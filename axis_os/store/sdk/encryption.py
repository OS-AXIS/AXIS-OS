"""
AXIS-OS — Chiffrement des apps (Play Store)
================================================
Comble une demande explicite, distincte de la signature (voir
signing.py) : une app, une fois livrée au Play Store, ne doit être
LISIBLE que par AXIS-OS lui-même — pas par n'importe qui qui
obtiendrait le fichier publié (copie du dépôt d'apps, interception
réseau, accès disque non autorisé sur l'infrastructure de
distribution). Le développeur garde évidemment son code source en
clair sur SA propre machine ; c'est le fichier LIVRÉ qui devient
illisible sans la clé privée plateforme.

Distinction avec signing.py, à ne jamais confondre :
  - signing.py (Ed25519) prouve l'ORIGINE et l'INTÉGRITÉ d'une app —
    "ce code vient bien de ce développeur, n'a pas été modifié" —
    le code reste LISIBLE par tous, seule son authenticité est
    vérifiable.
  - encryption.py (ce module) rend le code ILLISIBLE sans la clé
    privée plateforme — l'origine/l'intégrité du contenu déchiffré
    restent séparément garanties par sa propre signature (le flux
    normal est CHIFFRER PUIS SIGNER, ou l'inverse selon le besoin —
    ce module ne prend pas position sur l'ordre, orthogonal à
    signing.py).

Implémentation : HPKE (Hybrid Public Key Encryption, RFC 9180),
cipher suite DHKEM(X25519, HKDF-SHA256) / HKDF-SHA256 / AES-256-GCM —
exactement la configuration recommandée par Google Tink pour la
plupart des usages (DHKEM_X25519_HKDF_SHA256_HKDF_SHA256_AES_256_GCM).

Via la bibliothèque tierce `pyhpke` (github.com/dajiaji/pyhpke,
licence MIT), pas une composition manuelle des primitives — décision
prise après une première version qui composait X25519+HKDF+AES-GCM à
la main : les primitives individuelles étaient déjà correctes, mais
composer soi-même de la cryptographie comporte un vrai risque
d'erreur d'assemblage qu'aucune primitive individuellement solide ne
neutralise. `pyhpke` est testée contre les vecteurs de test OFFICIELS
du RFC 9180 (github.com/cfrg/draft-irtf-cfrg-hpke/test-vectors.json),
une garantie qu'une composition maison ne peut pas offrir. Le module
HPKE natif de `cryptography` (ajouté en version 50.0.1) a été
envisagé en premier mais écarté : il exige un backend AWS-LC ou
BoringSSL, indisponible avec l'installation OpenSSL standard utilisée
ici — recompiler `cryptography` contre un backend alternatif est un
chantier d'infrastructure disproportionné pour ce besoin précis.

Principe (chiffrement enveloppe / hybrid encryption) :
  - AXIS-OS génère UNE SEULE paire de clés X25519 "plateforme"
    (generate_platform_keypair(), exécuté une fois, hors ligne).
    La clé PRIVÉE ne quitte jamais l'infrastructure AXIS-OS. La clé
    PUBLIQUE est distribuée aux développeurs (n'importe qui peut
    chiffrer avec elle, personne ne peut déchiffrer avec elle).
  - Un développeur chiffre son app avec encrypt_app_code() — HPKE
    génère en coulisse une paire de clés éphémère, unique à CETTE
    app précise, encapsule un secret pour la clé publique plateforme,
    et chiffre le code avec AES-256-GCM (authentifié — toute
    altération du fichier chiffré est détectée au déchiffrement, pas
    seulement empêchée en lecture).
  - PlayStoreRuntime.install() déchiffre EN MÉMOIRE avec
    decrypt_app_code() et la clé privée plateforme — jamais
    réécrit en clair sur disque.

Pourquoi X25519 plutôt que réutiliser Ed25519 (déjà présent dans
signing.py) : Ed25519 est une clé de SIGNATURE, pas de chiffrement —
les deux usages ne doivent structurellement pas partager la même
paire de clés (principe de séparation des clés par usage). X25519
est la courbe du KEM (Key Encapsulation Mechanism) recommandé par
RFC 9180 pour HPKE — le standard de facto de tout schéma de
chiffrement enveloppe moderne.

Ce que ce module NE fait PAS (limites volontaires, documentées, même
esprit que signing.py) :
  - Pas de rotation de clé plateforme automatisée — un changement de
    clé privée plateforme rend illisibles toutes les apps déjà
    chiffrées avec l'ancienne clé publique. Chantier de gestion de
    clés distinct (KMS, versionnement de clé), hors de portée ici.
  - Pas de protection contre une clé privée plateforme compromise —
    si cette clé fuit, n'importe qui peut déchiffrer n'importe quelle
    app déjà publiée. Chaque app individuelle reste néanmoins
    protégée d'un secret éphémère compromis pris isolément (propriété
    de "forward secrecy" par message, inhérente à HPKE) — deux
    menaces différentes, la seconde seule est couverte ici.
  - Ne remplace ni le sandboxing ni les permissions — une app
    déchiffrée avec succès reste soumise à Permissions/Capabilities/
    SandboxRunner, exactement comme documenté dans signing.py :
    déchiffrer prouve seulement que le fichier a été chiffré pour
    AXIS-OS, pas que le CODE lui-même est bienveillant.

Licence : Apache 2.0
"""

from __future__ import annotations
import base64
from dataclasses import dataclass
from pathlib import Path
from typing import Union

from cryptography.hazmat.primitives.asymmetric.x25519 import (
    X25519PrivateKey, X25519PublicKey,
)
from pyhpke import AEADId, CipherSuite, KDFId, KEMId, KEMKey
from pyhpke.exceptions import PyHPKEError, OpenError as _PyHPKEOpenError


class EncryptionError(Exception):
    """
    Levée sur toute clé ou enveloppe chiffrée malformée — jamais sur
    un échec de DÉCHIFFREMENT dû à une mauvaise clé ou un fichier
    altéré (voir DecryptionError ci-dessous), même distinction que
    SignatureError/résultat booléen dans signing.py : ici la
    distinction se fait par exception dédiée plutôt que par valeur de
    retour, car un déchiffrement échoué n'a PAS de résultat partiel
    exploitable (contrairement à une vérification de signature, qui
    retourne sobrement False) — il n'y a rien d'autre à faire que de
    refuser l'installation.
    """
    pass


class DecryptionError(Exception):
    """
    Levée quand une enveloppe chiffrée ne peut pas être déchiffrée
    avec la clé privée plateforme fournie — mauvaise clé, fichier
    corrompu, ou fichier altéré après chiffrement (le composant AEAD
    d'HPKE, AES-256-GCM, est authentifié : toute modification, même
    d'un seul octet, du texte chiffré fait échouer le déchiffrement
    plutôt que de produire un texte clair corrompu silencieusement).
    """
    pass


@dataclass(frozen=True)
class PlatformKeypair:
    """
    Paire de clés X25519 "plateforme" — generate_platform_keypair()
    ne doit être exécutée QU'UNE SEULE FOIS, hors ligne, par
    l'exploitant d'AXIS-OS (jamais par un développeur d'app, jamais
    côté client). `private_key_b64` ne doit JAMAIS être distribuée,
    committée dans un dépôt, ni transmise à qui que ce soit — c'est
    littéralement la seule clé au monde qui permette de déchiffrer
    une app publiée. Seule `public_key_b64` est distribuée aux
    développeurs (documentation publique du SDK, ou embarquée dans
    l'outil de packaging officiel).
    """
    private_key_b64: str
    public_key_b64:  str


@dataclass(frozen=True)
class EncryptedApp:
    """
    Enveloppe chiffrée complète d'une app — tout ce qui est
    nécessaire pour déchiffrer (sauf la clé privée plateforme,
    jamais incluse ici) est contenu dans cette structure. Sérialisée
    par to_bytes()/from_bytes() pour être écrite/lue comme un seul
    fichier (l'extension .axisapp, voir packaging.py).

    `encapsulated_key_b64` correspond au `enc` du standard HPKE
    (RFC 9180 §5.1) — la clé publique éphémère encapsulée par HPKE,
    nécessaire au destinataire pour reconstruire le secret partagé.
    """
    encapsulated_key_b64: str
    ciphertext_b64:       str

    def to_bytes(self) -> bytes:
        """
        Sérialisation binaire simple — un format texte structuré
        (une ligne par champ, base64 déjà "safe" pour du texte),
        volontairement pas JSON : évite toute dépendance à un
        analyseur JSON pour ce format de fichier critique en
        sécurité, et rend le format trivialement inspectable/
        diffable en cas de besoin de diagnostic (même si le contenu
        reste illisible sans la clé, la STRUCTURE du fichier reste
        lisible — ça n'affaiblit en rien le chiffrement lui-même).
        """
        lines = [
            "AXISAPP-ENCRYPTED-V1",
            f"enc:{self.encapsulated_key_b64}",
            f"ct:{self.ciphertext_b64}",
        ]
        return "\n".join(lines).encode("ascii")

    @staticmethod
    def from_bytes(data: bytes) -> "EncryptedApp":
        try:
            text = data.decode("ascii")
        except UnicodeDecodeError as e:
            raise EncryptionError(
                f"Enveloppe chiffrée illisible (pas de l'ASCII valide) : {e}"
            ) from e
        lines = text.strip().split("\n")
        if len(lines) != 3 or lines[0] != "AXISAPP-ENCRYPTED-V1":
            raise EncryptionError(
                "Enveloppe chiffrée malformée — en-tête AXISAPP-ENCRYPTED-V1 "
                "absent ou nombre de champs incorrect."
            )
        fields = {}
        for line in lines[1:]:
            if ":" not in line:
                raise EncryptionError(f"Ligne malformée dans l'enveloppe : {line!r}")
            key, _, value = line.partition(":")
            fields[key] = value
        missing = {"enc", "ct"} - fields.keys()
        if missing:
            raise EncryptionError(f"Champ(s) manquant(s) dans l'enveloppe : {missing}")
        return EncryptedApp(
            encapsulated_key_b64=fields["enc"],
            ciphertext_b64=fields["ct"],
        )


# Cipher suite unique pour tout le module — DHKEM(X25519, HKDF-SHA256)
# / HKDF-SHA256 / AES-256-GCM, voir la docstring de module pour la
# justification de ce choix précis.
_SUITE = CipherSuite.new(
    KEMId.DHKEM_X25519_HKDF_SHA256, KDFId.HKDF_SHA256, AEADId.AES256_GCM,
)

# Chaîne "info" HPKE — liant contextuel qui distingue ce schéma
# d'un autre usage d'HPKE avec la même paire de clés (voir RFC 9180
# §5.1) — pas un secret, juste un identifiant de contexte fixe.
_HPKE_INFO = b"AXIS-OS app encryption v1"


def _decode_x25519_key_b64(key_b64: str, *, is_private: bool, label: str):
    """Décode une clé X25519 depuis base64 et retourne un objet
    X25519PrivateKey/X25519PublicKey natif de `cryptography` — jamais
    un objet interne à pyhpke exposé directement à l'appelant de ce
    module, pour garder encryption.py cohérent avec le reste du
    projet (signing.py utilise les mêmes types de clé natifs)."""
    try:
        key_bytes = base64.b64decode(key_b64, validate=True)
    except Exception as e:
        raise EncryptionError(f"{label} malformée (base64 invalide) : {e}") from e
    if len(key_bytes) != 32:
        raise EncryptionError(
            f"{label} X25519 invalide — attendu 32 octets, obtenu {len(key_bytes)}."
        )
    if is_private:
        return X25519PrivateKey.from_private_bytes(key_bytes)
    return X25519PublicKey.from_public_bytes(key_bytes)


def generate_platform_keypair() -> PlatformKeypair:
    """
    Génère une nouvelle paire de clés X25519 "plateforme" — voir
    PlatformKeypair pour le contexte complet. À exécuter UNE SEULE
    FOIS pour toute la durée de vie d'AXIS-OS (voir la limite
    documentée en tête de module sur l'absence de rotation
    automatisée) — un appel répété générerait une NOUVELLE clé,
    rendant illisibles toutes les apps déjà chiffrées avec l'ancienne
    clé publique.
    """
    private_key = X25519PrivateKey.generate()
    public_key = private_key.public_key()
    return PlatformKeypair(
        private_key_b64=base64.b64encode(
            private_key.private_bytes_raw()
        ).decode("ascii"),
        public_key_b64=base64.b64encode(
            public_key.public_bytes_raw()
        ).decode("ascii"),
    )


def encrypt_app_code(
    source_code: Union[str, bytes], platform_public_key_b64: str,
) -> EncryptedApp:
    """
    Chiffre le code source d'une app pour la clé publique plateforme
    fournie — exécuté par le développeur (ou l'outil de packaging
    officiel, voir packaging.py) AVANT publication, jamais côté
    AXIS-OS. Seule la clé PRIVÉE plateforme (jamais distribuée)
    permet de déchiffrer le résultat — le développeur lui-même ne
    peut PAS relire le fichier chiffré une fois produit (il garde
    évidemment son code source original en clair de son côté, c'est
    le FICHIER LIVRÉ qui devient illisible).

    HPKE génère en interne une nouvelle paire de clés éphémère à
    CHAQUE appel — deux chiffrements de la même app, même avec un
    contenu identique, produisent des enveloppes DIFFÉRENTES et non
    corrélables (propriété IND-CPA standard, garantie par le RFC,
    jamais un simple hachage déterministe du contenu).
    """
    if isinstance(source_code, str):
        source_code = source_code.encode("utf-8")

    platform_public_key = _decode_x25519_key_b64(
        platform_public_key_b64, is_private=False, label="Clé publique plateforme"
    )
    recipient_key = KEMKey.from_pyca_cryptography_key(platform_public_key)

    try:
        encapsulated_key, sender = _SUITE.create_sender_context(
            recipient_key, info=_HPKE_INFO,
        )
        ciphertext = sender.seal(source_code)
    except PyHPKEError as e:
        raise EncryptionError(f"Échec de préparation du chiffrement HPKE : {e}") from e

    return EncryptedApp(
        encapsulated_key_b64=base64.b64encode(encapsulated_key).decode("ascii"),
        ciphertext_b64=base64.b64encode(ciphertext).decode("ascii"),
    )


def decrypt_app_code(
    encrypted_app: EncryptedApp, platform_private_key_b64: str,
) -> bytes:
    """
    Déchiffre une app avec la clé privée plateforme — exécuté
    UNIQUEMENT par PlayStoreRuntime.install() (voir packaging.py pour
    le branchement complet), et TOUJOURS en mémoire, jamais réécrit
    en clair sur disque.

    Lève DecryptionError sur toute mauvaise clé ou altération du
    fichier — le composant AEAD d'HPKE (AES-256-GCM) est authentifié :
    un fichier corrompu, tronqué, ou modifié après chiffrement échoue
    ici plutôt que de produire un texte clair corrompu qui serait
    ensuite exécuté silencieusement comme du vrai code Python.
    """
    platform_private_key = _decode_x25519_key_b64(
        platform_private_key_b64, is_private=True, label="Clé privée plateforme"
    )
    recipient_key = KEMKey.from_pyca_cryptography_key(platform_private_key)

    try:
        encapsulated_key = base64.b64decode(
            encrypted_app.encapsulated_key_b64, validate=True
        )
        ciphertext = base64.b64decode(encrypted_app.ciphertext_b64, validate=True)
    except Exception as e:
        raise EncryptionError(f"Enveloppe chiffrée malformée (base64 invalide) : {e}") from e

    try:
        recipient = _SUITE.create_recipient_context(
            encapsulated_key, recipient_key, info=_HPKE_INFO,
        )
        plaintext = recipient.open(ciphertext)
    except _PyHPKEOpenError as e:
        raise DecryptionError(
            "Déchiffrement échoué — mauvaise clé privée plateforme, ou "
            "fichier corrompu/altéré après chiffrement (l'authentification "
            "AEAD a détecté une incohérence)."
        ) from e
    except PyHPKEError as e:
        raise EncryptionError(
            f"Enveloppe chiffrée invalide (clé encapsulée malformée) : {e}"
        ) from e

    return plaintext


def encrypt_app_file(file_path: Union[str, Path], platform_public_key_b64: str) -> EncryptedApp:
    """Raccourci : lit un fichier .py et le chiffre. Voir encrypt_app_code()."""
    content = Path(file_path).read_bytes()
    return encrypt_app_code(content, platform_public_key_b64)


def decrypt_app_file_to_bytes(
    encrypted_file_path: Union[str, Path], platform_private_key_b64: str,
) -> bytes:
    """Raccourci : lit un fichier .axisapp chiffré et le déchiffre en
    mémoire. Voir decrypt_app_code() — le résultat n'est JAMAIS
    réécrit en clair sur disque par cette fonction elle-même ; c'est
    la responsabilité de l'appelant de ne pas le faire non plus."""
    data = Path(encrypted_file_path).read_bytes()
    encrypted_app = EncryptedApp.from_bytes(data)
    return decrypt_app_code(encrypted_app, platform_private_key_b64)
