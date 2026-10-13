"""
AXIS-OS — Chiffrement des apps (Play Store)
================================================
Répond à un besoin exprimé explicitement, distinct de la signature (voir
signing.py) : une fois livrée au Play Store (la boutique d'apps
d'AXIS-OS), une app ne doit être LISIBLE que par AXIS-OS lui-même, et
non par quiconque obtiendrait le fichier publié (copie du dépôt d'apps,
interception réseau, accès disque non autorisé sur l'infrastructure de
distribution). Le développeur garde bien sûr son code source en clair
sur sa propre machine ; c'est le fichier LIVRÉ qui devient illisible
sans la clé privée de la plateforme.

Signature et chiffrement sont deux choses différentes, à ne jamais
confondre :
  - signing.py (Ed25519, un algorithme de signature numérique) prouve
    l'ORIGINE et l'INTÉGRITÉ d'une app : « ce code vient bien de ce
    développeur et n'a pas été modifié ». Le code reste lisible par
    tous ; seule son authenticité est vérifiable.
  - encryption.py (ce module) rend le code ILLISIBLE sans la clé privée
    de la plateforme. L'origine et l'intégrité du contenu déchiffré
    restent garanties à part, par sa propre signature. Ce module est
    indépendant de signing.py et ne prend pas position sur l'ordre des
    deux opérations : chiffrer puis signer, ou l'inverse, selon le
    besoin.

Implémentation : HPKE (Hybrid Public Key Encryption, « chiffrement
hybride à clé publique »), normalisé par le RFC 9180 (un RFC est une
norme publiée par l'IETF, l'organisme qui standardise les protocoles
d'Internet). La suite cryptographique est DHKEM(X25519, HKDF-SHA256) /
HKDF-SHA256 / AES-256-GCM :
  - X25519 : échange de clés sur courbe elliptique, qui établit un
    secret partagé avec la clé publique de la plateforme ;
  - HKDF-SHA256 : dérivation de clés à partir de ce secret, fondée sur
    la fonction de hachage SHA-256 ;
  - AES-256-GCM : chiffrement symétrique authentifié du code lui-même.
C'est exactement la configuration que Google Tink (la bibliothèque
cryptographique de Google) recommande pour la plupart des usages
(DHKEM_X25519_HKDF_SHA256_HKDF_SHA256_AES_256_GCM).

Le module passe par la bibliothèque tierce `pyhpke`
(github.com/dajiaji/pyhpke, licence MIT) au lieu d'assembler lui-même
les primitives (les briques cryptographiques de base). Une première
version composait X25519, HKDF et AES-GCM
à la main : chaque primitive était déjà correcte, mais assembler
soi-même de la cryptographie expose à de vraies erreurs d'assemblage,
qu'aucune primitive, aussi solide soit-elle, ne neutralise. `pyhpke`
est testée contre les vecteurs de test OFFICIELS du RFC 9180 (des
couples entrée/sortie de référence publiés avec la norme,
github.com/cfrg/draft-irtf-cfrg-hpke/test-vectors.json), une garantie
qu'une composition maison ne peut pas offrir.

Le module HPKE intégré à `cryptography` (la bibliothèque Python de
cryptographie déjà utilisée par le projet), ajouté dans sa version
50.0.1, a été envisagé en premier, puis écarté : il exige un backend
AWS-LC ou BoringSSL (des bibliothèques cryptographiques alternatives à
OpenSSL), indisponible avec l'installation OpenSSL standard utilisée ici.
Recompiler `cryptography` contre un autre backend serait un chantier
d'infrastructure disproportionné pour ce besoin précis.

Principe (chiffrement « enveloppe », ou hybride : une paire de clés
asymétrique protège un secret à usage unique, et c'est ce secret qui
chiffre le contenu) :
  - AXIS-OS génère UNE SEULE paire de clés X25519 « plateforme »
    (generate_platform_keypair(), exécutée une fois, hors ligne). La
    clé privée ne quitte jamais l'infrastructure AXIS-OS. La clé
    publique est distribuée aux développeurs : n'importe qui peut
    chiffrer avec elle, personne ne peut déchiffrer avec elle.
  - Le développeur chiffre son app avec encrypt_app_code(). En
    coulisse, HPKE génère une paire de clés éphémère (à usage unique),
    propre à cette app et même à chaque chiffrement, encapsule un
    secret pour la clé publique de la plateforme, puis chiffre le code
    avec AES-256-GCM. Ce chiffrement est authentifié : toute altération
    du fichier chiffré est détectée au déchiffrement, le fichier n'est
    donc pas seulement protégé en lecture.
  - À l'installation, le chargeur d'apps chiffrées
    (axis_os/store/runtime/packaging.py) déchiffre le fichier EN
    MÉMOIRE avec decrypt_app_code() et la clé privée de la plateforme,
    sans jamais le réécrire en clair sur disque, et en tire une
    instance de l'app, que l'appelant confie ensuite à
    PlayStoreRuntime.install().

Pourquoi X25519 plutôt que de réutiliser Ed25519 (déjà présent dans
signing.py) : Ed25519 est une clé de SIGNATURE, pas de chiffrement. Par
principe, les deux usages ne doivent pas partager la même paire de clés
(séparation des clés par usage). X25519 est la courbe du KEM (Key
Encapsulation Mechanism, le mécanisme qui transmet un secret, chiffré
pour une clé publique, au seul détenteur de la clé privée
correspondante) recommandé par le RFC 9180 pour HPKE, le standard de
fait de tout schéma moderne de chiffrement enveloppe.

Ce que ce module ne fait PAS (limites volontaires et documentées, dans
le même esprit que signing.py) :
  - Pas de rotation automatisée de la clé plateforme. Changer de clé
    privée plateforme rend illisibles toutes les apps déjà chiffrées
    avec l'ancienne clé publique. C'est un chantier distinct de gestion
    de clés (KMS, un service de gestion de clés ; versionnement des
    clés), hors de portée ici.
  - Pas de protection contre une clé privée plateforme compromise : si
    elle fuit, n'importe qui peut déchiffrer n'importe quelle app déjà
    publiée. En revanche, la fuite isolée du secret éphémère d'une app
    n'expose que cette app (propriété de « forward secrecy », ou
    confidentialité persistante, par message, inhérente à HPKE : chaque
    chiffrement a son propre secret). Ce sont deux menaces
    différentes ; seule la seconde est couverte ici.
  - Ne remplace ni la sandbox (le processus séparé dans lequel tourne
    une app) ni les permissions : une app déchiffrée avec succès reste
    soumise aux contrôles Permissions et Capabilities (les droits et
    les capacités déclarés dans son manifest, sa fiche descriptive) et
    à SandboxRunner (qui lance la sandbox), exactement comme le
    documente signing.py. Réussir à déchiffrer prouve seulement que le
    fichier a été chiffré pour AXIS-OS, pas que le code lui-même est
    bienveillant.

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
    Levée quand une clé ou une enveloppe chiffrée est malformée.

    Elle n'est jamais levée quand le DÉCHIFFREMENT échoue à cause d'une
    mauvaise clé ou d'un fichier altéré : c'est le rôle de
    DecryptionError, ci-dessous. On retrouve la même distinction que
    dans signing.py (SignatureError d'un côté, résultat booléen de
    l'autre), exprimée ici par une exception dédiée plutôt que par une
    valeur de retour : un déchiffrement échoué ne laisse aucun résultat
    partiel exploitable (alors qu'une vérification de signature
    retourne simplement False), et il n'y a rien d'autre à faire que
    refuser l'installation.
    """
    pass


class DecryptionError(Exception):
    """
    Levée quand une enveloppe chiffrée ne peut pas être déchiffrée avec
    la clé privée plateforme fournie : mauvaise clé, fichier corrompu,
    ou fichier altéré après chiffrement.

    C'est le composant AEAD de HPKE (Authenticated Encryption with
    Associated Data, un chiffrement qui authentifie aussi ce qu'il
    chiffre), ici AES-256-GCM, qui permet de le détecter : toute
    modification du texte chiffré, même d'un seul octet, fait échouer
    le déchiffrement au lieu de produire en silence un texte clair
    corrompu.
    """
    pass


@dataclass(frozen=True)
class PlatformKeypair:
    """
    Paire de clés X25519 « plateforme », encodées en base64 (un encodage
    qui représente des octets sous forme de texte).

    generate_platform_keypair() ne doit être exécutée QU'UNE SEULE FOIS,
    hors ligne, par l'exploitant d'AXIS-OS : jamais par un développeur
    d'app, jamais côté client. `private_key_b64` ne doit jamais être
    distribuée, committée dans un dépôt ni transmise à qui que ce soit :
    c'est littéralement la seule clé au monde qui permette de déchiffrer
    une app publiée. Seule `public_key_b64` est distribuée aux
    développeurs, dans la documentation publique du SDK (le kit de
    développement d'apps) ou intégrée à l'outil officiel de packaging
    (la mise en paquet d'une app avant sa livraison).
    """
    private_key_b64: str
    public_key_b64:  str


@dataclass(frozen=True)
class EncryptedApp:
    """
    Enveloppe chiffrée complète d'une app : elle contient tout ce qu'il
    faut pour déchiffrer, sauf la clé privée plateforme, qui n'y figure
    jamais. to_bytes() et from_bytes() la sérialisent pour l'écrire ou
    la lire comme un seul fichier (extension .axisapp, voir
    axis_os/store/runtime/packaging.py).

    `encapsulated_key_b64` correspond au champ `enc` de la norme HPKE
    (RFC 9180, §5.1) : la clé publique éphémère encapsulée par HPKE,
    dont le destinataire a besoin pour reconstruire le secret partagé.
    """
    encapsulated_key_b64: str
    ciphertext_b64:       str

    def to_bytes(self) -> bytes:
        """
        Sérialisation binaire simple : un format texte structuré, une
        ligne par champ (le base64 passe déjà sans risque dans du
        texte).

        Volontairement pas du JSON : pour ce format de fichier critique
        en sécurité, on évite toute dépendance à un analyseur JSON, et
        le format reste facile à inspecter ou à comparer (diff) lors
        d'un diagnostic. Le contenu reste illisible sans la clé ; seule
        la STRUCTURE du fichier est lisible, ce qui n'affaiblit en rien
        le chiffrement lui-même.
        """
        lines = [
            "AXISAPP-ENCRYPTED-V1",
            f"enc:{self.encapsulated_key_b64}",
            f"ct:{self.ciphertext_b64}",
        ]
        return "\n".join(lines).encode("ascii")

    @staticmethod
    def from_bytes(data: bytes) -> "EncryptedApp":
        """
        Relit une enveloppe écrite par to_bytes().

        Lève EncryptionError si les données ne sont pas de l'ASCII, si
        l'en-tête AXISAPP-ENCRYPTED-V1 manque, si le nombre de lignes
        est incorrect, si une ligne n'a pas la forme « nom:valeur », ou
        si le champ `enc` ou `ct` manque. La validité du base64 des
        champs n'est vérifiée qu'au déchiffrement (decrypt_app_code()).
        """
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


# Une seule suite cryptographique pour tout le module :
# DHKEM(X25519, HKDF-SHA256) / HKDF-SHA256 / AES-256-GCM. La
# justification de ce choix précis est dans la docstring du module.
_SUITE = CipherSuite.new(
    KEMId.DHKEM_X25519_HKDF_SHA256, KDFId.HKDF_SHA256, AEADId.AES256_GCM,
)

# Chaîne « info » de HPKE : un identifiant de contexte fixe, mêlé à la
# dérivation des clés, qui distingue ce schéma de tout autre usage de
# HPKE avec la même paire de clés (voir RFC 9180, §5.1). Ce n'est pas un
# secret.
_HPKE_INFO = b"AXIS-OS app encryption v1"


def _decode_x25519_key_b64(key_b64: str, *, is_private: bool, label: str):
    """Décode une clé X25519 depuis le base64 et retourne un objet
    X25519PrivateKey ou X25519PublicKey natif de `cryptography`.

    Aucun objet interne à pyhpke n'est exposé directement à l'appelant
    de ce module : encryption.py reste ainsi cohérent avec le reste du
    projet (signing.py utilise les mêmes types de clés natifs). Lève
    EncryptionError si le base64 est invalide ou si la clé ne fait pas
    32 octets."""
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
    Génère une nouvelle paire de clés X25519 « plateforme » (contexte
    complet dans PlatformKeypair).

    À exécuter UNE SEULE FOIS pour toute la durée de vie d'AXIS-OS : un
    nouvel appel produirait une nouvelle clé, et toutes les apps déjà
    chiffrées avec l'ancienne clé publique deviendraient illisibles
    (voir, en tête de module, la limite sur l'absence de rotation
    automatisée).
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
    fournie.

    Exécuté AVANT publication par le développeur ou par l'outil de
    packaging officiel, jamais côté AXIS-OS (le chargement côté
    AXIS-OS, lui, se trouve dans axis_os/store/runtime/packaging.py).
    Seule la clé privée plateforme, jamais distribuée, permet de
    déchiffrer le résultat : le développeur lui-même ne peut pas relire
    le fichier chiffré une fois produit. Il garde bien sûr son
    code source original en clair de son côté ; c'est le fichier livré
    qui devient illisible.

    HPKE génère en interne une nouvelle paire de clés éphémère à CHAQUE
    appel : deux chiffrements de la même app, même à contenu identique,
    produisent des enveloppes différentes, qu'on ne peut pas rapprocher
    l'une de l'autre. C'est la propriété standard IND-CPA
    (indistinguabilité face à une attaque à clair choisi : rien ne
    permet de dire si deux enveloppes contiennent le même code),
    garantie par le RFC ; ce n'est jamais un simple hachage
    déterministe du contenu.
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
    Déchiffre une app avec la clé privée plateforme.

    Appelé uniquement sur le chemin d'installation : le chargeur
    d'axis_os/store/runtime/packaging.py déchiffre l'app et en prépare
    une instance, que l'appelant confie ensuite à
    PlayStoreRuntime.install() (voir ce fichier pour le branchement
    complet). Le déchiffrement se fait TOUJOURS en mémoire, sans jamais
    réécrire le code en clair sur disque.

    Lève DecryptionError sur une mauvaise clé ou une altération du
    fichier : le composant AEAD de HPKE (le chiffrement authentifié,
    ici AES-256-GCM) fait échouer ici un fichier corrompu, tronqué ou
    modifié après chiffrement, au lieu de produire un texte clair
    corrompu qui serait ensuite exécuté silencieusement comme du vrai
    code Python. Lève EncryptionError si la clé ou l'enveloppe est
    elle-même malformée.
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
    """Raccourci : lit un fichier .py (en octets, tel quel) et le
    chiffre. Voir encrypt_app_code()."""
    content = Path(file_path).read_bytes()
    return encrypt_app_code(content, platform_public_key_b64)


def decrypt_app_file_to_bytes(
    encrypted_file_path: Union[str, Path], platform_private_key_b64: str,
) -> bytes:
    """Raccourci : lit un fichier .axisapp chiffré et le déchiffre en
    mémoire (voir decrypt_app_code()). Cette fonction ne réécrit JAMAIS
    elle-même le résultat en clair sur disque ; c'est à l'appelant de
    ne pas le faire non plus."""
    data = Path(encrypted_file_path).read_bytes()
    encrypted_app = EncryptedApp.from_bytes(data)
    return decrypt_app_code(encrypted_app, platform_private_key_b64)
