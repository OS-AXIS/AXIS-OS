"""
AXIS-OS — Signature des apps (Play Store)
=============================================
Le Play Store est la boutique d'apps d'AXIS-OS (axis_os/store/) : c'est
par lui qu'une app tierce s'installe sur le robot. Ce module comble un
trou de sécurité repéré dès le début du projet et resté ouvert jusque-là :
rien n'empêchait un fichier .py quelconque de se faire passer pour une
app AXIS-OS légitime, car l'installation ne vérifiait ni l'origine ni
l'intégrité du code.

Principe (Ed25519, un algorithme de signature numérique à clé publique) :
  - Le développeur d'une app génère une fois pour toutes une paire de
    clés, privée et publique, avec generate_developer_keypair().
  - Il signe le CODE SOURCE de son app avec sa clé privée
    (sign_app_code()). Cette clé n'est jamais transmise à AXIS-OS, qui
    ne la stocke jamais.
  - Avant d'installer l'app, PlayStoreRuntime.install() vérifie cette
    signature avec la clé publique du développeur
    (verify_app_signature()). Toute modification du code après
    signature rend la signature invalide.

Pourquoi Ed25519 plutôt que RSA (l'algorithme à clé publique historique) :
à niveau de sécurité équivalent, les signatures et les clés sont bien
plus courtes et la vérification nettement plus rapide. C'est aussi le
choix courant des écosystèmes modernes de signature de paquets : npm et
PyPI (les dépôts de paquets de JavaScript et de Python ; PyPI passe par
sigstore, un service public de signature de logiciels), ou encore, en
2026, des projets comparables comme les places de marché d'extensions
(plugins) pour agents IA. Ce n'est donc pas un format propre à AXIS-OS,
mais un choix d'alignement sur l'écosystème.

Ce que ce module ne fait PAS (limites volontaires et documentées) :
  - Pas d'autorité de certification (CA, un tiers de confiance qui
    atteste l'identité d'un développeur). La clé publique est
    aujourd'hui prise telle quelle dans le manifest de l'app (sa fiche
    descriptive, voir manifest.py) : c'est le modèle TOFU (Trust On
    First Use, « confiance à la première utilisation »). La signature
    empêche de falsifier le code après signature ; elle n'empêche pas
    un développeur malveillant de signer son propre code malveillant
    avec sa propre clé. La plupart des écosystèmes de paquets open
    source (npm, PyPI) ont la même limite à ce stade de maturité. Une
    vraie chaîne de confiance, comme Apple Developer ID (les certificats
    qu'Apple délivre aux développeurs identifiés) ou une CA à la manière
    de SROS2 (la sécurité de ROS 2, un cadre logiciel robotique
    répandu) et de DDS-Security (celle de DDS, Data Distribution
    Service, le standard d'échange de messages par lequel AXIS-OS parle
    aux moteurs de l'humanoïde Unitree G1), est un chantier distinct,
    pour plus tard : elle suppose une vraie infrastructure de
    distribution de clés.
  - Ne remplace pas la sandbox (le processus séparé dans lequel tourne
    une app, lancé par SandboxRunner) : une app signée reste soumise
    aux contrôles Permissions et Capabilities (les droits et les
    capacités déclarés dans son manifest), au verrou de mission de
    PlayStoreRuntime (une seule mission active à la fois sur le robot)
    et aux quotas de la sandbox (processeur, mémoire, durée). Le
    ResourceManager, qui réservera chaque actionneur à une seule app à
    la fois, n'est pas encore branché (voir resource_manager.py). La
    signature prouve l'origine et l'intégrité du code, pas ses bonnes
    intentions.

Licence : Apache 2.0
"""

from __future__ import annotations
import base64
import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Union

from cryptography.hazmat.primitives.asymmetric.ed25519 import (
    Ed25519PrivateKey, Ed25519PublicKey,
)
from cryptography.exceptions import InvalidSignature


class SignatureError(Exception):
    """Levée quand une clé ou une signature est malformée.

    Elle n'est jamais levée quand la VÉRIFICATION échoue : dans ce cas,
    verify_app_signature() retourne False. Cette séparation permet à
    l'appelant de distinguer clairement « la signature ne correspond
    pas » (un cas normal, à gérer) de « les données fournies sont
    structurellement invalides » (une erreur de programmation, à
    corriger)."""
    pass


@dataclass(frozen=True)
class DeveloperKeypair:
    """
    Paire de clés Ed25519 d'un développeur, encodées en base64 (un
    encodage qui représente des octets sous forme de texte).

    `private_key_b64` ne doit JAMAIS être transmise à AXIS-OS ni
    committée dans un dépôt : elle reste sur la machine du développeur
    et ne sert qu'à signer localement avant publication. Seule
    `public_key_b64` est partagée, dans le manifest de l'app.
    """
    private_key_b64: str
    public_key_b64:  str


def generate_developer_keypair() -> DeveloperKeypair:
    """
    Génère une nouvelle paire de clés Ed25519 pour un développeur.

    À exécuter une seule fois, hors ligne, sur la machine du
    développeur, jamais côté serveur AXIS-OS. Les clés sont encodées en
    base64 pour se ranger facilement dans un fichier texte ou une
    variable d'environnement, mais jamais dans du code source
    versionné : selon les bonnes pratiques de gestion des secrets, un
    secret permanent ne doit jamais être codé en dur (voir
    axis_os/hal/core/structured_log.py pour le même principe appliqué
    aux journaux).
    """
    private_key = Ed25519PrivateKey.generate()
    public_key = private_key.public_key()

    private_bytes = private_key.private_bytes_raw()
    public_bytes = public_key.public_bytes_raw()

    return DeveloperKeypair(
        private_key_b64=base64.b64encode(private_bytes).decode("ascii"),
        public_key_b64=base64.b64encode(public_bytes).decode("ascii"),
    )


def sign_app_code(source_code: Union[str, bytes], private_key_b64: str) -> str:
    """
    Signe le code source d'une app avec la clé privée du développeur.

    Retourne la signature encodée en base64, à inclure dans le manifest
    de l'app (voir AppManifest.code_signature). Une chaîne `str` est
    encodée en UTF-8 avant d'être signée. Lève SignatureError si la clé
    privée n'est pas du base64 valide ou ne fait pas 32 octets.

    `source_code` est en général le contenu du fichier .py principal de
    l'app (app.py). La vérification (verify_app_signature()) doit
    recevoir EXACTEMENT le même contenu : un seul octet différent
    invalide la signature. C'est voulu, c'est précisément la garantie
    d'intégrité.
    """
    if isinstance(source_code, str):
        source_code = source_code.encode("utf-8")

    try:
        key_bytes = base64.b64decode(private_key_b64, validate=True)
    except Exception as e:
        raise SignatureError(f"Clé privée malformée (base64 invalide) : {e}") from e
    if len(key_bytes) != 32:
        raise SignatureError(
            f"Clé privée Ed25519 invalide — attendu 32 octets, obtenu {len(key_bytes)}."
        )

    private_key = Ed25519PrivateKey.from_private_bytes(key_bytes)
    signature = private_key.sign(source_code)
    return base64.b64encode(signature).decode("ascii")


def verify_app_signature(
    source_code: Union[str, bytes],
    signature_b64: str,
    public_key_b64: str,
) -> bool:
    """
    Vérifie que la signature correspond bien au code source fourni,
    pour la clé publique donnée, et retourne True ou False.

    Un échec de vérification (signature invalide, code modifié) ne lève
    JAMAIS d'exception : c'est le résultat attendu face à une
    falsification, pas une erreur de programmation. SignatureError
    n'est levée que si les données elles-mêmes sont structurellement
    invalides (clé ou signature malformée), jamais sur une simple
    non-correspondance.
    """
    if isinstance(source_code, str):
        source_code = source_code.encode("utf-8")

    try:
        key_bytes = base64.b64decode(public_key_b64, validate=True)
    except Exception as e:
        raise SignatureError(f"Clé publique malformée (base64 invalide) : {e}") from e
    if len(key_bytes) != 32:
        raise SignatureError(
            f"Clé publique Ed25519 invalide — attendu 32 octets, obtenu {len(key_bytes)}."
        )

    try:
        signature_bytes = base64.b64decode(signature_b64, validate=True)
    except Exception as e:
        raise SignatureError(f"Signature malformée (base64 invalide) : {e}") from e

    public_key = Ed25519PublicKey.from_public_bytes(key_bytes)
    try:
        public_key.verify(signature_bytes, source_code)
        return True
    except InvalidSignature:
        return False


# ── Paquet signé : manifeste déclaré + code (format 2) ─────────────
#
# Pourquoi : pour lire le manifeste d'une app (identité, permissions,
# quotas) avant de demander quoi que ce soit à l'humain, l'OS exécutait son
# code. Avec un manifeste déclaré à côté du code et couvert par la MÊME
# signature, l'OS lit le manifeste sans rien exécuter. Une seule signature
# pour les deux : avec deux signatures séparées, on pourrait associer le
# manifeste d'une version au code d'une autre, signés par la même clé.

_DOMAINE_PAQUET = b"AXIS-OS-APP-V2\x00"


def message_paquet(manifeste: str, source: str) -> bytes:
    """Les octets exacts que signe sign_app_package(). Le préfixe contient
    un octet nul, qu'aucun code source Python ne peut contenir : une
    signature de paquet n'est jamais valable comme signature d'un code seul
    (format 1, sign_app_code), ni l'inverse. La longueur du manifeste fixe
    sans ambiguïté sa frontière avec le code."""
    try:
        m, s = manifeste.encode("utf-8"), source.encode("utf-8")
    except (AttributeError, UnicodeEncodeError) as e:
        raise SignatureError(f"manifeste ou code non encodable en UTF-8 : {e}") from None
    return _DOMAINE_PAQUET + len(m).to_bytes(8, "big") + m + s


def sign_app_package(manifeste: str, source: str, private_key_b64: str) -> str:
    """Signe ensemble le manifeste déclaré (texte JSON, voir
    manifeste_json.manifeste_texte) et le code source d'une app. C'est la
    signature qu'exige PlayStoreRuntime.install_source()."""
    return sign_app_code(message_paquet(manifeste, source), private_key_b64)


def verify_app_package(manifeste: str, source: str, signature_b64: str,
                       public_key_b64: str) -> bool:
    """Vérifie une signature de sign_app_package() ; mêmes règles que
    verify_app_signature() (False si elle ne correspond pas, SignatureError
    si une donnée est malformée)."""
    return verify_app_signature(message_paquet(manifeste, source), signature_b64,
                                public_key_b64)


def public_key_fingerprint(public_key_b64: str) -> str:
    """Empreinte courte d'une clé publique, à montrer à un humain :
    « SHA256:» suivi des 16 premiers chiffres hexadécimaux du SHA-256 des 32
    octets de la clé, par groupes de quatre. Deux clés différentes ont, en
    pratique, des empreintes différentes ; le nom d'auteur d'un manifeste,
    lui, peut être recopié par n'importe qui. Lève SignatureError si la clé
    est malformée."""
    try:
        octets = base64.b64decode(public_key_b64, validate=True)
    except Exception as e:
        raise SignatureError(f"Clé publique malformée (base64 invalide) : {e}") from e
    if len(octets) != 32:
        raise SignatureError(
            f"Clé publique Ed25519 invalide — attendu 32 octets, obtenu {len(octets)}.")
    h = hashlib.sha256(octets).hexdigest()[:16]
    return "SHA256:" + " ".join(h[i:i + 4] for i in range(0, 16, 4))


def sign_app_file(file_path: Union[str, Path], private_key_b64: str) -> str:
    """Raccourci : lit un fichier (en UTF-8) et le signe. Voir
    sign_app_code()."""
    content = Path(file_path).read_text(encoding="utf-8")
    return sign_app_code(content, private_key_b64)


def verify_app_file(
    file_path: Union[str, Path], signature_b64: str, public_key_b64: str,
) -> bool:
    """Raccourci : lit un fichier (en UTF-8) et vérifie sa signature.
    Voir verify_app_signature()."""
    content = Path(file_path).read_text(encoding="utf-8")
    return verify_app_signature(content, signature_b64, public_key_b64)
