"""
AXIS-OS — Signature des apps (Play Store)
=============================================
Comble un trou de sécurité identifié dès le début du projet et jamais
traité jusqu'ici : rien n'empêchait un fichier .py quelconque de se
faire passer pour une "app AXIS-OS" légitime — aucune vérification
d'origine ni d'intégrité à l'installation.

Principe (Ed25519, signature à clé publique) :
  - Un développeur d'app génère une paire de clés (privée/publique)
    une fois, avec generate_developer_keypair().
  - Il signe le CODE SOURCE de son app avec sa clé privée
    (sign_app_code()) — jamais transmise, jamais stockée par AXIS-OS.
  - PlayStoreRuntime.install() vérifie cette signature avec la clé
    PUBLIQUE du développeur (verify_app_signature()) avant d'installer.
    Toute modification du code après signature invalide la signature.

Pourquoi Ed25519 plutôt que RSA : signatures et clés bien plus courtes
à taille de sécurité équivalente, vérification nettement plus rapide,
c'est le choix standard des écosystèmes modernes de signature de
paquets (npm, PyPI via sigstore, et des projets comparables comme les
marketplaces de plugins pour agents IA en 2026) — pas une IP standard
imposée par AXIS-OS, un choix de convergence avec l'écosystème.

Ce que ce module NE fait PAS (limites volontaires, documentées) :
  - Pas d'autorité de certification (CA) centrale qui vérifierait
    l'IDENTITÉ du développeur — la clé publique est actuellement prise
    "telle quelle" dans le manifest (Trust On First Use). Empêche la
    falsification du CODE après signature, n'empêche PAS qu'un
    développeur malveillant signe son propre code malveillant avec sa
    propre clé. C'est la même limite que la plupart des écosystèmes de
    paquets open-source à ce stade de maturité (npm, PyPI) — une vraie
    chaîne de confiance (comme Apple Developer ID, ou une CA façon
    SROS2/DDS-Security) est un chantier ultérieur distinct, qui suppose
    une vraie infrastructure de distribution de clés.
  - Ne remplace PAS le sandboxing (SandboxRunner) : une app signée
    reste soumise à Permissions/Capabilities/ResourceManager/quotas —
    la signature prouve l'origine et l'intégrité, pas la bienveillance
    du code.

Licence : Apache 2.0
"""

from __future__ import annotations
import base64
from dataclasses import dataclass
from pathlib import Path
from typing import Union

from cryptography.hazmat.primitives.asymmetric.ed25519 import (
    Ed25519PrivateKey, Ed25519PublicKey,
)
from cryptography.exceptions import InvalidSignature


class SignatureError(Exception):
    """Levée sur toute clé ou signature malformée — jamais sur un
    échec de VÉRIFICATION de signature (qui retourne False, voir
    verify_app_signature), afin que l'appelant distingue clairement
    "la signature ne correspond pas" (cas normal à gérer) de "les
    données fournies sont structurellement invalides" (erreur de
    programmation à corriger)."""
    pass


@dataclass(frozen=True)
class DeveloperKeypair:
    """
    Paire de clés Ed25519 d'un développeur. `private_key_b64` ne doit
    JAMAIS être transmise à AXIS-OS ni committée dans un dépôt — elle
    reste sur la machine du développeur, utilisée uniquement pour
    signer localement avant publication. Seule `public_key_b64` est
    partagée (dans le manifest de l'app).
    """
    private_key_b64: str
    public_key_b64:  str


def generate_developer_keypair() -> DeveloperKeypair:
    """
    Génère une nouvelle paire de clés Ed25519 pour un développeur.
    À exécuter UNE FOIS, hors ligne, sur la machine du développeur —
    jamais côté serveur AXIS-OS. Les clés sont encodées en base64
    pour être facilement stockées dans un fichier texte/variable
    d'environnement (jamais dans le code source versionné, voir
    axis_os/hal/core/structured_log.py pour le même principe appliqué
    aux logs, et la recherche sur la gestion des secrets faite cette
    session : un secret permanent ne doit jamais être codé en dur).
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
    Retourne la signature encodée en base64, à inclure dans le
    manifest de l'app (voir AppManifest.code_signature).

    `source_code` est typiquement le contenu du fichier .py principal
    de l'app (app.py) — voir verify_app_signature() pour la
    contrepartie exacte de ce qui doit être fourni à la vérification :
    EXACTEMENT le même contenu, un seul octet différent invalide la
    signature (comportement voulu, c'est la garantie d'intégrité).
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
    Vérifie qu'une signature correspond bien au code source fourni,
    pour la clé publique donnée. Retourne True/False — ne lève JAMAIS
    d'exception sur un échec de vérification (signature invalide,
    code modifié) : c'est le résultat attendu d'une falsification,
    pas une erreur de programmation. Ne lève SignatureError que si
    les données elles-mêmes sont structurellement invalides (clé ou
    signature malformée), jamais sur un simple mismatch.
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


def sign_app_file(file_path: Union[str, Path], private_key_b64: str) -> str:
    """Raccourci : lit un fichier et le signe. Voir sign_app_code()."""
    content = Path(file_path).read_text(encoding="utf-8")
    return sign_app_code(content, private_key_b64)


def verify_app_file(
    file_path: Union[str, Path], signature_b64: str, public_key_b64: str,
) -> bool:
    """Raccourci : lit un fichier et vérifie sa signature. Voir
    verify_app_signature()."""
    content = Path(file_path).read_text(encoding="utf-8")
    return verify_app_signature(content, signature_b64, public_key_b64)
