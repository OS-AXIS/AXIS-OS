"""
AXIS-OS — Chargement d'une app depuis un fichier .axisapp chiffré
========================================================================
Complète encryption.py (chiffrement/déchiffrement bruts, en octets)
avec le VRAI chemin d'installation d'une app livrée sous forme
chiffrée : lire le fichier .axisapp, déchiffrer en mémoire avec la
clé privée plateforme, exécuter le code déchiffré dans un espace de
noms isolé pour en extraire la classe RobotApp, l'instancier, puis
l'installer normalement via PlayStoreRuntime.install().

Le code déchiffré N'EST JAMAIS réécrit en clair sur disque à aucune
étape de ce module — il transite uniquement en mémoire (une chaîne
Python), du déchiffrement jusqu'à l'exécution via exec().

Ce module ne remplace ni n'affaiblit aucune des vérifications déjà en
place dans PlayStoreRuntime.install() (signature, permissions,
capacités, ProtectionLevel.SIGNATURE) — il livre simplement une
instance de RobotApp à cette même méthode install() existante, qui
applique ensuite exactement les mêmes vérifications qu'une app
installée depuis une instance déjà en mémoire (voir
_load_app_instance_from_source() ci-dessous : exec() extrait la
classe, mais AUCUNE vérification de sécurité n'est court-circuitée —
elles ont toutes lieu APRÈS, dans install(), sur l'instance obtenue).

Licence : Apache 2.0
"""

from __future__ import annotations
from pathlib import Path
from typing import Optional, Union

from axis_os.store.sdk.app_base import RobotApp
from axis_os.store.sdk.encryption import (
    decrypt_app_file_to_bytes, decrypt_app_code, EncryptedApp,
    EncryptionError, DecryptionError,
)


class AppLoadError(Exception):
    """
    Levée quand le code déchiffré ne peut pas être chargé comme une
    app AXIS-OS valide — distincte de EncryptionError/DecryptionError
    (encryption.py), qui couvrent l'échec du DÉCHIFFREMENT lui-même.
    Cette exception couvre l'étape SUIVANTE : le déchiffrement a
    réussi (le contenu est un texte Python valide en soi), mais ce
    texte ne définit pas de classe RobotApp exploitable — code
    syntaxiquement invalide, aucune classe RobotApp trouvée, plusieurs
    classes ambiguës, ou une erreur levée par le code lui-même pendant
    son exécution (exec() peut exécuter du code arbitraire au niveau
    module, exactement comme un import Python normal — voir la
    docstring de _load_app_instance_from_source() pour la limite de
    sécurité assumée ici, cohérente avec le reste du projet).
    """
    pass


def _load_app_instance_from_source(source_code: str, *, origin_label: str) -> RobotApp:
    """
    Exécute `source_code` dans un module Python DÉDIÉ et unique
    (jamais le namespace global de ce module ni du process appelant),
    puis instancie la SEULE classe RobotApp qu'il définit.

    Le module dynamique est ENREGISTRÉ dans sys.modules — trouvé
    nécessaire par un test de bout en bout (installer une app chargée
    ainsi via un vrai PlayStoreRuntime) : sans cet enregistrement, la
    classe créée par exec() vit dans un namespace transitoire que
    `pickle` ne peut PAS localiser (`PicklingError: import of module
    ... failed`) — donc une instance de cette classe échoue à
    traverser la frontière de process de SandboxRunner (qui pickle
    l'app et ses arguments pour les transmettre au sous-process
    sandboxé, voir sandbox.py). Un nom de module UNIQUE par appel
    (préfixé de l'id de l'objet code compilé, jamais réutilisé) évite
    toute collision entre deux apps chargées dans le même process
    AXIS-OS — chaque app déchiffrée obtient son propre "faux module"
    isolé, jamais partagé.

    ⚠️ Limite de sécurité assumée, honnête, cohérente avec le reste du
    projet (voir la même discussion dans signing.py sur ce qu'une
    signature valide garantit ou non) : exec() exécute le code
    déchiffré avec les MÊMES privilèges que le process Python
    appelant — exactement le même niveau de confiance qu'un `import`
    Python normal sur un fichier .py, PAS un sandbox au sens où
    SandboxRunner (axis_os/store/runtime/sandbox.py) isole
    l'EXÉCUTION D'UNE MISSION (process séparé, quotas CPU/RAM/temps).
    Le déchiffrement + cette exécution transitoire au chargement,
    utilisée uniquement pour EXTRAIRE la classe et sa méthode
    manifest() (jamais pour exécuter run_mission() ici), précèdent
    TOUJOURS PlayStoreRuntime.install() — qui, lui, sandboxe
    réellement on_start()/on_stop()/run_mission() via SandboxRunner
    (voir le correctif C4, CHANGELOG.md). Déchiffrer et signer
    garantissent l'ORIGINE et la CONFIDENTIALITÉ du fichier, jamais
    la bienveillance du code lui-même — exactement la même limite
    documentée pour verify_app_signature() dans signing.py.
    """
    import types
    import sys as _sys

    try:
        code_obj = compile(source_code, filename=f"<{origin_label}>", mode="exec")
    except SyntaxError as e:
        raise AppLoadError(
            f"Code déchiffré syntaxiquement invalide ({origin_label}) : {e}"
        ) from e

    module_name = f"axis_os_encrypted_app_{id(code_obj)}_{origin_label}"
    module_name = module_name.replace(".", "_").replace("/", "_")
    module = types.ModuleType(module_name)
    module.__dict__["__name__"] = module_name
    _sys.modules[module_name] = module

    try:
        exec(code_obj, module.__dict__)
    except Exception as e:
        del _sys.modules[module_name]
        raise AppLoadError(
            f"Erreur lors de l'exécution du code déchiffré au chargement "
            f"({origin_label}) : {type(e).__name__}: {e}"
        ) from e

    candidates = [
        obj for obj in module.__dict__.values()
        if isinstance(obj, type)
        and issubclass(obj, RobotApp)
        and obj is not RobotApp
    ]
    if not candidates:
        del _sys.modules[module_name]
        raise AppLoadError(
            f"Aucune classe héritant de RobotApp trouvée dans le code "
            f"déchiffré ({origin_label})."
        )
    if len(candidates) > 1:
        names = ", ".join(c.__name__ for c in candidates)
        del _sys.modules[module_name]
        raise AppLoadError(
            f"Plusieurs classes RobotApp trouvées dans le code déchiffré "
            f"({origin_label}) — ambigu, une seule attendue par fichier : "
            f"{names}."
        )

    app_class = candidates[0]
    try:
        return app_class()
    except Exception as e:
        del _sys.modules[module_name]
        raise AppLoadError(
            f"Échec de l'instanciation de {app_class.__name__} "
            f"({origin_label}) : {type(e).__name__}: {e}"
        ) from e


def load_app_from_encrypted_bytes(
    encrypted_data: bytes, platform_private_key_b64: str, *, origin_label: str = "app",
) -> RobotApp:
    """
    Déchiffre `encrypted_data` (le contenu brut d'un fichier .axisapp,
    déjà lu en mémoire par l'appelant — jamais lu depuis un chemin ici,
    pour que cette fonction reste utilisable aussi bien avec un
    fichier disque qu'avec un contenu reçu par un autre canal, ex: un
    futur téléchargement réseau) et retourne une instance RobotApp
    prête à être passée à PlayStoreRuntime.install().

    Lève EncryptionError/DecryptionError (encryption.py) si le
    déchiffrement échoue, AppLoadError si le déchiffrement réussit
    mais que le contenu n'est pas une app valide.
    """
    encrypted_app = EncryptedApp.from_bytes(encrypted_data)
    plaintext_bytes = decrypt_app_code(encrypted_app, platform_private_key_b64)
    try:
        source_code = plaintext_bytes.decode("utf-8")
    except UnicodeDecodeError as e:
        raise AppLoadError(
            f"Code déchiffré n'est pas de l'UTF-8 valide ({origin_label}) : {e}"
        ) from e
    return _load_app_instance_from_source(source_code, origin_label=origin_label)


def load_app_from_encrypted_file(
    encrypted_file_path: Union[str, Path], platform_private_key_b64: str,
) -> RobotApp:
    """
    Raccourci : lit un fichier .axisapp depuis le disque et le
    déchiffre. Voir load_app_from_encrypted_bytes() pour le contexte
    complet — utilise `origin_label` = nom du fichier, pour que les
    messages d'erreur d'AppLoadError identifient précisément quel
    fichier a échoué à charger (utile dès qu'on installe plusieurs
    apps chiffrées à la suite).
    """
    path = Path(encrypted_file_path)
    data = path.read_bytes()
    return load_app_from_encrypted_bytes(
        data, platform_private_key_b64, origin_label=path.name,
    )
