"""
AXIS-OS — Chargement d'une app depuis un fichier .axisapp chiffré
========================================================================
Une app peut être livrée au Play Store sous forme chiffrée, dans un
fichier .axisapp, pour que seul AXIS-OS puisse en lire le code.
encryption.py fournit le chiffrement et le déchiffrement bruts (en
octets) ; ce module y ajoute le vrai chemin d'installation d'une telle
app :
  1. lire le fichier .axisapp ;
  2. le déchiffrer en mémoire avec la clé privée de la plateforme
     (la clé détenue par AXIS-OS, seule capable de le déchiffrer) ;
  3. vérifier la signature du développeur sur le code déchiffré, avant
     toute exécution : sans signature valide, aucune ligne de l'app
     n'est exécutée ;
  4. exécuter le code dans un espace de noms isolé (un module Python
     dédié) pour en extraire la classe RobotApp, c'est-à-dire la classe
     qui définit l'app ;
  5. l'instancier ;
  6. confier l'instance à PlayStoreRuntime.install() (le moteur qui
     installe et exécute les apps), qui l'installe normalement ; cette
     dernière étape est faite par l'appelant.

Le code déchiffré n'est jamais réécrit en clair sur disque, à aucune
étape de ce module : il ne transite qu'en mémoire (une chaîne Python),
du déchiffrement jusqu'à son exécution par exec() (la fonction Python
qui exécute du code fourni sous forme de texte).

La signature est vérifiée ici, et non seulement dans install(), parce
qu'exec() exécute déjà tout le code placé au niveau du module : une
vérification faite après aurait été trop tardive (voir
load_app_from_encrypted_bytes()). Ce module ne remplace ni n'affaiblit
ensuite aucune des vérifications de PlayStoreRuntime.install() :
permissions, capacités (ce que l'app déclare avoir besoin de faire),
permissions de niveau ProtectionLevel.SIGNATURE (réservées aux apps
signées avec la clé de la plateforme). Il livre une instance de
RobotApp à cette même méthode install(), qui applique exactement les
mêmes vérifications qu'à une app installée depuis une instance déjà en
mémoire.

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
from axis_os.store.sdk.signing import verify_app_signature, SignatureError


class AppLoadError(Exception):
    """
    Levée quand le code déchiffré ne peut pas être chargé comme une app
    AXIS-OS valide.

    Distincte d'EncryptionError et de DecryptionError (encryption.py),
    qui couvrent l'échec du déchiffrement lui-même. AppLoadError couvre
    l'étape suivante : le déchiffrement a réussi, mais la signature du
    développeur est invalide ou malformée (aucun code n'est alors
    exécuté), ou le contenu obtenu ne définit pas de classe RobotApp
    exploitable. Autres cas couverts : contenu
    qui n'est pas du texte UTF-8 valide (l'encodage attendu pour le code
    source), code syntaxiquement invalide, aucune classe RobotApp
    trouvée, plusieurs classes (ambiguïté), erreur levée par le code
    lui-même pendant son exécution, ou échec de l'instanciation de la
    classe.

    Le cas « erreur pendant l'exécution » existe parce qu'exec() exécute
    tout le code placé au niveau du module, exactement comme un import
    Python normal. La limite de sécurité qui en découle, assumée et
    cohérente avec le reste du projet, est décrite dans la docstring de
    _load_app_instance_from_source().
    """
    pass


def _load_app_instance_from_source(source_code: str, *, origin_label: str) -> RobotApp:
    """
    Exécute `source_code` dans un module Python dédié et unique (jamais
    l'espace de noms global de ce module ni celui du processus appelant),
    puis instancie la seule classe RobotApp qu'il définit.

    Pourquoi enregistrer ce module dynamique dans sys.modules (la table
    des modules importés de Python) : un test de bout en bout l'a rendu
    nécessaire, en installant une app chargée ainsi dans un vrai
    PlayStoreRuntime. Sans cet enregistrement, la classe créée par exec()
    vit dans un espace de noms transitoire que pickle (le format de
    sérialisation d'objets de Python) ne sait pas retrouver
    (`PicklingError: import of module ... failed`). Une instance de cette
    classe ne peut alors pas franchir la frontière de processus de
    SandboxRunner, qui sérialise l'app et ses arguments avec pickle pour
    les transmettre au sous-processus isolé (la sandbox, ou « bac à
    sable » : voir sandbox.py). Conséquence constatée avant ce
    correctif : on_start() et run_mission() échouaient systématiquement
    pour toute app chargée par ce module (voir CHANGELOG.md).

    Un nom de module distinct à chaque appel évite les collisions entre
    deux apps chargées dans le même processus AXIS-OS : chaque app
    déchiffrée obtient son propre « faux module » isolé, jamais partagé.
    Ce nom est construit avec l'id de l'objet code compilé et l'étiquette
    d'origine (origin_label). Python ne garantit l'unicité d'un id que
    tant que l'objet existe : l'unicité du nom est donc très probable,
    mais pas absolue.
    En cas d'échec, ce module est retiré de sys.modules, pour ne laisser
    aucun module fantôme (vérifié par test/test_encrypted_app_packaging.py).

    Limite de sécurité assumée, cohérente avec le reste du projet (voir
    la même discussion dans signing.py sur ce qu'une signature valide
    garantit ou non) : exec() exécute le code déchiffré avec les mêmes
    privilèges que le processus Python appelant. C'est exactement le
    niveau de confiance d'un `import` Python normal sur un fichier .py,
    et non une sandbox au sens où SandboxRunner
    (axis_os/store/runtime/sandbox.py) isole l'exécution d'une mission
    (processus séparé, quotas de CPU, de mémoire et de temps).

    Cette exécution au chargement sert uniquement à extraire la classe
    et sa méthode manifest() (le manifeste : ce que l'app déclare de
    ses permissions et de ses besoins), jamais à lancer run_mission()
    ici. Elle n'a lieu qu'après la vérification de la signature du
    développeur (load_app_from_encrypted_bytes()), et elle précède
    toujours PlayStoreRuntime.install(), qui, lui, fait réellement
    exécuter on_start(), on_stop() et run_mission() dans SandboxRunner
    (depuis un correctif décrit dans CHANGELOG.md : auparavant,
    on_start() et on_stop() tournaient hors sandbox). Cette isolation a
    elle-même des faiblesses connues face à une app hostile : voir
    docs/isolation_apps.md.

    Le chiffrement et la signature garantissent l'origine et la
    confidentialité du fichier, jamais la bienveillance du code
    lui-même : c'est la même limite que celle documentée pour
    verify_app_signature() dans signing.py.
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
    encrypted_data: bytes, platform_private_key_b64: str, *,
    code_signature: str, developer_public_key: str, origin_label: str = "app",
) -> RobotApp:
    """
    Déchiffre `encrypted_data`, vérifie la signature du développeur, et
    seulement alors retourne une instance de RobotApp prête à être passée
    à PlayStoreRuntime.install().

    `encrypted_data` est le contenu brut d'un fichier .axisapp, déjà lu
    en mémoire par l'appelant. Cette fonction ne lit jamais elle-même un
    chemin, pour rester utilisable aussi bien avec un fichier sur disque
    qu'avec un contenu reçu par un autre canal (par exemple, un futur
    téléchargement par le réseau).

    `code_signature` est la signature Ed25519 du code source en clair
    (sign_app_code() de signing.py). `developer_public_key` est la clé
    publique ATTENDUE du développeur, obtenue par un canal de confiance
    (par exemple le registre des développeurs du Play Store), jamais lue
    dans le paquet lui-même : une signature vérifiée avec une clé fournie
    par le paquet ne prouverait rien.

    Pourquoi vérifier ici : la clé de chiffrement de la plateforme est
    publique, n'importe qui peut donc produire un .axisapp que ce module
    sait déchiffrer. La signature est vérifiée sur les octets déchiffrés
    AVANT toute compilation ou exécution : une app dont la signature est
    invalide n'exécute aucune ligne (test/test_encrypted_app_packaging.py,
    section 7).

    Lève EncryptionError ou DecryptionError (encryption.py) si le
    déchiffrement échoue, et AppLoadError si la signature est invalide ou
    malformée, ou si le contenu n'est pas une app valide.
    """
    encrypted_app = EncryptedApp.from_bytes(encrypted_data)
    plaintext_bytes = decrypt_app_code(encrypted_app, platform_private_key_b64)
    try:
        signature_valide = verify_app_signature(
            plaintext_bytes, code_signature, developer_public_key,
        )
    except SignatureError as e:
        raise AppLoadError(
            f"Signature ou clé malformée ({origin_label}) : {e}"
        ) from e
    if not signature_valide:
        raise AppLoadError(
            f"Signature invalide ({origin_label}) : le code ne correspond pas "
            f"à la signature du développeur attendu ; il n'est pas exécuté."
        )
    try:
        source_code = plaintext_bytes.decode("utf-8")
    except UnicodeDecodeError as e:
        raise AppLoadError(
            f"Code déchiffré n'est pas de l'UTF-8 valide ({origin_label}) : {e}"
        ) from e
    return _load_app_instance_from_source(source_code, origin_label=origin_label)


def load_app_from_encrypted_file(
    encrypted_file_path: Union[str, Path], platform_private_key_b64: str, *,
    code_signature: str, developer_public_key: str,
) -> RobotApp:
    """
    Raccourci : lit un fichier .axisapp sur le disque, puis le déchiffre,
    vérifie sa signature et le charge (voir load_app_from_encrypted_bytes()
    pour le contexte complet, signature comprise).

    `origin_label` prend le nom du fichier, pour que les messages
    d'AppLoadError désignent précisément le fichier qui n'a pas pu être
    chargé (utile dès qu'on installe plusieurs apps chiffrées à la
    suite).
    """
    path = Path(encrypted_file_path)
    data = path.read_bytes()
    return load_app_from_encrypted_bytes(
        data, platform_private_key_b64, code_signature=code_signature,
        developer_public_key=developer_public_key, origin_label=path.name,
    )
