"""
AXIS-OS CLI
=============
Outil en ligne de commande (CLI, Command-Line Interface) destiné aux
auteurs d'apps pour le Play Store d'AXIS-OS, la boutique d'apps du robot.
Point d'entrée : python3 -m axis_os.cli.main <commande> (voir
docs/CREER_UNE_APP.md), ou axis-os <commande> une fois la commande
installée (voir « Installation » plus bas).

Trois commandes couvrent la vie d'une app :
  - init : crée le squelette d'une nouvelle app (code, manifeste, test,
    README) à partir des modèles de templates.py. Le manifeste est la
    fiche qui décrit l'app : identifiant, version, auteur, permissions ;
  - simulate : installe l'app sur un robot simulé et lui fait exécuter
    une mission, sans aucun robot réel ;
  - publish : vérifie le manifeste avant publication. L'envoi vers le
    registre en ligne n'est pas encore branché (voir cmd_publish).

Usage, une fois la commande axis-os installée (sinon, remplacer
axis-os par python3 -m axis_os.cli.main) :
    axis-os init mon-app
    axis-os simulate
    axis-os publish

Installation : le dépôt ne fournit pas encore de pyproject.toml ni de
setup.py, donc rien n'installe aujourd'hui la commande axis-os. Pour
l'obtenir, il faudra déclarer dans un pyproject.toml :
    [project.scripts]
    axis-os = "axis_os.cli.main:main"

Licence : Apache 2.0
"""

from __future__ import annotations
import sys
import os
import re
import json
import argparse
import importlib.util
from pathlib import Path

from .templates import (
    APP_PY_TEMPLATE, MANIFEST_JSON_TEMPLATE,
    TEST_APP_TEMPLATE, README_APP_TEMPLATE
)


def _slugify(name: str) -> str:
    """
    Transforme un nom libre en identifiant utilisable comme nom de dossier
    et dans un app_id : tout en minuscules, chaque suite de caractères
    autres que a-z et 0-9 (lettres accentuées comprises) remplacée par
    « _ », puis les « _ » de début et de fin retirés.
    Exemple : 'Mon App Cool' → 'mon_app_cool'.
    """
    s = re.sub(r"[^a-zA-Z0-9]+", "_", name.strip().lower())
    return s.strip("_")


def _class_name(name: str) -> str:
    """
    Construit le nom de la classe Python de l'app : le nom est coupé en
    mots à chaque caractère autre que a-z, A-Z et 0-9, chaque mot prend
    une majuscule initiale (le reste passe en minuscules), puis on ajoute
    le suffixe « App ».
    Exemple : 'mon-app-cool' → 'MonAppCoolApp'.
    """
    parts = re.split(r"[^a-zA-Z0-9]+", name)
    return "".join(p.capitalize() for p in parts if p) + "App"


# ─────────────────────────────────────────────────────────────────
#  axis-os init
# ─────────────────────────────────────────────────────────────────

def cmd_init(args: argparse.Namespace) -> int:
    """
    axis-os init <nom> : crée le dossier ./<slug>/ (le nom passé par
    _slugify) avec app.py, manifest.json, test_app.py et README.md,
    remplis à partir des modèles de templates.py. L'app_id suit la
    notation de domaine inversé com.<auteur>.<app>, convention courante
    qui évite que deux auteurs prennent le même identifiant. Refuse
    d'écraser un dossier existant. Renvoie le code de sortie du
    processus : 0 en cas de succès, 1 sinon.
    """
    app_name  = args.name
    app_id    = f"com.{_slugify(args.author)}.{_slugify(app_name)}"
    class_nm  = _class_name(app_name)
    slug      = _slugify(app_name)

    target = Path(slug)
    if target.exists():
        print(f"[ERREUR] Le dossier '{slug}' existe déjà.")
        return 1

    target.mkdir(parents=True)
    ctx = {
        "app_name": app_name, "app_id": app_id,
        "class_name": class_nm, "author": args.author,
        "author_email": args.email, "app_id_short": slug,
    }

    (target / "app.py").write_text(APP_PY_TEMPLATE.format(**ctx))
    (target / "manifest.json").write_text(MANIFEST_JSON_TEMPLATE.format(**ctx))
    (target / "test_app.py").write_text(TEST_APP_TEMPLATE.format(**ctx))
    (target / "README.md").write_text(README_APP_TEMPLATE.format(**ctx))

    print(f"\n✓ App '{app_name}' créée dans ./{slug}/")
    print(f"  app_id : {app_id}")
    print(f"\n  Prochaines étapes :")
    print(f"    cd {slug}")
    print(f"    # Éditez app.py")
    print(f"    axis-os simulate")
    return 0


# ─────────────────────────────────────────────────────────────────
#  axis-os simulate
# ─────────────────────────────────────────────────────────────────

def cmd_simulate(args: argparse.Namespace) -> int:
    """
    axis-os simulate : à lancer depuis le dossier de l'app. Charge app.py,
    y cherche la classe de l'app (une sous-classe de RobotApp, la classe
    de base de toute app AXIS-OS), puis l'installe, la démarre et lui fait
    exécuter une mission sur un humanoïde Unitree G1 simulé. Avec les deux
    choix de --backend (isaac ou webots), c'est la physique simplifiée
    intégrée (« stub ») qui sert : aucun simulateur 3D n'est lancé et
    aucun moteur réel n'est commandé. Renvoie 0 si la mission réussit,
    1 sinon.
    """
    app_file = Path("app.py")
    if not app_file.exists():
        print("[ERREUR] Aucun app.py trouvé. Lancez depuis le dossier de l'app.")
        return 1

    # Lecture facultative de manifest.json : un fichier absent ou illisible
    # n'arrête pas la simulation. La suite utilise l'app_id déclaré par la
    # classe de app.py (real_app_id plus bas), pas celui-ci.
    manifest_file = Path("manifest.json")
    app_id = "unknown"
    if manifest_file.exists():
        try:
            app_id = json.loads(manifest_file.read_text()).get("app_id", "unknown")
        except Exception:
            pass

    print(f"[axis-os] Simulation locale — backend={args.backend}")

    # Import dynamique de l'app locale : le fichier app.py du dossier
    # courant, chargé sous le nom de module « local_app ».
    spec = importlib.util.spec_from_file_location("local_app", app_file)
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(Path.cwd()))
    # Le module est enregistré dans sys.modules avant d'être exécuté :
    # c'est par ce nom et son fichier que le processus neuf de l'app (le
    # sandbox, voir store/runtime/hote_app.py) la recharge. Historique :
    # quand l'app passait par pickle, son absence de sys.modules faisait
    # planter `axis-os simulate` dès on_start (constaté en rédigeant
    # docs/CREER_UNE_APP.md).
    sys.modules["local_app"] = module
    spec.loader.exec_module(module)

    from axis_os.store.sdk.app_base import RobotApp
    import inspect

    # On ne retient que les sous-classes de RobotApp définies dans app.py
    # lui-même (__module__ == "local_app"), pas celles qu'il importe. S'il
    # y en a plusieurs, la première par ordre alphabétique de nom est
    # prise (inspect.getmembers trie par nom).
    app_classes = [
        obj for name, obj in inspect.getmembers(module, inspect.isclass)
        if issubclass(obj, RobotApp)
        and obj is not RobotApp
        and obj.__module__ == "local_app"
    ]
    if not app_classes:
        print("[ERREUR] Aucune classe RobotApp concrète trouvée dans app.py")
        return 1

    AppClass = app_classes[0]

    from axis_os.hal.drivers.isaac_sim.hal import IsaacSimHAL
    from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
    from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
    from axis_os.hal.watchdog.system import WatchdogSystem
    from axis_os.hal.watchdog.config import WatchdogConfig
    from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
    from axis_os.store.runtime.profil import Profil

    # Choix du simulateur. Sans backend fourni (backend=None), IsaacSimHAL
    # se branche sur le pont vers Isaac Sim (le simulateur de NVIDIA) forcé
    # en mode stub, une physique simplifiée intégrée. Webots, un autre
    # simulateur robotique, est lui aussi forcé en mode stub
    # (force_stub=True), même s'il est installé.
    backend = None
    if args.backend == "webots":
        from axis_os.hal.drivers.webots.backend import WebotsBackend
        backend = WebotsBackend(force_stub=True)

    # Assemblage du robot simulé :
    #  - IsaacSimHAL : le HAL (Hardware Abstraction Layer, couche
    #    d'abstraction matérielle, seul chemin vers les moteurs), branché
    #    ici sur le simulateur choisi ;
    #  - NormalizedRobotHAL : traduit les angles propres au G1
    #    (build_g1_normalizer) dans la convention commune, la seule que
    #    voient les apps, pour qu'elles n'aient pas à connaître le robot ;
    #  - WatchdogSystem : le chien de garde qui surveille en continu les
    #    articulations. Son profil simulation() émet les alertes mais ne
    #    déclenche jamais de lui-même l'e-stop (arrêt d'urgence).
    hal = IsaacSimHAL(robot_id="axis_sim", backend=backend)
    hal.initialize_all()
    norm = NormalizedRobotHAL(hal, build_g1_normalizer())
    wd   = WatchdogSystem(hal, WatchdogConfig.simulation())
    wd.start()

    # « axis-os simulate » ne pilote que le simulateur : les permissions
    # DANGEROUS (celles qui exigent l'accord explicite d'un humain, comme
    # commander les moteurs) y sont accordées explicitement, sans
    # confirmation. Sur un vrai robot, le runtime (PlayStoreRuntime, qui
    # installe et exécute les apps) exige une confirmation humaine et
    # refuse accepter_tout_SIMULATION quand il reconnaît un robot réel
    # (voir axis_os/store/runtime/play_store.py).
    runtime = PlayStoreRuntime(norm, wd, confirmer_permissions=accepter_tout_SIMULATION,
                               profil=Profil.SIMULATION)
    app_instance = AppClass()

    if not runtime.install(app_instance):
        print("[ERREUR] Installation échouée — vérifiez le manifest.")
        wd.stop(); hal.shutdown_all()
        return 1

    real_app_id = app_instance.manifest().app_id
    if not runtime.start_app(real_app_id):
        print("[ERREUR] on_start() a échoué.")
        wd.stop(); hal.shutdown_all()
        return 1

    params = json.loads(args.params) if args.params else {}
    result = runtime.run_mission(real_app_id, params=params)

    runtime.stop_app(real_app_id)
    wd.stop()
    hal.shutdown_all()

    if result and result.success:
        print(f"\n✓ Simulation réussie : {result}")
        return 0
    else:
        print(f"\n✗ Simulation échouée : {result}")
        return 1


# ─────────────────────────────────────────────────────────────────
#  axis-os publish
# ─────────────────────────────────────────────────────────────────

def cmd_publish(args: argparse.Namespace) -> int:
    """
    axis-os publish : vérifie, depuis le dossier de l'app, que
    manifest.json et app.py existent et que le manifeste remplit les
    champs obligatoires (app_id, name, version, author, author_email).
    Limite connue : l'envoi vers le registre en ligne du Play Store n'est
    pas encore branché. La commande s'arrête donc après cette validation
    locale, avec ou sans --dry-run (essai à blanc, sans aucun envoi).
    Renvoie 0 si le manifeste est valide, 1 sinon.
    """
    manifest_file = Path("manifest.json")
    app_file      = Path("app.py")

    if not manifest_file.exists() or not app_file.exists():
        print("[ERREUR] manifest.json ou app.py manquant.")
        return 1

    manifest = json.loads(manifest_file.read_text())
    errors = []
    for field in ["app_id", "name", "version", "author", "author_email"]:
        if not manifest.get(field):
            errors.append(f"Champ manquant : {field}")

    if errors:
        print("[ERREUR] Manifeste invalide :")
        for e in errors: print(f"  • {e}")
        return 1

    print(f"[axis-os] Empaquetage '{manifest['name']}' v{manifest['version']}")
    print(f"  app_id : {manifest['app_id']}")
    print(f"  auteur : {manifest['author']}")

    if args.dry_run:
        print("\n  [DRY-RUN] Aucun push effectué.")
        print("  ✓ Package prêt (validation locale uniquement)")
        return 0

    print("\n  ⚠ Registre cloud non encore connecté (Sprint 3 en cours).")
    print("  Le package est validé localement et prêt pour publication.")
    return 0


# ─────────────────────────────────────────────────────────────────
#  Entrée principale
# ─────────────────────────────────────────────────────────────────

def main() -> int:
    """
    Lit la ligne de commande et lance la sous-commande demandée. Sans
    sous-commande, affiche l'aide et renvoie 1. Le nombre renvoyé devient
    le code de sortie du processus (sys.exit en bas du fichier).
    """
    parser = argparse.ArgumentParser(prog="axis-os",
        description="CLI AXIS-OS — développement d'apps Play Store")
    sub = parser.add_subparsers(dest="command")

    p_init = sub.add_parser("init", help="Crée le squelette d'une nouvelle app")
    p_init.add_argument("name", help="Nom de l'app (ex: 'Mon App')")
    p_init.add_argument("--author", default="Anonymous")
    p_init.add_argument("--email", default="dev@example.com")
    p_init.set_defaults(func=cmd_init, author="Anonymous", email="dev@example.com")

    p_sim = sub.add_parser("simulate", help="Teste l'app dans le simulateur")
    p_sim.add_argument("--backend", choices=["isaac", "webots"], default="isaac")
    p_sim.add_argument("--params", default=None, help="JSON des params de mission")
    p_sim.set_defaults(func=cmd_simulate)

    p_pub = sub.add_parser("publish", help="Publie l'app sur le Play Store")
    p_pub.add_argument("--dry-run", action="store_true")
    p_pub.set_defaults(func=cmd_publish)

    args = parser.parse_args()
    if not hasattr(args, "func"):
        parser.print_help()
        return 1
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
