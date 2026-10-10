"""
AXIS-OS CLI
=============
Point d'entrée : axis-os <commande>

Usage :
    axis-os init mon-app
    axis-os simulate
    axis-os publish

Installation (à ajouter dans setup.py / pyproject.toml) :
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
    """'Mon App Cool' → 'mon_app_cool'"""
    s = re.sub(r"[^a-zA-Z0-9]+", "_", name.strip().lower())
    return s.strip("_")


def _class_name(name: str) -> str:
    """'mon-app-cool' → 'MonAppCoolApp'"""
    parts = re.split(r"[^a-zA-Z0-9]+", name)
    return "".join(p.capitalize() for p in parts if p) + "App"


# ─────────────────────────────────────────────────────────────────
#  axis-os init
# ─────────────────────────────────────────────────────────────────

def cmd_init(args: argparse.Namespace) -> int:
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
    app_file = Path("app.py")
    if not app_file.exists():
        print("[ERREUR] Aucun app.py trouvé. Lancez depuis le dossier de l'app.")
        return 1

    manifest_file = Path("manifest.json")
    app_id = "unknown"
    if manifest_file.exists():
        try:
            app_id = json.loads(manifest_file.read_text()).get("app_id", "unknown")
        except Exception:
            pass

    print(f"[axis-os] Simulation locale — backend={args.backend}")

    # Import dynamique de l'app locale
    spec = importlib.util.spec_from_file_location("local_app", app_file)
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(Path.cwd()))
    # ENREGISTRER le module avant de l'exécuter : le sandbox transmet
    # l'app au processus isolé par sérialisation (pickle), qui retrouve
    # la classe par le nom de son module dans sys.modules. Sans cette
    # ligne, TOUTE app générée par `axis-os init` plantait dès
    # `axis-os simulate` (on_start : CRASHED / SANDBOX_ERROR, sans
    # autre explication). Constaté en rédigeant docs/CREER_UNE_APP.md.
    sys.modules["local_app"] = module
    spec.loader.exec_module(module)

    from axis_os.store.sdk.app_base import RobotApp
    import inspect

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

    backend = None
    if args.backend == "webots":
        from axis_os.hal.drivers.webots.backend import WebotsBackend
        backend = WebotsBackend(force_stub=True)

    hal = IsaacSimHAL(robot_id="axis_sim", backend=backend)
    hal.initialize_all()
    norm = NormalizedRobotHAL(hal, build_g1_normalizer())
    wd   = WatchdogSystem(hal, WatchdogConfig.simulation())
    wd.start()

    # « axis-os simulate » ne pilote que le SIMULATEUR : les permissions
    # DANGEROUS y sont accordées explicitement, sans confirmation. Sur un vrai
    # robot, le runtime exige une confirmation humaine (voir play_store.py).
    runtime = PlayStoreRuntime(norm, wd, confirmer_permissions=accepter_tout_SIMULATION)
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
