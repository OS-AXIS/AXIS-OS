"""
SDK, partie 7 : kit de développement.

Vérifié ici :
  1. pyproject.toml : paquet « axis-os-sdk », version SemVer égale à
     l'API du SDK, dépendances minimales, commande « axis », données
     (animations, URDF) incluses ;
  2. ctx.api_version, en direct et dans le processus de l'app ;
  3. min_sdk_version du manifeste : format vérifié, installation refusée
     si l'API de l'OS est trop ancienne ou d'une autre version majeure ;
  4. `axis validate` : une app à problèmes est refusée, chaque problème
     dit (les exemples, eux, sont vérifiés par test/test_exemples_sdk.py) ;
  5. `axis init` : le gabarit montre has_capability, should_stop et
     report_progress ; l'app générée passe `axis validate` et son propre
     test, sur le robot simulé léger.
"""
import contextlib
import io
import json
import subprocess
import sys
import tempfile
import tomllib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os import SemVer  # noqa: E402
from axis_os.store.runtime.hal_bridge import HALBridgeClient  # noqa: E402
from axis_os.store.runtime.manifeste_json import (  # noqa: E402
    manifeste_depuis_json, manifeste_texte, manifeste_vers_json,
)
from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext  # noqa: E402
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402
from axis_os.store.sdk.version import API_VERSION  # noqa: E402
from axis_os.testing import AppTestHarness, SimulatedRobot  # noqa: E402

FAIL = 0
RACINE = Path(__file__).resolve().parents[1]


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"\n{detail}" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def axis(*args, cwd=None):
    """La commande « axis » (python -m axis_os.cli.main). cwd=None : la
    racine du dépôt (pas de valeur calculée par défaut : le processus de
    l'app recharge ce module sans ses affectations calculées)."""
    cwd = cwd or RACINE
    env_path = str(RACINE)
    import os
    env = dict(os.environ, PYTHONPATH=env_path + os.pathsep + os.environ.get("PYTHONPATH", ""))
    return subprocess.run([sys.executable, "-m", "axis_os.cli.main", *args], cwd=cwd,
                          capture_output=True, text=True, timeout=300, env=env)


def manifeste(**kw):
    return AppManifest(app_id="com.test.devkit", name="Devkit", version="1.0.0", author="T",
                       author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
                       **kw)


section("1. pyproject.toml")
projet = tomllib.loads((RACINE / "pyproject.toml").read_text())["project"]
check("Paquet « axis-os-sdk »", projet["name"] == "axis-os-sdk")
check(f"Version SemVer, égale à l'API du SDK ({API_VERSION})",
      projet["version"] == API_VERSION and SemVer.parse(projet["version"]) is not None)
check("Dépendances minimales : numpy seul", projet["dependencies"] == ["numpy>=1.24"],
      str(projet["dependencies"]))
check("Commande « axis » (et l'ancien nom « axis-os »)",
      projet["scripts"].get("axis") == "axis_os.cli.main:main"
      and projet["scripts"].get("axis-os") == "axis_os.cli.main:main")
outils = tomllib.loads((RACINE / "pyproject.toml").read_text())["tool"]["setuptools"]
donnees = outils["package-data"]["axis_os.hal.drivers.unitree_g1"]
check("Les animations du G1 font partie du paquet",
      "animations/*.json" in donnees and list((RACINE / "axis_os/hal/drivers/unitree_g1/animations")
                                                .glob("*.json")))
check("Le SDK, le robot simulé et la CLI font partie du paquet (axis_os*, avec __init__.py)",
      outils["packages"]["find"]["include"] == ["axis_os*"]
      and all((RACINE / p / "__init__.py").exists()
              for p in ("axis_os/store/sdk", "axis_os/testing", "axis_os/cli")))

section("2. ctx.api_version")
check("RobotContext.api_version et HALBridgeClient.api_version valent API_VERSION",
      RobotContext.api_version == API_VERSION == HALBridgeClient.api_version)


class AppVersion(RobotApp):
    def manifest(self):
        return AppManifest(app_id="com.test.version", name="Version", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES, permissions=[],
                           min_sdk_version="1.0.0")

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             metrics={"api": ctx.api_version})


with AppTestHarness(AppVersion()) as h:
    res = h.run()
    check("Dans le processus de l'app, ctx.api_version vaut celle de l'OS",
          res is not None and res.metrics.get("api") == API_VERSION, str(getattr(res, "metrics", None)))

section("3. min_sdk_version")
check("Format vérifié : « 1.0 » refusé", manifeste(min_sdk_version="1.0").validate() != [])
check("Format vérifié : « 1.0.0 » accepté", manifeste(min_sdk_version="1.0.0").validate() == [])
m = manifeste(min_sdk_version="1.0.0")
check("Aller-retour JSON du manifeste", manifeste_depuis_json(manifeste_vers_json(m)).min_sdk_version
      == "1.0.0")
check("Absent : pas de clé dans le texte du manifeste (signatures existantes intactes)",
      "min_sdk_version" not in manifeste_texte(manifeste()))
api = SemVer.parse(API_VERSION)
for version, attendu in [(API_VERSION, True), (f"{api.major}.{api.minor + 1}.0", False),
                         (f"{api.major + 1}.0.0", False), (None, True)]:
    class AppExigeante(AppVersion):
        def manifest(self, _v=version):
            m = super().manifest()
            m.app_id = "com.test.exigeante"
            m.min_sdk_version = _v
            return m
    with SimulatedRobot() as robot:
        runtime = robot.make_runtime()
        with contextlib.redirect_stdout(io.StringIO()):
            ok = runtime.install(AppExigeante())
    check(f"min_sdk_version {version} face à l'API {API_VERSION} : "
          f"{'installée' if attendu else 'installation refusée'}", ok is attendu)

section("4. axis validate")
with tempfile.TemporaryDirectory() as dossier:
    d = Path(dossier)
    (d / "app.py").write_text('''
from axis_os.store.sdk.app_base import MissionResult, RobotApp
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission, ResourceQuota


class AppBancale(RobotApp):
    def manifest(self):
        return AppManifest(app_id="com.test.bancale", name="Bancale", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.ANIMATION_PLAY, Permission.TELEOP_STREAM],
                           quota=ResourceQuota(max_runtime_s=100000.0),
                           min_sdk_version="9.0.0")

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)
''')
    (d / "manifest.json").write_text(json.dumps({"app_id": "com.test.autre", "version": "1.0.0"}))
    (d / "animations").mkdir()
    (d / "animations" / "trop_vite.json").write_text(json.dumps({
        "format": "axis-animation", "format_version": 1, "name": "trop_vite", "version": "1.0.0",
        "robots": ["G1"], "keyframes": [
            {"t": 0.0, "positions": {"right_shoulder_pitch_joint": 0.0}},
            {"t": 0.2, "positions": {"right_shoulder_pitch_joint": -2.0}}]}))
    r = axis("validate", str(d))
    sortie = r.stdout
    check("App à problèmes : refusée (code 1)", r.returncode == 1, sortie + r.stderr)
    for attendu in ["ANIMATION_PLAY exige aussi", "TELEOP_STREAM exige aussi", "max_runtime_s",
                    "min_sdk_version 9.0.0", "manifest.json : app_id",
                    "animation trop_vite.json"]:
        check(f"… et le dit : « {attendu} »", attendu in sortie, sortie)
    r = axis("validate", str(d / "absent"))
    check("Dossier sans app.py : code 1", r.returncode == 1)

section("5. axis init")
with tempfile.TemporaryDirectory() as dossier:
    r = axis("init", "Mon App", "--author", "Ada", cwd=dossier)
    app = Path(dossier) / "mon_app" / "app.py"
    check("axis init : l'app est créée", r.returncode == 0 and app.exists(), r.stdout + r.stderr)
    code = app.read_text() if app.exists() else ""
    check("Le gabarit montre has_capability, should_stop et report_progress",
          all(f"ctx.{api}(" in code for api in ("has_capability", "should_stop", "report_progress")))
    check("… et déclare min_sdk_version", 'min_sdk_version="1.0.0"' in code)
    r = axis("validate", str(Path(dossier) / "mon_app"))
    check("L'app générée passe axis validate", r.returncode == 0, r.stdout + r.stderr)
    import os
    r = subprocess.run([sys.executable, "test_app.py"], cwd=Path(dossier) / "mon_app",
                       capture_output=True, text=True, timeout=300,
                       env=dict(os.environ, PYTHONPATH=str(RACINE)))
    check("… et son propre test, sur le robot simulé léger", r.returncode == 0
          and "Test passé" in r.stdout, r.stdout[-2000:] + r.stderr[-2000:])

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
