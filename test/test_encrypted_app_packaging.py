"""
AXIS-OS — Tests du chargement d'apps chiffrées (packaging.py)
===================================================================
Valide load_app_from_encrypted_bytes()/load_app_from_encrypted_file()
— le VRAI chemin d'installation d'une app livrée sous forme chiffrée,
de bout en bout : déchiffrement en mémoire, chargement dynamique du
code, instanciation, et installation via un vrai PlayStoreRuntime,
y compris à travers le sandboxing complet (on_start/start_app/
run_mission, tous sandboxés depuis le correctif C4).

Point technique critique testé explicitement (section 2) : une classe
créée par exec() dans un namespace transitoire n'est PAS picklable
par défaut — trouvé en testant ce module contre un vrai
PlayStoreRuntime (SandboxRunner pickle l'app pour la transmettre au
sous-process sandboxé). Corrigé en enregistrant chaque app chargée
comme un vrai module dans sys.modules ; ce test vérifie explicitement
que ce mécanisme fonctionne ET qu'il ne laisse aucune fuite dans
sys.modules après un échec de chargement.
"""

import sys
sys.path.insert(0, ".")

from axis_os.store.sdk.encryption import generate_platform_keypair, encrypt_app_code
from axis_os.store.runtime.packaging import (
    load_app_from_encrypted_bytes, load_app_from_encrypted_file, AppLoadError,
)
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.store.runtime.play_store import PlayStoreRuntime
from axis_os.store.sdk.app_base import RobotApp

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


VALID_APP_SOURCE = """
from axis_os.store.sdk.app_base import RobotApp, MissionResult
from axis_os.store.sdk.manifest import AppManifest, AppCategory

class MySecretApp(RobotApp):
    def manifest(self):
        return AppManifest(
            app_id="{app_id}", name="Secret App", version="1.0.0",
            author="Dev", author_email="dev@dev.com", description="d",
            category=AppCategory.UTILITIES,
        )
    def on_start(self, ctx):
        return True
    def on_stop(self, ctx):
        pass
    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)
"""


def make_runtime():
    robot = NormalizedRobotHAL(
        UnitreeG1HAL(f"g1_pkg_{id(object())}"), build_g1_normalizer(),
    )
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    return PlayStoreRuntime(robot, wd)


# ══════════════════════════════════════════════════════════════════

section("1 — Chargement et instanciation depuis du code chiffré")

kp = generate_platform_keypair()
source = VALID_APP_SOURCE.format(app_id="com.developer.secretapp_w1")
encrypted = encrypt_app_code(source, kp.public_key_b64)

app_instance = load_app_from_encrypted_bytes(encrypted.to_bytes(), kp.private_key_b64)
check("Instance retournée hérite bien de RobotApp", isinstance(app_instance, RobotApp))
check("Le manifest de l'app chargée est correct",
      app_instance.manifest().app_id == "com.developer.secretapp_w1")

# ══════════════════════════════════════════════════════════════════

section("2 — CRITIQUE : l'app chargée survit au sandboxing complet "
        "(pickle via multiprocessing)")

runtime2 = make_runtime()
source2 = VALID_APP_SOURCE.format(app_id="com.developer.secretapp_w2")
encrypted2 = encrypt_app_code(source2, kp.public_key_b64)
app2 = load_app_from_encrypted_bytes(encrypted2.to_bytes(), kp.private_key_b64)

result_install = runtime2.install(app2)
check("install() réussit avec une app chargée depuis du code chiffré",
      result_install is True)

result_start = runtime2.start_app("com.developer.secretapp_w2")
check("CRITIQUE : start_app() réussit — on_start() est sandboxé "
      "(SandboxRunner pickle l'app pour un sous-process séparé), "
      "confirme que le mécanisme d'enregistrement de module fonctionne",
      result_start is True)

mission_result = runtime2.run_mission("com.developer.secretapp_w2", params={})
check("run_mission() (aussi sandboxé) réussit également",
      mission_result is not None and mission_result.success is True)

# ══════════════════════════════════════════════════════════════════

section("3 — Pas de fuite dans sys.modules après un chargement réussi ou échoué")

modules_before_ok = len(sys.modules)
source3 = VALID_APP_SOURCE.format(app_id="com.developer.secretapp_w3")
encrypted3 = encrypt_app_code(source3, kp.public_key_b64)
load_app_from_encrypted_bytes(encrypted3.to_bytes(), kp.private_key_b64)
modules_after_ok = len(sys.modules)
check("Un chargement RÉUSSI enregistre bien un nouveau module (attendu, "
      "nécessaire au pickling — voir section 2) mais un seul",
      modules_after_ok == modules_before_ok + 1)

modules_before_fail = len(sys.modules)
encrypted_bad = encrypt_app_code("x = 42  # pas de classe RobotApp", kp.public_key_b64)
try:
    load_app_from_encrypted_bytes(encrypted_bad.to_bytes(), kp.private_key_b64)
except AppLoadError:
    pass
modules_after_fail = len(sys.modules)
check("Un chargement ÉCHOUÉ ne laisse AUCUN module fantôme dans sys.modules",
      modules_after_fail == modules_before_fail)

# ══════════════════════════════════════════════════════════════════

section("4 — Cas d'erreur : contenu invalide après déchiffrement réussi")

try:
    enc_no_class = encrypt_app_code("x = 42", kp.public_key_b64)
    load_app_from_encrypted_bytes(enc_no_class.to_bytes(), kp.private_key_b64)
    check("Aucune classe RobotApp dans le code déchiffré -> AppLoadError", False)
except AppLoadError:
    check("Aucune classe RobotApp dans le code déchiffré -> AppLoadError", True)

try:
    enc_syntax = encrypt_app_code("def broken(:\n    pass", kp.public_key_b64)
    load_app_from_encrypted_bytes(enc_syntax.to_bytes(), kp.private_key_b64)
    check("Code syntaxiquement invalide -> AppLoadError", False)
except AppLoadError:
    check("Code syntaxiquement invalide -> AppLoadError", True)

AMBIGUOUS_SOURCE = """
from axis_os.store.sdk.app_base import RobotApp
from axis_os.store.sdk.manifest import AppManifest, AppCategory

class AppA(RobotApp):
    def manifest(self):
        return AppManifest(app_id="a", name="A", version="1.0.0", author="a",
                           author_email="a@a.com", description="d",
                           category=AppCategory.UTILITIES)
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass

class AppB(RobotApp):
    def manifest(self):
        return AppManifest(app_id="b", name="B", version="1.0.0", author="a",
                           author_email="a@a.com", description="d",
                           category=AppCategory.UTILITIES)
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass
"""
try:
    enc_ambig = encrypt_app_code(AMBIGUOUS_SOURCE, kp.public_key_b64)
    load_app_from_encrypted_bytes(enc_ambig.to_bytes(), kp.private_key_b64)
    check("Deux classes RobotApp (ambigu) -> AppLoadError", False)
except AppLoadError:
    check("Deux classes RobotApp (ambigu) -> AppLoadError", True)

# ══════════════════════════════════════════════════════════════════

section("5 — Isolation : deux apps chargées dans le même process "
        "n'interfèrent jamais entre elles")

source_a = VALID_APP_SOURCE.format(app_id="com.developer.appA_w5")
source_b = VALID_APP_SOURCE.format(app_id="com.developer.appB_w5")
enc_a = encrypt_app_code(source_a, kp.public_key_b64)
enc_b = encrypt_app_code(source_b, kp.public_key_b64)

app_a = load_app_from_encrypted_bytes(enc_a.to_bytes(), kp.private_key_b64)
app_b = load_app_from_encrypted_bytes(enc_b.to_bytes(), kp.private_key_b64)

check("Deux apps chargées séparément ont bien des app_id distincts",
      app_a.manifest().app_id != app_b.manifest().app_id)
check("Les deux classes sont bien des types Python DIFFÉRENTS "
      "(modules distincts, pas de collision de nom de classe)",
      type(app_a) is not type(app_b))

runtime5 = make_runtime()
check("Les deux s'installent ensemble sans conflit",
      runtime5.install(app_a) is True and runtime5.install(app_b) is True)

# ══════════════════════════════════════════════════════════════════

section("6 — Chargement depuis un vrai fichier disque (.axisapp)")

import tempfile
import shutil
from pathlib import Path

tmp_dir = tempfile.mkdtemp(prefix="axis_os_packaging_test_")
axisapp_path = Path(tmp_dir) / "my_app.axisapp"
source6 = VALID_APP_SOURCE.format(app_id="com.developer.fileapp_w6")
encrypted6 = encrypt_app_code(source6, kp.public_key_b64)
axisapp_path.write_bytes(encrypted6.to_bytes())

app6 = load_app_from_encrypted_file(axisapp_path, kp.private_key_b64)
check("Chargement depuis un vrai fichier .axisapp sur disque réussit",
      isinstance(app6, RobotApp)
      and app6.manifest().app_id == "com.developer.fileapp_w6")

shutil.rmtree(tmp_dir, ignore_errors=True)

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ Une app chargée depuis du code déchiffré survit au")
    print("    sandboxing complet (pickle via multiprocessing)")
    print("  ✓ Aucune fuite de module fantôme après un chargement échoué")
    print("  ✓ Deux apps chargées séparément n'interfèrent jamais entre")
    print("    elles, même en même temps dans le même process")
    print("  ✓ Confirmé de bout en bout via un vrai fichier .axisapp")

sys.exit(0 if FAIL == 0 else 1)
