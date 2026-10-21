"""
AXIS-OS — Tests du manifest étendu (Capability, ProtectionLevel)
=====================================================================
Valide capabilities_required et intents sur AppManifest, ainsi que
ProtectionLevel — ajoutés maintenant, même peu utilisés aujourd'hui,
pour ne jamais avoir à migrer le format du manifest une fois des
apps tierces en nombre (voir docs/architecture_services*.md).

Section 1 : une app exigeant CAMERA_DEPTH est refusée sur le G1
(qui ne déclare que CAMERA générique, jamais assez précis pour
affirmer la profondeur disponible).
Section 2 : une app tierce déclarant une permission SIGNATURE
(ACTUATORS_RAW) est refusée à l'installation — jamais accordée par
défaut, fail-safe tant qu'aucune clé plateforme n'est configurée.
Section 3 : une app RÉELLEMENT signée par la clé plateforme obtient
bien cette permission — confirme le mécanisme dans les deux sens.
"""

import sys
import os
import tempfile
sys.path.insert(0, ".")

from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
from axis_os.store.runtime.profil import Profil
from axis_os.store.sdk.manifest import (
    AppManifest, AppCategory, Permission, Capability, IntentDecl,
    ProtectionLevel, get_protection_level,
)
from axis_os.store.sdk.app_base import RobotApp
from axis_os.store.sdk.signing import generate_developer_keypair

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def make_runtime(**kwargs):
    robot = NormalizedRobotHAL(UnitreeG1HAL(f"g1_manifest_{id(kwargs)}"),
                               build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    return PlayStoreRuntime(robot, wd, **kwargs, confirmer_permissions=accepter_tout_SIMULATION,
                            profil=Profil.SIMULATION)


# ══════════════════════════════════════════════════════════════════

section("0 — Le manifest étendu reste rétrocompatible")

manifest_minimal = AppManifest(
    app_id="com.test.minimal", name="T", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
)
check("capabilities_required vide par défaut", manifest_minimal.capabilities_required == [])
check("intents vide par défaut", manifest_minimal.intents == [])

# ══════════════════════════════════════════════════════════════════

section("1 — Capability : refus d'installation si capacité fonctionnelle absente")


class _DepthApp(RobotApp):
    def manifest(self):
        return AppManifest(
            app_id="com.test.depth_w4", name="T", version="1.0.0", author="T",
            author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
            capabilities_required=[Capability.CAMERA_DEPTH],
        )
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass


runtime1 = make_runtime()
result1 = runtime1.install(_DepthApp())
check("App exigeant CAMERA_DEPTH refusée (G1 ne déclare que CAMERA générique, "
      "mapping conservateur — voir _available_capabilities())",
      result1 is False)


class _RgbApp(RobotApp):
    def manifest(self):
        return AppManifest(
            app_id="com.test.rgb_w4", name="T", version="1.0.0", author="T",
            author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
            capabilities_required=[Capability.CAMERA_RGB],
        )
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass


runtime1b = make_runtime()
result1b = runtime1b.install(_RgbApp())
check("Non-régression : app exigeant seulement CAMERA_RGB s'installe normalement",
      result1b is True)

# ══════════════════════════════════════════════════════════════════

section("2 — ProtectionLevel.SIGNATURE : refusée par défaut à toute app tierce")


class _RawApp(RobotApp):
    def manifest(self):
        return AppManifest(
            app_id="com.attacker.raw_w4", name="T", version="1.0.0", author="T",
            author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
            permissions=[Permission.ACTUATORS_RAW],
        )
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass


check("get_protection_level(ACTUATORS_RAW) == SIGNATURE",
      get_protection_level(Permission.ACTUATORS_RAW) == ProtectionLevel.SIGNATURE)

runtime2 = make_runtime()  # SANS platform_public_key
result2 = runtime2.install(_RawApp())
check("Sans clé plateforme configurée (fail-safe) : app tierce avec "
      "ACTUATORS_RAW refusée", result2 is False)

# ══════════════════════════════════════════════════════════════════

section("3 — ProtectionLevel.SIGNATURE : accordée à une vraie app système")

kp = generate_developer_keypair()
runtime3 = make_runtime(platform_public_key=kp.public_key_b64)

tmp_dir = tempfile.mkdtemp()
app_file = os.path.join(tmp_dir, "platform_app_w4.py")
with open(app_file, "w") as f:
    f.write(f'''
import sys
sys.path.insert(0, "{os.getcwd()}")
import inspect
from axis_os.store.sdk.signing import sign_app_code
from axis_os.store.sdk.manifest import AppManifest, AppCategory, Permission
from axis_os.store.sdk.app_base import RobotApp

_PRIVATE_KEY = "{kp.private_key_b64}"
_PUBLIC_KEY = "{kp.public_key_b64}"

class PlatformDiagnosticApp(RobotApp):
    def manifest(self):
        source_file = inspect.getsourcefile(type(self))
        with open(source_file, "r", encoding="utf-8") as f:
            source = f.read()
        sig = sign_app_code(source, _PRIVATE_KEY)
        return AppManifest(
            app_id="com.axisos.diagnostic_w4", name="Diagnostic", version="1.0.0",
            author="AXIS-OS", author_email="t@t.com", description="d",
            category=AppCategory.UTILITIES,
            permissions=[Permission.ACTUATORS_RAW],
            code_signature=sig, developer_public_key=_PUBLIC_KEY,
        )
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass
''')
sys.path.insert(0, tmp_dir)
from platform_app_w4 import PlatformDiagnosticApp

result3 = runtime3.install(PlatformDiagnosticApp())
check("App RÉELLEMENT signée par la clé plateforme obtient ACTUATORS_RAW",
      result3 is True)

# Non-régression : une app signée par un AUTRE développeur (pas la
# plateforme) reste refusée, même correctement signée.
kp_other = generate_developer_keypair()
runtime3b = make_runtime(platform_public_key=kp.public_key_b64)  # même clé plateforme que ci-dessus
app_file_other = os.path.join(tmp_dir, "other_dev_app_w4.py")
with open(app_file_other, "w") as f:
    f.write(f'''
import sys
sys.path.insert(0, "{os.getcwd()}")
import inspect
from axis_os.store.sdk.signing import sign_app_code
from axis_os.store.sdk.manifest import AppManifest, AppCategory, Permission
from axis_os.store.sdk.app_base import RobotApp

_PRIVATE_KEY = "{kp_other.private_key_b64}"
_PUBLIC_KEY = "{kp_other.public_key_b64}"

class OtherDevRawApp(RobotApp):
    def manifest(self):
        source_file = inspect.getsourcefile(type(self))
        with open(source_file, "r", encoding="utf-8") as f:
            source = f.read()
        sig = sign_app_code(source, _PRIVATE_KEY)
        return AppManifest(
            app_id="com.otherdev.raw_w4", name="T", version="1.0.0",
            author="AutreDev", author_email="t@t.com", description="d",
            category=AppCategory.UTILITIES,
            permissions=[Permission.ACTUATORS_RAW],
            code_signature=sig, developer_public_key=_PUBLIC_KEY,
        )
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass
''')
from other_dev_app_w4 import OtherDevRawApp

result3b = runtime3b.install(OtherDevRawApp())
check("Une app correctement signée par un AUTRE développeur (pas la "
      "plateforme) reste refusée pour ACTUATORS_RAW",
      result3b is False)

# ══════════════════════════════════════════════════════════════════

section("4 — IntentDecl : simple champ déclaratif, aucune logique de correspondance")

intent = IntentDecl(
    intent_id="logistics.sort_packages",
    description="trier des colis par destination",
    examples=["trie-moi ces colis", "range les paquets"],
)
manifest_intent = AppManifest(
    app_id="com.test.intent_w4", name="T", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
    intents=[intent],
)
check("IntentDecl correctement porté par le manifest",
      manifest_intent.intents[0].intent_id == "logistics.sort_packages")

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

sys.exit(0 if FAIL == 0 else 1)
