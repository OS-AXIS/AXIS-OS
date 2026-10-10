"""
AXIS-OS — Tests des capacités déclarées sur les 3 apps de référence
=========================================================================
Valide capabilities_required sur PackageSortingApp, SecurityPatrolApp,
WarehouseOpsApp (dernier point de l'ordre demandé pour l'architecture
services système) — et confirme que _available_capabilities()
(axis_os/store/runtime/play_store.py) dérive correctement WALK/
GRASP_SINGLE/CAMERA_RGB depuis les RobotCapability du G1.

Section 1 : les 3 apps de référence s'installent avec leurs vraies
capacités (pas une liste vide qui ne prouve rien).
Section 2 : une app exigeant CAMERA_DEPTH est refusée (déjà couvert
par test_extended_manifest.py avec une app dédiée — reconfirmé ici
dans le contexte des 3 vraies apps installées ensemble).
Section 3 : une app tierce déclarant ACTUATORS_RAW (SIGNATURE) est
refusée à l'installation, même aux côtés des 3 apps légitimes.
"""

import sys
sys.path.insert(0, ".")

from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
from axis_os.store.sdk.manifest import AppManifest, AppCategory, Permission, Capability
from axis_os.store.sdk.app_base import RobotApp
from axis_os.apps.package_sorting.app import PackageSortingApp
from axis_os.apps.security_patrol.app import SecurityPatrolApp
from axis_os.apps.warehouse_ops.app import WarehouseOpsApp

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def make_runtime():
    robot = NormalizedRobotHAL(UnitreeG1HAL(f"g1_refapps_{id(object())}"),
                               build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    return robot, PlayStoreRuntime(robot, wd, confirmer_permissions=accepter_tout_SIMULATION)


# ══════════════════════════════════════════════════════════════════

section("1 — Les 3 apps de référence déclarent de vraies Capability et s'installent")

check("PackageSortingApp déclare capabilities_required non vide",
      PackageSortingApp().manifest().capabilities_required != [])
check("PackageSortingApp exige GRASP_SINGLE (un seul bras à la fois — voir "
      "DEPOSIT_LEFT/DEPOSIT_RIGHT dans le code, jamais les deux ensemble)",
      Capability.GRASP_SINGLE in PackageSortingApp().manifest().capabilities_required)

check("SecurityPatrolApp déclare capabilities_required non vide",
      SecurityPatrolApp().manifest().capabilities_required != [])
check("SecurityPatrolApp exige WALK et CAMERA_RGB",
      {Capability.WALK, Capability.CAMERA_RGB}
      <= set(SecurityPatrolApp().manifest().capabilities_required))

check("WarehouseOpsApp déclare capabilities_required non vide",
      WarehouseOpsApp().manifest().capabilities_required != [])
check("WarehouseOpsApp exige WALK, GRASP_SINGLE et CAMERA_RGB",
      {Capability.WALK, Capability.GRASP_SINGLE, Capability.CAMERA_RGB}
      <= set(WarehouseOpsApp().manifest().capabilities_required))

robot1, runtime1 = make_runtime()
r_pkg = runtime1.install(PackageSortingApp())
r_sec = runtime1.install(SecurityPatrolApp())
r_wh  = runtime1.install(WarehouseOpsApp())
check("Les 3 apps de référence s'installent ENSEMBLE avec leurs vraies "
      "capacités déclarées sur le G1",
      r_pkg is True and r_sec is True and r_wh is True)

# ══════════════════════════════════════════════════════════════════

section("2 — Une app exigeant CAMERA_DEPTH est refusée, même aux côtés "
        "des 3 apps légitimes déjà installées")


class _DepthApp(RobotApp):
    def manifest(self):
        return AppManifest(
            app_id="com.test.depth_refapps", name="T", version="1.0.0", author="T",
            author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
            capabilities_required=[Capability.CAMERA_DEPTH],
        )
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass


r_depth = runtime1.install(_DepthApp())
check("App exigeant CAMERA_DEPTH refusée — le G1 ne déclare que CAMERA "
      "générique, mapping conservateur (voir _available_capabilities())",
      r_depth is False)

# ══════════════════════════════════════════════════════════════════

section("3 — Une app tierce déclarant ACTUATORS_RAW (SIGNATURE) est refusée, "
        "même aux côtés des 3 apps légitimes")


class _RawApp(RobotApp):
    def manifest(self):
        return AppManifest(
            app_id="com.attacker.raw_refapps", name="T", version="1.0.0", author="T",
            author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
            permissions=[Permission.ACTUATORS_RAW],
        )
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass


r_raw = runtime1.install(_RawApp())
check("App tierce avec ACTUATORS_RAW refusée (aucune clé plateforme "
      "configurée sur ce runtime — fail-safe)",
      r_raw is False)

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ Les 3 apps de référence déclarent de vraies capacités, pas")
    print("    une liste vide qui ne prouve rien")
    print("  ✓ Une app malveillante (CAMERA_DEPTH factice, ou ACTUATORS_RAW")
    print("    sans clé plateforme) reste refusée aux côtés d'apps légitimes")

sys.exit(0 if FAIL == 0 else 1)
