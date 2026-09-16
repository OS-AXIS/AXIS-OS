"""
AXIS-OS — Tests move_to_pose
================================
Valide RobotContext.move_to_pose() et son équivalent HALBridgeClient.
move_to_pose(), ajoutés suite à un point pratique relevé après le
correctif C5 (audit de sécurité) : position_delta_max protège
correctement contre un saut de position dangereux, mais n'offrait à
aucune app un moyen d'atteindre une cible légitime au-delà de cette
limite en un seul appel — bloquant package_sorting.on_start() sur
tout G1 qui ne démarre pas déjà bras à zéro.
"""

import sys
import multiprocessing as mp
sys.path.insert(0, ".")

from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.core.types import ActuatorCommand, ControlMode, HALStatus
from axis_os.store.sdk.app_base import RobotContext
from axis_os.store.sdk.manifest import AppManifest, AppCategory, Permission
from axis_os.store.runtime.hal_bridge import HALBridgeServer
from axis_os.store.runtime.play_store import PlayStoreRuntime
from axis_os.apps.package_sorting.app import PackageSortingApp

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def make_ctx():
    robot = NormalizedRobotHAL(UnitreeG1HAL("g1_movepose"), build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    manifest = AppManifest(
        app_id="com.test.movepose", name="T", version="1.0.0", author="T",
        author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
        permissions=[Permission.ACTUATORS_READ, Permission.ACTUATORS_WRITE_ARMS],
    )
    ctx = RobotContext.for_testing(robot, wd, manifest)
    return robot, ctx


# ══════════════════════════════════════════════════════════════════

section("1 — move_to_pose() atteint une cible au-delà de position_delta_max")

robot1, ctx1 = make_ctx()
act1 = robot1._hal._actuators["left_elbow_joint"]
delta_max = act1._guard.limits.position_delta_max
act1._position = 1.2  # au-delà du delta max depuis 0.0

direct_result = ctx1.move_arms({
    "left_elbow_joint": ActuatorCommand(mode=ControlMode.POSITION, position=0.0)
})
check("Confirme le problème : un move_arms() direct est bien refusé",
      direct_result["left_elbow_joint"] == HALStatus.ERROR_LIMIT)

act1._position = 1.2  # reset après le test de confirmation ci-dessus
reached1 = ctx1.move_to_pose({"left_elbow_joint": 0.0}, duration_s=0.3, step_s=0.03)
check("move_to_pose() atteint la cible malgré position_delta_max",
      reached1 is True)
check("Position réelle finale proche de la cible (0.0)",
      abs(act1._position) < 0.01)

# ══════════════════════════════════════════════════════════════════

section("2 — move_to_pose() échoue proprement sur un joint introuvable")

robot2, ctx2 = make_ctx()
result2 = ctx2.move_to_pose({"joint_qui_nexiste_pas": 0.0}, duration_s=0.1, step_s=0.05)
check("Joint introuvable -> retourne False (pas de crash)", result2 is False)

# ══════════════════════════════════════════════════════════════════

section("3 — move_to_pose() s'arrête immédiatement dès qu'un pas échoue")

robot3, ctx3 = make_ctx()
act3 = robot3._hal._actuators["left_elbow_joint"]
act3._position = 0.0
# Cible hors des vraies limites (pos_max ~2.09 pour le coude) — même
# en interpolant, le dernier pas dépassera forcément pos_max.
result3 = ctx3.move_to_pose({"left_elbow_joint": 99.0}, duration_s=0.2, step_s=0.05)
check("Cible finale hors limites physiques -> échoue proprement (False)",
      result3 is False)

# ══════════════════════════════════════════════════════════════════

section("4 — Non-régression : package_sorting démarre avec les bras repliés")

robot4 = NormalizedRobotHAL(UnitreeG1HAL("g1_folded"), build_g1_normalizer())
robot4.initialize_all()
for jid in ("left_elbow_joint", "right_elbow_joint"):
    robot4._hal._actuators[jid]._position = 1.2  # bras repliés au démarrage

wd4 = WatchdogSystem(robot4)
runtime4 = PlayStoreRuntime(robot4, wd4)
runtime4.install(PackageSortingApp())
ready4 = runtime4.start_app("com.acme.package_sorting")
check("package_sorting démarre malgré des bras repliés à 1.2 rad", ready4 is True)
check("Position finale du coude gauche proche de HOME (0.0)",
      abs(robot4._hal._actuators["left_elbow_joint"]._position) < 0.01)

# Vérifie qu'une vraie mission fonctionne ensuite normalement.
result4 = runtime4.run_mission("com.acme.package_sorting", params={})
check("Mission après ce démarrage interpolé réussit normalement",
      result4 is not None and result4.success)

# ══════════════════════════════════════════════════════════════════

section("5 — HALBridgeClient.move_to_pose() : même comportement en sandbox")

robot5 = NormalizedRobotHAL(UnitreeG1HAL("g1_sandbox_movepose"), build_g1_normalizer())
robot5.initialize_all()
robot5._hal._actuators["left_elbow_joint"]._position = 1.2
wd5 = WatchdogSystem(robot5)
manifest5 = PackageSortingApp().manifest()
ctx5 = RobotContext.for_testing(robot5, wd5, manifest5)

cq, rq = mp.Queue(), mp.Queue()
server5 = HALBridgeServer(ctx5, cq, rq)
server5.start()

from axis_os.store.runtime.hal_bridge import HALBridgeClient
bridge_client = HALBridgeClient(cq, rq)
reached5 = bridge_client.move_to_pose({"left_elbow_joint": 0.0}, duration_s=0.3, step_s=0.03)
check("HALBridgeClient.move_to_pose() atteint la cible en sandbox",
      reached5 is True)
server5.stop()

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ move_to_pose() atteint des cibles au-delà de position_delta_max")
    print("  ✓ Échoue proprement (jamais de crash) sur joint introuvable ou")
    print("    cible finale hors limites")
    print("  ✓ package_sorting démarre maintenant même avec les bras repliés")
    print("  ✓ Même comportement en sandbox (HALBridgeClient)")

sys.exit(0 if FAIL == 0 else 1)
