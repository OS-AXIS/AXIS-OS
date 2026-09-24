"""
AXIS-OS — Tests des 3 actions ajoutées au pont HAL
=======================================================
Valide get_joint_state, move_all, subscribe_alerts côté
HALBridgeClient/HALBridgeServer — ajoutées suite au test de
non-divergence (test_context_parity.py) qui a révélé ces trois
méthodes manquantes côté sandbox.

Point de sécurité central testé explicitement (section 1) : la
vérification de permission pour move_all() a lieu CÔTÉ SERVEUR
(HALBridgeServer._handle(), via le vrai RobotContext), jamais côté
client — le process enfant sandboxé est non fiable, sa propre
vérification ne compterait pour rien.
"""

import sys
import multiprocessing as mp
import time
sys.path.insert(0, ".")

from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem, WatchdogConfig
from axis_os.hal.core.types import ActuatorCommand, ControlMode, HALStatus
from axis_os.store.sdk.app_base import RobotContext
from axis_os.store.sdk.manifest import AppManifest, AppCategory, Permission
from axis_os.store.runtime.hal_bridge import HALBridgeServer, HALBridgeClient

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def make_bridge(permissions):
    robot = NormalizedRobotHAL(UnitreeG1HAL(f"g1_bridge_{id(permissions)}"),
                               build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    manifest = AppManifest(
        app_id="com.test.bridge3", name="T", version="1.0.0", author="T",
        author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
        permissions=permissions,
    )
    ctx = RobotContext.for_testing(robot, wd, manifest)
    cq, rq = mp.Queue(), mp.Queue()
    server = HALBridgeServer(ctx, cq, rq)
    server.start()
    client = HALBridgeClient(cq, rq)
    return robot, wd, server, client


# ══════════════════════════════════════════════════════════════════

section("1 — CRITIQUE : move_all() vérifié côté SERVEUR, pas côté client")

robot1, wd1, server1, client1 = make_bridge([Permission.ACTUATORS_READ])  # SANS write_all
from axis_os.store.sdk.app_base import PermissionError as _PermErr

try:
    client1.move_all({"waist_yaw_joint": ActuatorCommand(mode=ControlMode.POSITION, position=0.1)})
    check("Sans ACTUATORS_WRITE_ALL, move_all() refusé", False)
except _PermErr as e:
    # Attendait auparavant un RuntimeError générique : le pont ne
    # transportait que le MESSAGE de l'erreur, pas son TYPE. Une app
    # sandboxée ne pouvait donc pas distinguer une permission manquante
    # d'une autre panne, alors qu'en direct elle recevait une
    # PermissionError.
    check("Sans ACTUATORS_WRITE_ALL, move_all() refusé (le client n'a "
          "AUCUN moyen de le contourner — vérification côté serveur)",
          "Permission" in str(e))
server1.stop()

robot2, wd2, server2, client2 = make_bridge(
    [Permission.ACTUATORS_READ, Permission.ACTUATORS_WRITE_ALL])
result2 = client2.move_all({
    "waist_yaw_joint": ActuatorCommand(mode=ControlMode.POSITION, position=0.1)
})
check("Avec ACTUATORS_WRITE_ALL, move_all() réussit", result2["waist_yaw_joint"] == HALStatus.OK)
check("waist_yaw_joint (ni bras ni jambe) désormais atteignable en sandbox "
      "— seul move_all() peut le router",
      "waist_yaw_joint" in result2)
server2.stop()

# ══════════════════════════════════════════════════════════════════

section("2 — get_joint_state()")

robot3, wd3, server3, client3 = make_bridge([Permission.ACTUATORS_READ])
state = client3.get_joint_state("left_knee_joint")
# Contrat mis à jour : le pont reconstruit désormais de vrais
# ActuatorFeedback au lieu de rendre les dicts sérialisés du transport.
# Ce test vérifiait l'ancien comportement, qui était précisément la
# divergence à fermer — une app écrite contre RobotContext plantait
# uniquement en sandbox (AttributeError: 'dict' object has no attribute
# 'position'). Voir test_context_parity.py, section types de retour.
from axis_os.hal.core.types import ActuatorFeedback
check("get_joint_state() retourne un ActuatorFeedback (pas un dict)",
      isinstance(state, ActuatorFeedback))
check("get_joint_state() expose les champs en attributs",
      state is not None and all(hasattr(state, a) for a in
      ("position", "velocity", "torque", "temperature", "state")))

state_missing = client3.get_joint_state("joint_inexistant")
check("get_joint_state() sur un joint inexistant retourne None (pas d'exception)",
      state_missing is None)
server3.stop()

# ══════════════════════════════════════════════════════════════════

section("3 — subscribe_alerts() (polling, voir sa docstring pour le contexte)")

robot4 = NormalizedRobotHAL(UnitreeG1HAL("g1_bridge_alerts"), build_g1_normalizer())
robot4.initialize_all()
cfg4 = WatchdogConfig.standard()
cfg4.poll_interval_s = 0.02
wd4 = WatchdogSystem(robot4, cfg4)
wd4.start()
manifest4 = AppManifest(
    app_id="com.test.bridge3b", name="T", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
    permissions=[Permission.WATCHDOG_READ],
)
ctx4 = RobotContext.for_testing(robot4, wd4, manifest4)
cq4, rq4 = mp.Queue(), mp.Queue()
server4 = HALBridgeServer(ctx4, cq4, rq4)
server4.start()
client4 = HALBridgeClient(cq4, rq4, alert_poll_interval_s=0.05)

received = []
client4.subscribe_alerts(lambda a: received.append(a))

act4 = robot4._hal._actuators["left_knee_joint"]
act4._position = 99.0  # au-delà de pos_max — déclenche POSITION_LIMIT_BREACH
time.sleep(0.35)

server4.stop()
wd4.stop()

check("Au moins une alerte reçue via le polling", len(received) > 0)
check("Le polling relaie bien une vraie alerte POSITION_LIMIT_BREACH "
      "(confirme aussi que le bug de délégation get_motion_tracker "
      "est corrigé — voir NormalizedRobotHAL)",
      any(a["kind"] == "POSITION_LIMIT_BREACH" for a in received))

# ══════════════════════════════════════════════════════════════════

section("4 — Non-régression : NormalizedRobotHAL.get_motion_tracker() délègue bien")

robot5 = NormalizedRobotHAL(UnitreeG1HAL("g1_tracker_delegation"), build_g1_normalizer())
robot5.initialize_all()
tracker = robot5.get_motion_tracker()
check("NormalizedRobotHAL.get_motion_tracker() ne lève plus AttributeError",
      tracker is not None)

# ══════════════════════════════════════════════════════════════════

# ══════════════════════════════════════════════════════════════════

section("Le TYPE d'erreur traverse le pont, pas seulement son message")

# Divergence constatée en démonstration : en direct, une permission
# manquante lève PermissionError ; à travers le pont, elle devenait un
# RuntimeError générique que les méthodes du client avalaient en rendant
# False. Une app sandboxée ne pouvait plus distinguer « je n'ai pas le
# droit » de « ce robot n'a pas de jambes ».
from axis_os.store.sdk.app_base import PermissionError as _PermErr

robot_p, wd_p, server_p, client_p = make_bridge([Permission.ACTUATORS_READ])
try:
    client_p.move_arms({"left_elbow_joint":
                        ActuatorCommand(mode=ControlMode.POSITION, position=0.1)})
    leve = None
except _PermErr as exc:
    leve = "permission"
except RuntimeError:
    leve = "runtime"
check("Permission manquante : PermissionError, comme en direct — "
      f"pas un RuntimeError générique (obtenu: {leve})",
      leve == "permission")

check("PermissionError n'hérite PAS de RuntimeError, donc elle n'est "
      "pas avalée par les except RuntimeError du client",
      not issubclass(_PermErr, RuntimeError))
server_p.stop()

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

sys.exit(0 if FAIL == 0 else 1)
