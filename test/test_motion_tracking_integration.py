"""
AXIS-OS — Test Motion Tracking Integration (drift detection)
================================================================
Referme une dépendance identifiée par TROIS revues externes
indépendantes (Safety Layer, Watchdog, Conventions) : l'absence
d'historique de commande empêchait toute détection d'écart
commande/feedback. MotionStateTracker existait déjà (session
précédente) mais n'était branché nulle part — ce test valide son
câblage réel dans UnitreeG1HAL et son exploitation par WatchdogSystem.

    python test_motion_tracking_integration.py
"""

import sys
import time
sys.path.insert(0, ".")

from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.watchdog.config import WatchdogConfig
from axis_os.hal.watchdog.events import AlertKind, RobotHealthState
from axis_os.hal.core.types import ActuatorCommand, ControlMode
from axis_os.hal.core.motion_state import MotionStateTracker

PASS = 0
FAIL = 0


def section(title: str):
    print(f"\n{'═'*66}\n  {title}\n{'═'*66}")


def check(label: str, condition: bool) -> bool:
    global PASS, FAIL
    icon = "✓" if condition else "✗"
    print(f"  [{icon}] {label:<58} {'OK' if condition else 'FAILED'}")
    if condition: PASS += 1
    else: FAIL += 1
    return condition


def make_robot():
    r = UnitreeG1HAL("g1_tracking_test")
    r.initialize_all()
    return r


# ══════════════════════════════════════════════════════════════════

section("1 — UnitreeG1HAL expose un vrai MotionStateTracker")
robot = make_robot()
tracker = robot.get_motion_tracker()
check("get_motion_tracker() retourne une instance, pas None",
      tracker is not None)
check("Le tracker connaît les 23 joints du G1 (fichier URDF officiel)",
      isinstance(tracker, MotionStateTracker) and len(tracker.joint_ids) == 23)
robot.shutdown_all()

section("2 — IRobotHAL générique : no-op sûr par défaut")
from axis_os.hal.core.interfaces import IRobotHAL

class DummyHAL(IRobotHAL):
    """Backend minimal qui n'implémente PAS get_motion_tracker —
    doit hériter le comportement par défaut (None) sans erreur."""
    def initialize_all(self): pass
    def shutdown_all(self): pass
    def emergency_stop_all(self): pass
    def get_actuator(self, joint_id): return None
    def get_all_actuators(self): return {}
    def get_imu(self, imu_id="base_imu"): return None
    def get_force_torque(self, sensor_id): return None
    def send_joint_commands(self, commands): return {}
    def get_full_state(self): return {}
    def get_joint_definitions(self): return []
    def get_robot_info(self): return {}

dummy = DummyHAL("dummy")
check("Un HAL qui n'implémente pas get_motion_tracker() hérite "
      "None par défaut (rétrocompatibilité stricte)",
      dummy.get_motion_tracker() is None)

section("3 — send_joint_commands() enregistre réellement dans le tracker")
robot2 = make_robot()
cmd = ActuatorCommand(ControlMode.POSITION, position=0.3)
robot2.send_joint_commands({"left_elbow_joint": cmd})
tracker2 = robot2.get_motion_tracker()
recorded = tracker2.get("left_elbow_joint").last_command
check("La commande envoyée via le HAL est bien enregistrée dans le tracker",
      recorded is not None and recorded.position == 0.3)
robot2.shutdown_all()

section("4 — get_full_state() enregistre réellement les feedbacks")
robot3 = make_robot()
state = robot3.get_full_state()  # lit tous les joints, doit peupler le tracker
tracker3 = robot3.get_motion_tracker()
check("Au moins un feedback enregistré après get_full_state()",
      tracker3.get("left_knee_joint").last_feedback is not None)
robot3.shutdown_all()

section("5 — Détection d'écart : moteur qui suit sa consigne → pas d'alerte")
robot4 = make_robot()
cfg = WatchdogConfig.standard(); cfg.poll_interval_s = 0.01
wd4 = WatchdogSystem(robot4, cfg)
wd4.start()
time.sleep(0.05)

robot4.send_joint_commands({
    "left_knee_joint": ActuatorCommand(ControlMode.POSITION, position=0.1)
})
act4 = robot4.get_actuator("left_knee_joint")
act4._position = 0.1  # le moteur suit correctement sa consigne
time.sleep(0.15)

check("health_state == HEALTHY quand le feedback suit la commande",
      wd4.health_state == RobotHealthState.HEALTHY)
wd4.stop(); robot4.shutdown_all()

section("6 — Détection d'écart : moteur bloqué → DEGRADED (le vrai test)")
robot5 = make_robot()
cfg5 = WatchdogConfig.standard()
cfg5.poll_interval_s = 0.01
cfg5.position_drift_max_rad = 0.1
wd5 = WatchdogSystem(robot5, cfg5)
wd5.start()
time.sleep(0.05)

robot5.send_joint_commands({
    "left_knee_joint": ActuatorCommand(ControlMode.POSITION, position=0.5)
})
act5 = robot5.get_actuator("left_knee_joint")
act5._position = 0.0  # le moteur NE bouge PAS malgré la commande
time.sleep(0.15)

check("health_state == DEGRADED quand un moteur ne suit plus sa consigne "
      "(bug réel qu'un vrai robot pourrait rencontrer)",
      wd5.health_state == RobotHealthState.DEGRADED)
wd5.stop(); robot5.shutdown_all()

section("7 — Anti faux-positif : confirmation multi-échantillons requise")
robot6 = make_robot()
cfg6 = WatchdogConfig.standard()
cfg6.poll_interval_s = 0.05  # cycle plus lent : 1 seul échantillon aura le temps
cfg6.position_drift_max_rad = 0.05
wd6 = WatchdogSystem(robot6, cfg6)
wd6.start()
time.sleep(0.02)

robot6.send_joint_commands({
    "left_knee_joint": ActuatorCommand(ControlMode.POSITION, position=0.5)
})
act6 = robot6.get_actuator("left_knee_joint")
act6._position = 0.0
time.sleep(0.03)  # 1 seul cycle écoulé — pas encore confirm_samples atteint
early_state = wd6.health_state
act6._position = 0.5  # redresse avant confirmation
time.sleep(0.15)
check("Pas de DEGRADED avant confirm_samples atteint (anti faux-positif)",
      early_state != RobotHealthState.DEGRADED)
wd6.stop(); robot6.shutdown_all()

section("8 — Robustesse : NaN dans le feedback n'affole pas le check")
robot7 = make_robot()
cfg7 = WatchdogConfig.standard(); cfg7.poll_interval_s = 0.01
wd7 = WatchdogSystem(robot7, cfg7)
wd7.start()
time.sleep(0.05)

robot7.send_joint_commands({
    "left_knee_joint": ActuatorCommand(ControlMode.POSITION, position=0.5)
})
act7 = robot7.get_actuator("left_knee_joint")
act7._position = float("nan")
time.sleep(0.1)
check("Aucun crash du Watchdog avec un feedback NaN", wd7.is_running)
wd7.stop(); robot7.shutdown_all()

section("RÉSUMÉ")
total = PASS + FAIL
print(f"  Résultats : {PASS}/{total} tests passés\n")
if FAIL == 0:
    print("  ✓ MotionStateTracker réellement branché dans UnitreeG1HAL")
    print("  ✓ IRobotHAL générique : no-op sûr pour tout backend sans tracker")
    print("  ✓ Commandes ET feedbacks enregistrés via le vrai pipeline HAL")
    print("  ✓ Détection d'écart commande/feedback fonctionnelle (POSITION_DRIFT)")
    print("  ✓ Anti faux-positif confirmé, robustesse NaN confirmée")
    print("  → Dépendance identifiée par 3 revues externes indépendantes,")
    print("    désormais refermée et testée.")
else:
    print(f"  ✗ {FAIL} test(s) échoué(s).")

sys.exit(0 if FAIL == 0 else 1)
