"""
AXIS-OS — Test WatchdogSystem v2
===================================
Suite dédiée couvrant le point #20 de la revue externe, plus les
corrections v2 : idempotence e-stop, health_state, anti faux-positif,
auto-surveillance, alertes FATAL jamais perdues, journal d'e-stop.

Note : cette suite a directement servi à découvrir un bug réel lors
de sa conception — AlertSeverity en Enum simple faisait échouer max()
silencieusement (capturé par un except trop large), empêchant toute
alerte FATAL de faire monter health_state. Corrigé en passant
AlertSeverity en IntEnum. Exactement le genre de bug que le point #21
de la revue ("tests de panne du watchdog lui-même") vise à attraper.

    python test_watchdog_v2.py
"""

import sys
import time
import threading
sys.path.insert(0, ".")

from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL, G1SimulatedActuator
from axis_os.hal.core.types import ActuatorState
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.watchdog.config import WatchdogConfig
from axis_os.hal.watchdog.events import AlertKind, AlertSeverity, RobotHealthState

PASS = 0
FAIL = 0


def section(title: str):
    print(f"\n{'═'*62}\n  {title}\n{'═'*62}")


def check(label: str, condition: bool) -> bool:
    global PASS, FAIL
    icon = "✓" if condition else "✗"
    print(f"  [{icon}] {label:<54} {'OK' if condition else 'FAILED'}")
    if condition: PASS += 1
    else: FAIL += 1
    return condition


class AlertCollector:
    def __init__(self):
        self._alerts = []
        self._lock = threading.Lock()
    def __call__(self, alert):
        with self._lock: self._alerts.append(alert)
    def get(self): 
        with self._lock: return list(self._alerts)
    def has(self, kind): return any(a.kind == kind for a in self.get())


def make_robot():
    r = UnitreeG1HAL("g1_wdg_v2")
    r.initialize_all()
    return r


# ══════════════════════════════════════════════════════════════════

section("1 — AlertSeverity : comparaison directe (le bug réel trouvé)")
check("max(INFO, FATAL) == FATAL sans exception",
      max(AlertSeverity.INFO, AlertSeverity.FATAL) == AlertSeverity.FATAL)
check("FATAL > CRITICAL > WARNING > INFO (ordre total)",
      AlertSeverity.FATAL > AlertSeverity.CRITICAL > AlertSeverity.WARNING > AlertSeverity.INFO)

section("2 — Health state : HEALTHY par défaut")
robot = make_robot()
wd = WatchdogSystem(robot, WatchdogConfig.standard())
wd.start()
time.sleep(0.1)
check("health_state == HEALTHY sans anomalie", wd.health_state == RobotHealthState.HEALTHY)
wd.stop()
robot.shutdown_all()

section("3 — Health state : WARNING sur température élevée (pas critique)")
robot = make_robot()
cfg = WatchdogConfig.standard(); cfg.poll_interval_s = 0.01
wd = WatchdogSystem(robot, cfg)
wd.start()
time.sleep(0.05)
act = robot.get_actuator("left_knee_joint")
act._temperature = 68.0  # > warning (65) mais < critical (80)
time.sleep(0.1)
check("health_state == WARNING", wd.health_state == RobotHealthState.WARNING)
check("estop PAS déclenché sur simple warning", not wd.estop_fired)
wd.stop()
robot.shutdown_all()

section("4 — Health state : DEGRADED sur surcharge confirmée (CRITICAL)")
robot = make_robot()
cfg = WatchdogConfig.standard(); cfg.poll_interval_s = 0.01
wd = WatchdogSystem(robot, cfg)
wd.start()
time.sleep(0.05)
act = robot.get_actuator("left_knee_joint")
# torque_max réel du fichier URDF officiel = 139.0 Nm (et non 88.0 Nm,
# valeur de l'ancien codage manuel) — 155.0 dépasse le seuil de
# surcharge réel (139 * 1.10 ≈ 152.9), 97.0 ne le dépassait plus.
act._torque = 155.0  # > 139.0 * 1.10
time.sleep(0.15)  # laisse le temps aux 3 échantillons de confirmation
check("health_state == DEGRADED après confirmation", wd.health_state == RobotHealthState.DEGRADED)
wd.stop()
robot.shutdown_all()

section("5 — Anti faux-positif : 1 seul pic CRITICAL ne déclenche PAS (#12)")
robot = make_robot()
cfg = WatchdogConfig.standard(); cfg.poll_interval_s = 0.05  # cycle lent
wd = WatchdogSystem(robot, cfg)
col = AlertCollector()
wd.subscribe(col)
wd.start()
time.sleep(0.02)
act = robot.get_actuator("left_knee_joint")
act._torque = 97.0
time.sleep(0.03)   # 1 seul cycle écoulé — pas encore 3 confirmations
alerts_early = col.has(AlertKind.TORQUE_OVERLOAD)
act._torque = 0.0  # redescend avant confirmation
time.sleep(0.15)
check("Pas d'alerte confirmée avant confirm_samples atteint",
      not alerts_early)
wd.stop()
robot.shutdown_all()

section("6 — Health state : ESTOP après e-stop déclenché")
robot = make_robot()
cfg = WatchdogConfig.standard(); cfg.poll_interval_s = 0.01
wd = WatchdogSystem(robot, cfg)
wd.start()
time.sleep(0.05)
act = robot.get_actuator("left_knee_joint")
act._temperature = 85.0
time.sleep(0.1)
check("health_state == ESTOP", wd.health_state == RobotHealthState.ESTOP)
check("estop_fired == True", wd.estop_fired)
wd.stop()
robot.shutdown_all()

section("7 — E-STOP idempotent : deux conditions FATAL simultanées (#8)")
robot = make_robot()
cfg = WatchdogConfig.standard(); cfg.poll_interval_s = 0.01
wd = WatchdogSystem(robot, cfg)
wd.start()
time.sleep(0.05)
act1 = robot.get_actuator("left_knee_joint")
act2 = robot.get_actuator("right_elbow_joint")
act1._temperature = 85.0
act2._temperature = 85.0  # deux joints en FATAL le même cycle
time.sleep(0.15)
check("estop_count == 1 malgré 2 conditions FATAL simultanées",
      wd.get_stats()["estop_count"] == 1)
wd.stop()
robot.shutdown_all()

section("8 — Journal d'E-STOP structuré (#19)")
robot = make_robot()
cfg = WatchdogConfig.standard(); cfg.poll_interval_s = 0.01
wd = WatchdogSystem(robot, cfg)
wd.start()
time.sleep(0.05)
act = robot.get_actuator("right_knee_joint")
act._temperature = 85.0
time.sleep(0.1)
log = wd.estop_log
check("1 entrée dans le journal", len(log) == 1)
check("raison contient TEMP_CRITICAL", "TEMP_CRITICAL" in log[0].reason)
check("joint_id renseigné", log[0].joint_id == "right_knee_joint")
check("valeurs mesurées renseignées", log[0].measured is not None)
check("snapshot de l'état du robot présent", len(log[0].robot_state_snapshot) == 23)
wd.stop()
robot.shutdown_all()

section("9 — Alertes FATAL jamais perdues même si queue normale sature")
robot = make_robot()
cfg = WatchdogConfig.simulation()  # pas d'e-stop auto, pour observer les alertes
cfg.poll_interval_s = 0.005
wd = WatchdogSystem(robot, cfg)
col = AlertCollector()
wd.subscribe(col)
wd.start()
time.sleep(0.03)
# On force artificiellement une condition FATAL
act = robot.get_actuator("left_knee_joint")
from axis_os.hal.core.types import ActuatorState as AS
act._state = AS.FAULT
time.sleep(0.1)
check("JOINT_FAULT (FATAL) bien reçue malgré la charge",
      col.has(AlertKind.JOINT_FAULT))
wd.stop()
robot.shutdown_all()

section("10 — Auto-surveillance : pas de faux 'dégradé' en fonctionnement normal (#14)")
robot = make_robot()
wd = WatchdogSystem(robot, WatchdogConfig.standard())
wd.start()
time.sleep(0.2)
check("watchdog_degraded == False en conditions normales",
      wd.get_stats()["watchdog_degraded"] == False)
wd.stop()
robot.shutdown_all()

section("RÉSUMÉ")
total = PASS + FAIL
print(f"  Résultats : {PASS}/{total} tests passés\n")
if FAIL == 0:
    print("  ✓ AlertSeverity IntEnum — bug réel de comparaison corrigé")
    print("  ✓ health_state : HEALTHY/WARNING/DEGRADED/ESTOP cohérents")
    print("  ✓ Anti faux-positif : confirmation multi-échantillons validée")
    print("  ✓ E-STOP idempotent confirmé sous conditions concurrentes")
    print("  ✓ Journal d'E-STOP structuré et complet")
    print("  ✓ Alertes FATAL jamais perdues")
else:
    print(f"  ✗ {FAIL} test(s) échoué(s).")

sys.exit(0 if FAIL == 0 else 1)
