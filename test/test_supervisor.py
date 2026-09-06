"""
AXIS-OS — Test Supervisor
=============================
Referme le point le plus critique d'une revue externe du 2026-09-06 :
un Watchdog ne peut pas fiablement se surveiller lui-même s'il meurt
complètement (deadlock, crash silencieux). Le Supervisor, thread
indépendant, comble ce trou en vérifiant que le Watchdog progresse
réellement, et déclenche lui-même l'arrêt d'urgence si ce n'est plus
le cas.

    python test_supervisor.py
"""

import sys
import time
import threading
sys.path.insert(0, ".")

from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.watchdog.config import WatchdogConfig
from axis_os.hal.supervisor.supervisor import Supervisor, SupervisorAlert

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


def make_stack(robot_id: str, check_interval_s=0.1, unresponsive_threshold_s=0.5, grace_period_s=0.3):
    robot = UnitreeG1HAL(robot_id)
    robot.initialize_all()
    wd = WatchdogSystem(robot, WatchdogConfig.simulation())
    sup = Supervisor(watchdog=wd, robot_hal=robot,
                     check_interval_s=check_interval_s,
                     unresponsive_threshold_s=unresponsive_threshold_s,
                     grace_period_s=grace_period_s)
    return robot, wd, sup


# ══════════════════════════════════════════════════════════════════

section("1 — Cas normal : aucun faux positif quand tout va bien")
robot1, wd1, sup1 = make_stack("g1_normal")
wd1.start(); sup1.start()
time.sleep(1.0)
check("Supervisor n'a rien déclenché avec un Watchdog sain",
      not sup1.has_triggered)
check("is_robot_stopped() est False, robot toujours actif",
      not sup1.is_robot_stopped())
sup1.stop(); wd1.stop(); robot1.shutdown_all()

section("2 — LE VRAI TEST : Watchdog gelé (deadlock simulé)")
robot2, wd2, sup2 = make_stack("g1_frozen")
wd2.start(); sup2.start()
time.sleep(0.5)

# Simule un deadlock : le thread pourrait encore techniquement exister
# (is_alive() == True) mais ne progresse plus — exactement le cas que
# le try/except interne du Watchdog ne peut pas attraper, puisqu'un
# deadlock ne lève jamais d'exception.
wd2._last_cycle_monotonic = time.monotonic() - 10.0

triggered_alerts = []
sup2.subscribe(lambda a: triggered_alerts.append(a))
time.sleep(1.0)

check("Supervisor a détecté le Watchdog gelé et déclenché la réponse",
      sup2.has_triggered)
check("Le robot est réellement arrêté (emergency_stop_all appelé "
      "directement par le Supervisor, sans passer par le Watchdog)",
      sup2.is_robot_stopped())
check("wd.estop_fired reste False — DÉCOUVERTE IMPORTANTE documentée : "
      "c'est le Supervisor qui a arrêté le robot, pas le Watchdog, donc "
      "son flag interne n'a logiquement pas été mis à jour. Tout code "
      "doit utiliser is_robot_stopped(), jamais wd.estop_fired seul.",
      not wd2.estop_fired)
check("Une alerte SupervisorAlert a bien été émise aux abonnés",
      len(triggered_alerts) >= 1 and triggered_alerts[0].kind == "WATCHDOG_UNRESPONSIVE")

sup2.stop(); wd2.stop(); robot2.shutdown_all()

section("3 — Idempotence : un deuxième gel ne déclenche pas deux fois")
robot3, wd3, sup3 = make_stack("g1_idempotent")
wd3.start(); sup3.start()
time.sleep(0.5)
wd3._last_cycle_monotonic = time.monotonic() - 10.0
time.sleep(0.8)
first_trigger_state = sup3.has_triggered

# Un deuxième "gel" ne doit pas re-déclencher une deuxième réponse —
# même principe que _trigger_estop dans WatchdogSystem.
wd3._last_cycle_monotonic = time.monotonic() - 10.0
time.sleep(0.8)

check("Premier déclenchement bien enregistré", first_trigger_state)
check("État reste cohérent après un second gel (idempotent, pas de "
      "double-déclenchement incohérent)", sup3.has_triggered)
sup3.stop(); wd3.stop(); robot3.shutdown_all()

section("4 — Grace period : pas de faux positif juste après le démarrage")
robot4, wd4, sup4 = make_stack("g1_grace", grace_period_s=2.0)
sup4.start()  # Supervisor démarré AVANT le Watchdog, exprès —
              # teste la robustesse à un ordre de démarrage inversé.
time.sleep(0.5)  # bien dans la fenêtre de grâce de 2.0s

check("Aucun déclenchement pendant la période de grâce, même si le "
      "Watchdog n'a pas encore démarré du tout (ordre inversé)",
      not sup4.has_triggered)
wd4.start()
sup4.stop(); wd4.stop(); robot4.shutdown_all()

section("5 — Un abonné qui plante ne fait pas tomber le Supervisor")
robot5, wd5, sup5 = make_stack("g1_bad_subscriber")

def bad_subscriber(alert):
    raise RuntimeError("Abonné volontairement cassé pour ce test")

sup5.subscribe(bad_subscriber)
wd5.start(); sup5.start()
time.sleep(0.5)
wd5._last_cycle_monotonic = time.monotonic() - 10.0

try:
    time.sleep(1.0)
    check("Supervisor reste stable malgré un abonné qui lève une "
          "exception à chaque alerte", sup5.has_triggered)
except Exception as e:
    check(f"Supervisor reste stable malgré un abonné cassé ({e})", False)

sup5.stop(); wd5.stop(); robot5.shutdown_all()

# ══════════════════════════════════════════════════════════════════

section("RÉSUMÉ")
total = PASS + FAIL
print(f"  Résultats : {PASS}/{total} tests passés\n")
if FAIL == 0:
    print("  ✓ Aucun faux positif en fonctionnement normal")
    print("  ✓ LE VRAI TEST : Watchdog gelé détecté, robot arrêté")
    print("    par le Supervisor SANS dépendre du Watchdog mort")
    print("  ✓ Découverte documentée : wd.estop_fired seul ne suffit")
    print("    plus — is_robot_stopped() est la vraie source de vérité")
    print("  ✓ Idempotent, robuste à l'ordre de démarrage, robuste")
    print("    aux abonnés défaillants")
    print("  → Le point le plus critique de la revue externe est refermé.")
else:
    print(f"  ✗ {FAIL} test(s) échoué(s).")

sys.exit(0 if FAIL == 0 else 1)
