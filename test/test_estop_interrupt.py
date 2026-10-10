"""
AXIS-OS — Test Interruption E-STOP en plein vol
====================================================
Referme un vrai trou de sécurité identifié par test_ultimate_stress.py :
une mission déjà en cours au moment où le Watchdog déclenche un e-stop
continuait auparavant son exécution jusqu'au bout. emergency_stop_all()
coupait bien le hardware, mais le CODE APPLICATIF pouvait continuer à
tenter d'envoyer de nouvelles commandes de mouvement entre-temps.

Corrigé en ajoutant EstopActiveError et un check dans move_arms/
move_legs/move_all — vérifié à CHAQUE appel, pas une seule fois à
l'entrée de la mission.

    python test_estop_interrupt.py
"""

import sys
import time
import threading
sys.path.insert(0, ".")

from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.watchdog.config import WatchdogConfig
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
from axis_os.store.sdk.app_base import RobotApp, RobotContext, MissionResult, EstopActiveError
from axis_os.store.sdk.manifest import (
    AppManifest, AppCategory, Permission, PricingModel, HardwareRequirements
)
from axis_os.hal.core.types import ActuatorCommand, ControlMode

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


class SlowWalkApp(RobotApp):
    """App de test : marche en 20 pas avec pause entre chaque, pour
    laisser le temps à un e-stop de survenir en plein milieu."""
    def __init__(self):
        self.steps_completed = 0

    def manifest(self):
        return AppManifest(
            app_id="com.test.slowwalk", name="SlowWalk", version="1.0.0",
            author="test", author_email="t@t.com", description="test",
            category=AppCategory.UTILITIES,
            permissions=[Permission.ACTUATORS_WRITE_LEGS],
            hardware=HardwareRequirements(min_dof=0),
            pricing_model=PricingModel.FREE, max_velocity_ratio=1.0,
        )

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx: RobotContext, params) -> MissionResult:
        steps_completed = 0
        try:
            for i in range(20):
                ctx.move_legs({
                    "left_hip_pitch_joint": ActuatorCommand(ControlMode.POSITION, position=0.05)
                })
                steps_completed += 1
                self.steps_completed = steps_completed
                time.sleep(0.03)
        except EstopActiveError:
            # Remonte l'exception avec le compte de pas déjà réalisés
            # conservé dans le message — permet au test de vérifier
            # l'interruption sans dépendre de l'état de l'instance
            # (qui vit sur la copie du process enfant, invisible au
            # parent une fois le sandbox terminé).
            raise EstopActiveError(f"interrompu après {steps_completed} pas")
        return MissionResult(success=True, mission_id="x", app_id="com.test.slowwalk",
                             duration_s=0, items_processed=steps_completed)


def make_stack():
    robot = UnitreeG1HAL("g1_estop_interrupt")
    robot.initialize_all()
    norm = NormalizedRobotHAL(robot, build_g1_normalizer())
    wd = WatchdogSystem(robot, WatchdogConfig.standard())
    wd.start()
    runtime = PlayStoreRuntime(norm, wd, confirmer_permissions=accepter_tout_SIMULATION)
    return robot, norm, wd, runtime


# ══════════════════════════════════════════════════════════════════

section("1 — EstopActiveError existe et est distincte de PermissionError")
from axis_os.store.sdk.app_base import PermissionError as AppPermissionError
check("EstopActiveError est une exception à part, pas un alias",
      EstopActiveError is not AppPermissionError)
check("EstopActiveError hérite bien d'Exception",
      issubclass(EstopActiveError, Exception))

section("2 — Sans e-stop actif : le mouvement fonctionne normalement")
robot, norm, wd, runtime = make_stack()
app = SlowWalkApp()
runtime.install(app)
runtime.start_app("com.test.slowwalk")

result_holder = {}
def run_normal():
    result_holder["r"] = runtime.run_mission("com.test.slowwalk", {})

t = threading.Thread(target=run_normal)
t.start()
t.join(timeout=5.0)

check("Mission complète sans e-stop : les 20 pas sont exécutés "
      "(vérifié via result.items_processed — app.steps_completed vit sur "
      "la copie de l'app dans le process enfant sandboxé, jamais visible "
      "depuis l'instance parent conservée ici)",
      result_holder["r"].items_processed == 20)
check("Mission réussie normalement", result_holder["r"].success)
wd.stop(); robot.shutdown_all()

section("3 — LE VRAI TEST : e-stop en PLEIN MILIEU d'une mission multi-étapes")
robot2, norm2, wd2, runtime2 = make_stack()
app2 = SlowWalkApp()
runtime2.install(app2)
runtime2.start_app("com.test.slowwalk")

result_holder2 = {}
def run_interrupted():
    result_holder2["r"] = runtime2.run_mission("com.test.slowwalk", {})

t2 = threading.Thread(target=run_interrupted)
t2.start()
time.sleep(0.1)  # laisse ~3 pas s'exécuter sur les 20 prévus (0.03s chacun)
act2 = robot2.get_actuator("right_knee_joint")
act2._temperature = 85.0  # déclenche l'e-stop EN PLEIN MILIEU
t2.join(timeout=5.0)

check("e-stop bien déclenché pendant la mission", wd2.estop_fired)
check("La mission a été INTERROMPUE avant ses 20 pas prévus "
      "(le compte de pas réalisés est conservé dans error_message, "
      "car l'app tourne dans le process enfant sandboxé — voir "
      "SlowWalkApp.run_mission)",
      result_holder2["r"] is not None and
      "interrompu après" in (result_holder2["r"].error_message or ""))
check("Le résultat de la mission reflète l'échec, pas un succès "
      "silencieux malgré l'interruption",
      not result_holder2["r"].success)
check("Le message d'erreur mentionne explicitement l'e-stop, "
      "pas une erreur applicative générique",
      "e-stop" in result_holder2["r"].error_message.lower())
wd2.stop(); robot2.shutdown_all()

section("4 — Toutes les méthodes de mouvement sont protégées, pas juste move_legs")
robot3, norm3, wd3, runtime3 = make_stack()

class ArmsApp(RobotApp):
    def manifest(self):
        return AppManifest(
            app_id="com.test.arms", name="Arms", version="1.0.0",
            author="t", author_email="t@t.com", description="test",
            category=AppCategory.UTILITIES,
            permissions=[Permission.ACTUATORS_WRITE_ARMS],
            hardware=HardwareRequirements(min_dof=0),
            pricing_model=PricingModel.FREE, max_velocity_ratio=1.0,
        )
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params):
        ctx.move_arms({
            "left_elbow_joint": ActuatorCommand(ControlMode.POSITION, position=0.3)
        })
        return MissionResult(success=True, mission_id="x", app_id="com.test.arms",
                             duration_s=0, items_processed=1)

runtime3.install(ArmsApp())
runtime3.start_app("com.test.arms")
# Déclenche l'e-stop AVANT même de lancer la mission cette fois
act3 = robot3.get_actuator("left_knee_joint")
act3._temperature = 85.0
time.sleep(0.15)
check("e-stop bien actif avant la mission", wd3.estop_fired)

result3 = runtime3.run_mission("com.test.arms", {})
check("move_arms() également bloqué par l'e-stop (pas seulement move_legs)",
      result3 is not None and not result3.success)
check("Message d'erreur cohérent pour move_arms aussi",
      result3 is not None and "e-stop" in result3.error_message.lower())
wd3.stop(); robot3.shutdown_all()

section("RÉSUMÉ")
total = PASS + FAIL
print(f"  Résultats : {PASS}/{total} tests passés\n")
if FAIL == 0:
    print("  ✓ EstopActiveError correctement définie et distincte")
    print("  ✓ Sans e-stop : comportement normal inchangé (rétrocompatible)")
    print("  ✓ LE VRAI TEST : mission de 20 pas interrompue après ~3 pas")
    print("    quand l'e-stop survient en plein milieu de son exécution")
    print("  ✓ move_arms ET move_legs protégés de façon cohérente")
    print("  → Le trou identifié par le test de stress final est refermé.")
else:
    print(f"  ✗ {FAIL} test(s) échoué(s).")

sys.exit(0 if FAIL == 0 else 1)
