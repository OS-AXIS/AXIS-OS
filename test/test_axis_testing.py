"""
axis_os.testing : le robot simulé léger et AppTestHarness.

Vérifié ici :
  1. la base avance de la vitesse demandée, à travers le vrai service de
     locomotion, et s'arrête quand la consigne n'est plus rafraîchie ;
  2. le gardien de proximité (le vrai) arrête le robot devant une personne
     ajoutée au monde ;
  3. sous arrêt d'urgence, la base ne bouge plus ; une chute est détectée ;
  4. la batterie : inconnue par défaut, mesurée quand le test la fixe ;
  5. la perception simulée voit les objets du monde, dans le repère
     demandé ;
  6. AppTestHarness installe une app, la démarre et lance sa mission dans
     son processus séparé, comme le Play Store sur un robot.
"""
import contextlib
import io
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.frames import FRAME_WORLD  # noqa: E402
from axis_os.services.locomotion.service import LocomotionState  # noqa: E402
from axis_os.store.sdk.app_base import MissionResult, RobotApp  # noqa: E402
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402
from axis_os.testing import AppTestHarness, SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def marcher(robot, vx, duree_s):
    """Rafraîchit une consigne de marche pendant `duree_s`, comme une app."""
    fin = time.monotonic() + duree_s
    while time.monotonic() < fin:
        robot.locomotion.set_velocity(vx, 0.0, 0.0)
        time.sleep(0.05)


section("1. La base suit la vitesse demandée")
with SimulatedRobot() as robot:
    marcher(robot, 0.3, 1.0)
    x = robot.pose[0]
    check("0.3 m/s pendant 1 s : environ 0.3 m parcourus", 0.2 < x < 0.4, f"x = {x:.3f}")
    time.sleep(1.0)
    check("Consigne non rafraîchie : le robot s'arrête (délai de la locomotion)",
          abs(robot.pose[0] - x) < 0.2, f"{robot.pose[0]:.3f} contre {x:.3f}")
    check("Aucune erreur dans la boucle des services", robot.errors == [], str(robot.errors))

section("2. Le gardien de proximité voit les personnes")
with SimulatedRobot() as robot:
    robot.add_person(0.6, 0.0)
    marcher(robot, 0.3, 0.6)
    v = robot.locomotion.dernier_verdict
    check("Personne à 0.6 m : le gardien arrête le robot",
          v is not None and v.facteur == 0.0 and abs(robot.pose[0]) < 0.05, f"{v} {robot.pose}")
    robot.clear_people()
    time.sleep(1.5)
    marcher(robot, 0.3, 0.5)
    check("Zone dégagée : le robot repart", robot.pose[0] > 0.05, str(robot.pose))

section("3. Arrêt d'urgence et chute")
with SimulatedRobot() as robot:
    with contextlib.redirect_stdout(io.StringIO()):
        robot.hal.emergency_stop_all()
    marcher(robot, 0.3, 0.5)
    check("Sous arrêt d'urgence, la base ne bouge plus", robot.pose[0] == 0.0, str(robot.pose))
with SimulatedRobot() as robot:
    robot.fall()
    time.sleep(0.2)
    check("Chute détectée par la locomotion", robot.locomotion.state is LocomotionState.FALLEN)

section("4. Batterie")
robot = SimulatedRobot()
check("Inconnue par défaut", robot.hal.get_battery() is None)
robot.set_battery(18.0)
check("Fixée par le test : mesurée", robot.hal.get_battery().percent == 18.0)

section("5. Perception simulée")
with SimulatedRobot(start=(1.0, 0.0, math.pi / 2),
                    objects=[SimulatedObject("red box", (1.0, 2.0, 0.9)),
                             SimulatedObject("blue box", (3.0, 0.0, 0.9))]) as robot:
    r = robot.perception.find("red", frame=FRAME_WORLD)
    check("« red » : la boîte rouge seule, position monde exacte",
          r.ok and len(r.value) == 1 and r.value[0].label == "red box"
          and abs(r.value[0].position_3d.y - 2.0) < 1e-9, str(r))
    r = robot.perception.find("red")
    p = r.value[0].position_3d
    check("Repère du robot (tourné de 90°) : la boîte est 2 m devant",
          abs(p.x - 2.0) < 1e-6 and abs(p.y) < 1e-6, f"{p}")


class AppQuiMarche(RobotApp):
    """Marche droit devant pendant une seconde, puis s'arrête."""
    def manifest(self):
        return AppManifest(app_id="com.test.qui_marche", name="Marche", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.LOCOMOTION_WALK,
                                        Permission.ACTUATORS_WRITE_LEGS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        fin = ctx.now() + 1.0
        while ctx.now() < fin:
            ctx.walk(0.3)
            ctx.sleep(0.05)
        ctx.stop_walking()
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


section("6. AppTestHarness")
with AppTestHarness(AppQuiMarche()) as h:
    res = h.run()
    check("La mission tourne dans son processus et réussit", res is not None and res.success,
          getattr(res, "error_message", None))
    check("… et l'app a fait avancer la base simulée", h.robot.pose[0] > 0.1, str(h.robot.pose))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
