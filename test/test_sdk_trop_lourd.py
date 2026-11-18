"""
SDK 1.3.0 : un code de refus propre pour un objet trop lourd, TOO_HEAVY,
distinct de OUT_OF_REACH (qu'une app traiterait en se rapprochant).

  1. Les raisons du service de manipulation et leur code : « trop lourd »
     (pesé au-delà de masse_max_kg, ou serrage requis hors des plafonds des
     poignets) donne TOO_HEAVY ; « trop lourd ou trop glissant » (l'objet
     n'a pas suivi le levage : rien ne dit lequel) reste FAILED ; un objet
     hors de portée reste OUT_OF_REACH.
  2. pick() d'une boîte de 1,6 kg, sur le robot simulé léger : FAILED avec
     TOO_HEAVY, la boîte reposée sur la table, à sa place, mains vides.
  3. La version de l'API du SDK a monté d'un cran mineur (nouveau code).
"""
import contextlib
import io
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.frames import FRAME_WORLD  # noqa: E402
from axis_os.store.sdk.app_base import RobotContext  # noqa: E402
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402
from axis_os.store.sdk.results import RefusalCode  # noqa: E402
from axis_os.store.sdk.version import API_VERSION  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402
from axis_os import SemVer  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


section("1. Raison du service de manipulation -> code")
code = RobotContext._code_refus_manipulation
cas = [
    ("prise refusée — trop lourd : 1.61 kg pesés, au plus 1.2 kg (masse_max_kg, la charge pour "
     "laquelle la prise est vérifiée) ; ni serré plus, ni porté : carton reposé sur la table",
     RefusalCode.TOO_HEAVY),
    ("prise refusée — trop lourd pour ces mains (serrage insuffisant dans leurs plafonds) : il "
     "faudrait 12.5 N par main pour 10.0 N de poids", RefusalCode.TOO_HEAVY),
    ("prise NON confirmée — l'objet n'a pas suivi (2 essais) : il glisse entre les mains serrées à "
     "6.5 N, trop lourd ou trop glissant pour ce serrage", RefusalCode.FAILED),
    ("prise refusée — hors de portée des bras : objet à 0.46 m devant le bassin",
     RefusalCode.OUT_OF_REACH),
    ("dépose impossible : plateau à 0.70 m, hors d'atteinte des bras", RefusalCode.OUT_OF_REACH),
]
for raison, attendu in cas:
    check(f"« {raison[:60]}… » : {attendu.value}", code(raison) is attendu, code(raison).value)

section("2. pick() d'une boîte de 1,6 kg : FAILED/TOO_HEAVY, reposée à sa place")
manifeste = AppManifest(app_id="com.test.trop_lourd", name="Trop lourd", version="1.0.0",
                        author="T", author_email="t@t.com", description="d",
                        category=AppCategory.UTILITIES,
                        permissions=[Permission.MANIPULATION, Permission.ACTUATORS_READ,
                                     Permission.ACTUATORS_WRITE_ARMS, Permission.SENSORS_CAMERA])
with SimulatedRobot(start=(1.0, 0.0, 0.0), poignets_29dof=True) as robot:
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog, manifeste)
    ctx._set_clock(robot.clock)
    ctx._attach_services(locomotion=robot.locomotion, navigation=robot.navigation,
                         manipulation=robot.manipulation, perception=robot.perception)
    boite = SimulatedObject("red box", (robot.x + 0.30, robot.y, 0.90), masse_kg=1.6)
    robot.objects.append(boite)
    with contextlib.redirect_stdout(io.StringIO()):
        vues = ctx.look_for("red", FRAME_WORLD)
        r = ctx.pick(vues[0], size_m=(0.24, 0.20, 0.20), timeout_s=30.0)
    check("pick : FAILED avec TOO_HEAVY, et la raison dit « trop lourd »",
          r.status == "FAILED" and r.code == RefusalCode.TOO_HEAVY.value
          and "trop lourd" in r.message, str(r))
    t0 = robot.clock()                 # les bras se rouvrent progressivement
    while robot.held is not None and robot.clock() - t0 < 5.0:
        time.sleep(0.01)
    check("… boîte reposée sur la table, à sa place, mains vides",
          robot.held is None and abs(boite.position[2] - 0.90) < 0.03
          and abs(boite.position[0] - (robot.x + 0.30)) < 0.03, str(boite.position))

section("3. Version de l'API du SDK")
check(f"API_VERSION {API_VERSION} : au moins 1.3.0, le code TOO_HEAVY en fait partie",
      SemVer.parse(API_VERSION) >= SemVer.parse("1.3.0") and RefusalCode("TOO_HEAVY"))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
