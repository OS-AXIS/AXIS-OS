"""
Dépose à la limite de portée des bras : refusée proprement, jamais lâchée
de haut.

Carton de la scène T1 (24 × 20 × 20 cm) pris par pick(), bras à 7
articulations (G1 29 DOF : porté en corps rigide), robot simulé léger.
Porté là où les mains l'ont serré, il est bien posé jusqu'à un plateau de
72 cm, à 3 mm près (docs/validation_port_carton.md, §8). Plus bas :
  1. plateau à 70 cm, hors d'atteinte du chemin en corps rigide : place()
     rend FAILED/OUT_OF_REACH ; le carton reste tenu, rien ne bouge (bras,
     carton, base) ;
  2. plateau à 71 cm : le chemin de descente passe, mais en bas les mains
     ne pourraient pas s'ouvrir sur place. Avant, le repli d'_ouvrir
     rouvrait les roulis : elles remontaient de 2,4 cm en s'ouvrant, le
     carton lâché 3,4 cm au-dessus du plateau. Désormais vérifié avant de
     descendre : même refus, tenu, rien ne bouge ;
  3. la vérification d'avant la descente se fait à quelques mm près (les
     mains mesurées, d'où _ouvrir part, ne sont pas tout à fait à leur
     consigne) : si, en bas, les mains ne peuvent pas s'ouvrir sur place
     (simulé ici), l'objet n'est jamais lâché, il est remonté, tenu ;
  4. plateau à 73 cm, juste au-dessus : posé, mains ouvertes sur place, le
     carton à 1 cm au plus au-dessus du plateau (jeu_depose).
"""
import contextlib
import io
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.frames import FRAME_WORLD  # noqa: E402
from axis_os.services.manipulation.service import Etat  # noqa: E402
from axis_os.store.sdk.app_base import RobotContext  # noqa: E402
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402
from axis_os.store.sdk.results import RefusalCode  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
TAILLE = (0.24, 0.20, 0.20)


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def silencieux(f, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return f(*a, **kw)


def contexte(robot):
    """RobotContext en direct, branché sur les services du robot simulé."""
    manifeste = AppManifest(
        app_id="com.test.depose_limite", name="Dépose", version="1.0.0", author="T",
        author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
        permissions=[Permission.MANIPULATION, Permission.ACTUATORS_READ,
                     Permission.ACTUATORS_WRITE_ARMS, Permission.SENSORS_CAMERA])
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog, manifeste)
    ctx._set_clock(robot.clock)
    ctx._attach_services(locomotion=robot.locomotion, navigation=robot.navigation,
                         manipulation=robot.manipulation, perception=robot.perception)
    return ctx


def attendre(robot, duree_s):
    """Laisse tourner le robot simulé `duree_s` secondes de son horloge."""
    t0, limite = robot.clock(), time.monotonic() + 30.0
    while robot.clock() - t0 < duree_s and time.monotonic() < limite:
        time.sleep(0.01)


def deposer(plateau, ouverture_impossible_en_bas=False):
    """Prend le carton puis le pose sur un plateau à `plateau` m : (résultat
    de pick, de place, robot, carton, état avant place)."""
    robot = SimulatedRobot(start=(1.0, 0.0, 0.0), poignets_29dof=True).__enter__()
    ctx = contexte(robot)
    boite = SimulatedObject("red box", (robot.x + 0.30, robot.y, 0.90))
    robot.objects.append(boite)
    vues = silencieux(ctx.look_for, "red", FRAME_WORLD)
    prise = silencieux(ctx.pick, vues[0], size_m=TAILLE, timeout_s=15.0)
    attendre(robot, 0.5)
    m = robot.manipulation
    if ouverture_impossible_en_bas:
        m._q_ouverture = lambda: None      # là où sont les mains, en bas
    avant = {"q": dict(m._q), "carton": tuple(boite.position),
             "base": (robot.x, robot.y, robot.heading)}
    pose = silencieux(ctx.place, plateau, timeout_s=20.0)
    attendre(robot, 1.0)                 # rien ne doit bouger après un refus
    return prise, pose, robot, boite, avant


def refus_propre(plateau, motif, label):
    prise, pose, robot, boite, avant = deposer(plateau)
    try:
        m = robot.manipulation
        check(f"Plateau à {100 * plateau:.0f} cm : carton pris (HELD)", prise.ok
              and prise.status == "HELD", str(prise))
        check(f"… place() : FAILED/OUT_OF_REACH, {label}", pose.status == "FAILED"
              and pose.code == RefusalCode.OUT_OF_REACH.value
              and pose.message.startswith("dépose impossible") and motif in pose.message,
              str(pose))
        check("… carton toujours tenu, l'OS prêt à le porter (TENU)",
              robot.held is boite and m.etat is Etat.TENU, f"{m.etat} {robot.held}")
        bouge = max(abs(v - avant["q"].get(j, v)) for j, v in m._q.items())
        deplace = max(abs(a - b) for a, b in zip(boite.position, avant["carton"]))
        base = max(abs(a - b) for a, b in zip((robot.x, robot.y, robot.heading), avant["base"]))
        check(f"… aucun mouvement : consignes des bras à {bouge:.1e} rad, carton à "
              f"{1000 * deplace:.1f} mm, base à {1000 * base:.1f} mm de leur état d'avant",
              bouge <= 1e-3 and deplace <= 0.001 and base <= 0.001,
              f"{bouge} {deplace} {base}")
        check("… jamais lâché, aucun geste lancé : ni descente, ni ouverture",
              not robot.chutes and m.stats.get("deposes_plateau", 0) == 0, str(robot.chutes))
    finally:
        robot.__exit__(None, None, None)


section("1. Plateau à 70 cm, hors d'atteinte : refusé proprement")
refus_propre(0.70, "hors d'atteinte des bras", "chemin hors d'atteinte des bras")

section("2. Plateau à 71 cm : chemin possible, mains qui ne s'ouvriraient pas sur place")
refus_propre(0.71, "ne pourraient pas s'y ouvrir sur place",
             "les mains ne pourraient pas s'y ouvrir sur place (vérifié avant de descendre)")

section("3. Ouverture possible avant de descendre, impossible vue en bas : jamais lâché")
prise, pose, robot, boite, avant = deposer(0.75, ouverture_impossible_en_bas=True)
try:
    m = robot.manipulation
    check("Carton pris (HELD), descendu vers un plateau à 75 cm", prise.ok
          and m.stats.get("deposes_impossibles", 0) == 1, str(prise))
    check("… place() : FAILED/OUT_OF_REACH, « les mains ne s'y ouvriraient pas sur place (vu en "
          "bas) »", pose.status == "FAILED" and pose.code == RefusalCode.OUT_OF_REACH.value
          and "vu en bas" in pose.message, str(pose))
    check("… jamais lâché : ni ouverture, ni chute ; remonté, tenu (TENU)",
          robot.held is boite and m.etat is Etat.TENU and not robot.chutes
          and m.stats.get("deposes_plateau", 0) == 0, f"{m.etat} {robot.chutes}")
    check(f"… le carton remonté à {100 * (boite.position[2] - avant['carton'][2]):+.1f} cm de sa "
          f"hauteur de port (à 3 cm près)", abs(boite.position[2] - avant["carton"][2]) <= 0.03,
          f"{boite.position} {avant['carton']}")
finally:
    robot.__exit__(None, None, None)

section("4. Plateau à 73 cm, juste au-dessus de la limite : posé, mains ouvertes sur place")
prise, pose, robot, boite, _ = deposer(0.73)
try:
    fond = boite.position[2] - TAILLE[2] / 2.0
    check("Carton pris, puis posé (PLACED), mains vides", prise.ok and pose.ok
          and pose.status == "PLACED" and robot.held is None
          and robot.manipulation.etat is Etat.LIBRE, f"{prise} {pose}")
    check(f"… lâché à {100 * (fond - 0.73):.1f} cm au-dessus du plateau (au plus 1, jeu_depose) : "
          f"les mains s'ouvrent sur place, sans remonter", -0.005 <= fond - 0.73 <= 0.01,
          str(boite.position))
    check("… aucune chute", not robot.chutes, str(robot.chutes))
finally:
    robot.__exit__(None, None, None)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
