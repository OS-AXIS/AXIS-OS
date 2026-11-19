"""
pick() et le planificateur de prise de l'OS : quand la carte de navigation
connaît la table sous l'objet, pick() planifie (paire de faces, position,
contrôles faits sans bouger) avant de toucher à quoi que ce soit.

  1. Robot à la pose que le planificateur retient : HELD, l'objet pris d'ici,
     le bord de la table connu de la prise.
  2. Robot trop loin : REFUSED/OUT_OF_REACH, la raison d'ici et la pose d'où
     la prise est possible ; rien n'a bougé (bras au repos, pas d'exemption).
  3. Aucune prise possible (boîte de 46 cm) : REFUSED, « même la meilleure
     prise : … », rien n'a bougé.
  4. Sans la table dans la carte : pick() comme avant (pas de planificateur).
  5. Table déclarée à sa hauteur de chargement (zmax = plateau + 25 cm, la
     convention du gardien de proximité de l'app de tri) : planifiée aussi,
     le plateau pris sous l'objet.
"""
import contextlib
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.frames import FRAME_WORLD  # noqa: E402
from axis_os.services.manipulation import planificateur as P  # noqa: E402
from axis_os.services.manipulation.service import Etat  # noqa: E402
from axis_os.services.navigation.service import Carte, Rectangle  # noqa: E402
from axis_os.store.sdk.app_base import RobotContext  # noqa: E402
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402
from axis_os.store.sdk.results import RefusalCode  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
TABLE = (1.30, 2.00, -0.50, 0.50, 0.80)           # xmin, xmax, ymin, ymax, plateau
BOITE = (0.24, 0.20, 0.20)                         # largeur, profondeur, hauteur
CENTRE = (TABLE[0] + BOITE[1] / 2.0, 0.0, TABLE[4] + BOITE[2] / 2.0)


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


MANIFESTE = AppManifest(app_id="com.test.pick_planifie", name="Pick planifié", version="1.0.0",
                        author="T", author_email="t@t.com", description="d",
                        category=AppCategory.UTILITIES,
                        permissions=[Permission.MANIPULATION, Permission.ACTUATORS_READ,
                                     Permission.ACTUATORS_WRITE_ARMS, Permission.SENSORS_CAMERA])


def essai(depart, largeur=BOITE[0], table_dans_la_carte=True, zmax=TABLE[4]):
    """pick() de la boîte rouge posée au bord de la table : (résultat, objet
    que la prise a reçu, rien n'a bougé)."""
    obstacles = [Rectangle(TABLE[0], TABLE[2], TABLE[1], TABLE[3], zmax=zmax)] \
        if table_dans_la_carte else []
    carte = Carte(-5.0, -5.0, 5.0, 5.0, obstacles=obstacles)
    robot = SimulatedRobot(start=depart, poignets_29dof=True, contact_main_m=0.11, carte=carte,
                           plateaux=[TABLE])
    with robot:
        ctx = RobotContext.for_testing(robot.hal, robot.watchdog, MANIFESTE)
        ctx._set_clock(robot.clock)
        ctx._attach_services(locomotion=robot.locomotion, navigation=robot.navigation,
                             manipulation=robot.manipulation, perception=robot.perception)
        robot.objects.append(SimulatedObject("red box", CENTRE, size=(largeur, BOITE[1],
                                                                      BOITE[2]), masse_kg=0.8))
        recu = []
        prendre = robot.manipulation.prendre
        robot.manipulation.prendre = lambda o: (recu.append(o), prendre(o))[1]
        avant = dict(robot.manipulation._q)
        with contextlib.redirect_stdout(io.StringIO()):
            vues = ctx.look_for("red", FRAME_WORLD)
            r = ctx.pick(vues[0], size_m=(largeur, BOITE[1], BOITE[2]), timeout_s=30.0)
        immobile = (robot.manipulation._q == avant and robot.guard._exemption is None
                    and robot.manipulation.etat is Etat.LIBRE)
        return r, recu, immobile


# La pose que le planificateur retient pour cette boîte (sans robot : géométrie).
section("1. Robot à la pose retenue : pris d'ici")
with contextlib.redirect_stdout(io.StringIO()):
    banc = SimulatedRobot(start=(0.5, 0.0, 0.0), poignets_29dof=True, contact_main_m=0.11)
plan = banc.manipulation.planifier_prise(
    P.coins_boite(CENTRE, 0.0, BOITE[1], BOITE[0], BOITE[2]),
    P.BordTable((TABLE[0], 0.0), 0.0, TABLE[4]))
ch = plan.choisie
check(f"Retenue : {ch and ch.explication()[:110]}…", ch is not None, plan.raison)
r, recu, _ = essai((ch.x, ch.y, ch.cap))
check(f"pick : {r.status} ({r.message[:80]})", r.ok and r.status == "HELD", str(r))
check("La prise a reçu l'objet avec le bord de la table (prise planifiée)",
      recu and recu[0].lacet_bord is not None and abs(recu[0].bord_m) < 1e-6, str(recu))

section("2. Robot trop loin : la pose d'où prendre, rien ne bouge")
r, recu, immobile = essai((ch.x - 0.40, ch.y, 0.0))
check(f"pick : {r.status}/{r.code} ({r.message[:120]}…)", r.status == "REFUSED"
      and r.code == RefusalCode.OUT_OF_REACH.value and "prise possible depuis" in r.message
      and f"({ch.x:.2f}, {ch.y:.2f})" in r.message, str(r))
check("Rien n'a bougé : prise jamais lancée, bras au repos, pas d'exemption",
      not recu and immobile)

section("3. Aucune prise possible : la raison de la meilleure")
r, recu, immobile = essai((ch.x, ch.y, ch.cap), largeur=0.46)
check(f"pick : {r.status}/{r.code} ({r.message[:120]}…)", r.status == "REFUSED"
      and "même la meilleure prise" in r.message, str(r))
check("Rien n'a bougé", not recu and immobile)

section("4. Sans la table dans la carte : comme avant")
r, recu, _ = essai((ch.x, ch.y, ch.cap), table_dans_la_carte=False)
check(f"pick : {r.status}, l'objet reçu sans bord de table (prise d'ici, comme avant)",
      r.ok and recu and recu[0].lacet_bord is None, str(r))

section("5. Table déclarée à sa hauteur de chargement (plateau + 25 cm)")
r, recu, _ = essai((ch.x, ch.y, ch.cap), zmax=TABLE[4] + 0.25)
check(f"pick : {r.status}, prise planifiée, plateau pris sous l'objet",
      r.ok and recu and recu[0].lacet_bord is not None and abs(recu[0].bord_m) < 1e-6, str(r))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
