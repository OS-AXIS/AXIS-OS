"""
Navigation — contourner ce qui reste planté sur le chemin, jamais foncer.

  - une personne IMMOBILE en zone ouverte : contournée, sans jamais
    déclencher l'arrêt du gardien ;
  - une personne qui TRAVERSE : aucun contournement (elle bouge) ;
  - une personne plantée dans un couloir ÉTROIT : pas d'autre passage, le
    robot attend à distance, puis repart quand elle s'en va ;
  - l'obstacle temporaire EXPIRE : la carte revient à la normale.

Robot simulé à la GR00T WBC (bande morte, erreur), vrai gardien de
proximité sur un lidar synthétique, consigne plafonnée comme le fait le
LocomotionService.
"""
import math
import random
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.locomotion.proximite import GardienProximite
from axis_os.services.navigation.service import (
    Carte, EtatNavigation, NavigationService, Pose2D, Rectangle,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


DT = 0.02


class Robot:
    def __init__(self, x, y, cap, graine=0):
        self.x, self.y, self.cap, self.t = x, y, cap, 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.rng = random.Random(graine)

    def pas(self):
        vx, vy, wz = self.cmd
        if math.sqrt(vx * vx + wz * wz) < 0.05:
            return
        f = 1.0 + self.rng.uniform(-0.08, 0.08)
        self.cap += wz * f * DT
        self.x += vx * f * math.cos(self.cap) * DT
        self.y += vx * f * math.sin(self.cap) * DT

    def pose(self):
        return Pose2D(self.x, self.y, self.cap, horodatage=self.t)


class LocoGardee:
    """Ce que fait le LocomotionService : chaque consigne passe par le gardien."""

    def __init__(self, robot, gardien):
        self.r, self.g = robot, gardien
        self.voulu = (0.0, 0.0, 0.0)
        self.dernier_verdict = None

    def set_velocity(self, vx=0.0, vy=0.0, wz=0.0):
        self.voulu = (vx, vy, wz)

    def halt(self):
        self.voulu = (0.0, 0.0, 0.0)

    def tick(self):
        v = self.g.evaluer(self.r.t)
        self.dernier_verdict = v
        f = max(0.0, v.facteur)
        self.r.cmd = tuple(c * f for c in self.voulu)


def personne(x, y):
    a = np.linspace(0, 2 * math.pi, 16, endpoint=False)
    z = np.linspace(0.1, 1.7, 6)
    A, Z = np.meshgrid(a, z)
    return np.c_[x + 0.2 * np.cos(A.ravel()), y + 0.2 * np.sin(A.ravel()), Z.ravel()]


def murs(obstacles):
    pts = []
    for o in obstacles:
        xs = np.arange(o.xmin, o.xmax + 1e-9, 0.1)
        ys = np.arange(o.ymin, o.ymax + 1e-9, 0.1)
        X, Y = np.meshgrid(xs, ys)
        for z in (0.3, 1.0):
            pts.append(np.c_[X.ravel(), Y.ravel(), np.full(X.size, z)])
    return np.vstack(pts) if pts else np.zeros((0, 3))


def scenario(carte, depart, but, position_personne, t_max=120.0):
    """position_personne(t) -> (x, y) ou None."""
    R = Robot(*depart)
    fixes = murs(carte.obstacles)

    def scan():
        p = position_personne(R.t)
        pts = fixes if p is None else np.vstack([fixes, personne(*p)])
        return pts, R.t

    g = GardienProximite(scan, lambda: (R.x, R.y, R.cap), obstacles=carte.obstacles,
                         horloge=lambda: R.t)
    loco = LocoGardee(R, g)
    nav = NavigationService(carte, R.pose, loco, horloge=lambda: R.t)
    nav.start()
    nav.aller_vers(but)
    d_min, bouge_pres = float("inf"), 0
    while nav.etat is EtatNavigation.EN_ROUTE and R.t < t_max:
        R.t += DT
        nav.tick()
        loco.tick()
        x0, y0 = R.x, R.y
        R.pas()
        p = position_personne(R.t)
        if p is not None:
            d = math.hypot(p[0] - R.x, p[1] - R.y) - 0.2      # à sa surface
            d_min = min(d_min, d)
            if d < 0.8 and math.hypot(R.x - x0, R.y - y0) > 1e-6:
                bouge_pres += 1
    return nav, R, g, d_min, bouge_pres


ouvert = Carte(0.0, 0.0, 10.0, 6.0, postes={"est": Pose2D(9.0, 3.0, 0.0)})

section("1 — Personne IMMOBILE en zone ouverte : contournée")
nav, R, g, d_min, bouge = scenario(ouvert, (1.0, 3.0, 0.0), "est", lambda t: (5.0, 3.0))
print(f"    {nav.stats}, arrêts gardien {g.stats['arrets']}, plus près {d_min:.2f} m, "
      f"arrivé à t={R.t:.0f} s")
check("Contournement décidé", nav.stats["contournements"] >= 1)
check("Arrivé à destination", nav.etat is EtatNavigation.ARRIVE)
check(f"Jamais à moins de 0.8 m de la personne ({d_min:.2f} m)", d_min >= 0.8)
check("Jamais un mouvement du robot à moins de 0.8 m", bouge == 0)

section("2 — Personne qui TRAVERSE : pas de contournement")
traverse = lambda t: (5.0, 0.3 + 0.8 * t) if 0.3 + 0.8 * t < 5.7 else None
nav, R, g, d_min, bouge = scenario(ouvert, (1.0, 3.0, 0.0), "est", traverse)
print(f"    {nav.stats}, plus près {d_min:.2f} m")
check("Aucun contournement : elle bouge", nav.stats["contournements"] == 0)
check("Arrivé", nav.etat is EtatNavigation.ARRIVE)
check("Jamais un mouvement du robot à moins de 0.8 m", bouge == 0)

section("3 — Couloir étroit, personne plantée : attente, puis reprise")
etroit = Carte(0.0, 0.0, 10.0, 6.0, obstacles=[
    Rectangle(3.0, 0.0, 7.0, 2.4), Rectangle(3.0, 3.6, 7.0, 6.0)],
    postes={"est": Pose2D(9.0, 3.0, 0.0)})
part = lambda t: (5.0, 3.0) if t < 50.0 else None
nav, R, g, d_min, bouge = scenario(etroit, (1.0, 3.0, 0.0), "est", part)
print(f"    {nav.stats}, plus près {d_min:.2f} m, arrivé à t={R.t:.0f} s")
check("Aucun autre passage : pas de contournement inventé",
      nav.stats["contournements"] == 0 and nav.stats["contournements_impossibles"] >= 1)
check("Le robot a attendu, puis est arrivé une fois la personne partie",
      nav.etat is EtatNavigation.ARRIVE and R.t > 50.0)
check("Jamais un mouvement du robot à moins de 0.8 m", bouge == 0)

section("4 — L'obstacle temporaire expire")
nav, R, g, d_min, bouge = scenario(ouvert, (1.0, 3.0, 0.0), "est", lambda t: (5.0, 3.0))
check("Après contournement, un obstacle temporaire est posé",
      len(nav.planif.temporaires) == 1)
R.t += 61.0
nav.aller_vers(Pose2D(1.0, 3.0, math.pi))
check("60 s plus tard, il a disparu : la carte revient à la normale",
      len(nav.planif.temporaires) == 0 and nav.planif.libre(5.0, 3.0))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
