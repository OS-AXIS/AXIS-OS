"""
Lidar simulé (rayons PhysX) — testé sans GPU avec une géométrie analytique :
un sol, une personne (cylindre), le corps du robot (sphère).
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.isaac_sim.sim_lidar import (
    ELEVATIONS_MID360_INVERSE, IsaacSimLidar, directions_balayage,
)
from axis_os.services.locomotion.proximite import GardienProximite

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Scene:
    """Sol z=0, personne = cylindre vertical, robot = sphère autour du capteur."""

    def __init__(self, personne=(1.5, 0.0), rayon_p=0.2, h_p=1.7, corps=True):
        self.p, self.rp, self.hp = personne, rayon_p, h_p
        self.corps = corps

    def lancer(self, o, d, portee):
        o, d = np.array(o), np.array(d)
        touches = []
        if d[2] < -1e-9:                          # sol
            t = -o[2] / d[2]
            touches.append((t, "/World/sol"))
        if self.p is not None:                    # cylindre
            ox, oy = o[0] - self.p[0], o[1] - self.p[1]
            a = d[0] ** 2 + d[1] ** 2
            b = 2 * (ox * d[0] + oy * d[1])
            c = ox * ox + oy * oy - self.rp ** 2
            disc = b * b - 4 * a * c
            if a > 1e-12 and disc >= 0:
                t = (-b - math.sqrt(disc)) / (2 * a)
                if t > 0 and 0 <= o[2] + t * d[2] <= self.hp:
                    touches.append((t, "/World/personne"))
        if self.corps:                            # sphère du robot, r=0.3, centre 0.4 m sous le capteur
            cc = np.array([0.0, 0.0, 0.9])
            oc = o - cc
            b = np.dot(oc, d)
            c = np.dot(oc, oc) - 0.09
            disc = b * b - c
            if disc >= 0:
                for t in (-b - math.sqrt(disc), -b + math.sqrt(disc)):
                    if t > 1e-6:
                        touches.append((t, "/World/g1/torso_link"))
                        break
        touches = [x for x in touches if 0 < x[0] <= portee]
        if not touches:
            return None
        t, chemin = min(touches)
        return tuple(o + t * d), chemin


T = [0.0]
POSE = (np.array([0.0, 0.0, 1.3]), np.eye(3))


def lidar(scene, **kw):
    li = IsaacSimLidar("tete", lambda: POSE, lambda: T[0], lanceur=scene.lancer, **kw)
    li.initialize()
    return li


section("1 — Géométrie du balayage")
d = directions_balayage(180, ELEVATIONS_MID360_INVERSE)
check(f"{len(d)} rayons (180 × {len(ELEVATIONS_MID360_INVERSE)} couches)",
      len(d) == 180 * len(ELEVATIONS_MID360_INVERSE))
check("Tous unitaires", np.allclose(np.linalg.norm(d, axis=1), 1.0))
check("Champ vertical du Mid-360 inversé : -50° à +7°",
      math.isclose(math.degrees(math.asin(d[:, 2].min())), -50, abs_tol=0.1)
      and math.isclose(math.degrees(math.asin(d[:, 2].max())), 7, abs_tol=0.1))

section("2 — Il voit une personne, y compris DERRIÈRE le robot")
for nom, pos in (("devant", (1.5, 0.0)), ("derrière", (-1.2, 0.3)), ("à gauche", (0.1, 1.0))):
    s = lidar(Scene(personne=pos)).scan()
    p = s.points
    sur = p[(np.hypot(p[:, 0] - pos[0], p[:, 1] - pos[1]) < 0.25) & (p[:, 2] > 0.05)]
    check(f"Personne {nom} : {len(sur)} points sur elle", len(sur) >= 5)
check("Points en repère MONDE, horodatés à l'horloge du robot",
      s.frame_id == "world" and s.timestamp_ns == 0)

section("3 — Il ne se voit pas lui-même")
li = lidar(Scene(personne=(1.0, 0.0)))
s = li.scan()
p = s.points
check(f"Rayons qui ont touché le corps du robot : {li.stats['impacts_sur_soi']}, relancés",
      li.stats["impacts_sur_soi"] > 0)
check("Aucun point rendu sur le corps du robot",
      not np.any(np.linalg.norm(p - np.array([0, 0, 0.9]), axis=1) < 0.29))
sur = p[(np.hypot(p[:, 0] - 1.0, p[:, 1]) < 0.25) & (p[:, 2] > 0.05)]
check("… et la personne à 1 m reste vue malgré le corps dans le champ bas",
      len(sur) >= 5)

section("4 — 10 Hz : le balayage n'est pas refait à chaque appel")
li = lidar(Scene())
a = li.scan()
T[0] = 0.05
b = li.scan()
T[0] = 0.11
c = li.scan()
check("À 50 ms : même balayage ; à 110 ms : nouveau", a is b and c is not b
      and li.stats["balayages"] == 2)
T[0] = 0.0

section("5 — Branché sur le gardien de proximité")
for pos, attendu in (((0.7, 0.0), "arrêt"), ((1.5, 0.0), "ralenti"), ((3.5, 0.0), "libre"),
                     ((-0.6, 0.2), "arrêt")):
    li = lidar(Scene(personne=pos))
    g = GardienProximite(li.source_gardien(), lambda: (0.0, 0.0, 0.0), horloge=lambda: T[0])
    v = g.evaluer(T[0])
    ok = {"arrêt": v.facteur == 0.0, "ralenti": 0.0 < v.facteur < 1.0,
          "libre": v.facteur == 1.0}[attendu]
    check(f"Personne en {pos} : {attendu} (facteur {v.facteur:.2f}, "
          f"distance {v.distance if v.distance is None else round(v.distance, 2)})", ok)
li = lidar(Scene(personne=None))
g = GardienProximite(li.source_gardien(), lambda: (0.0, 0.0, 0.0), horloge=lambda: T[0])
check("Sol seul : le sol n'arrête pas le robot", g.evaluer(0.0).facteur == 1.0)
li.shutdown()
check("Lidar arrêté : scan() rend None -> le gardien ARRÊTE le robot",
      g.evaluer(0.0).facteur == 0.0)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
