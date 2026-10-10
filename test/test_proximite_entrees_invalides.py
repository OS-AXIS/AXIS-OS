"""
Gardien de proximité — une entrée INVALIDE arrête le robot.

Trouvé par l'audit du 2026-10-10 : avec NaN, toute comparaison est fausse,
donc un point NaN n'était ni « proche » ni « expliqué » — il disparaissait.
Reproduit avant correction :
  - pose du robot NaN, obstacle à 0.58 m  -> « zone dégagée » ;
  - balayage entièrement NaN             -> « zone dégagée » ;
  - horodatage NaN ou dans le futur      -> balayage jamais périmé.
Chaque cas doit désormais rendre un ARRÊT, comme un capteur muet.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.locomotion.proximite import ChampsProtection, GardienProximite

FAIL = 0
NAN = float("nan")


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


# Un obstacle à 0.58 m, hors du volume de port, plus le sol.
OBSTACLE = np.array([[0.5, 0.3, 0.3]])
SOL = np.c_[np.linspace(-3, 3, 50), np.linspace(-3, 3, 50), np.zeros(50)]
LOIN = np.array([[3.0, 3.0, 1.0]])                 # rien de proche
POSE = (0.0, 0.0, 0.0)


def verdict(scan, pose=POSE, maintenant=0.0):
    g = GardienProximite(lambda: scan, lambda: pose)
    return g, g.evaluer(maintenant)


section("1 — Référence : l'obstacle à 0.58 m arrête le robot")

g, v = verdict((np.vstack([OBSTACLE, SOL]), 0.0))
check(f"Entrées valides : ARRÊT ({v.raison})", v.facteur == 0.0)

section("2 — Pose du robot non finie : ARRÊT (avant : « zone dégagée »)")

for nom, pose in {"x NaN": (NAN, 0.0, 0.0), "y NaN": (0.0, NAN, 0.0),
                  "cap NaN": (0.0, 0.0, NAN), "x infini": (math.inf, 0.0, 0.0)}.items():
    g, v = verdict((np.vstack([OBSTACLE, SOL]), 0.0), pose=pose)
    check(f"Pose {nom} : ARRÊT ({v.raison})", v.facteur == 0.0 and g.arrete)

# Même sans obstacle : sans savoir OÙ est le robot, on ne sait rien.
g, v = verdict((LOIN, 0.0), pose=(NAN, 0.0, 0.0))
check("Pose NaN, rien de proche : ARRÊT quand même", v.facteur == 0.0)
check("Compté comme entrée invalide", g.stats["entrees_invalides"] == 1)


def pose_en_panne():
    raise RuntimeError("localisation perdue")


g = GardienProximite(lambda: (LOIN, 0.0), pose_en_panne)
v = g.evaluer(0.0)
check(f"Source de pose en ERREUR : ARRÊT, sans exception ({v.raison})",
      v.facteur == 0.0)

section("3 — Balayage sans aucun point fini : ARRÊT")

g, v = verdict((np.full((50, 3), NAN), 0.0))
check(f"Balayage entièrement NaN : ARRÊT ({v.raison})", v.facteur == 0.0)
g, v = verdict((np.full((50, 3), math.inf), 0.0))
check("Balayage entièrement infini : ARRÊT", v.facteur == 0.0)
g, v = verdict((np.vstack([np.full((10, 3), NAN), OBSTACLE]), 0.0))
check("Quelques points NaN parmi de vrais points : les vrais comptent "
      "(obstacle à 0.58 m -> ARRÊT)", v.facteur == 0.0 and v.distance is not None
      and abs(v.distance - math.hypot(0.5, 0.3)) < 1e-9)
g, v = verdict((np.vstack([np.full((10, 3), NAN), LOIN]), 0.0))
check("Quelques points NaN, le reste loin : pleine vitesse (pas d'excès "
      "de zèle)", v.facteur == 1.0)

section("4 — Balayage mal formé : ARRÊT, sans exception")

g, v = verdict((np.zeros((10, 2)), 0.0))
check(f"Points en 2D : ARRÊT ({v.raison})", v.facteur == 0.0)
g, v = verdict(("pas un tableau", 0.0))
check("Points illisibles : ARRÊT", v.facteur == 0.0)
g, v = verdict((LOIN,))
check("Balayage sans horodatage : ARRÊT", v.facteur == 0.0)


def lidar_en_panne():
    raise OSError("câble débranché")


g = GardienProximite(lidar_en_panne, lambda: POSE)
v = g.evaluer(0.0)
check(f"Source de balayage en ERREUR : ARRÊT, sans exception ({v.raison})",
      v.facteur == 0.0)

section("5 — Horodatage : NaN ou dans le futur -> ARRÊT")

g, v = verdict((LOIN, NAN))
check(f"Horodatage NaN : ARRÊT (avant : jamais périmé) ({v.raison})",
      v.facteur == 0.0)
g, v = verdict((LOIN, 100.0), maintenant=0.0)
check(f"Horodatage 100 s dans le futur : ARRÊT ({v.raison})", v.facteur == 0.0)
c = ChampsProtection()
g, v = verdict((LOIN, c.avance_max_s / 2), maintenant=0.0)
check(f"Léger décalage d'horloge (< {c.avance_max_s} s) : toléré",
      v.facteur == 1.0)
g, v = verdict((LOIN, 0.0), maintenant=NAN)
check("Horloge du robot NaN : ARRÊT", v.facteur == 0.0)

section("6 — Après une panne de capteur, le délai de reprise repart de zéro")

etat = {"scan": (np.vstack([OBSTACLE, SOL]), 0.0)}
g = GardienProximite(lambda: etat["scan"], lambda: POSE)
check("Obstacle : arrêt", g.evaluer(0.0).facteur == 0.0)
etat["scan"] = (LOIN, 0.5)
check("Zone dégagée : reprise pas encore autorisée", g.evaluer(0.5).facteur == 0.0)
etat["scan"] = None                                 # lidar muet un instant
check("Lidar muet : arrêt", g.evaluer(1.0).facteur == 0.0)
etat["scan"] = (LOIN, 1.6)
v = g.evaluer(1.6)
check("Lidar revenu à 1.6 s (1.1 s après la première vue dégagée) : PAS de "
      "reprise immédiate — le délai compte depuis le retour du capteur",
      v.facteur == 0.0)
etat["scan"] = (LOIN, 2.7)
check("1.1 s de zone dégagée vue en continu : reprise",
      g.evaluer(2.7).facteur == 1.0)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
