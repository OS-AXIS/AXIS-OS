"""
Gardien de proximité — la sécurité des humains, garantie par l'OS.

Ce que ce test protège :
  - tout obstacle INEXPLIQUÉ arrête le robot, qu'il soit reconnu ou non ;
  - à 360° : un humain derrière le robot compte autant que devant ;
  - les étagères de la carte et le carton porté n'arrêtent PAS le robot ;
  - capteur muet ou périmé -> ARRÊT ;
  - AUCUNE app ne peut désactiver le gardien.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.locomotion.proximite import ChampsProtection, GardienProximite
from axis_os.services.locomotion.service import (
    BaseState, ILocomotionAdapter, LocomotionService, Twist,
)
from axis_os.services.navigation.service import Rectangle

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


# ── Nuages de points synthétiques, repère MONDE ─────────────────────
def humain(x, y, rayon=0.2, hauteur=1.75, n=400):
    a = np.random.RandomState(0).uniform(0, 2 * math.pi, n)
    z = np.random.RandomState(1).uniform(0.0, hauteur, n)
    return np.c_[x + rayon * np.cos(a), y + rayon * np.sin(a), z]


def boite(xmin, ymin, xmax, ymax, zmin, zmax, n=400):
    r = np.random.RandomState(2)
    return np.c_[r.uniform(xmin, xmax, n), r.uniform(ymin, ymax, n),
                 r.uniform(zmin, zmax, n)]


def sol(n=500):
    r = np.random.RandomState(3)
    return np.c_[r.uniform(-4, 4, n), r.uniform(-4, 4, n), np.zeros(n)]


ETAGERE = Rectangle(0.4, -1.0, 0.8, 1.0)          # connue de la carte
POSE = (0.0, 0.0, 0.0)                              # robot à l'origine, face +x


def gardien(points, t_scan=0.0, obstacles=(ETAGERE,), champs=ChampsProtection()):
    etat = {"points": points, "t": t_scan}
    g = GardienProximite(lambda: None if etat["points"] is None
                         else (etat["points"], etat["t"]),
                         lambda: POSE, obstacles=obstacles, champs=champs)
    return g, etat


def verdict(points, obstacles=()):
    g, _ = gardien(points, obstacles=obstacles)
    return g.evaluer(0.0)


section("1 — Vitesse selon la distance")
check("Rien autour (sol seulement) : pleine vitesse",
      verdict(sol()).facteur == 1.0)
check("Humain à 3 m : pleine vitesse", verdict(humain(3.0, 0.0)).facteur == 1.0)
v = verdict(humain(1.4, 0.0))
check(f"Humain à 1.4 m : vitesse RÉDUITE (×{v.facteur:.2f})", 0.2 < v.facteur < 0.8)
check("Humain à 0.7 m : ARRÊT", verdict(humain(0.7, 0.0)).facteur == 0.0)

section("2 — À 360° : ce que la caméra ne voit pas")
check("Humain DERRIÈRE le robot, à 0.6 m : ARRÊT",
      verdict(humain(-0.6, 0.0)).facteur == 0.0)
check("Humain sur le CÔTÉ, à 0.7 m : ARRÊT",
      verdict(humain(0.0, 0.7)).facteur == 0.0)

section("3 — Ce qui est expliqué n'arrête pas le robot")
check("Étagère DE LA CARTE à 0.4 m : pas d'arrêt — il doit pouvoir s'en "
      "approcher pour y prendre un carton",
      verdict(boite(0.4, -1.0, 0.8, 1.0, 0.0, 1.8), obstacles=(ETAGERE,)).facteur == 1.0)
check("Carton PORTÉ devant le torse : pas d'arrêt",
      verdict(boite(0.25, -0.12, 0.45, 0.12, 0.80, 1.00)).facteur == 1.0)

section("4 — Ce qui n'est expliqué par rien arrête le robot")
check("Carton OUBLIÉ au sol à 0.6 m : ARRÊT — mieux vaut s'arrêter pour "
      "rien que heurter", verdict(boite(0.5, -0.15, 0.7, 0.15, 0.0, 0.3)).facteur == 0.0)
check("Humain collé au carton porté (0.45 m) : ses JAMBES, sous le volume "
      "de port, le rendent visible — ARRÊT",
      verdict(humain(0.45, 0.0, rayon=0.1)).facteur == 0.0)
check("Obstacle inexpliqué : même hors de la carte connue, il compte",
      verdict(boite(1.0, 0.5, 1.2, 0.7, 0.3, 1.2), obstacles=(ETAGERE,)).facteur < 1.0)

section("5 — Pas de reptation dans la bande morte")
v = verdict(humain(0.84, 0.0))
check(f"Humain à 0.84 m : plafond sous la vitesse minimale de marche -> "
      f"ARRÊT franc, pas une reptation à {0.3 * (0.04 / 1.2):.3f} m/s",
      v.facteur == 0.0)

section("6 — Reprise : hystérésis et délai")
# Scène SANS étagère. Première version : l'humain se tenait à l'intérieur
# de l'emprise de l'étagère de la carte — ses points étaient « expliqués »
# et ignorés. C'est ce qui a révélé que la carte filtrait en 2D.
g, etat = gardien(humain(0.7, 0.0), obstacles=())
check("Arrêt à 0.7 m", g.evaluer(0.0).facteur == 0.0)
etat["points"], etat["t"] = humain(0.9, 0.0), 0.5
check("L'humain recule à 0.9 m : TOUJOURS arrêté (reprise au-delà de 1 m)",
      g.evaluer(0.5).facteur == 0.0)
etat["points"], etat["t"] = humain(1.5, 0.0), 1.0
check("À 1.5 m : encore arrêté pendant le délai de reprise",
      g.evaluer(1.0).facteur == 0.0)
etat["t"] = 2.1
v = g.evaluer(2.1)
check(f"Après 1.1 s de zone dégagée : reprise, à vitesse RÉDUITE (×{v.facteur:.2f})",
      0.0 < v.facteur < 1.0)

section("6b — Obstacles connus : en TROIS dimensions")
TABLE = Rectangle(0.30, -0.40, 0.80, 0.40, zmax=0.80)       # plateau à 80 cm
plateau = boite(0.30, -0.40, 0.80, 0.40, 0.0, 0.80)
check("La table elle-même (jusqu'à son plateau) : expliquée, pas d'arrêt",
      verdict(plateau, obstacles=(TABLE,)).facteur == 1.0)
check("Humain PENCHÉ au-dessus de la table (buste au-dessus du plateau) : "
      "ARRÊT — en 2D, il aurait été ignoré",
      verdict(np.r_[plateau, boite(0.45, -0.2, 0.75, 0.2, 0.9, 1.5)],
              obstacles=(TABLE,)).facteur == 0.0)
carton_sur_table = boite(0.25, -0.12, 0.45, 0.12, 0.80, 1.00)
g, _ = gardien(np.r_[plateau, carton_sur_table], obstacles=(TABLE,))
g.c = ChampsProtection(port_x=(0.0, 0.0))                     # bras vides
check("Carton posé SUR la table, sans exemption : ARRÊT — rien ne l'explique",
      g.evaluer(0.0).facteur == 0.0)

section("6c — Exemption de manipulation, contrôlée")
g, etat = gardien(np.r_[plateau, carton_sur_table], obstacles=(TABLE,))
g.c = ChampsProtection(port_x=(0.0, 0.0))
ok, _ = g.exempter((0.20, 0.50, -0.17, 0.17, 0.78, 1.05), 10.0, 0.0,
                   motif="prise du carton")
check("Exemption autour du carton : accordée", ok)
g._arrete = False
check("… et le robot peut s'approcher du carton qu'il doit prendre",
      g.evaluer(0.0).facteur == 1.0)
etat["points"] = np.r_[plateau, carton_sur_table, humain(0.55, 0.55)]
check("Pendant l'exemption, un humain qui s'approche HORS du volume : ARRÊT",
      g.evaluer(0.0).facteur == 0.0)
ok, raison = g.exempter((0.0, 2.0, -1.0, 1.0, 0.0, 2.0), 10.0, 0.0)
check(f"Exemption trop grande : REFUSÉE (« {raison[:40]}… »)", not ok)
ok, _ = g.exempter((0.20, 0.50, -0.17, 0.17, 0.78, 1.05), 600.0, 0.0)
check("Exemption trop longue : REFUSÉE", not ok)
check("Chaque exemption accordée est journalisée, avec son motif",
      g.journal_exemptions and g.journal_exemptions[0][3] == "prise du carton")
g2, e2 = gardien(np.r_[plateau, carton_sur_table], obstacles=(TABLE,))
g2.c = ChampsProtection(port_x=(0.0, 0.0))
g2.exempter((0.20, 0.50, -0.17, 0.17, 0.78, 1.05), 2.0, 0.0)
e2["t"] = 3.0
check("Exemption EXPIRÉE : la surveillance reprend, ARRÊT",
      g2.evaluer(3.0).facteur == 0.0)

section("7 — Capteur muet : arrêt")
g, etat = gardien(None)
check("Aucun balayage : ARRÊT", g.evaluer(0.0).facteur == 0.0)
g, etat = gardien(sol(), t_scan=0.0)
check("Balayage vieux de 0.5 s : ARRÊT — un capteur de sécurité muet n'est "
      "pas un capteur qui ne voit rien", g.evaluer(0.5).facteur == 0.0)

section("8 — Dans l'OS : aucune app ne peut passer outre")


class Adaptateur(ILocomotionAdapter):
    def __init__(self):
        self.recu = []

    @property
    def control_dt(self): return 0.02

    @property
    def max_twist(self): return Twist(0.8, 0.5, 1.5)

    def reset(self): pass

    def step(self, state, twist):
        self.recu.append(twist)
        return {}

    def is_fallen(self, state): return False


ETAT = BaseState((1, 0, 0, 0), (0, 0, 0), {}, {})
t = [0.0]
nuage = {"p": humain(0.7, 0.0)}
g = GardienProximite(lambda: (nuage["p"], t[0]), lambda: POSE,
                     horloge=lambda: t[0])
ad = Adaptateur()
loco = LocomotionService(ad, lambda: ETAT, lambda cibles: {}, clock=lambda: t[0],
                         gardien=g)
loco.start()
loco.set_velocity(0.3, 0.0, 0.0)          # l'app demande d'avancer
loco.tick()
check("Humain à 0.7 m : l'app demande 0.3 m/s, les jambes reçoivent ZÉRO",
      ad.recu[-1] == Twist())
nuage["p"] = humain(3.0, 0.0)
t[0] = 0.05
g._arrete = False                          # (repartir d'une zone dégagée)
loco.set_velocity(0.3, 0.0, 0.0)
loco.tick()
check("Humain à 3 m : la consigne passe intacte", ad.recu[-1].vx == 0.3)
methodes = [m for m in dir(loco) if not m.startswith("_")]
contournement = [m for m in methodes
                 if any(k in m.lower() for k in ("gardien", "proxim", "securit", "desactiv", "disable"))]
check(f"Aucune méthode publique pour désactiver le gardien "
      f"(trouvé : {contournement or 'aucune'})", not contournement)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
