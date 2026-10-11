"""
L'entrepôt de la démonstration de tri — UNE seule définition.

La répétition générale sans GPU (test/test_tri_pile_logicielle.py) et la
scène Isaac Sim (sim/validation/test_isaac_t1_tri_usine.py) lisent toutes deux ce module :
ce qui a été validé sur CPU est exactement ce qui tournera sur le GPU.

Disposition (10 m × 8 m, vue de dessus, y vers le haut) :

    y=8 ┌──────────────────────────────────────────┐
        │        ║           [C]          ║        │
        │        ║                        ║        │
        │  [B]   ║ étagère       étagère  ║  [A]   │
        │        ║                        ║        │
        │   ← humain qui traverse (y = 3.2) →      │
        │            [table de prise]    ↑ humain  │
        │  collègue ↗     (prise)        │ (x=6.4) │
    y=0 └──────────────────────────────────────────┘

Postes de dépose à 4-6 m du poste de prise : c'est le levier de débit
identifié par la première répétition (postes à 7 m : ~95 s par colis).

Tables : plateau à 0.80 m, comme dans W3/W4 — un carton de 20 cm posé
dessus a son centre à 0.90 m, la hauteur des mains en posture de port.
"""
from __future__ import annotations

import math
from typing import Dict, List

import numpy as np

from axis_os.apps.tri_colis.facade import PosteDepose
from axis_os.services.navigation.service import Carte, Pose2D, Rectangle

PLATEAU = 0.80                       # hauteur des plateaux (m)
LARGEUR_CARTON, PROFONDEUR_CARTON, HAUTEUR_CARTON = 0.24, 0.20, 0.20
MAINS_DEVANT = 0.29                  # centre des mains devant le bassin
DANS_PLATEAU = 0.17                  # centre du carton à 17 cm du bord (12 au départ : sur GPU, carton posé de travers au bord, tombé)

CAP_PRISE = math.pi / 2              # face à +y pour prendre
DEPART = Pose2D(4.0, 0.5, CAP_PRISE)

# Zones de poste : zmax = plateau + hauteur de chargement. Les cartons posés
# dessus sont « expliqués » pour le gardien ; un humain penché, plus haut, non.
# 1.6 m de long : 28 cm entre les cartons des bouts et le bord de la table.
# (1.2 m au départ : 8 cm seulement. Constaté sur GPU, vidéo T1 : en
# serrant, une main a poussé le carton du bout, qui a basculé par-dessus le
# bord et est tombé.)
TABLE_PRISE = Rectangle(3.2, 2.0, 4.8, 2.6, zmax=PLATEAU + 0.25)
TABLES_DEPOSE: Dict[str, Rectangle] = {
    "A": Rectangle(7.0, 4.4, 8.0, 5.0, zmax=PLATEAU + 0.30),
    "B": Rectangle(0.8, 4.4, 1.8, 5.0, zmax=PLATEAU + 0.30),
    "C": Rectangle(3.5, 6.6, 4.5, 7.2, zmax=PLATEAU + 0.30),
}
ETAGERES: List[Rectangle] = [Rectangle(2.4, 3.6, 2.8, 7.6),
                             Rectangle(5.2, 3.6, 5.6, 7.6)]
OBSTACLES: List[Rectangle] = [TABLE_PRISE, *TABLES_DEPOSE.values(), *ETAGERES]

# Emplacements des cartons sur la table de prise : resserrés pour rester
# dans le champ de la caméra (±30° avec une focale de 18 mm) depuis le
# poste de prise, à 1 m.
# Espacés de 40 cm (35 au premier essai GPU) : 16 cm entre deux cartons,
# la place de glisser une main ouverte de part et d'autre du carton visé.
EMPLACEMENTS_X = [3.60, 4.00, 4.40]
Y_CARTON = TABLE_PRISE.ymin + 0.10   # centre des cartons

POSTES = {
    "prise": Pose2D(4.0, 1.0, CAP_PRISE),
    "A": Pose2D(7.5, 3.8, math.pi / 2),
    "B": Pose2D(1.3, 3.8, math.pi / 2),
    "C": Pose2D(4.0, 6.0, math.pi / 2),
}
REGLES_INITIALES = {"rouge": "A", "bleu": "B"}
COULEURS = ["rouge", "bleu", "vert"]
COULEURS_RGB = {"rouge": (0.80, 0.10, 0.10), "bleu": (0.10, 0.25, 0.80),
                "vert": (0.10, 0.60, 0.20)}


def carte() -> Carte:
    return Carte(0.0, 0.0, 10.0, 8.0, obstacles=list(OBSTACLES), postes=dict(POSTES))


def poste_depose(nom: str) -> PosteDepose:
    """Accostage jusqu'à ce que le carton tenu soit DANS_PLATEAU à
    l'intérieur du plateau (tables abordées par leur bord ymin)."""
    bord = TABLES_DEPOSE[nom].ymin
    return PosteDepose(avance_m=(bord + DANS_PLATEAU - MAINS_DEVANT) - POSTES[nom].y,
                       hauteur=PLATEAU)


def postes_depose() -> Dict[str, PosteDepose]:
    return {n: poste_depose(n) for n in TABLES_DEPOSE}


# ── Figurants : des personnes qui circulent ───────────────────────────────

class Figurant:
    """Fait des allers-retours ; s'écarte d'un robot trop proche et fait
    demi-tour si son but est à côté du robot, comme le ferait une personne.
    Ce n'est PAS lui qui garantit l'absence de contact : les tests vérifient
    que le ROBOT ne s'approche jamais d'une personne proche."""

    def __init__(self, a, b, vitesse=1.0, pause=4.0, evitement=1.1,
                 pause_a=None):
        """pause : à l'arrivée en b ; pause_a : à l'arrivée en a (par
        défaut la même)."""
        self.a, self.b = np.array(a, float), np.array(b, float)
        self.v, self.pause, self.ev = vitesse, pause, evitement
        self.pause_a = pause if pause_a is None else pause_a
        self.p, self.vers_b, self.attente = np.array(a, float), True, 0.0

    def pas(self, dt: float, rx: float, ry: float) -> None:
        if self.attente > 0:
            self.attente -= dt
            return
        but = self.b if self.vers_b else self.a
        d = but - self.p
        n = float(np.linalg.norm(d))
        if n < 0.05:
            self.attente = self.pause if self.vers_b else self.pause_a
            self.vers_b = not self.vers_b
            return
        suiv = self.p + d / n * min(self.v * dt, n)
        r = np.array([rx, ry])
        e = suiv - r
        ne = max(float(np.linalg.norm(e)), 1e-6)
        if ne < self.ev and n < 1.5:
            self.vers_b = not self.vers_b      # son but est à côté du robot
            return
        if ne < self.ev:
            # Contourne le robot en tangente, du côté de son but.
            tang = np.array([-e[1], e[0]]) / ne
            if np.dot(tang, d) < 0:
                tang = -tang
            suiv = self.p + tang * self.v * dt
            e = suiv - r
            suiv = r + e / max(float(np.linalg.norm(e)), 1e-6) * max(self.ev, ne)
        self.p = suiv


def figurants() -> List[Figurant]:
    return [
        Figurant((6.4, 0.5), (6.4, 7.5)),                       # traverse vers A
        Figurant((0.3, 3.2), (9.7, 3.2), vitesse=0.8, pause=10.0),
        # un collègue qui vient régulièrement près du poste de prise
        # Pause de 15 s (45 s au premier essai GPU : trop de temps à attendre
        # pour une démonstration, sans rien montrer de plus).
        # Visite toutes les ~2 min (au premier essai GPU : toutes les 40 s,
        # le robot ne faisait plus qu'attendre).
        # Arrêt à (2.6, 1.1) : 1.4 m du poste de prise (3.2 au départ — sur
        # GPU, il se tenait à 0.7 m du robot 15 s à chaque visite : 2 min
        # d'accostage perdues dès le début de l'essai de 30 min). Le robot
        # ralentit à son passage, s'arrête s'il travaille à l'emplacement
        # de gauche.
        # (2.2, 0.8) ensuite : à (2.6, 1.1), il se tenait à 1.0 m de la
        # pré-pose de l'emplacement de GAUCHE (3.60) — robot arrêté par la
        # sécurité, deux accostages du carton bleu perdus par délai (RTX 5090).
        Figurant((0.8, 0.5), (2.2, 0.8), vitesse=0.5, pause=15.0, evitement=0.6,
                 pause_a=90.0),
    ]
