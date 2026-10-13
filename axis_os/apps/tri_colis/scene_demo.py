"""
L'entrepôt de la démonstration de tri, défini en un seul endroit.

Pourquoi une seule définition : la répétition générale sans GPU (carte
graphique), test/test_tri_pile_logicielle.py, qui exécute toute la pile
logicielle sur le processeur (CPU), et la scène de simulation physique
Isaac Sim, sim/validation/test_isaac_t1_tri_usine.py, lisent toutes deux ce
module. Ce qui a été validé sur CPU est donc exactement ce qui tournera sur
GPU. Voir docs/demo_tri_usine.md.

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

Les postes de dépose sont à 4-6 m du poste de prise : c'est le levier de
débit identifié par la première répétition (avec des postes à 7 m, il
fallait environ 95 s par colis).

Tables : plateau à 0.80 m, comme lors des essais en simulation où le robot
prend un carton sur une table et l'emporte, puis va le chercher sans
connaître sa position (docs/validation_port_carton.md). Un carton de 20 cm
posé dessus a son centre à 0.90 m, la hauteur des mains en posture de port
(celle où le robot tient un carton devant lui).

Dans ce fichier, « sur GPU » renvoie aux essais de cette scène sous
Isaac Sim, « RTX 5090 » à l'une des cartes graphiques utilisées.
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
# Le carton déposé a son centre à 17 cm à l'intérieur du bord du plateau.
# Au départ, 12 cm : sur GPU, un carton posé de travers au bord est tombé.
DANS_PLATEAU = 0.17

CAP_PRISE = math.pi / 2              # orientation pour prendre : face à +y
DEPART = Pose2D(4.0, 0.5, CAP_PRISE)  # pose de départ du robot (x, y, cap)

# Tables, telles que les voit le gardien de proximité (le service qui
# ralentit ou arrête le robot quand quelque chose est trop près) :
# Rectangle(xmin, ymin, xmax, ymax), et zmax = plateau + hauteur de
# chargement. Les cartons posés dessus sont ainsi « expliqués » par la
# table et n'arrêtent pas le robot ; un humain penché, plus haut, reste
# détecté.
# Table de prise de 1.6 m de long : 28 cm entre les cartons des bouts et le
# bord de la table. (Au départ, 1.2 m, soit 8 cm seulement. Constaté sur
# GPU, dans la vidéo d'un essai de cette scène : en serrant, une main a
# poussé le carton du bout, qui a basculé par-dessus le bord et est tombé.)
TABLE_PRISE = Rectangle(3.2, 2.0, 4.8, 2.6, zmax=PLATEAU + 0.25)
TABLES_DEPOSE: Dict[str, Rectangle] = {
    "A": Rectangle(7.0, 4.4, 8.0, 5.0, zmax=PLATEAU + 0.30),
    "B": Rectangle(0.8, 4.4, 1.8, 5.0, zmax=PLATEAU + 0.30),
    "C": Rectangle(3.5, 6.6, 4.5, 7.2, zmax=PLATEAU + 0.30),
}
ETAGERES: List[Rectangle] = [Rectangle(2.4, 3.6, 2.8, 7.6),
                             Rectangle(5.2, 3.6, 5.6, 7.6)]
OBSTACLES: List[Rectangle] = [TABLE_PRISE, *TABLES_DEPOSE.values(), *ETAGERES]

# Emplacements des cartons sur la table de prise (abscisses x). Ils sont
# resserrés pour rester dans le champ de la caméra (±30° avec une focale de
# 18 mm) vue du poste de prise, à 1 m. Ils sont espacés de 40 cm (35 au
# premier essai sur GPU) : il reste 16 cm entre deux cartons, la place de
# glisser une main ouverte de part et d'autre du carton visé.
EMPLACEMENTS_X = [3.60, 4.00, 4.40]
# Centre des cartons, à 10 cm du bord de la table côté robot.
Y_CARTON = TABLE_PRISE.ymin + 0.10

# Poses des postes dans la carte (x, y, cap) : là où la navigation amène le
# robot.
POSTES = {
    "prise": Pose2D(4.0, 1.0, CAP_PRISE),
    "A": Pose2D(7.5, 3.8, math.pi / 2),
    "B": Pose2D(1.3, 3.8, math.pi / 2),
    "C": Pose2D(4.0, 6.0, math.pi / 2),
}
# Règles de tri au démarrage : le vert n'en a pas.
REGLES_INITIALES = {"rouge": "A", "bleu": "B"}
# Couleurs des cartons de la scène, et leur couleur RGB (composantes de 0
# à 1) dans la simulation.
COULEURS = ["rouge", "bleu", "vert"]
COULEURS_RGB = {"rouge": (0.80, 0.10, 0.10), "bleu": (0.10, 0.25, 0.80),
                "vert": (0.10, 0.60, 0.20)}


def carte() -> Carte:
    """Carte de l'entrepôt : 10 m × 8 m, avec ses obstacles et ses postes."""
    return Carte(0.0, 0.0, 10.0, 8.0, obstacles=list(OBSTACLES), postes=dict(POSTES))


def poste_depose(nom: str) -> PosteDepose:
    """Poste de dépose `nom` : l'accostage fin avance jusqu'à ce que le
    centre du carton tenu soit à DANS_PLATEAU à l'intérieur du plateau.
    Les tables sont abordées par leur bord ymin, robot face à +y."""
    bord = TABLES_DEPOSE[nom].ymin
    return PosteDepose(avance_m=(bord + DANS_PLATEAU - MAINS_DEVANT) - POSTES[nom].y,
                       hauteur=PLATEAU)


def postes_depose() -> Dict[str, PosteDepose]:
    """Les postes de dépose de la scène (A, B et C)."""
    return {n: poste_depose(n) for n in TABLES_DEPOSE}


# ── Figurants : des personnes qui circulent ───────────────────────────────

class Figurant:
    """Une personne simulée, qui fait des allers-retours entre a et b.

    Comme le ferait une personne, elle s'écarte d'un robot trop proche, et
    fait demi-tour si son but est juste à côté du robot. Ce n'est pas elle
    qui garantit l'absence de contact : les tests vérifient que c'est le
    robot qui ne s'approche jamais d'une personne proche."""

    def __init__(self, a, b, vitesse=1.0, pause=4.0, evitement=1.1,
                 pause_a=None):
        """a, b : extrémités du trajet (x, y), en mètres ; vitesse en m/s ;
        evitement : distance au robot en deçà de laquelle elle s'écarte ;
        pause : attente à l'arrivée en b, en secondes ; pause_a : attente à
        l'arrivée en a (par défaut la même)."""
        self.a, self.b = np.array(a, float), np.array(b, float)
        self.v, self.pause, self.ev = vitesse, pause, evitement
        self.pause_a = pause if pause_a is None else pause_a
        self.p, self.vers_b, self.attente = np.array(a, float), True, 0.0

    def pas(self, dt: float, rx: float, ry: float) -> None:
        """Avance de dt secondes ; (rx, ry) est la position du robot."""
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
    """Les trois personnes simulées de la démonstration."""
    return [
        Figurant((6.4, 0.5), (6.4, 7.5)),                       # traverse vers A
        Figurant((0.3, 3.2), (9.7, 3.2), vitesse=0.8, pause=10.0),
        # Un collègue qui vient régulièrement près du poste de prise. Ses
        # réglages ont été ajustés au fil des essais sur GPU :
        # - pause de 15 s (45 s au premier essai : trop de temps d'attente
        #   pour une démonstration, sans rien montrer de plus) ;
        # - une visite toutes les ~2 min (toutes les 40 s au premier essai :
        #   le robot ne faisait plus qu'attendre) ;
        # - arrêt déplacé à (2.6, 1.1), à 1.4 m du poste de prise (il était
        #   au départ en (3.2, 1.2) : sur GPU, il se tenait alors à 0.7 m du
        #   robot pendant 15 s à chaque visite, et 2 min d'accostage étaient
        #   perdues dès le début de l'essai de 30 min). Le robot ralentit à
        #   son passage, et s'arrête s'il travaille à l'emplacement de
        #   gauche ;
        # - arrêt enfin déplacé en (2.2, 0.8) : à (2.6, 1.1), il se tenait à
        #   1.0 m de la pré-pose (position d'attente devant la table) de
        #   l'emplacement de gauche (x = 3.60) ; le robot était arrêté par
        #   la sécurité, et deux accostages du carton bleu ont été perdus
        #   par dépassement de délai (essai sur RTX 5090).
        Figurant((0.8, 0.5), (2.2, 0.8), vitesse=0.5, pause=15.0, evitement=0.6,
                 pause_a=90.0),
    ]
