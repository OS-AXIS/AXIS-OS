"""
AXIS-OS — Ce que la prise mesurée lit, calculé des seuls signaux du vrai
G1 à mains caoutchouc : angles articulaires (codeurs) et orientation de la
centrale inertielle. Aucun capteur de contact ni de force dans les mains.

En simulation (banc Isaac T1), ces estimations remplacent la vérité de la
physique (racine PhysX, pose du carton) que le banc passait à l'OS.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Dict

import numpy as np

from axis_os.services.manipulation.cinematique import URDF_G1_29DOF, ChaineBras


class HauteurBassin:
    """Hauteur du bassin au-dessus du sol (m) : cinématique des jambes sur
    les angles mesurés, tournée par l'orientation de la centrale
    inertielle ; le pied le plus bas est le pied d'appui."""

    # Dessous du pied sous ankle_roll_link : sphères de contact de l'URDF,
    # 3 cm plus bas, rayon 5 mm.
    SEMELLE_M = 0.035

    def __init__(self, chemin: Path = URDF_G1_29DOF):
        self._pieds = [ChaineBras.depuis_urdf(c, chemin, fin=f"{c}_ankle_roll_link")
                       for c in ("left", "right")]

    def __call__(self, angles: Dict[str, float], rotation: np.ndarray) -> float:
        r = np.asarray(rotation, dtype=float)
        return max(-float((r @ p.main(angles))[2]) for p in self._pieds) + self.SEMELLE_M


def inclinaison_mains(main_g, main_d, rotation: np.ndarray) -> float:
    """Inclinaison (degrés) de l'objet serré entre les mains : celle de la
    droite qui joint les mains (cinématique, repère du bassin) dans le
    repère monde (orientation de la centrale inertielle). Un objet qui
    tourne autour de cette droite (il glisse en tangage) n'est pas vu."""
    v = np.asarray(rotation, dtype=float) @ (np.asarray(main_g, float) - np.asarray(main_d, float))
    return math.degrees(math.atan2(abs(float(v[2])), math.hypot(float(v[0]), float(v[1]))))
