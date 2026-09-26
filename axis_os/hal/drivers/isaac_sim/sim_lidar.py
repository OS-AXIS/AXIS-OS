"""
Lidar simulé pour Isaac Sim — par lancers de rayons PhysX.

Modelé sur le Livox Mid-360 de la tête du G1, monté TÊTE EN BAS : son
champ vertical, ~-52° à +7°, regarde le sol et les jambes autour du
robot, ce qui est exactement ce dont le gardien de proximité a besoin. Le
vrai Mid-360 balaie en motif non répétitif ; ici, une grille régulière
(azimut × couches) suffit à la sécurité de proximité et reste
déterministe.

Pourquoi des rayons PhysX plutôt que le capteur RTX d'Isaac : aucune
dépendance à une extension dont le nom et l'API changent d'une version à
l'autre (constaté entre 4.x et 6.x), et un coût maîtrisé. Les rayons ne
voient que les objets qui ont une COLLISION — c'est aussi ce que « voit »
la physique.

Le robot ne doit pas se voir lui-même : un rayon qui touche un de ses
corps est relancé juste derrière le point d'impact (les bras du G1 sont
dans le champ des couches basses).

Points rendus dans le repère MONDE (frame_id "world") : c'est ce qu'attend
le GardienProximite, qui compare au plan du site.
"""
from __future__ import annotations

import math
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from axis_os.hal.core.interfaces import ILidar
from axis_os.hal.core.types import HALStatus, LidarScan, SensorType

# Mid-360 inversé : couches plus serrées vers l'horizontale, où sont les
# jambes des personnes à distance ; plus espacées vers le bas.
ELEVATIONS_MID360_INVERSE = (-50.0, -40.0, -31.0, -23.0, -16.0, -10.0, -5.0,
                             -1.0, 3.0, 7.0)

Lanceur = Callable[[Tuple[float, float, float], Tuple[float, float, float], float],
                   Optional[Tuple[Tuple[float, float, float], str]]]


def directions_balayage(n_azimut: int, elevations_deg: Sequence[float]) -> np.ndarray:
    """Vecteurs unitaires (N x 3) dans le repère du CAPTEUR (x devant,
    y à gauche, z en haut)."""
    az = np.linspace(-math.pi, math.pi, n_azimut, endpoint=False)
    dirs = []
    for e in elevations_deg:
        el = math.radians(e)
        ce, se = math.cos(el), math.sin(el)
        dirs.append(np.c_[ce * np.cos(az), ce * np.sin(az), np.full(n_azimut, se)])
    return np.vstack(dirs)


def lanceur_physx() -> Lanceur:
    """Rayon le plus proche via la scène PhysX d'Isaac Sim."""
    from omni.physx import get_physx_scene_query_interface
    sqi = get_physx_scene_query_interface()

    def lancer(origine, direction, portee):
        h = sqi.raycast_closest(origine, direction, portee)
        if not h or not h.get("hit"):
            return None
        p = h["position"]
        chemin = str(h.get("rigidBody") or h.get("collision") or "")
        return (float(p[0]), float(p[1]), float(p[2])), chemin
    return lancer


class IsaacSimLidar(ILidar):
    def __init__(self, sensor_id: str,
                 pose_capteur: Callable[[], Tuple[np.ndarray, np.ndarray]],
                 horloge: Callable[[], float],
                 lanceur: Optional[Lanceur] = None,
                 n_azimut: int = 180,
                 elevations_deg: Sequence[float] = ELEVATIONS_MID360_INVERSE,
                 portee_m: float = 8.0, periode_s: float = 0.1,
                 prefixes_ignores: Sequence[str] = ("/World/g1",),
                 relances_max: int = 2):
        """
        pose_capteur : () -> (position monde (3,), rotation monde 3x3) du
                       capteur. Sur le G1 : la tête, 1.3 m environ.
        horloge      : horloge du ROBOT (temps simulé), pour l'horodatage
                       que le gardien compare à age_max_s.
        lanceur      : (origine, direction, portée) -> (point, chemin du
                       corps touché) ou None. Par défaut : PhysX.
        periode_s    : 10 Hz, comme le Mid-360.
        """
        super().__init__(sensor_id, SensorType.LIDAR)
        self._pose = pose_capteur
        self._horloge = horloge
        self._lanceur = lanceur
        self._dirs = directions_balayage(n_azimut, elevations_deg)
        self._portee = portee_m
        self._periode = periode_s
        self._ignores = tuple(prefixes_ignores)
        self._relances = relances_max
        self._dernier: Optional[LidarScan] = None
        self._t_dernier: Optional[float] = None
        self.stats = {"balayages": 0, "rayons": 0, "impacts_sur_soi": 0}

    # ---- ISensor ------------------------------------------------------------
    def initialize(self) -> HALStatus:
        if self._lanceur is None:
            self._lanceur = lanceur_physx()
        self._initialized = True
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._initialized = False
        return HALStatus.OK

    def is_healthy(self) -> bool:
        return self._initialized

    # ---- balayage -------------------------------------------------------------
    def _sur_soi(self, chemin: str) -> bool:
        return any(chemin.startswith(p) for p in self._ignores)

    def _lancer(self, o: np.ndarray, d: np.ndarray) -> Optional[Tuple[float, float, float]]:
        reste, origine = self._portee, o
        for _ in range(self._relances + 1):
            r = self._lanceur(tuple(float(v) for v in origine),
                              tuple(float(v) for v in d), reste)
            self.stats["rayons"] += 1
            if r is None:
                return None
            p, chemin = r
            if not self._sur_soi(chemin):
                return p
            self.stats["impacts_sur_soi"] += 1
            p = np.asarray(p, dtype=float)
            parcouru = float(np.dot(p - origine, d)) + 0.02
            reste -= parcouru
            if reste <= 0:
                return None
            origine = p + 0.02 * d
        return None

    def balayer(self) -> LidarScan:
        pos, R = self._pose()
        o = np.asarray(pos, dtype=float)
        dirs_monde = self._dirs @ np.asarray(R, dtype=float).T
        pts: List[Tuple[float, float, float]] = []
        for d in dirs_monde:
            p = self._lancer(o, d)
            if p is not None:
                pts.append(p)
        self.stats["balayages"] += 1
        t = self._horloge()
        return LidarScan(points=np.asarray(pts, dtype=float).reshape(-1, 3),
                         frame_id="world", timestamp_ns=int(t * 1e9))

    def scan(self) -> Optional[LidarScan]:
        if not self._initialized:
            return None
        t = self._horloge()
        if self._t_dernier is None or t - self._t_dernier >= self._periode:
            self._dernier, self._t_dernier = self.balayer(), t
        return self._dernier

    def source_gardien(self) -> Callable[[], Optional[Tuple[np.ndarray, float]]]:
        """Adaptateur pour GardienProximite(source_scan=...)."""
        def source():
            s = self.scan()
            if s is None:
                return None
            return s.points, s.timestamp_ns / 1e9
        return source
