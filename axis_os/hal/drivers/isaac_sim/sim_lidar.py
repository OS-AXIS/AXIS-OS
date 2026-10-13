"""
Lidar simulé pour Isaac Sim, par lancers de rayons PhysX.

Un lidar mesure des distances en balayant l'espace avec des faisceaux
laser ; il rend un nuage de points. Isaac Sim est le simulateur robotique
de NVIDIA, et PhysX son moteur physique : on lui demande, rayon par rayon,
le premier objet touché.

Le modèle est le Livox Mid-360 de la tête du G1 (le robot humanoïde
d'Unitree), monté tête en bas : son champ vertical, d'environ -52° à +7°,
regarde le sol et les jambes autour du robot, ce qui est exactement ce
dont le gardien de proximité a besoin. Ce gardien (GardienProximite, dans
axis_os/services/locomotion/proximite.py) réduit la vitesse de marche,
jusqu'à l'arrêt, quand un obstacle est trop proche. Le vrai Mid-360
balaie selon un motif non répétitif ; ici, une grille régulière (azimut,
c'est-à-dire angle horizontal, × couches) suffit à la sécurité de
proximité et reste déterministe : le même balayage donne les mêmes
points.

Pourquoi des rayons PhysX plutôt que le capteur lidar RTX d'Isaac Sim
(simulé par le moteur de rendu de la carte graphique) : aucune dépendance
à une extension dont le nom et l'API (l'interface de programmation)
changent d'une version à l'autre (constaté entre 4.x et 6.x), et un coût
maîtrisé. Les rayons ne voient que les objets qui ont une forme de
collision ; c'est aussi ce que « voit » la physique.

Le robot ne doit pas se voir lui-même : un rayon qui touche un de ses
corps est relancé juste derrière le point d'impact (les bras du G1 sont
dans le champ des couches basses).

Les points sont rendus dans le repère monde (frame_id "world") : c'est ce
qu'attend GardienProximite, qui les compare au plan du site.
"""
from __future__ import annotations

import math
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from axis_os.hal.core.interfaces import ILidar
from axis_os.hal.core.types import HALStatus, LidarScan, SensorType

# Élévations des couches, en degrés (0 = horizontale), pour un Mid-360
# monté tête en bas : couches plus serrées vers l'horizontale, où se
# trouvent les jambes des personnes à distance, et plus espacées vers le
# bas.
ELEVATIONS_MID360_INVERSE = (-50.0, -40.0, -31.0, -23.0, -16.0, -10.0, -5.0,
                             -1.0, 3.0, 7.0)

# Lanceur de rayon : (origine, direction, portée) -> (point touché, chemin
# dans la scène du corps touché), ou None si rien n'est touché.
Lanceur = Callable[[Tuple[float, float, float], Tuple[float, float, float], float],
                   Optional[Tuple[Tuple[float, float, float], str]]]


def directions_balayage(n_azimut: int, elevations_deg: Sequence[float]) -> np.ndarray:
    """Vecteurs unitaires (tableau N x 3) des rayons d'un balayage, dans le
    repère du capteur (x devant, y à gauche, z en haut).

    Pour chaque élévation de elevations_deg (en degrés), n_azimut
    directions réparties régulièrement sur un tour complet : N vaut donc
    n_azimut × le nombre de couches."""
    az = np.linspace(-math.pi, math.pi, n_azimut, endpoint=False)
    dirs = []
    for e in elevations_deg:
        el = math.radians(e)
        ce, se = math.cos(el), math.sin(el)
        dirs.append(np.c_[ce * np.cos(az), ce * np.sin(az), np.full(n_azimut, se)])
    return np.vstack(dirs)


def lanceur_physx() -> Lanceur:
    """Lanceur par défaut : le point touché le plus proche, demandé à la
    scène PhysX d'Isaac Sim. Le chemin rendu est celui du corps rigide
    touché, ou à défaut de sa forme de collision ; il sert à reconnaître
    les impacts sur le robot lui-même."""
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
    """Lidar simulé conforme à ILidar, l'interface lidar de la HAL
    (Hardware Abstraction Layer, la couche qui cache au reste du système
    les détails du matériel) : un service qui lit un lidar ne voit pas
    s'il est réel ou simulé."""

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
                       capteur. Sur le G1 : la tête, à 1.3 m environ.
        horloge      : horloge du robot (temps simulé), pour l'horodatage
                       que le gardien compare à age_max_s (au-delà, le
                       balayage est jugé périmé et le robot s'arrête).
        lanceur      : (origine, direction, portée) -> (point, chemin du
                       corps touché) ou None. Par défaut : PhysX, créé par
                       initialize().
        n_azimut, elevations_deg : grille de balayage, voir
                       directions_balayage().
        portee_m     : portée maximale d'un rayon, en mètres.
        periode_s    : intervalle entre deux balayages ; 0.1 s, soit 10 Hz,
                       comme le Mid-360.
        prefixes_ignores : débuts des chemins de scène du robot lui-même
                       (par défaut "/World/g1"). Un rayon qui touche un
                       corps dont le chemin commence ainsi est relancé
                       plus loin.
        relances_max : nombre maximal de relances d'un même rayon après un
                       impact sur le robot.
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
        # Compteurs de diagnostic. balayer() y ajoute le nombre de points
        # du dernier balayage et le nombre de balayages vides.
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
        """Premier point touché, hors du robot lui-même, par le rayon parti
        de o dans la direction d. Rend None si rien n'est touché dans la
        portée, ou si les relances sont épuisées."""
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
            parcouru = float(np.dot(p - origine, d))
            # Impact à distance quasi nulle : le rayon est parti de
            # l'intérieur d'un corps du robot (PhysX signale ce
            # chevauchement initial comme un impact). Un pas de 2 cm ne
            # suffit pas à en sortir : on saute 15 cm.
            saut = 0.15 if parcouru < 0.01 else 0.02
            reste -= parcouru + saut
            if reste <= 0:
                return None
            origine = p + saut * d
        return None

    def balayer(self) -> LidarScan:
        """Balayage complet, fait tout de suite, sans tenir compte de la
        période (scan() s'en charge)."""
        pos, R = self._pose()
        o = np.asarray(pos, dtype=float)
        dirs_monde = self._dirs @ np.asarray(R, dtype=float).T
        pts: List[Tuple[float, float, float]] = []
        for d in dirs_monde:
            p = self._lancer(o, d)
            if p is not None:
                pts.append(p)
        self.stats["balayages"] += 1
        self.stats["points_dernier_balayage"] = len(pts)
        if not pts:
            self.stats["balayages_vides"] = self.stats.get("balayages_vides", 0) + 1
        t = self._horloge()
        return LidarScan(points=np.asarray(pts, dtype=float).reshape(-1, 3),
                         frame_id="world", timestamp_ns=int(t * 1e9))

    def scan(self) -> Optional[LidarScan]:
        """Dernier balayage, refait seulement quand la période est écoulée
        sur l'horloge du robot. None tant que initialize() n'a pas été
        appelé."""
        if not self._initialized:
            return None
        t = self._horloge()
        if self._t_dernier is None or t - self._t_dernier >= self._periode:
            self._dernier, self._t_dernier = self.balayer(), t
        return self._dernier

    def source_gardien(self) -> Callable[[], Optional[Tuple[np.ndarray, float]]]:
        """Adaptateur pour GardienProximite(source_scan=...) : rend une
        fonction qui donne (points, horodatage en secondes), ou None."""
        def source():
            s = self.scan()
            if s is None:
                return None
            return s.points, s.timestamp_ns / 1e9
        return source
