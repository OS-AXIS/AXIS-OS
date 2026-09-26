"""
AXIS-OS — Gardien de proximité : champs de protection autour du robot.

Le principe des scanners de sécurité des chariots autonomes industriels,
appliqué à un humanoïde. Le lidar voit tout autour ; tout ce qu'il voit et
qui n'est PAS expliqué est un obstacle potentiellement humain :

    au-delà de d_ralenti   pleine vitesse
    entre les deux         vitesse plafonnée, proportionnellement
    en deçà de d_arret     ARRÊT (en équilibre, pas une coupure)

LA SÉCURITÉ NE REPOSE PAS SUR LA RECONNAISSANCE. S'arrêter seulement devant
ce qu'un modèle identifie comme « une personne », c'est ignorer l'humain
vu de dos, accroupi, à contre-jour, ou simplement mal reconnu. Ici, un
obstacle inexpliqué suffit. La reconnaissance peut ENRICHIR l'affichage
(« humain détecté ») ; elle ne décide jamais de la sécurité.

CE QUI EST EXPLIQUÉ, et donc ignoré :
  - le SOL et le PLAFOND (bande de hauteur utile) ;
  - le ROBOT LUI-MÊME (son empreinte) ;
  - le VOLUME DE PORT devant le torse, à hauteur des bras : le carton
    porté ne doit pas arrêter le robot. Les JAMBES d'un humain qui s'y
    tiendrait restent visibles en dessous — il n'est pas ignoré ;
  - les OBSTACLES FIXES DE LA CARTE (étagères, tables) : le robot doit
    pouvoir s'en approcher pour y prendre un carton.

Un carton oublié par terre, lui, n'est expliqué par rien : le robot
s'arrête. C'est voulu — mieux vaut s'arrêter pour rien que heurter.

FAIL-SAFE : pas de balayage, ou balayage périmé -> ARRÊT. Un capteur de
sécurité muet n'est pas un capteur qui ne voit rien.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class ChampsProtection:
    d_arret: float = 0.80          # m, depuis le centre du robot
    d_ralenti: float = 2.00
    d_reprise: float = 1.00        # hystérésis : ne repart pas à 0.81 m
    delai_reprise_s: float = 1.0   # ... ni aussitôt la zone dégagée
    rayon_robot: float = 0.35      # empreinte, ignorée
    h_min: float = 0.05            # au-dessus du sol
    h_max: float = 1.90            # sous le plafond
    # Volume de port, repère du robot (x devant, y à gauche), hauteur monde.
    port_x: tuple = (0.15, 0.60)
    port_y: float = 0.30
    port_z: tuple = (0.60, 1.30)
    marge_carte: float = 0.05
    age_max_s: float = 0.3         # au-delà : balayage périmé -> arrêt
    # Exemption de manipulation : volume MAXIMAL par côté, et durée maximale.
    exemption_cote_max: float = 0.70
    exemption_duree_max_s: float = 30.0


@dataclass(frozen=True)
class Verdict:
    facteur: float                 # 0 = arrêt, 1 = pleine vitesse
    distance: Optional[float]      # obstacle inexpliqué le plus proche
    raison: str


class GardienProximite:
    """Évalue un balayage et rend un facteur de vitesse.

    source_scan  : () -> (points_monde N x 3, horodatage) ou None
    source_pose  : () -> (x, y, cap) du robot dans le monde
    obstacles    : rectangles fixes de la carte (xmin, ymin, xmax, ymax)
    """

    def __init__(self, source_scan: Callable, source_pose: Callable,
                 obstacles: Sequence = (), champs: ChampsProtection = ChampsProtection(),
                 horloge: Callable[[], float] = None, v_min: float = 0.06,
                 v_max: float = 0.30):
        self._scan = source_scan
        self._pose = source_pose
        self._obstacles = list(obstacles)
        self.c = champs
        self._horloge = horloge
        self._v_min, self._v_max = v_min, v_max
        self._arrete = False
        self._degage_depuis: Optional[float] = None
        self._exemption = None           # (xmin, xmax, ymin, ymax, zmin, zmax, fin)
        self.journal_exemptions = []
        self.stats = {"arrets": 0, "ralentissements": 0, "capteur_muet": 0,
                      "exemptions": 0, "exemptions_refusees": 0}

    def exempter(self, volume, duree_s: float, maintenant: float,
                 motif: str = "") -> tuple:
        """EXEMPTION CONTRÔLÉE pendant une manipulation.

        Le carton à prendre est posé sur une table, au-dessus de son plateau :
        rien ne l'explique, et le robot s'arrêterait devant l'objet qu'il doit
        saisir. Comme les chariots industriels qui neutralisent un champ de
        protection pour s'amarrer à un poste, on exclut UN petit volume
        autour de l'objet ciblé.

        Bornée : chaque côté ≤ exemption_cote_max, durée ≤
        exemption_duree_max_s. Refusée au-delà. Journalisée. Une seule à la
        fois. Tout ce qui est HORS du volume reste surveillé — les jambes
        d'un humain qui s'approche de la table, par exemple, sous le plateau.

        volume : (xmin, xmax, ymin, ymax, zmin, zmax), repère MONDE.
        """
        xmin, xmax, ymin, ymax, zmin, zmax = volume
        cotes = (xmax - xmin, ymax - ymin, zmax - zmin)
        if min(cotes) <= 0:
            self.stats["exemptions_refusees"] += 1
            return False, "volume vide ou inversé"
        if max(cotes) > self.c.exemption_cote_max:
            self.stats["exemptions_refusees"] += 1
            return False, (f"volume trop grand ({max(cotes):.2f} m > "
                           f"{self.c.exemption_cote_max} m) — une exemption ne "
                           f"peut couvrir que l'objet manipulé")
        if duree_s <= 0 or duree_s > self.c.exemption_duree_max_s:
            self.stats["exemptions_refusees"] += 1
            return False, (f"durée {duree_s} s hors de ]0, "
                           f"{self.c.exemption_duree_max_s}] s")
        self._exemption = (xmin, xmax, ymin, ymax, zmin, zmax, maintenant + duree_s)
        self.stats["exemptions"] += 1
        self.journal_exemptions.append((maintenant, volume, duree_s, motif))
        return True, ""

    def lever_exemption(self) -> None:
        self._exemption = None

    @property
    def arrete(self) -> bool:
        """Le gardien tient-il le robot à l'arrêt ? Lu par les autres
        services (bras) : un arrêt de sécurité fige TOUT le robot, pas
        seulement les jambes."""
        return self._arrete

    def _dans_carte(self, x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
        """Points expliqués par un obstacle CONNU — en trois dimensions.

        Première version en deux dimensions : tout ce qui se trouvait
        au-dessus de l'emprise d'une table était ignoré, à n'importe quelle
        hauteur. Un humain penché au-dessus d'une table aurait été invisible.
        """
        m = np.zeros(x.shape, dtype=bool)
        mc = self.c.marge_carte
        for o in self._obstacles:
            dedans = ((x >= o.xmin - mc) & (x <= o.xmax + mc)
                      & (y >= o.ymin - mc) & (y <= o.ymax + mc))
            zmax = getattr(o, "zmax", None)
            if zmax is not None:
                dedans &= z <= zmax + mc
            m |= dedans
        return m

    def _dans_exemption(self, x, y, z, maintenant):
        e = self._exemption
        if e is None:
            return np.zeros(x.shape, dtype=bool)
        if maintenant > e[6]:
            self._exemption = None       # expirée : la surveillance reprend
            return np.zeros(x.shape, dtype=bool)
        return ((x >= e[0]) & (x <= e[1]) & (y >= e[2]) & (y <= e[3])
                & (z >= e[4]) & (z <= e[5]))

    def distance_inexpliquee(self, points: np.ndarray, pose,
                             maintenant: float = 0.0) -> Optional[float]:
        """Distance horizontale au point inexpliqué le plus proche."""
        if points is None or len(points) == 0:
            return None
        px, py, cap = pose
        P = np.asarray(points, dtype=float)
        x, y, z = P[:, 0], P[:, 1], P[:, 2]
        dx, dy = x - px, y - py
        d = np.hypot(dx, dy)
        # Repère du robot, pour le volume de port.
        c, s = math.cos(cap), math.sin(cap)
        xr, yr = c * dx + s * dy, -s * dx + c * dy
        garde = (z >= self.c.h_min) & (z <= self.c.h_max)          # sol, plafond
        garde &= d > self.c.rayon_robot                              # soi-même
        port = ((xr >= self.c.port_x[0]) & (xr <= self.c.port_x[1])
                & (np.abs(yr) <= self.c.port_y)
                & (z >= self.c.port_z[0]) & (z <= self.c.port_z[1]))
        garde &= ~port                                               # carton porté
        garde &= ~self._dans_carte(x, y, z)                          # étagères, tables
        garde &= ~self._dans_exemption(x, y, z, maintenant)          # objet manipulé
        if not garde.any():
            return float("inf")
        return float(d[garde].min())

    def evaluer(self, maintenant: float) -> Verdict:
        s = self._scan()
        if s is None:
            self.stats["capteur_muet"] += 1
            self._arrete = True
            return Verdict(0.0, None, "lidar muet — ARRÊT par sécurité")
        points, horodatage = s
        if maintenant - horodatage > self.c.age_max_s:
            self.stats["capteur_muet"] += 1
            self._arrete = True
            return Verdict(0.0, None, f"balayage périmé de "
                                      f"{maintenant - horodatage:.2f} s — ARRÊT")
        d = self.distance_inexpliquee(points, self._pose(), maintenant)

        if d is not None and d < self.c.d_arret:
            if not self._arrete:
                self.stats["arrets"] += 1
            self._arrete, self._degage_depuis = True, None
            return Verdict(0.0, d, f"obstacle inexpliqué à {d:.2f} m — ARRÊT")

        if self._arrete:
            # Reprise : zone dégagée au-delà de d_reprise, pendant un délai.
            if d is None or d < self.c.d_reprise:
                self._degage_depuis = None
                return Verdict(0.0, d, "en attente : zone pas encore dégagée")
            if self._degage_depuis is None:
                self._degage_depuis = maintenant
            if maintenant - self._degage_depuis < self.c.delai_reprise_s:
                return Verdict(0.0, d, "zone dégagée — reprise imminente")
            self._arrete, self._degage_depuis = False, None

        if d is not None and d < self.c.d_ralenti:
            f = (d - self.c.d_arret) / (self.c.d_ralenti - self.c.d_arret)
            # Un plafond sous la vitesse minimale de marche ne ferait pas
            # ralentir le robot : il le ferait tenir debout en piétinant
            # (bande morte de GR00T WBC). Alors on arrête franchement.
            if f * self._v_max < self._v_min:
                # Arrêt VERROUILLÉ, comme sous d_arret : sans verrou, le robot
                # oscillerait entre arrêt et marche à la frontière.
                if not self._arrete:
                    self.stats["arrets"] += 1
                self._arrete, self._degage_depuis = True, None
                return Verdict(0.0, d, f"obstacle à {d:.2f} m — arrêt "
                                       f"(vitesse plafonnée sous le minimum)")
            self.stats["ralentissements"] += 1
            return Verdict(f, d, f"obstacle à {d:.2f} m — vitesse ×{f:.2f}")
        return Verdict(1.0, d, "zone dégagée")
