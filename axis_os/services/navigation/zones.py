"""
AXIS-OS — Gardien de zones : le robot n'entre jamais dans une zone interdite,
et marche lentement dans une zone lente.

Branché sur le service de locomotion (LocomotionService, paramètre `zones`),
il s'applique à CHAQUE consigne de marche, d'où qu'elle vienne : walk(), go_to
(la navigation), follow (le suivi d'une personne), un flux « walk ». Aucune
app ne peut le régler ni l'ignorer. Il ne fait que réduire une consigne,
jamais l'augmenter, et vient après le gardien de proximité.

Règles, à chaque pas de commande :
  - le robot (un disque de rayon `rayon_m`, plus `marge_m`) ne doit jamais
    toucher une zone interdite : on prolonge la consigne sur `horizon_s` et,
    si le disque y entrerait, la translation est annulée (la rotation sur
    place reste permise, le disque ne bouge pas) ;
  - un robot déjà trop près ou dedans (localisation qui dérive, robot porté)
    ne peut que s'en éloigner ;
  - dans une zone lente, ou sur le point d'y entrer, la vitesse de
    translation est plafonnée à la plus basse des zones concernées ;
  - pose du robot inconnue, non finie ou carte illisible : arrêt (mieux vaut
    s'arrêter que d'entrer quelque part sans le savoir), dès que la carte a
    au moins une zone interdite ou lente.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from .site_map import FORBIDDEN, SLOW


@dataclass(frozen=True)
class VerdictZones:
    vx: float
    vy: float
    wz: float
    raison: str
    zone: Optional[str] = None


class GardienZones:
    """Voir l'en-tête du module.

    site_map    : SiteMap (la version courante est relue à chaque pas).
    source_pose : fonction sans argument -> (x, y, cap), repère monde.
    """

    def __init__(self, site_map, source_pose: Callable[[], Tuple[float, float, float]], *,
                 rayon_m: float = 0.35, marge_m: float = 0.10, horizon_s: float = 0.8):
        self._carte = site_map
        self._pose = source_pose
        self.rayon = rayon_m + marge_m
        self.horizon = horizon_s
        self.stats = {"bloquees": 0, "ralenties": 0, "pose_invalide": 0}

    def limiter(self, vx: float, vy: float, wz: float) -> VerdictZones:
        """Consigne (repère du robot) -> consigne autorisée par les zones."""
        version = self._carte.current
        interdites = version.forbidden()
        lentes = [z for z in version.zones if z.kind == SLOW]
        if not interdites and not lentes:
            return VerdictZones(vx, vy, wz, "aucune zone")
        try:
            x, y, cap = (float(v) for v in self._pose())
        except Exception:                      # noqa: BLE001 — localisation en panne
            x = y = cap = math.nan
        if not all(math.isfinite(v) for v in (x, y, cap)):
            self.stats["pose_invalide"] += 1
            return VerdictZones(0.0, 0.0, 0.0, "position du robot inconnue : arrêt")
        if vx == 0.0 and vy == 0.0:
            return VerdictZones(vx, vy, wz, "rotation sur place")
        c, s = math.cos(cap), math.sin(cap)
        dx, dy = (vx * c - vy * s) * self.horizon, (vx * s + vy * c) * self.horizon
        n = max(2, int(math.hypot(dx, dy) / 0.05) + 1)
        for z in interdites:
            maintenant = z.distance(x, y)
            for k in range(1, n + 1):
                px, py = x + dx * k / n, y + dy * k / n
                d = z.distance(px, py)
                # Trop près au point prévu : interdit, sauf si l'on s'éloigne
                # d'une zone dont on est déjà trop près.
                if d < self.rayon and not (maintenant < self.rayon and d > maintenant):
                    self.stats["bloquees"] += 1
                    return VerdictZones(0.0, 0.0, wz, f"zone interdite « {z.name} » : "
                                                      f"translation refusée", z.name)
        plafond, nom = None, None
        for z in lentes:
            if z.distance(x, y) < self.rayon or z.distance(x + dx, y + dy) < self.rayon:
                if plafond is None or z.max_speed_mps < plafond:
                    plafond, nom = z.max_speed_mps, z.name
        if plafond is not None:
            v = math.hypot(vx, vy)
            if v > plafond:
                self.stats["ralenties"] += 1
                k = plafond / v
                return VerdictZones(vx * k, vy * k, wz, f"zone lente « {nom} » : "
                                                        f"{plafond:g} m/s au plus", nom)
        return VerdictZones(vx, vy, wz, "zones respectées")
