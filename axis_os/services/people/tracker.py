"""
AXIS-OS — Suivi multi-personnes (« tracking ») et ré-identification courte.

Le suivi donne à chaque personne vue un NUMÉRO, stable tant qu'elle reste
visible. Ce numéro est temporaire : il ne désigne personne en dehors de
cette piste, n'est jamais réutilisé, et disparaît avec elle.

Trois pièces, écrites ici (numpy seul, licence du dépôt) :

  Filtre de mouvement  — un filtre de Kalman à vitesse constante (la
                         méthode classique pour prédire où sera un objet qui
                         bouge, à partir de mesures bruitées), dans le repère
                         MONDE, au sol : la piste ne saute pas quand le
                         robot tourne la tête.
  Association          — à la manière de ByteTrack (Zhang et al., 2022) :
                         les détections sûres d'abord, puis les détections
                         peu sûres pour les pistes restées sans mesure (une
                         personne à moitié cachée). Une détection n'est
                         rattachée à une piste que si elle est assez près de
                         sa position prédite ET si les apparences ne se
                         contredisent pas : une personne habillée autrement
                         qui passe tout près ne prend jamais le numéro d'une
                         autre.
  Ré-identification    — une piste sans mesure depuis plus de
  courte                 ACTIVE_MISS_S est « perdue ». Pendant REID_MAX_S au
                         plus, une nouvelle détection peut la reprendre, si
                         son apparence globale (appearance.py : couleurs des
                         vêtements et silhouette, jamais le visage) est très
                         proche, si la distance parcourue est possible pour
                         un humain, et si AUCUNE autre piste ou détection ne
                         lui ressemble autant. Dans le doute, on ne reprend
                         pas : mieux vaut perdre quelqu'un que suivre la
                         mauvaise personne.

Mémoire bornée : au plus MAX_TRACKS pistes, et une piste perdue est oubliée,
apparence comprise, REID_MAX_S secondes après la dernière fois que la
personne a été vue.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .appearance import appearance_distance

# Ré-identification : quelques secondes au plus, puis on oublie.
REID_MAX_S = 5.0
# Sans mesure depuis plus longtemps : piste perdue (ré-identification seule).
ACTIVE_MISS_S = 0.6
# Détections nécessaires pour confirmer une piste (une fausse détection
# isolée ne devient pas une personne).
MIN_HITS = 2
# Seuils de score (ByteTrack) : sûres au-dessus de HIGH, peu sûres entre
# LOW et HIGH, ignorées en dessous.
HIGH_SCORE = 0.5
LOW_SCORE = 0.25
# Association : rayon de base autour de la position prédite (m), allongé du
# chemin qu'un humain peut faire pendant le temps sans mesure.
GATE_M = 0.8
GATE_MAX_M = 2.0
HUMAN_MAX_SPEED = 2.0          # m/s : un pas rapide
# Apparences trop différentes pour la même personne (association).
APPEARANCE_GATE = 0.45
# Ré-identification : apparence très proche exigée, et aucune autre
# candidate à moins de REID_MARGIN de ce seuil.
REID_THRESHOLD = 0.25
REID_MARGIN = 0.10
# Rayon (m) sous lequel deux détections sont « l'une contre l'autre » :
# l'apparence n'est alors pas mise à jour (elle mélangerait les deux).
CROWD_RADIUS = 1.0
MAX_TRACKS = 32
# Bruit du filtre : accélération d'un humain (m/s²) et précision de la
# position mesurée, qui se dégrade avec la distance.
ACCEL_SIGMA = 1.5


def mesure_sigma(portee_m: float) -> float:
    """Écart type (m) d'une position mesurée à `portee_m` de la caméra."""
    return 0.10 + 0.05 * max(0.0, portee_m)


@dataclass
class Observation:
    """Une détection située au sol, dans le repère monde."""
    xy: Tuple[float, float]
    score: float
    appearance: Optional[np.ndarray] = None
    range_m: float = 2.0
    bbox: Optional[Tuple[float, float, float, float]] = None


@dataclass
class Track:
    """Une piste : une personne suivie. `state` = (x, y, vx, vy), repère
    monde ; `cov` : sa covariance (l'incertitude du filtre)."""
    track_id: int
    state: np.ndarray
    cov: np.ndarray
    first_seen: float
    last_seen: float
    last_t: float
    hits: int = 1
    confirmed: bool = False
    appearance: Optional[np.ndarray] = None
    score: float = 0.0
    bbox: Optional[Tuple[float, float, float, float]] = None
    reid_count: int = 0
    last_reid: Optional[float] = None


@dataclass(frozen=True)
class TrackSnapshot:
    """Copie d'une piste confirmée, à un instant donné, pour les autres
    services de l'OS. `appearance` est une copie : le service de suivi
    (follow.py) s'en sert pour vérifier qu'il suit toujours la même
    personne ; elle ne sort jamais de l'OS."""
    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    first_seen: float
    last_seen: float
    score: float
    reid_count: int
    last_reid: Optional[float]
    appearance: Optional[Tuple[float, ...]] = field(default=None, repr=False)


class PersonTracker:
    """Suivi multi-personnes : voir l'en-tête du module. Le temps est passé
    à chaque update() (secondes, horloge du robot)."""

    def __init__(self):
        self._tracks: Dict[int, Track] = {}
        self._next_id = 1
        self.stats = {"tracks_created": 0, "tracks_confirmed": 0, "tracks_lost": 0,
                      "reid": 0, "reid_ambiguous": 0, "saturated": 0}

    # ---- filtre de Kalman ---------------------------------------------------

    @staticmethod
    def _predire(tr: Track, t: float) -> None:
        dt = max(0.0, t - tr.last_t)
        if dt <= 0.0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        tr.state = F @ tr.state
        tr.cov = F @ tr.cov @ F.T + (ACCEL_SIGMA ** 2) * (G @ G.T)
        v = math.hypot(tr.state[2], tr.state[3])
        vmax = 1.25 * HUMAN_MAX_SPEED
        if v > vmax:                        # jamais plus vite qu'un humain
            tr.state[2:] *= vmax / v
        tr.last_t = t

    @staticmethod
    def _corriger(tr: Track, obs: Observation) -> None:
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1.0
        R = (mesure_sigma(obs.range_m) ** 2) * np.eye(2)
        z = np.asarray(obs.xy, dtype=float)
        S = H @ tr.cov @ H.T + R
        K = tr.cov @ H.T @ np.linalg.inv(S)
        tr.state = tr.state + K @ (z - H @ tr.state)
        tr.cov = (np.eye(4) - K @ H) @ tr.cov

    @staticmethod
    def _neuve_cov(obs: Observation) -> np.ndarray:
        s = mesure_sigma(obs.range_m)
        return np.diag([s * s, s * s, 1.0, 1.0])

    # ---- association ---------------------------------------------------------

    @staticmethod
    def _apparence_compatible(tr: Track, obs: Observation, seuil: float) -> Tuple[bool, float]:
        d = appearance_distance(tr.appearance, obs.appearance)
        if d is None:
            return True, 0.0
        return d <= seuil, d

    def _associer(self, pistes: List[Track], obs: List[Observation], idx: List[int],
                  t: float) -> List[Tuple[int, int]]:
        """Appariement glouton par coût croissant (distance à la position
        prédite, plus l'écart d'apparence), sous les deux portes. Rend des
        couples (indice de piste dans `pistes`, indice d'observation)."""
        couples = []
        for a, tr in enumerate(pistes):
            porte = min(GATE_MAX_M, GATE_M + HUMAN_MAX_SPEED * max(0.0, t - tr.last_seen))
            for i in idx:
                o = obs[i]
                d = math.hypot(o.xy[0] - tr.state[0], o.xy[1] - tr.state[1])
                if d > porte:
                    continue
                ok, da = self._apparence_compatible(tr, o, APPEARANCE_GATE)
                if not ok:
                    continue
                couples.append((d / porte + 0.5 * da, a, i))
        couples.sort()
        prises_a, prises_i, out = set(), set(), []
        for _, a, i in couples:
            if a in prises_a or i in prises_i:
                continue
            prises_a.add(a)
            prises_i.add(i)
            out.append((a, i))
        return out

    def _reidentifier(self, perdues: List[Track], obs: List[Observation], idx: List[int],
                      t: float) -> List[Tuple[int, int]]:
        """Ré-identification courte : voir l'en-tête du module. Un couple
        n'est retenu que s'il est sans rival, dans les deux sens."""
        def candidat(tr: Track, o: Observation, seuil: float) -> Optional[float]:
            if tr.appearance is None or o.appearance is None:
                return None              # sans apparence : jamais de reprise
            portee = HUMAN_MAX_SPEED * max(0.0, t - tr.last_seen) + 0.5
            if math.hypot(o.xy[0] - tr.state[0], o.xy[1] - tr.state[1]) > portee:
                return None
            d = appearance_distance(tr.appearance, o.appearance)
            return d if d is not None and d <= seuil else None

        out, prises = [], set()
        relache = REID_THRESHOLD + REID_MARGIN
        for a, tr in enumerate(perdues):
            meilleurs = [(candidat(tr, obs[i], REID_THRESHOLD), i) for i in idx if i not in prises]
            meilleurs = [(d, i) for d, i in meilleurs if d is not None]
            if not meilleurs:
                continue
            d, i = min(meilleurs)
            rivales_obs = [j for j in idx if j != i and candidat(tr, obs[j], relache) is not None]
            rivales_pistes = [b for b, autre in enumerate(perdues)
                              if b != a and candidat(autre, obs[i], relache) is not None]
            if rivales_obs or rivales_pistes:
                self.stats["reid_ambiguous"] += 1
                continue
            prises.add(i)
            out.append((a, i))
        return out

    # ---- mise à jour -----------------------------------------------------------

    def update(self, observations: Sequence[Observation], t: float) -> None:
        """Un cycle de détection à l'instant `t` : prédiction, association
        en deux temps, ré-identification, création et oubli des pistes."""
        obs = [o for o in observations
               if o.score >= LOW_SCORE and all(math.isfinite(v) for v in o.xy)]
        for tr in self._tracks.values():
            self._predire(tr, t)
        actives = [tr for tr in self._tracks.values() if t - tr.last_seen <= ACTIVE_MISS_S]
        perdues = [tr for tr in self._tracks.values()
                   if tr.confirmed and ACTIVE_MISS_S < t - tr.last_seen <= REID_MAX_S]
        sures = [i for i, o in enumerate(obs) if o.score >= HIGH_SCORE]
        faibles = [i for i, o in enumerate(obs) if o.score < HIGH_SCORE]

        mesurees = []
        for a, i in self._associer(actives, obs, sures, t):
            mesurees.append((actives[a], i))
        restantes = [tr for tr in actives if all(tr is not m for m, _ in mesurees)]
        for a, i in self._associer(restantes, obs, faibles, t):
            mesurees.append((restantes[a], i))
        utilisees = {i for _, i in mesurees}
        libres = [i for i in sures if i not in utilisees]
        for a, i in self._reidentifier(perdues, obs, libres, t):
            tr, o = perdues[a], obs[i]
            tr.state = np.array([o.xy[0], o.xy[1], 0.0, 0.0])
            tr.cov = self._neuve_cov(o)
            tr.reid_count += 1
            tr.last_reid = t
            self.stats["reid"] += 1
            mesurees.append((tr, i))
            utilisees.add(i)
            # La piste est reprise : ne pas la corriger une seconde fois.
            tr.hits += 1
            tr.last_seen = tr.last_t = t
            tr.score = o.score
            tr.bbox = o.bbox

        for tr, i in mesurees:
            o = obs[i]
            if tr.last_seen != t:
                self._corriger(tr, o)
                tr.hits += 1
                tr.last_seen = t
                tr.score = o.score
                tr.bbox = o.bbox
            if not tr.confirmed and tr.hits >= MIN_HITS:
                tr.confirmed = True
                self.stats["tracks_confirmed"] += 1
            isolee = all(math.hypot(obs[j].xy[0] - o.xy[0], obs[j].xy[1] - o.xy[1]) > CROWD_RADIUS
                         for j in range(len(obs)) if j != i)
            if o.appearance is not None and isolee:
                if tr.appearance is None:
                    tr.appearance = np.asarray(o.appearance, dtype=float).copy()
                else:
                    tr.appearance = 0.8 * tr.appearance + 0.2 * np.asarray(o.appearance, dtype=float)

        for i in sures:
            if i in utilisees:
                continue
            if len(self._tracks) >= MAX_TRACKS:
                self.stats["saturated"] += 1
                continue
            o = obs[i]
            tr = Track(track_id=self._next_id, state=np.array([o.xy[0], o.xy[1], 0.0, 0.0]),
                       cov=self._neuve_cov(o), first_seen=t, last_seen=t, last_t=t,
                       appearance=(None if o.appearance is None
                                   else np.asarray(o.appearance, dtype=float).copy()),
                       score=o.score, bbox=o.bbox)
            self._next_id += 1
            self._tracks[tr.track_id] = tr
            self.stats["tracks_created"] += 1

        for tid, tr in list(self._tracks.items()):
            vu_il_y_a = t - tr.last_seen
            if (not tr.confirmed and vu_il_y_a > 0.25) or vu_il_y_a > REID_MAX_S:
                if tr.confirmed:
                    self.stats["tracks_lost"] += 1
                tr.appearance = None             # oubli explicite
                del self._tracks[tid]

    def forget_all(self) -> None:
        """Oublie toutes les pistes et leurs apparences. Les numéros déjà
        donnés ne sont jamais réutilisés."""
        for tr in self._tracks.values():
            tr.appearance = None
        self._tracks.clear()

    def snapshots(self) -> List[TrackSnapshot]:
        """Copies des pistes confirmées (visibles ou perdues depuis moins de
        REID_MAX_S)."""
        return [TrackSnapshot(
            track_id=tr.track_id, x=float(tr.state[0]), y=float(tr.state[1]),
            vx=float(tr.state[2]), vy=float(tr.state[3]), first_seen=tr.first_seen,
            last_seen=tr.last_seen, score=tr.score, reid_count=tr.reid_count,
            last_reid=tr.last_reid,
            appearance=None if tr.appearance is None else tuple(float(v) for v in tr.appearance))
            for tr in self._tracks.values() if tr.confirmed]

    def __len__(self) -> int:
        return len(self._tracks)
