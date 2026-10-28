"""
AXIS-OS — Suivi d'une personne (« suis-moi »), dans l'OS.

Une app désigne une personne (son numéro temporaire, voir service.py) ;
l'OS la suit. La boucle tourne ici, à la cadence de la locomotion, et non
dans l'app : une app sandboxée ne garantit pas de répondre 50 fois par
seconde. Chaque consigne passe par le service de locomotion, donc par le
gardien de proximité et par le garde-fou des articulations : le suivi n'a
aucune exemption, il s'adapte au gardien, jamais l'inverse.

Règles, chacune testée (test/test_suivi_personne.py) :
  - distance : le robot garde la distance cible (1.5 m par défaut), toujours
    au-delà de la zone où le gardien l'arrête (GardienProximite.
    distance_arret) d'au moins MARGE_GARDIEN_M ; une distance plus faible
    est refusée. La distance retenue est la plus courte de deux mesures :
    la caméra (la personne suivie) et le lidar (l'obstacle le plus proche
    devant le robot) ;
  - vitesse : au plus MAX_SPEED (plus lent que la navigation), de plus en
    plus lent en approchant ; jamais en arrière, jamais de côté ;
  - obstacles de la carte : le gardien les ignore (ils sont « expliqués »,
    c'est la navigation qui les contourne) ; le suivi n'avance donc jamais
    si le point situé ANTICIPATION_M devant le robot est dans un obstacle
    de la carte, élargi comme pour la navigation ;
  - la personne s'arrête : le robot s'arrête à la distance cible ;
  - la personne disparaît : le robot s'arrête (HOLD_S), puis cherche en
    tournant sur place vers l'endroit où elle devait être, jusqu'à SEARCH_S
    après l'avoir vue ; ensuite LOST ;
  - une autre personne passe tout près : le robot s'immobilise ; si elle
    ressemble à la personne suivie (ou si on ne peut pas le savoir), le
    suivi s'arrête avec LOST plutôt que de risquer de suivre la mauvaise
    personne. Le suivi ne change JAMAIS de cible ; il rend LOST quand il
    doute ;
  - arrêt d'urgence, homme mort de l'app (plus de nouvelles depuis
    deadman_s), fin ou mort de l'app, demande d'arrêt : arrêt immédiat ;
  - à la fin du suivi, quelle qu'en soit l'issue, l'apparence de référence
    est effacée et le service « personnes » oublie toutes ses pistes.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from axis_os.hal.core.types import HALStatus
from axis_os.services.base import IService, ServiceHealth, ServiceResult, sous_verrou
from axis_os.store.sdk.manifest import Capability

from .appearance import appearance_distance
from .tracker import ACTIVE_MISS_S, APPEARANCE_GATE, REID_MARGIN, REID_MAX_S, REID_THRESHOLD


@dataclass(frozen=True)
class FollowOutcome:
    """Issue d'un suivi, ou refus de le commencer. `status` : STOPPED_BY_APP,
    LOST, TIMEOUT, ESTOP, CANCELLED, DEADMAN, FALLEN, REFUSED ; `code` : la
    RefusalCode (None pour STOPPED_BY_APP)."""
    status: str
    code: Optional[str]
    message: str
    person_id: Optional[int] = None
    reacquisitions: int = 0
    moved: bool = False             # le robot a réellement marché pendant ce suivi


class FollowService(IService):
    """Suivi d'une personne (voir l'en-tête du module)."""

    MAX_SPEED = 0.25            # m/s, sous les 0.30 de la navigation
    MAX_TURN = 0.5              # rad/s, sous les 0.6 de la navigation
    V_MIN = 0.06                # au-dessus de la bande morte de la marche
    W_MIN = 0.08
    GAIN_V = 0.5                # (m/s) par mètre d'écart à la distance cible
    GAIN_W = 1.0
    DISTANCE_DEFAULT = 1.5
    DISTANCE_MAX = 3.0
    MARGE_GARDIEN_M = 0.25
    BANDE_M = 0.10              # écart sous lequel le robot ne bouge pas
    REPRISE_M = 0.20            # écart au-delà duquel il repart (hystérésis)
    CAP_TOL = 0.10              # rad : en face
    TOURNER_SUR_PLACE = 0.6     # rad : au-delà, il tourne sans avancer
    SECTEUR_AVANT = 1.0         # rad : le lidar compte dans ce secteur
    DONNEES_MAX_S = 0.5         # cycle de détection plus vieux : arrêt
    HOLD_S = 1.0
    SEARCH_S = REID_MAX_S
    SEARCH_TURN = 0.35
    CROSS_RADIUS = 0.8
    SIMILAR = REID_THRESHOLD + REID_MARGIN
    MISMATCH = APPEARANCE_GATE
    MISMATCH_CYCLES = 3
    DEADMAN_MIN_S, DEADMAN_MAX_S = 0.3, 2.0
    ANTICIPATION_M = 0.4

    def __init__(self, people, locomotion, gardien=None, *,
                 source_pose: Optional[Callable[[], Tuple[float, float, float]]] = None,
                 horloge: Callable[[], float] = time.monotonic, navigation=None):
        """navigation : NavigationService du site, s'il y en a un ; sa carte
        (obstacles élargis) arrête l'avance du suivi devant une table ou une
        étagère. Sans carte, le gardien voit tout obstacle comme inexpliqué
        et arrête le robot de toute façon."""
        self._people = people
        self._nav = navigation
        self._loco = locomotion
        self._gardien = gardien
        self._pose = source_pose
        self._horloge = horloge
        self._verrou = threading.RLock()
        self._demarre = False
        self._proprio = None
        self._issue: Optional[FollowOutcome] = None
        self._issue_proprio = None
        self.stats = {"follows": 0, "lost": 0, "identity_doubts": 0, "reacquisitions": 0,
                      "max_vx": 0.0, "max_abs_wz": 0.0, "map_stops": 0}
        self._raz()

    def _raz(self) -> None:
        self._cible: Optional[int] = None
        self._reference = None
        self._distance = self.DISTANCE_DEFAULT
        self._echeance = 0.0
        self._deadman = 1.0
        self._battement = 0.0
        self._interruption: Optional[Callable[[], Optional[tuple]]] = None
        self._phase = "idle"
        self._avance = False
        self._reid_vus = 0
        self._dernier_cycle_vu: Optional[float] = None
        self._ecarts = 0
        self._doute = False
        self._invisible_depuis: Optional[float] = None
        self._reprises = 0
        self._debut = 0.0
        self._mesure: Optional[Tuple[float, float]] = None
        self._a_bouge = False

    # ---- IService ------------------------------------------------------------

    @property
    def service_id(self) -> str:
        return "follow"

    @property
    def capability(self) -> Capability:
        return Capability.WALK

    def start(self) -> HALStatus:
        self._demarre = True
        return HALStatus.OK

    def stop(self) -> None:
        with self._verrou:
            if self._cible is not None:
                self._finir("CANCELLED", "CANCELLED", "service de suivi arrêté")
            self._demarre = False

    def health(self) -> ServiceHealth:
        if not self._demarre:
            return ServiceHealth(running=False, healthy=False, detail="service non démarré")
        if self._gardien is None:
            return ServiceHealth(running=True, healthy=False,
                                 detail="pas de gardien de proximité : suivi refusé")
        return ServiceHealth(running=True, healthy=True, detail=self._phase)

    # ---- lecture -----------------------------------------------------------------

    @property
    def distance_min(self) -> Optional[float]:
        """Distance cible minimale (m) : la zone où le gardien arrête le robot,
        plus MARGE_GARDIEN_M ; None sans gardien (le suivi est alors
        refusé)."""
        if self._gardien is None:
            return None
        return round(self._gardien.distance_arret() + self.MARGE_GARDIEN_M, 3)

    @property
    def active(self) -> bool:
        return self._cible is not None

    def owner(self):
        return self._proprio if self._cible is not None else None

    # ---- commandes -----------------------------------------------------------

    @sous_verrou
    def suivre(self, person_id: int, *, distance_m: float, timeout_s: float,
               deadman_s: float, owner, interruption: Callable[[], Optional[tuple]]
               ) -> Optional[FollowOutcome]:
        """Commence à suivre la piste `person_id`. Rend None si le suivi
        commence, sinon un FollowOutcome REFUSED (son code dit pourquoi)."""
        def refus(code, msg):
            return FollowOutcome("REFUSED", code, msg, person_id)
        if not self._demarre:
            return refus("CAPABILITY_MISSING", "service de suivi non démarré")
        if self._gardien is None or not getattr(self._loco, "is_guarded", False):
            return refus("CAPABILITY_MISSING",
                         "pas de gardien de proximité : l'OS ne suit personne sans lui")
        if self._cible is not None and self._proprio is not owner:
            return refus("BUSY", "le robot suit déjà une personne pour une autre app")
        dmin = self.distance_min
        if not (math.isfinite(distance_m) and dmin <= distance_m <= self.DISTANCE_MAX):
            return refus("INVALID_ARGUMENT",
                         f"distance_m = {distance_m} hors de [{dmin}, {self.DISTANCE_MAX}] m")
        if not (self.DEADMAN_MIN_S <= deadman_s <= self.DEADMAN_MAX_S):
            return refus("INVALID_ARGUMENT", f"deadman_s = {deadman_s} hors de "
                                             f"[{self.DEADMAN_MIN_S}, {self.DEADMAN_MAX_S}] s")
        t = self._horloge()
        snap = self._people.track(int(person_id))
        if snap is None or not self._people.is_visible(snap, t):
            return refus("TARGET_LOST", f"personne n° {person_id} introuvable (numéro "
                                        f"inconnu, expiré, ou plus visible)")
        if self._people.relative(snap) is None:
            return refus("CAPABILITY_MISSING", "position du robot inconnue")
        if self._cible is not None:
            # Même app : le nouveau suivi remplace l'ancien.
            self._finir("STOPPED_BY_APP", None, "remplacé par un nouveau suivi", effacer=False)
        self._raz()
        self._cible = int(person_id)
        self._reference = snap.appearance
        self._distance = float(distance_m)
        self._debut = t
        self._echeance = t + float(timeout_s)
        self._deadman = float(deadman_s)
        self._battement = t
        self._interruption = interruption
        self._proprio = owner
        self._reid_vus = snap.reid_count
        self._dernier_cycle_vu = snap.last_seen
        self._phase = "following"
        self._issue = None
        self.stats["follows"] += 1
        return None

    @sous_verrou
    def battement(self, owner) -> None:
        """Homme mort : l'app donne de ses nouvelles (follow_status)."""
        if self._cible is not None and owner is self._proprio:
            self._battement = self._horloge()

    @sous_verrou
    def arreter(self, owner, status: str = "STOPPED_BY_APP", code: Optional[str] = None,
                message: str = "arrêté par l'app") -> Optional[FollowOutcome]:
        """Arrête le suivi de `owner` (s'il en a un) et rend son issue ; sinon
        la dernière issue de ce propriétaire, ou None."""
        if self._cible is not None and owner is self._proprio:
            self._finir(status, code, message)
        return self._issue if owner is self._issue_proprio else None

    @sous_verrou
    def status(self, owner) -> dict:
        """État du suivi de `owner` : en cours (FOLLOWING, avec la phase),
        sinon sa dernière issue, sinon IDLE."""
        if self._cible is not None and owner is self._proprio:
            d = self._mesure
            return {"status": "FOLLOWING", "phase": self._phase, "person_id": self._cible,
                    "distance_m": None if d is None else round(d[0], 3),
                    "bearing_rad": None if d is None else round(d[1], 3),
                    "target_distance_m": self._distance, "reacquisitions": self._reprises,
                    "elapsed_s": round(self._horloge() - self._debut, 3)}
        if self._issue is not None and owner is self._issue_proprio:
            i = self._issue
            return {"status": i.status, "code": i.code, "message": i.message,
                    "person_id": i.person_id, "reacquisitions": i.reacquisitions,
                    "moved": i.moved}
        return {"status": "IDLE"}

    # ---- boucle ------------------------------------------------------------------

    def _finir(self, status: str, code: Optional[str], message: str, effacer: bool = True) -> None:
        """Fin du suivi : arrêt en équilibre, issue gardée, apparence de
        référence effacée, et le service « personnes » oublie ses pistes et
        cesse de regarder (jusqu'à ce qu'une app le lui redemande)."""
        self._loco.halt()
        self._issue = FollowOutcome(status, code, message, self._cible, self._reprises,
                                    self._a_bouge)
        self._issue_proprio = self._proprio
        if status == "LOST":
            self.stats["lost"] += 1
        self._reference = None
        self._raz()
        self._proprio = None
        if effacer:
            self._people.release()

    def _devant_libre(self) -> bool:
        """Faux si le point ANTICIPATION_M devant le robot est dans un
        obstacle de la carte (élargi du rayon du robot et d'une marge)."""
        planif = getattr(self._nav, "planif", None)
        if planif is None or self._pose is None:
            return True
        x, y, cap = self._pose()
        return planif.libre(x + self.ANTICIPATION_M * math.cos(cap),
                            y + self.ANTICIPATION_M * math.sin(cap))

    def _commander(self, vx: float, wz: float) -> None:
        """Seule sortie vers la locomotion : jamais en arrière, jamais de
        côté, jamais au-delà des bornes du suivi, jamais dans un obstacle de
        la carte."""
        vx = min(self.MAX_SPEED, max(0.0, vx))
        if vx > 0.0 and not self._devant_libre():
            vx = 0.0
            self.stats["map_stops"] += 1
        wz = max(-self.MAX_TURN, min(self.MAX_TURN, wz))
        self.stats["max_vx"] = max(self.stats["max_vx"], vx)
        self.stats["max_abs_wz"] = max(self.stats["max_abs_wz"], abs(wz))
        self._a_bouge = self._a_bouge or vx > 0.0 or wz != 0.0
        self._loco.set_velocity(vx, 0.0, wz)

    def _franche(self, v: float, vmin: float, vmax: float) -> float:
        if abs(v) < 1e-9:
            return 0.0
        return math.copysign(min(vmax, max(vmin, abs(v))), v)

    def _distance_lidar_avant(self) -> Optional[float]:
        """Distance de l'obstacle le plus proche vu par le gardien, s'il est
        dans le secteur avant du robot."""
        v = getattr(self._loco, "dernier_verdict", None)
        if v is None or v.distance is None or v.point is None or self._pose is None:
            return None
        if not math.isfinite(v.distance):
            return None
        rx, ry, cap = self._pose()
        b = math.atan2(v.point[1] - ry, v.point[0] - rx) - cap
        b = (b + math.pi) % (2 * math.pi) - math.pi
        return float(v.distance) if abs(b) <= self.SECTEUR_AVANT else None

    def _autres_proches(self, cible, t: float):
        """Autres personnes visibles à moins de CROSS_RADIUS de la personne
        suivie (ou de sa position prédite si elle est cachée)."""
        dt = max(0.0, t - cible.last_seen)
        px, py = cible.x + cible.vx * dt, cible.y + cible.vy * dt
        return [s for s in self._people.snapshots()
                if s.track_id != cible.track_id and self._people.is_visible(s, t)
                and math.hypot(s.x - px, s.y - py) < self.CROSS_RADIUS]

    @sous_verrou
    def tick(self) -> ServiceResult:
        """Un pas du suivi, à la cadence de la locomotion."""
        if self._cible is None:
            return ServiceResult(ok=True, value="idle")
        t = self._horloge()
        try:
            motif = self._interruption() if self._interruption is not None else None
        except Exception as exc:        # noqa: BLE001 — dans le doute, arrêter
            motif = ("CANCELLED", f"vérification impossible ({type(exc).__name__})")
        if motif is not None:
            code = getattr(motif[0], "value", motif[0])
            self._finir("ESTOP" if code == "ESTOP_ACTIVE" else "CANCELLED", code,
                        f"suivi interrompu : {motif[1]}")
            return ServiceResult(ok=False, reason=self._issue.message)
        if getattr(getattr(self._loco, "state", None), "name", "") == "FALLEN":
            self._finir("FALLEN", "SAFETY_LIMIT", "chute détectée")
            return ServiceResult(ok=False, reason="chute détectée")
        if t - self._battement > self._deadman:
            self._finir("DEADMAN", "DEADMAN", f"aucune nouvelle de l'app depuis "
                                              f"{t - self._battement:.2f} s (homme mort)")
            return ServiceResult(ok=False, reason=self._issue.message)
        if t > self._echeance:
            self._finir("TIMEOUT", "TIMEOUT", "délai du suivi dépassé")
            return ServiceResult(ok=False, reason="délai dépassé")
        # Le service « personnes » regarde tant que le suivi dure.
        self._people.demander()
        cible = self._people.track(self._cible)
        if cible is None:
            return self._perdue("personne oubliée par le suivi des pistes")
        dernier = self._people.last_cycle
        if dernier is None or t - dernier > self.DONNEES_MAX_S:
            # Détection muette : on ne bouge plus. Le temps sans voir la
            # personne continue de courir.
            self._loco.halt()
            self._phase = "waiting"
            if t - cible.last_seen > self.SEARCH_S:
                return self._perdue("plus de détection")
            return ServiceResult(ok=True, value="waiting")
        visible = self._people.is_visible(cible, t)
        nouveau_cycle = cible.last_seen != self._dernier_cycle_vu
        self._dernier_cycle_vu = cible.last_seen

        # Une autre personne tout près : on s'immobilise, et on doute si
        # elle ressemble à la personne suivie (ou si on ne peut pas le dire).
        proches = self._autres_proches(cible, t)
        for autre in proches:
            d = appearance_distance(autre.appearance, self._reference)
            if d is None or d <= self.SIMILAR:
                if not self._doute:
                    self.stats["identity_doubts"] += 1
                self._doute = True
        if proches:
            self._loco.halt()
            self._phase = "holding"
            self._avance = False
            if not visible and t - cible.last_seen > self.SEARCH_S:
                return self._perdue("cachée par une autre personne trop longtemps")
            return ServiceResult(ok=True, value="holding")
        if self._doute:
            # Les deux se sont séparées : on ne continue que si la personne
            # suivie se distingue clairement de toutes les autres visibles.
            if not self._identite_claire(cible, t):
                return self._perdue("identité incertaine : une personne d'apparence proche "
                                    "est passée tout près")
            self._doute = False

        if not visible:
            if self._invisible_depuis is None:
                self._invisible_depuis = cible.last_seen
            perdue_depuis = t - cible.last_seen
            if perdue_depuis > self.SEARCH_S:
                return self._perdue(f"personne non vue depuis {perdue_depuis:.1f} s")
            self._avance = False
            if perdue_depuis <= self.HOLD_S:
                self._loco.halt()
                self._phase = "waiting"
                return ServiceResult(ok=True, value="waiting")
            # Recherche : tourner sur place vers là où elle devait être.
            rel = self._relatif(cible.x + cible.vx * perdue_depuis,
                                cible.y + cible.vy * perdue_depuis)
            sens = 0.0 if rel is None or abs(rel[1]) < self.CAP_TOL else math.copysign(1.0, rel[1])
            self._commander(0.0, sens * self.SEARCH_TURN)
            self._phase = "searching"
            return ServiceResult(ok=True, value="searching")

        # Visible. Reprise après une absence : vérifier que c'est bien elle.
        if self._invisible_depuis is not None:
            absence = cible.last_seen - self._invisible_depuis
            self._invisible_depuis = None
            repris = cible.reid_count > self._reid_vus or absence > ACTIVE_MISS_S
            if repris:
                self._reprises += 1
                self.stats["reacquisitions"] += 1
                d = appearance_distance(cible.appearance, self._reference)
                if d is None or d > REID_THRESHOLD:
                    return self._perdue("reprise sans apparence vérifiable")
        self._reid_vus = cible.reid_count
        if nouveau_cycle:
            d = appearance_distance(cible.appearance, self._reference)
            if d is not None and d > self.MISMATCH:
                self._ecarts += 1
                if self._ecarts >= self.MISMATCH_CYCLES:
                    return self._perdue("l'apparence de la personne suivie ne correspond plus")
            else:
                self._ecarts = 0
                if d is not None and d <= REID_THRESHOLD and self._reference is not None:
                    # Lente mise à jour (éclairage qui change), seulement
                    # quand personne n'est à côté.
                    self._reference = tuple(0.9 * r + 0.1 * c for r, c in
                                            zip(self._reference, cible.appearance))
        rel = self._people.relative(cible)
        if rel is None:
            self._loco.halt()
            return ServiceResult(ok=False, reason="position du robot inconnue")
        d_vue, cap = rel
        self._mesure = rel
        d_lidar = self._distance_lidar_avant()
        d = d_vue if d_lidar is None else min(d_vue, d_lidar)
        wz = 0.0 if abs(cap) < self.CAP_TOL else self._franche(
            self.GAIN_W * cap, self.W_MIN, self.MAX_TURN)
        ecart = d - self._distance
        if abs(cap) > self.TOURNER_SUR_PLACE:
            self._avance = False
        elif self._avance:
            self._avance = ecart >= self.BANDE_M
        else:
            self._avance = ecart >= self.REPRISE_M
        vx = 0.0
        if self._avance:
            vmax = min(self.MAX_SPEED, getattr(getattr(self._loco, "max_twist", None), "vx",
                                               self.MAX_SPEED))
            vx = self._franche(self.GAIN_V * ecart * max(0.0, math.cos(cap)), self.V_MIN, vmax)
        self._commander(vx, wz)
        self._phase = "following"
        return ServiceResult(ok=True, value="following")

    def _relatif(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        if self._pose is None:
            return None
        rx, ry, cap = self._pose()
        b = math.atan2(y - ry, x - rx) - cap
        return math.hypot(x - rx, y - ry), (b + math.pi) % (2 * math.pi) - math.pi

    def _identite_claire(self, cible, t: float) -> bool:
        """Vrai si la personne suivie ressemble à la référence ET se
        distingue nettement de toute autre personne visible."""
        d = appearance_distance(cible.appearance, self._reference)
        if d is None or d > REID_THRESHOLD:
            return False
        for s in self._people.snapshots():
            if s.track_id == cible.track_id or not self._people.is_visible(s, t):
                continue
            da = appearance_distance(s.appearance, self._reference)
            if da is None or da <= d + REID_MARGIN:
                return False
        return True

    def _perdue(self, raison: str) -> ServiceResult:
        self._finir("LOST", "TARGET_LOST", raison)
        return ServiceResult(ok=False, reason=raison)
