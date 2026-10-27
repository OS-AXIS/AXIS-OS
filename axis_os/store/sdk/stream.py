"""
AXIS-OS — Flux de consignes de téléopération locale (axis_os.store.sdk.stream)
==============================================================================
ctx.stream_targets("arms" | "walk", ...) ouvre un flux : l'app y pousse,
à son rythme, des cibles d'articulations des bras (horodatées) ou des
vitesses de marche, typiquement lues sur une manette branchée au robot.
LOCAL seulement : le flux passe par le pont JSON de l'app, jamais par le
réseau.

Ce que l'OS garantit, quoi que fasse l'app (TargetStreamEngine, côté OS) :
  - chaque consigne trop vieille (STALE_TARGET), trop fréquente
    (RATE_LIMITED) ou hors bornes (OUT_OF_BOUNDS) est jetée, et comptée ;
  - vitesse propre au flux, plus basse que celle du robot : bras à
    `max_speed` rad/s au plus (MAX_ARM_SPEED_RADPS), marche à `max_speed`
    fois la vitesse maximale du robot (MAX_WALK_RATIO) ;
  - homme mort : chaque consigne porte `deadman`. Relâché (False), ou plus
    de consigne valide depuis `deadman_ms`, et c'est l'arrêt sûr : la
    marche ralentit puis s'arrête (le contrôleur de marche garde
    l'équilibre, jamais de coupure des jambes), les bras s'arrêtent là où
    ils sont. Une perte du flux ou un retard excessif (consignes toutes
    périmées) mène donc au même arrêt ;
  - après un arrêt sûr, plus rien ne passe tant que l'app n'a pas demandé
    explicitement la reprise (resume()) ;
  - bras seulement, jamais les jambes : un bipède debout ne se pilote pas
    articulation par articulation ;
  - chaque pas passe par les mêmes contrôles qu'une commande directe :
    move_arms() (permissions, arrêt d'urgence, JointSafetyGuard) par
    SetpointFollower, ou walk() (gardien de proximité, vitesse maximale).

Mesures (metrics()) : consignes reçues, acceptées, rejetées par motif,
latence (âge d'une consigne à son arrivée), gigue (écart type de
l'intervalle entre deux consignes acceptées), arrêts sûrs.

Licence : Apache 2.0 (open-source — SDK public)
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

from .results import (
    EXCEPTIONS_PAR_CODE, ActionResult, AxisRefusal, InvalidArgumentError, RefusalCode,
    refused, require_number,
)

STREAM_KINDS = ("arms", "walk")
# Bornes des réglages d'un flux.
DEADMAN_MS = (50.0, 1000.0)
# Marche : pas plus que le délai de validité d'une consigne du service de
# locomotion (0.5 s) ; sinon le flux ferait rouler le robot plus longtemps
# sans consigne que walk() (constaté par une revue adversariale : 1.07 s).
DEADMAN_WALK_MAX_MS = 500.0
MAX_AGE_MS = (10.0, 1000.0)
MAX_RATE_HZ = 100.0
MAX_ARM_SPEED_RADPS = 2.0
MAX_WALK_RATIO = 0.5
MAX_JOINTS = 32
# Décélération de l'arrêt sûr de la marche (m/s² en translation, rad/s² en
# rotation) : de la vitesse maximale du flux à l'arrêt en moins d'une
# demi-seconde, sans à-coup.
DECEL_LIN = 1.0
DECEL_ROT = 2.0


class CapabilityMissingError(AxisRefusal):
    """Le robot n'a pas ce qu'il faut (pas de bras, pas de marche)."""
    code = RefusalCode.CAPABILITY_MISSING


class BusyError(AxisRefusal):
    """Un flux est déjà ouvert par ce contexte."""
    code = RefusalCode.BUSY


EXCEPTIONS_PAR_CODE[RefusalCode.CAPABILITY_MISSING.value] = CapabilityMissingError
EXCEPTIONS_PAR_CODE[RefusalCode.BUSY.value] = BusyError


def stream_params(kind: Any, deadman_ms: Any, max_rate_hz: Any, max_age_ms: Any,
                  max_speed: Any) -> Dict[str, Any]:
    """Réglages validés d'un flux, ou InvalidArgumentError."""
    if kind not in STREAM_KINDS:
        raise InvalidArgumentError(f"kind : {STREAM_KINDS[0]!r} ou {STREAM_KINDS[1]!r}")
    vmax = MAX_ARM_SPEED_RADPS if kind == "arms" else MAX_WALK_RATIO
    if max_speed is None:
        max_speed = 1.0 if kind == "arms" else 0.3
    return {
        "kind": kind,
        "deadman_ms": require_number("deadman_ms", deadman_ms, mini=DEADMAN_MS[0],
                                     maxi=DEADMAN_MS[1] if kind == "arms"
                                     else DEADMAN_WALK_MAX_MS),
        "max_rate_hz": require_number("max_rate_hz", max_rate_hz, mini=1.0, maxi=MAX_RATE_HZ),
        "max_age_ms": require_number("max_age_ms", max_age_ms, mini=MAX_AGE_MS[0],
                                     maxi=MAX_AGE_MS[1]),
        "max_speed": require_number("max_speed", max_speed, mini=0.0, maxi=vmax,
                                    strict_min=True),
    }


def stream_target(kind: str, target: Any):
    """Cible validée (types et valeurs finies) : dict {articulation:
    position} pour « arms », (vx, vy, wz) pour « walk ». Les bornes, elles,
    sont vérifiées par l'OS (rejet OUT_OF_BOUNDS, compté)."""
    if kind == "arms":
        if not isinstance(target, dict) or not target or len(target) > MAX_JOINTS:
            raise InvalidArgumentError(f"cible des bras : un dict non vide d'au plus "
                                       f"{MAX_JOINTS} articulations")
        sortie = {}
        for jid, pos in target.items():
            if not isinstance(jid, str) or len(jid) > 128:
                raise InvalidArgumentError("nom d'articulation : un texte court")
            sortie[jid] = require_number(f"cible[{jid}]", pos)
        return sortie
    if not isinstance(target, (list, tuple)) or len(target) != 3:
        raise InvalidArgumentError("cible de marche : (vx, vy, wz)")
    return tuple(require_number(n, v) for n, v in zip(("vx", "vy", "wz"), target))


@dataclass
class StreamAck:
    """Réponse à une consigne poussée : acceptée ou non (et pourquoi),
    état du flux (ACTIVE, STOPPING, STOPPED, CLOSED)."""
    accepted: bool
    code: Optional[str]
    state: str

    def to_json(self) -> Dict[str, Any]:
        return {"accepted": self.accepted, "code": self.code, "state": self.state}

    @classmethod
    def from_json(cls, d: Dict[str, Any]) -> "StreamAck":
        return cls(accepted=d.get("accepted") is True, code=d.get("code"),
                   state=str(d.get("state", "")))


@dataclass
class StreamMetrics:
    """Mesures d'un flux. Latence : âge d'une consigne acceptée à son
    arrivée dans l'OS ; gigue : écart type de l'intervalle entre deux
    consignes acceptées (millisecondes)."""
    state: str
    received: int = 0
    accepted: int = 0
    rejected: Dict[str, int] = field(default_factory=dict)
    latency_ms_mean: float = 0.0
    latency_ms_max: float = 0.0
    jitter_ms: float = 0.0
    safe_stops: int = 0
    stop_reason: str = ""

    def to_json(self) -> Dict[str, Any]:
        return dict(self.__dict__, rejected=dict(self.rejected))

    @classmethod
    def from_json(cls, d: Dict[str, Any]) -> "StreamMetrics":
        champs = {k: d[k] for k in cls.__dataclass_fields__ if k in d}
        champs["rejected"] = dict(champs.get("rejected") or {})
        return cls(**champs)


class TargetStream:
    """Le flux, côté app (rendu par ctx.stream_targets). Même objet en
    direct et dans le processus de l'app : seul son contexte change.

        with ctx.stream_targets("walk", deadman_ms=200) as flux:
            while manette.connectee():
                ack = flux.push(manette.vitesse(), deadman=manette.bouton())
                ctx.sleep(0.02)
    """

    def __init__(self, ctx, stream_id: int, kind: str, info: Dict[str, Any]):
        self._ctx = ctx
        self._id = stream_id
        self.kind = kind
        # Articulations pilotables (« arms ») ; bornes appliquées par l'OS.
        self.joints = tuple(info.get("joints") or ())
        self.limits = dict(info.get("limits") or {})

    def push(self, target, *, deadman: bool, t: Optional[float] = None) -> StreamAck:
        """Pousse une consigne : dict {articulation: position} (« arms »)
        ou (vx, vy, wz) (« walk »). `deadman` : vrai tant que l'opérateur
        tient le bouton homme mort. `t` : heure du robot (ctx.now()) où la
        consigne a été lue ; par défaut, maintenant. Lève
        InvalidArgumentError (types, valeurs non finies) et
        EstopActiveError sous arrêt d'urgence."""
        if not isinstance(deadman, bool):
            raise InvalidArgumentError("deadman doit être un booléen")
        cible = stream_target(self.kind, target)
        if t is not None:
            t = require_number("t", t)
        return self._ctx._flux_push(self._id, cible, deadman, t)

    def resume(self) -> ActionResult:
        """Reprend après un arrêt sûr (status RESUMED). Refusé sous arrêt
        d'urgence (ESTOP_ACTIVE) et sur un flux fermé (STREAM_STOPPED). La
        consigne suivante doit porter deadman=True."""
        return self._ctx._flux_resume(self._id)

    def metrics(self) -> StreamMetrics:
        return self._ctx._flux_metrics(self._id)

    def close(self) -> None:
        """Arrêt sûr puis fermeture. Sans effet si déjà fermé."""
        self._ctx._flux_close(self._id)

    def __enter__(self) -> "TargetStream":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class TargetStreamEngine:
    """Le flux, côté OS : vérifie chaque consigne et pilote le robot à
    PERIOD_S (voir l'en-tête du module). `ctx` est le RobotContext de l'app,
    dans le processus de l'OS : tout passe par ses méthodes, donc par ses
    contrôles."""

    PERIOD_S = 0.02

    def __init__(self, ctx, params: Dict[str, Any],
                 joints: Optional[Dict[str, Tuple[float, float]]] = None,
                 max_twist=None):
        self._ctx = ctx
        self.kind = params["kind"]
        self._deadman_s = params["deadman_ms"] / 1000.0
        self._age_max_s = params["max_age_ms"] / 1000.0
        self._rate = params["max_rate_hz"]
        self._joints = dict(joints or {})
        self.state = "ACTIVE"
        self._verrou = threading.Lock()
        self._dernier_ok: Optional[float] = None     # heure de la dernière consigne valide
        self._acceptees_1s: deque = deque()
        self._vitesse = (0.0, 0.0, 0.0)
        self.m = StreamMetrics(state="ACTIVE")
        self._lat_total = 0.0
        self._t_prec: Optional[float] = None
        self._n_int, self._moy_int, self._m2_int = 0, 0.0, 0.0
        if self.kind == "arms":
            from axis_os.services.policy.setpoint_follower import SetpointFollower
            pas = params["max_speed"] * self.PERIOD_S
            # SetpointFollower avance d'au plus delta_max × safety_factor par
            # pas : delta_max = pas / safety_factor donne exactement `pas`.
            # damping_after_s infini : jamais de relâchement automatique,
            # qui enverrait aussi des consignes aux jambes.
            self._suiveur = SetpointFollower(
                ctx, {j: pas / 0.5 for j in self._joints}, step_s=self.PERIOD_S,
                max_age_s=self._deadman_s, damping_after_s=float("inf"), safety_factor=0.5)
            self.limits = {"max_speed_radps": params["max_speed"]}
        else:
            self._suiveur = None
            r = params["max_speed"]
            self._vmax = (r * max_twist.vx, r * max_twist.vy, r * max_twist.wz)
            self.limits = {"max_vx_mps": self._vmax[0], "max_vy_mps": self._vmax[1],
                           "max_wz_radps": self._vmax[2]}
        self._fil = threading.Thread(target=self._boucle, daemon=True, name="axis-flux")
        self._fil.start()

    # ---- consignes (fil du pont) ------------------------------------------

    def _rejeter(self, code: RefusalCode) -> StreamAck:
        self.m.rejected[code.value] = self.m.rejected.get(code.value, 0) + 1
        return StreamAck(False, code.value, self.state)

    def push(self, cible, deadman: bool, t: Optional[float]) -> StreamAck:
        maintenant = self._ctx.now()
        with self._verrou:
            self.m.received += 1
            if self._ctx._arret_demande is not None:
                # Arrêt demandé : sans attendre le prochain pas du flux.
                self._arret_sur(f"arrêt demandé ({self._ctx._arret_demande})")
            if self.state != "ACTIVE":
                return self._rejeter(RefusalCode.STREAM_STOPPED)
            if not deadman:
                self._arret_sur("homme mort relâché")
                return StreamAck(False, RefusalCode.STREAM_STOPPED.value, self.state)
            t = maintenant if t is None else t
            age = maintenant - t
            # Une consigne venue du futur (au-delà de 50 ms) n'a pas d'âge
            # fiable : elle est tenue pour périmée.
            if age > self._age_max_s or age < -0.05:
                return self._rejeter(RefusalCode.STALE_TARGET)
            while self._acceptees_1s and maintenant - self._acceptees_1s[0] >= 1.0:
                self._acceptees_1s.popleft()
            if len(self._acceptees_1s) >= self._rate:
                return self._rejeter(RefusalCode.RATE_LIMITED)
            if self.kind == "arms":
                for jid, pos in cible.items():
                    bornes = self._joints.get(jid)
                    if bornes is None or not bornes[0] <= pos <= bornes[1]:
                        return self._rejeter(RefusalCode.OUT_OF_BOUNDS)
                self._suiveur.set_target(cible)
            else:
                if any(abs(v) > vm + 1e-9 for v, vm in zip(cible, self._vmax)):
                    return self._rejeter(RefusalCode.OUT_OF_BOUNDS)
                self._vitesse = cible
            self._acceptees_1s.append(maintenant)
            self._dernier_ok = maintenant
            self._mesurer(max(0.0, age), maintenant)
            return StreamAck(True, None, self.state)

    def _mesurer(self, age: float, maintenant: float) -> None:
        self.m.accepted += 1
        self._lat_total += age * 1000.0
        self.m.latency_ms_mean = self._lat_total / self.m.accepted
        self.m.latency_ms_max = max(self.m.latency_ms_max, age * 1000.0)
        if self._t_prec is not None:
            # Écart type de l'intervalle (algorithme de Welford).
            x = (maintenant - self._t_prec) * 1000.0
            self._n_int += 1
            d = x - self._moy_int
            self._moy_int += d / self._n_int
            self._m2_int += d * (x - self._moy_int)
            self.m.jitter_ms = math.sqrt(self._m2_int / self._n_int)
        self._t_prec = maintenant

    def resume(self) -> ActionResult:
        if self._ctx._watchdog.estop_fired or self._ctx._robot.estop_active:
            return refused("REFUSED", RefusalCode.ESTOP_ACTIVE, "arrêt d'urgence actif")
        if self._ctx._arret_demande is not None:
            return refused("REFUSED", RefusalCode.CANCELLED,
                           f"arrêt demandé ({self._ctx._arret_demande})")
        with self._verrou:
            if self.state == "CLOSED":
                return refused("REFUSED", RefusalCode.STREAM_STOPPED, "flux fermé")
            if self.state == "STOPPING":
                return refused("REFUSED", RefusalCode.BUSY, "arrêt sûr en cours")
            if self.state == "ACTIVE":
                return ActionResult(ok=True, status="ACTIVE", message="flux déjà actif")
            # Rien de l'ancienne consigne ne repart : la prochaine doit être
            # neuve, homme mort tenu.
            self._dernier_ok, self._vitesse = None, (0.0, 0.0, 0.0)
            if self._suiveur is not None:
                self._suiveur.set_target({})
            self._t_prec = None
            self.state = self.m.state = "ACTIVE"
            self.m.stop_reason = ""
        return ActionResult(ok=True, status="RESUMED", message="flux repris")

    def metrics(self) -> StreamMetrics:
        with self._verrou:
            return StreamMetrics.from_json(self.m.to_json())

    def close(self, attendre_s: float = 1.0) -> None:
        with self._verrou:
            if self.state == "CLOSED":
                return
            self._arret_sur("flux fermé")
            self._fermer = True
        self._fil.join(attendre_s)

    # ---- pilotage (fil du flux) -------------------------------------------

    _fermer = False

    def _arret_sur(self, raison: str) -> None:
        """Sous le verrou. Marche : ralentir puis s'arrêter (STOPPING, puis
        STOPPED) ; bras : arrêt sur place (STOPPED)."""
        if self.state in ("STOPPING", "STOPPED", "CLOSED"):
            return
        self.m.safe_stops += 1
        self.m.stop_reason = raison
        self.state = "STOPPING" if self.kind == "walk" else "STOPPED"
        self.m.state = self.state
        if self._suiveur is not None:
            self._suiveur.set_target({})

    def _boucle(self) -> None:
        from .app_base import EstopActiveError
        prec = time.monotonic()
        while True:
            debut = time.monotonic()
            dt, prec = debut - prec, debut
            try:
                if not self._pas(min(dt, 0.1)):
                    return
            except EstopActiveError:
                with self._verrou:
                    self._arret_sur("arrêt d'urgence")
                    self.state = self.m.state = "STOPPED"
            except Exception as exc:          # noqa: BLE001 — jamais de flux qui continue à l'aveugle
                with self._verrou:
                    self._arret_sur(f"erreur : {type(exc).__name__}")
                    self.state = self.m.state = "STOPPED"
                try:
                    self._ctx.stop_walking() if self.kind == "walk" else None
                except Exception:             # noqa: BLE001
                    pass
            time.sleep(max(0.0, self.PERIOD_S - (time.monotonic() - debut)))

    def _pas(self, dt: float) -> bool:
        """Un pas du flux ; False quand il est fermé et arrêté."""
        ctx = self._ctx
        estop = ctx._watchdog.estop_fired or ctx._robot.estop_active
        with self._verrou:
            if estop or ctx._ferme:
                self._arret_sur("arrêt d'urgence" if estop else "fin du hook")
                if ctx._ferme:
                    self._fermer = True
            elif ctx._arret_demande is not None:
                # Arrêt demandé par un humain ou par l'OS : arrêt sûr, comme
                # go_to et les animations (constaté par une revue : le flux
                # continuait).
                self._arret_sur(f"arrêt demandé ({ctx._arret_demande})")
            if self.state == "ACTIVE" and self._dernier_ok is not None \
                    and ctx.now() - self._dernier_ok > self._deadman_s:
                self._arret_sur("plus de consigne valide (homme mort, perte ou retard du flux)")
            etat, vitesse = self.state, self._vitesse
        if etat == "ACTIVE" and self._dernier_ok is not None:
            if self.kind == "arms":
                if self._suiveur.tick().name == "FAULT":
                    with self._verrou:
                        self._arret_sur("pas refusé par le garde-fou des articulations")
            else:
                ctx.walk(*vitesse)
            return True
        if etat == "STOPPING":
            # Ralentir à décélération bornée, puis s'arrêter.
            vx, vy, wz = vitesse
            def vers_zero(v, a):
                return math.copysign(max(0.0, abs(v) - a * dt), v)
            vitesse = (vers_zero(vx, DECEL_LIN), vers_zero(vy, DECEL_LIN),
                       vers_zero(wz, DECEL_ROT))
            if estop:
                vitesse = (0.0, 0.0, 0.0)
            elif any(vitesse):
                ctx.walk(*vitesse)
            with self._verrou:
                self._vitesse = vitesse
                if not any(vitesse):
                    ctx.stop_walking()
                    self.state = self.m.state = "STOPPED"
            return True
        with self._verrou:
            if self._fermer and self.state in ("STOPPED", "CLOSED"):
                self.state = self.m.state = "CLOSED"
                return False
        return True
