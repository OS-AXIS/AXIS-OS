"""
AXIS-OS — Service de locomotion.

Troisième service système. Rend la marche accessible à une app par une
consigne de vitesse — « avance à 0.5 m/s », « tourne à 0.6 rad/s » —
sans que l'app ait à connaître le robot.

UNIVERSEL PAR CONSTRUCTION : le service ne sait rien de la façon dont un
robot se déplace. Il délègue à un ILocomotionAdapter, déclaré par le
DRIVER : politique de marche pour un bipède, cinématique différentielle
pour une base à roues. Même principe que les groupes d'articulations :
le cadre est commun, la connaissance du robot est dans son driver.

TROIS EXIGENCES DE SÉCURITÉ, chacune testée :

1. ÉCHÉANCE DE FRAÎCHEUR. Une app qui cesse de rafraîchir sa consigne
   ne laisse PAS le robot marcher indéfiniment : passé command_timeout_s,
   la consigne retombe à zéro. Même principe que SetpointFollower.

2. S'ARRÊTER N'EST PAS COUPER. Sur un bipède, cesser d'envoyer des
   consignes ou couper le couple, c'est tomber. Arrêter = continuer à
   faire tourner l'adaptateur avec une vitesse NULLE : le robot s'immobilise
   en gardant son équilibre. C'est l'adaptateur qui sait ce qu'est un
   arrêt sûr pour son robot ; le service lui demande juste « vitesse zéro ».

3. APRÈS UNE CHUTE, PLUS RIEN. Une politique de marche qui continue à
   commander un robot à terre se débat. Chute détectée -> le service
   cesse d'envoyer des consignes et le signale. La suite (amortissement,
   arrêt d'urgence) relève du système de sécurité, pas de la marche.

Toutes les consignes passent par `send_targets`, qui DOIT être la
méthode gardée du HAL (JointSafetyGuard) : un service ne commande
jamais un actionneur directement.
"""
from __future__ import annotations

import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, Optional, Tuple

from axis_os.hal.core.types import HALStatus
from axis_os.services.base import IService, ServiceHealth, ServiceResult
from axis_os.store.sdk.manifest import Capability


@dataclass(frozen=True)
class Twist:
    """Consigne de vitesse dans le repère du robot : vx en avant, vy à
    gauche (m/s), wz rotation autour de la verticale (rad/s)."""
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def is_zero(self) -> bool:
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0


@dataclass(frozen=True)
class BaseState:
    """État du robot nécessaire à la locomotion.

    `ang_vel_body` porte son repère DANS SON NOM : repère du CORPS, comme
    le rend une vraie centrale inertielle. Une source qui lit le repère du
    monde (Isaac Sim) doit convertir avant de construire cet objet. Le
    repère d'une vitesse angulaire ne se devine pas — mesuré, une erreur
    de repère fait tomber le G1 après 103° de virage.
    """
    quat_wxyz: Tuple[float, float, float, float]
    ang_vel_body: Tuple[float, float, float]
    joint_pos: Dict[str, float]
    joint_vel: Dict[str, float]
    timestamp_ns: int = field(default_factory=time.monotonic_ns)


class ILocomotionAdapter(ABC):
    """Traduction consigne de vitesse -> consignes articulaires, propre à
    un robot. Déclaré par le driver."""

    @property
    @abstractmethod
    def control_dt(self) -> float:
        """Période de commande attendue par l'adaptateur (s)."""

    @property
    @abstractmethod
    def max_twist(self) -> Twist:
        """Vitesses maximales. Au-delà, un adaptateur appris extrapole
        hors de son domaine d'entraînement — sans le dire."""

    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def step(self, state: BaseState, twist: Twist) -> Dict[str, float]:
        """Un pas de commande. Twist nul = ARRÊT SÛR pour ce robot."""

    @abstractmethod
    def is_fallen(self, state: BaseState) -> bool: ...


class LocomotionState(Enum):
    STOPPED = auto()
    RUNNING = auto()
    FALLEN = auto()


class LocomotionService(IService):

    def __init__(
        self,
        adapter: ILocomotionAdapter,
        read_state: Callable[[], BaseState],
        send_targets: Callable[[Dict[str, float]], Dict[str, HALStatus]],
        command_timeout_s: float = 0.5,
        clock: Callable[[], float] = time.monotonic,
        gardien=None,
    ):
        """
        gardien : GardienProximite, optionnel. S'il est fourni, CHAQUE
            consigne de vitesse — d'une app, de la navigation, de qui que
            ce soit — est plafonnée par lui avant d'atteindre les jambes.
            Il n'existe aucune méthode pour le désactiver : une app ne
            choisit pas la sécurité des humains autour du robot.
        """
        self._adapter = adapter
        self._read_state = read_state
        self._send = send_targets
        self._timeout = command_timeout_s
        self._clock = clock
        self._gardien = gardien
        self.dernier_verdict = None
        self._twist = Twist()
        self._twist_t: Optional[float] = None
        self.state = LocomotionState.STOPPED
        self.stats = {"ticks": 0, "stale": 0, "clamped": 0,
                      "refused": 0, "falls": 0, "non_finies": 0}

    # ---- IService --------------------------------------------------------

    @property
    def service_id(self) -> str:
        return "locomotion"

    @property
    def capability(self) -> Capability:
        return Capability.WALK

    def start(self) -> HALStatus:
        self._adapter.reset()
        self._twist, self._twist_t = Twist(), None
        self.state = LocomotionState.RUNNING
        return HALStatus.OK

    def stop(self) -> None:
        self.state = LocomotionState.STOPPED

    def health(self) -> ServiceHealth:
        if self.state is LocomotionState.FALLEN:
            return ServiceHealth(running=False, healthy=False,
                                 detail="chute détectée — consignes suspendues")
        return ServiceHealth(running=self.state is LocomotionState.RUNNING,
                             healthy=True, detail=self.state.name)

    # ---- API des apps ----------------------------------------------------

    def set_velocity(self, vx: float = 0.0, vy: float = 0.0,
                     wz: float = 0.0) -> ServiceResult:
        """Nouvelle consigne. Non bloquant. DOIT être rafraîchie avant
        command_timeout_s, sinon le robot s'arrête (en équilibre)."""
        if self.state is not LocomotionState.RUNNING:
            return ServiceResult(ok=False,
                                 reason=f"service {self.state.name}")
        # Consigne NON FINIE refusée, et le robot s'arrête en équilibre.
        # POURQUOI (audit du 2026-10-10) : le bornage ci-dessous,
        # max(-m, min(m, v)), transforme NaN en +m — min(m, nan) rend m,
        # car toute comparaison avec NaN est fausse. Un ctx.walk(nan)
        # d'une app, ou un connecteur VLA qui laisse passer un NaN,
        # faisait marcher le robot à PLEINE VITESSE vers l'avant ; seul le
        # gardien de proximité freinait, et seulement près d'un obstacle.
        # Un infini n'a pas davantage de sens : refusé aussi, plutôt que
        # ramené à la vitesse maximale. On remet la consigne à zéro (pas
        # seulement « on ignore ») : la consigne précédente, peut-être
        # rapide, ne doit pas continuer jusqu'à son échéance.
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            self._twist, self._twist_t = Twist(), self._clock()
            self.stats["non_finies"] += 1
            return ServiceResult(
                ok=False,
                reason=f"consigne non finie ({vx}, {vy}, {wz}) refusée — "
                       f"arrêt en équilibre")
        m = self._adapter.max_twist
        c = Twist(max(-m.vx, min(m.vx, vx)),
                  max(-m.vy, min(m.vy, vy)),
                  max(-m.wz, min(m.wz, wz)))
        self._twist, self._twist_t = c, self._clock()
        if c != Twist(vx, vy, wz):
            self.stats["clamped"] += 1
            return ServiceResult(
                ok=True, value=c,
                reason=f"consigne bornée à ({c.vx}, {c.vy}, {c.wz}) — "
                       f"au-delà, l'adaptateur sort de son domaine")
        return ServiceResult(ok=True, value=c)

    def halt(self) -> ServiceResult:
        """Arrêt EN ÉQUILIBRE : vitesse nulle, l'adaptateur continue de
        tourner. Ce n'est pas un arrêt d'urgence."""
        return self.set_velocity(0.0, 0.0, 0.0)

    # ---- boucle de commande ----------------------------------------------

    def _effective_twist(self) -> Twist:
        if self._twist_t is None:
            return Twist()
        if self._clock() - self._twist_t > self._timeout:
            if not self._twist.is_zero:
                self.stats["stale"] += 1
            self._twist, self._twist_t = Twist(), None
            return Twist()
        return self._twist

    def tick(self) -> ServiceResult:
        """Un pas de commande, à appeler tous les adapter.control_dt."""
        if self.state is LocomotionState.FALLEN:
            return ServiceResult(ok=False, reason="robot à terre — aucune consigne")
        if self.state is not LocomotionState.RUNNING:
            return ServiceResult(ok=False, reason="service arrêté")

        st = self._read_state()
        if self._adapter.is_fallen(st):
            self.state = LocomotionState.FALLEN
            self.stats["falls"] += 1
            return ServiceResult(
                ok=False, reason="chute détectée — plus aucune consigne "
                                 "de marche envoyée ; relève du système de "
                                 "sécurité")

        twist = self._effective_twist()
        if self._gardien is not None:
            v = self._gardien.evaluer(self._clock())
            self.dernier_verdict = v
            if v.facteur <= 0.0:
                # Arrêt EN ÉQUILIBRE : consigne nulle, la politique continue
                # de tenir le robot debout. Jamais une coupure des moteurs.
                twist = Twist()
            elif v.facteur < 1.0:
                twist = Twist(twist.vx * v.facteur, twist.vy * v.facteur,
                              twist.wz * v.facteur)
        targets = self._adapter.step(st, twist)
        statuses = self._send(targets)
        refused = sum(1 for s in statuses.values() if s != HALStatus.OK)
        self.stats["refused"] += refused
        self.stats["ticks"] += 1
        return ServiceResult(ok=True, value=twist,
                             reason=(f"{refused} consigne(s) refusée(s)"
                                     if refused else None))
