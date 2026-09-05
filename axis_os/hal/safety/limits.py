"""
AXIS-OS — HAL Safety Layer v2
================================
Validateur de commandes interposé entre l'orchestrateur et chaque actionneur.
Règle absolue : si doute → on coupe. Sécurité > Performance.

Corrections v2 suite à revue externe :
  - NaN/Infinity détectés EXPLICITEMENT (rule=INVALID_VALUE), au lieu
    d'être rejetés implicitement sous rule=POSITION_LIMIT avec un
    message trompeur (pos_min <= NaN <= pos_max vaut False en Python,
    donc la commande était déjà bloquée, mais le diagnostic était faux)
  - validate_limits() : vérifie qu'une JointLimits est cohérente
    (min < max, valeurs finies) AVANT de servir à valider quoi que ce
    soit — un JointLimits mal configuré ne doit jamais passer inaperçu

Portée volontairement pas étendue à cette version (dépendance commune
identifiée à la fois par la revue de ce fichier et celle du Watchdog) :
  - Limites d'accélération / slew rate / comparaison commande↔feedback
    nécessitent de connaître la commande PRÉCÉDENTE, que JointSafetyGuard
    ne conserve pas aujourd'hui. Les deux revues externes (celle-ci et
    celle du Watchdog) pointent la même dépendance manquante — signal
    qu'elle doit être traitée une fois, proprement, pas dupliquée ici
    à moitié.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, List

from ..core.types import (
    ActuatorCommand, ActuatorFeedback,
    JointLimits, ControlMode, HALStatus
)
from ..core.motion_state import MotionStateTracker
from ..core.structured_log import StructuredLogger, LogCategory, get_default_logger


class SafetyErrorKind(Enum):
    """Taxonomie cohérente avec SandboxErrorKind et ConventionErrorKind."""
    INVALID_VALUE      = auto()   # NaN / Infinity
    POSITION_LIMIT      = auto()
    VELOCITY_LIMIT       = auto()
    TORQUE_LIMIT          = auto()
    CURRENT_LIMIT           = auto()
    TEMPERATURE_CRITICAL      = auto()
    TORQUE_OVERLOAD             = auto()
    INVALID_LIMITS_CONFIG         = auto()  # JointLimits elle-même incohérente
    ACCELERATION_LIMIT              = auto()  # rate limiting — BLOCK, jamais lissé


@dataclass
class SafetyViolation:
    joint_id:    str
    rule:        str            # nom lisible, conservé pour compat ascendante
    value:       float
    limit:       float
    kind:        Optional[SafetyErrorKind] = None
    timestamp_ns: int = 0

    def __post_init__(self):
        if self.timestamp_ns == 0:
            import time
            self.timestamp_ns = time.time_ns()

    def __str__(self) -> str:
        return (
            f"[SAFETY VIOLATION] joint={self.joint_id} | "
            f"rule={self.rule} | value={self.value} | limit={self.limit}"
        )


@dataclass
class LimitsValidationResult:
    """Résultat de validate_limits() — point #5 revue externe."""
    valid:  bool
    errors: List[str]

    def __str__(self) -> str:
        if self.valid:
            return "[LIMITS] Configuration valide."
        return "[LIMITS] " + "; ".join(self.errors)


def validate_limits(joint_id: str, limits: JointLimits) -> LimitsValidationResult:
    """
    Vérifie qu'une JointLimits est cohérente AVANT utilisation.
    Point #5 revue externe : min < max, valeurs finies, positives
    où c'est physiquement requis (vitesses, couples, courants max
    sont des amplitudes, jamais négatives).
    """
    errors: List[str] = []

    for name, val in [("pos_min", limits.pos_min), ("pos_max", limits.pos_max),
                      ("vel_max", limits.vel_max), ("torque_max", limits.torque_max),
                      ("current_max", limits.current_max),
                      ("temperature_max", limits.temperature_max)]:
        if not math.isfinite(val):
            errors.append(f"{name}={val} n'est pas une valeur finie")

    if math.isfinite(limits.pos_min) and math.isfinite(limits.pos_max):
        if limits.pos_min >= limits.pos_max:
            errors.append(f"pos_min({limits.pos_min}) >= pos_max({limits.pos_max})")

    for name, val in [("vel_max", limits.vel_max), ("torque_max", limits.torque_max),
                      ("current_max", limits.current_max)]:
        if math.isfinite(val) and val <= 0:
            errors.append(f"{name}={val} doit être strictement positif (amplitude)")

    if joint_id.strip() == "":
        errors.append("joint_id vide")

    return LimitsValidationResult(valid=(len(errors) == 0), errors=errors)


class JointSafetyGuard:
    """
    Gardien de sécurité pour UN joint.
    Instancié par chaque IActuator dans son __init__.

    La validation des limites elle-même (validate_limits) est appelée
    à la construction — un JointSafetyGuard configuré avec des limites
    incohérentes lève une exception immédiatement plutôt que de laisser
    passer des commandes silencieusement mal contraintes.
    """

    TEMPERATURE_CRITICAL_C = 80.0
    TEMPERATURE_WARNING_C  = 65.0

    def __init__(self, joint_id: str, limits: JointLimits, strict_init: bool = True,
                motion_tracker: Optional[MotionStateTracker] = None,
                logger: Optional[StructuredLogger] = None):
        result = validate_limits(joint_id, limits)
        if not result.valid and strict_init:
            raise ValueError(
                f"JointSafetyGuard({joint_id}) refusé : {result}"
            )
        self.joint_id = joint_id
        self.limits   = limits
        self._last_violation: Optional[SafetyViolation] = None
        # Même pattern que WatchdogSystem : injecté, instance partagée
        # par défaut si rien n'est fourni. Chaque violation détectée par
        # ce Guard (rejetée, jamais exécutée sur le hardware) est
        # persistée — utile pour repérer après coup une app qui envoie
        # systématiquement des commandes invalides, même si le HAL n'a
        # jamais vu passer ces commandes.
        self._struct_logger = logger or get_default_logger()

        # MotionStateTracker optionnel — injecté par IRobotHAL, jamais créé
        # ici (décision de conception : composant partagé générique, voir
        # hal/core/motion_state.py). None = rate limiting désactivé, tout
        # le reste du comportement du Guard est inchangé.
        self._motion_tracker = motion_tracker

    def _record_violation(self, v: SafetyViolation) -> SafetyViolation:
        """
        Point unique de journalisation d'une violation — centralise ce
        qui était auparavant répété (self._last_violation = v) à chaque
        cas de rejet dans validate_command(), pour garantir qu'aucun ne
        soit oublié lors d'un futur ajout de règle.
        """
        self._last_violation = v
        self._struct_logger.log(
            LogCategory.SAFETY_VIOLATION, v, robot_id=None
        )
        return v

    def validate_command(
        self,
        command: ActuatorCommand,
        feedback: Optional[ActuatorFeedback] = None
    ) -> tuple[bool, Optional[SafetyViolation]]:

        # NaN/Infinity — point #1 revue externe. Vérifié explicitement
        # AVANT toute comparaison, pour un diagnostic correct (au lieu
        # de "POSITION_LIMIT" trompeur quand la vraie cause est NaN).
        for field_name, val in [("position", command.position),
                                ("velocity", command.velocity),
                                ("torque", command.torque),
                                ("current", command.current)]:
            if val is not None and not math.isfinite(val):
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="INVALID_VALUE",
                    value=val, limit=0.0, kind=SafetyErrorKind.INVALID_VALUE,
                )
                self._record_violation(v)
                return False, v

        # Position
        if command.mode == ControlMode.POSITION and command.position is not None:
            if not (self.limits.pos_min <= command.position <= self.limits.pos_max):
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="POSITION_LIMIT",
                    value=command.position,
                    limit=self.limits.pos_max if command.position > 0
                          else self.limits.pos_min,
                    kind=SafetyErrorKind.POSITION_LIMIT,
                )
                self._record_violation(v)
                return False, v

        # Vitesse
        if command.velocity is not None:
            if abs(command.velocity) > self.limits.vel_max:
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="VELOCITY_LIMIT",
                    value=abs(command.velocity), limit=self.limits.vel_max,
                    kind=SafetyErrorKind.VELOCITY_LIMIT,
                )
                self._record_violation(v)
                return False, v

        # Couple
        if command.torque is not None:
            if abs(command.torque) > self.limits.torque_max:
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="TORQUE_LIMIT",
                    value=abs(command.torque), limit=self.limits.torque_max,
                    kind=SafetyErrorKind.TORQUE_LIMIT,
                )
                self._record_violation(v)
                return False, v

        # Courant
        if command.current is not None:
            if abs(command.current) > self.limits.current_max:
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="CURRENT_LIMIT",
                    value=abs(command.current), limit=self.limits.current_max,
                    kind=SafetyErrorKind.CURRENT_LIMIT,
                )
                self._record_violation(v)
                return False, v

        # Température (feedback)
        if feedback is not None and math.isfinite(feedback.temperature):
            if feedback.temperature >= self.TEMPERATURE_CRITICAL_C:
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="TEMPERATURE_CRITICAL",
                    value=feedback.temperature, limit=self.TEMPERATURE_CRITICAL_C,
                    kind=SafetyErrorKind.TEMPERATURE_CRITICAL,
                )
                self._record_violation(v)
                return False, v

        # Accélération / slew rate — BLOCK, jamais lissé (décision de
        # conception actée : le Guard reste une barrière accepter/refuser,
        # jamais un contrôleur de mouvement qui modifierait la commande).
        # Nécessite un MotionStateTracker injecté et limits.accel_max défini ;
        # sans l'un ou l'autre, ce check est un no-op (comportement inchangé).
        if (self._motion_tracker is not None and self.limits.accel_max is not None
                and command.mode == ControlMode.POSITION):
            delta = self._motion_tracker.compute_command_delta(self.joint_id, command)
            if delta is not None and delta.has_history and delta.implied_acceleration is not None:
                accel_abs = abs(delta.implied_acceleration)
                if accel_abs > self.limits.accel_max:
                    v = SafetyViolation(
                        joint_id=self.joint_id, rule="ACCELERATION_LIMIT",
                        value=accel_abs, limit=self.limits.accel_max,
                        kind=SafetyErrorKind.ACCELERATION_LIMIT,
                    )
                    self._record_violation(v)
                    return False, v

        return True, None

    def check_feedback_health(
        self, feedback: ActuatorFeedback
    ) -> list[SafetyViolation]:
        """
        Vérifie un feedback reçu du hardware, indépendamment de toute
        commande. Traite aussi les feedbacks eux-mêmes non-finis —
        un capteur qui renvoie NaN est une anomalie, pas une donnée
        exploitable.
        """
        violations = []

        for field_name in ("position", "velocity", "torque", "current", "temperature"):
            val = getattr(feedback, field_name)
            if not math.isfinite(val):
                violations.append(SafetyViolation(
                    joint_id=self.joint_id, rule="INVALID_VALUE",
                    value=val, limit=0.0, kind=SafetyErrorKind.INVALID_VALUE,
                ))
                return violations  # feedback inexploitable, on s'arrête là

        if feedback.temperature >= self.TEMPERATURE_CRITICAL_C:
            violations.append(SafetyViolation(
                joint_id=self.joint_id, rule="TEMPERATURE_CRITICAL",
                value=feedback.temperature, limit=self.TEMPERATURE_CRITICAL_C,
                kind=SafetyErrorKind.TEMPERATURE_CRITICAL,
            ))
        if abs(feedback.torque) > self.limits.torque_max * 1.1:
            violations.append(SafetyViolation(
                joint_id=self.joint_id, rule="TORQUE_OVERLOAD",
                value=abs(feedback.torque), limit=self.limits.torque_max * 1.1,
                kind=SafetyErrorKind.TORQUE_OVERLOAD,
            ))
        return violations

    @property
    def last_violation(self) -> Optional[SafetyViolation]:
        return self._last_violation
