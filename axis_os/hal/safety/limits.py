"""
AXIS-OS — HAL Safety Layer
============================
Validateur de commandes interposé entre l'orchestrateur et chaque actionneur.
Règle absolue : si doute → on coupe. Sécurité > Performance.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import time

from ..core.types import (
    ActuatorCommand, ActuatorFeedback,
    JointLimits, ControlMode, HALStatus
)


@dataclass
class SafetyViolation:
    joint_id:    str
    rule:        str
    value:       float
    limit:       float
    timestamp_ns: int = 0

    def __post_init__(self):
        if self.timestamp_ns == 0:
            self.timestamp_ns = time.time_ns()

    def __str__(self) -> str:
        return (
            f"[SAFETY VIOLATION] joint={self.joint_id} | "
            f"rule={self.rule} | value={self.value:.4f} | limit={self.limit:.4f}"
        )


class JointSafetyGuard:
    """
    Gardien de sécurité pour UN joint.
    Instancié par chaque IActuator dans son __init__.
    """

    TEMPERATURE_CRITICAL_C = 80.0
    TEMPERATURE_WARNING_C  = 65.0

    def __init__(self, joint_id: str, limits: JointLimits):
        self.joint_id = joint_id
        self.limits   = limits
        self._last_violation: Optional[SafetyViolation] = None

    def validate_command(
        self,
        command: ActuatorCommand,
        feedback: Optional[ActuatorFeedback] = None
    ) -> tuple[bool, Optional[SafetyViolation]]:

        # Position
        if command.mode == ControlMode.POSITION and command.position is not None:
            if not (self.limits.pos_min <= command.position <= self.limits.pos_max):
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="POSITION_LIMIT",
                    value=command.position,
                    limit=self.limits.pos_max if command.position > 0
                          else self.limits.pos_min
                )
                self._last_violation = v
                return False, v

        # Vitesse
        if command.velocity is not None:
            if abs(command.velocity) > self.limits.vel_max:
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="VELOCITY_LIMIT",
                    value=abs(command.velocity), limit=self.limits.vel_max
                )
                self._last_violation = v
                return False, v

        # Couple
        if command.torque is not None:
            if abs(command.torque) > self.limits.torque_max:
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="TORQUE_LIMIT",
                    value=abs(command.torque), limit=self.limits.torque_max
                )
                self._last_violation = v
                return False, v

        # Courant
        if command.current is not None:
            if abs(command.current) > self.limits.current_max:
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="CURRENT_LIMIT",
                    value=abs(command.current), limit=self.limits.current_max
                )
                self._last_violation = v
                return False, v

        # Température
        if feedback is not None:
            if feedback.temperature >= self.TEMPERATURE_CRITICAL_C:
                v = SafetyViolation(
                    joint_id=self.joint_id, rule="TEMPERATURE_CRITICAL",
                    value=feedback.temperature, limit=self.TEMPERATURE_CRITICAL_C
                )
                self._last_violation = v
                return False, v

        return True, None

    def check_feedback_health(
        self, feedback: ActuatorFeedback
    ) -> list[SafetyViolation]:
        violations = []
        if feedback.temperature >= self.TEMPERATURE_CRITICAL_C:
            violations.append(SafetyViolation(
                joint_id=self.joint_id, rule="TEMPERATURE_CRITICAL",
                value=feedback.temperature, limit=self.TEMPERATURE_CRITICAL_C
            ))
        if abs(feedback.torque) > self.limits.torque_max * 1.1:
            violations.append(SafetyViolation(
                joint_id=self.joint_id, rule="TORQUE_OVERLOAD",
                value=abs(feedback.torque), limit=self.limits.torque_max * 1.1
            ))
        return violations

    @property
    def last_violation(self) -> Optional[SafetyViolation]:
        return self._last_violation
