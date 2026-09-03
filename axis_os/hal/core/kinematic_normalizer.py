"""
AXIS-OS — KinematicNormalizer
================================
Traduit les angles entre l'espace normalisé (apps) et l'espace raw hardware.

Formule :
    raw        = normalized × sign + offset
    normalized = (raw - offset) × sign

Principe : positif = flexion, TOUJOURS, quel que soit le robot.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Dict

from .types import ActuatorCommand, ActuatorFeedback, ControlMode


@dataclass(frozen=True)
class ConventionEntry:
    sign:       float   # +1.0 ou -1.0
    offset_rad: float   # décalage zéro (0.0 dans la majorité des cas)


ConventionMap = Dict[str, ConventionEntry]


class KinematicNormalizer:
    """
    Stateless — fonctions pures.
    Bidirectionnel : normalized→raw ET raw→normalized.
    Robot-agnostique : prend une ConventionMap, pas un robot spécifique.
    """

    def __init__(self, convention_map: ConventionMap, verbose: bool = False):
        self._map     = convention_map
        self._verbose = verbose

    def normalized_to_raw(self, joint_id: str, normalized: float) -> float:
        entry = self._get(joint_id)
        raw   = (normalized * entry.sign) + entry.offset_rad
        if self._verbose:
            print(f"  [Normalizer] {joint_id}: norm={normalized:+.4f} → raw={raw:+.4f}")
        return raw

    def raw_to_normalized(self, joint_id: str, raw: float) -> float:
        entry      = self._get(joint_id)
        normalized = (raw - entry.offset_rad) * entry.sign
        if self._verbose:
            print(f"  [Normalizer] {joint_id}: raw={raw:+.4f} → norm={normalized:+.4f}")
        return normalized

    def command_to_raw(
        self, command: ActuatorCommand, joint_id: str
    ) -> ActuatorCommand:
        raw = ActuatorCommand(
            mode=command.mode, position=command.position,
            velocity=command.velocity, torque=command.torque,
            current=command.current, stiffness=command.stiffness,
            damping=command.damping, timestamp_ns=command.timestamp_ns,
        )
        if command.mode in (ControlMode.POSITION, ControlMode.IMPEDANCE):
            if command.position is not None:
                raw.position = self.normalized_to_raw(joint_id, command.position)
            if command.velocity is not None:
                raw.velocity = self.normalized_to_raw(joint_id, command.velocity)
        elif command.mode == ControlMode.VELOCITY:
            if command.velocity is not None:
                raw.velocity = self.normalized_to_raw(joint_id, command.velocity)
        return raw

    def feedback_to_normalized(self, feedback: ActuatorFeedback) -> ActuatorFeedback:
        entry = self._get(feedback.joint_id)
        return ActuatorFeedback(
            joint_id=    feedback.joint_id,
            state=       feedback.state,
            position=    self.raw_to_normalized(feedback.joint_id, feedback.position),
            velocity=    self.raw_to_normalized(feedback.joint_id, feedback.velocity),
            torque=      feedback.torque * entry.sign,
            current=     feedback.current,
            temperature= feedback.temperature,
            voltage=     feedback.voltage,
            error_code=  feedback.error_code,
            timestamp_ns=feedback.timestamp_ns,
        )

    def commands_batch_to_raw(
        self, commands: Dict[str, ActuatorCommand]
    ) -> Dict[str, ActuatorCommand]:
        return {jid: self.command_to_raw(cmd, jid) for jid, cmd in commands.items()}

    def feedbacks_batch_to_normalized(
        self, feedbacks: Dict[str, ActuatorFeedback]
    ) -> Dict[str, ActuatorFeedback]:
        return {jid: self.feedback_to_normalized(fb) for jid, fb in feedbacks.items()}

    def get_normalized_limits(
        self, joint_id: str, raw_min: float, raw_max: float
    ) -> tuple[float, float]:
        n_min = self.raw_to_normalized(joint_id, raw_min)
        n_max = self.raw_to_normalized(joint_id, raw_max)
        return (min(n_min, n_max), max(n_min, n_max))

    def is_inverted(self, joint_id: str) -> bool:
        return self._get(joint_id).sign < 0

    def _get(self, joint_id: str) -> ConventionEntry:
        if joint_id not in self._map:
            raise KeyError(f"AXIS-OS: joint '{joint_id}' absent de la ConventionMap.")
        return self._map[joint_id]


def build_g1_normalizer(verbose: bool = False) -> KinematicNormalizer:
    """Factory : normalizer configuré pour le Unitree G1."""
    from axis_os.hal.drivers.unitree_g1.conventions import G1_JOINT_CONVENTIONS
    convention_map: ConventionMap = {
        jid: ConventionEntry(sign=c.sign, offset_rad=c.offset_rad)
        for jid, c in G1_JOINT_CONVENTIONS.items()
    }
    return KinematicNormalizer(convention_map, verbose=verbose)
