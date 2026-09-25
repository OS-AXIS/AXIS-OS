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
import math
from dataclasses import dataclass
from typing import Optional, Dict

from .types import ActuatorCommand, ActuatorFeedback, ControlMode


class InvalidAngleError(ValueError):
    """
    Levée quand une valeur NaN/Infinity/non-finie traverse le normalizer.
    Trou identifié par revue externe : une commande contenant NaN passait
    silencieusement jusqu'au JointSafetyGuard, où pos_min <= NaN <= pos_max
    est TOUJOURS False en Python — donc rejetée, mais avec un message
    trompeur ("hors limite") au lieu de la vraie cause ("valeur invalide").
    """
    def __init__(self, joint_id: str, value: float, context: str):
        self.joint_id = joint_id
        self.value    = value
        self.context  = context
        super().__init__(
            f"Valeur non-finie rejetée : joint={joint_id} value={value} "
            f"contexte={context} (NaN/Infinity interdits)"
        )


def _check_finite(joint_id: str, value: Optional[float], context: str) -> None:
    if value is not None and not math.isfinite(value):
        raise InvalidAngleError(joint_id, value, context)


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
        _check_finite(joint_id, normalized, "normalized_to_raw")
        entry = self._get(joint_id)
        raw   = (normalized * entry.sign) + entry.offset_rad
        if self._verbose:
            print(f"  [Normalizer] {joint_id}: norm={normalized:+.4f} → raw={raw:+.4f}")
        return raw

    def raw_to_normalized(self, joint_id: str, raw: float) -> float:
        _check_finite(joint_id, raw, "raw_to_normalized")
        entry      = self._get(joint_id)
        normalized = (raw - entry.offset_rad) * entry.sign
        if self._verbose:
            print(f"  [Normalizer] {joint_id}: raw={raw:+.4f} → norm={normalized:+.4f}")
        return normalized

    def command_to_raw(
        self, command: ActuatorCommand, joint_id: str
    ) -> ActuatorCommand:
        entry = self._get(joint_id)
        raw = ActuatorCommand(
            mode=command.mode, position=command.position,
            velocity=command.velocity, torque=command.torque,
            current=command.current, stiffness=command.stiffness,
            damping=command.damping, timestamp_ns=command.timestamp_ns,
        )
        # Position : sign ET offset s'appliquent (l'offset compense un
        # zéro mécanique différent entre la convention AXIS-OS et le
        # zéro raw du joint — n'a de sens QUE pour une position).
        if command.mode in (ControlMode.POSITION, ControlMode.IMPEDANCE):
            if command.position is not None:
                raw.position = self.normalized_to_raw(joint_id, command.position)

        # Vitesse : SIGN SEUL, jamais l'offset — bug trouvé par un audit
        # de sécurité (exécution réelle). La version précédente
        # appelait normalized_to_raw() pour la vitesse, qui ajoute
        # entry.offset_rad. Un offset de position n'a aucun sens
        # physique appliqué à une vitesse : "vitesse 0" aurait produit
        # une consigne raw = offset_rad ≠ 0, donc un joint qui continue
        # de bouger alors qu'on lui demande explicitement de s'arrêter.
        # Latent tant que tous les offsets du G1 sont à 0.0 (c'est le
        # cas aujourd'hui), mais une mine pour le premier robot/joint
        # qui en aura un non nul.
        if command.mode in (ControlMode.POSITION, ControlMode.IMPEDANCE, ControlMode.VELOCITY):
            if command.velocity is not None:
                raw.velocity = command.velocity * entry.sign

        # Couple et courant : SIGN SEUL — même bug que la vitesse,
        # trouvé par le même audit, mais plus grave : contrairement à
        # la vitesse (qui n'était qu'un no-op tant que offset=0), le
        # couple/courant n'était ICI PAS DU TOUT converti — ni sign, ni
        # offset (l'offset n'a de toute façon aucun sens pour un
        # couple). Sur les 9 joints à sign=-1 du G1 (right_hip_*,
        # right_knee, right_ankle_*, right_shoulder_roll/yaw,
        # right_wrist_roll — voir conventions.py), une commande de
        # couple était transmise avec le signe INVERSÉ au moteur réel
        # par rapport à ce que l'app avait demandé, alors que le
        # feedback (feedback_to_normalized ci-dessous) applique bien
        # "* entry.sign" en lecture. Cette asymétrie lecture/écriture
        # est corrigée en traitant l'écriture exactement comme la
        # lecture le fait déjà.
        if command.torque is not None:
            raw.torque = command.torque * entry.sign
        if command.current is not None:
            raw.current = command.current * entry.sign

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


def build_g1_normalizer(verbose: bool = False,
                        variante: str = "23dof") -> KinematicNormalizer:
    """Factory : normalizer configuré pour le Unitree G1."""
    from axis_os.hal.drivers.unitree_g1.conventions import conventions_pour
    convention_map: ConventionMap = {
        jid: ConventionEntry(sign=c.sign, offset_rad=c.offset_rad)
        for jid, c in conventions_pour(variante).items()
    }
    return KinematicNormalizer(convention_map, verbose=verbose)
