"""
AXIS-OS — KinematicNormalizer
================================
Traduit les angles articulaires entre deux espaces : l'espace normalisé,
le seul que voient les apps (les programmes installés sur le robot), et
l'espace brut (raw) du matériel, propre à chaque robot.

Pourquoi : chaque robot a, pour chaque joint (articulation), son propre
sens positif et son propre zéro mécanique. Cette couche les absorbe,
pour qu'une app n'ait jamais à connaître le robot précis sur lequel elle
tourne.

Formule :
    raw        = normalized × sign + offset
    normalized = (raw - offset) × sign

sign vaut +1 ou -1 (sens du moteur par rapport à la convention) ; offset
est le décalage, en radians, entre le zéro brut du joint et le zéro
AXIS-OS.

Principe : positif = flexion, toujours, quel que soit le robot.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Optional, Dict

from .types import ActuatorCommand, ActuatorFeedback, ControlMode


class InvalidAngleError(ValueError):
    """
    Levée quand une valeur non finie (NaN, « Not a Number », ou l'infini)
    traverse le normalizer.

    Une revue extérieure du code a relevé ce trou : une commande contenant
    NaN passait silencieusement jusqu'au JointSafetyGuard (le garde qui
    vérifie chaque commande contre les limites du joint). Là, le test
    pos_min <= NaN <= pos_max vaut toujours False en Python : la commande
    était donc bien rejetée, mais avec un message trompeur (« hors
    limite ») au lieu de la vraie cause (« valeur invalide »).
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
    """Lève InvalidAngleError si value n'est pas finie ; None (champ
    absent) est accepté."""
    if value is not None and not math.isfinite(value):
        raise InvalidAngleError(joint_id, value, context)


@dataclass(frozen=True)
class ConventionEntry:
    """Convention d'un joint : son sens et le décalage de son zéro."""
    sign:       float   # +1.0, ou -1.0 si le moteur tourne à l'envers
    offset_rad: float   # décalage du zéro (0.0 dans la majorité des cas)


# Convention de chaque joint, indexée par identifiant de joint.
ConventionMap = Dict[str, ConventionEntry]


class KinematicNormalizer:
    """
    Sans état : chaque méthode est une fonction pure de ses arguments et
    de la table de conventions.
    Bidirectionnel : normalisé vers brut, et brut vers normalisé.
    Indépendant du robot : il reçoit une ConventionMap, pas un robot
    précis.

    verbose=True affiche, pour le débogage, chaque conversion d'angle
    faite par normalized_to_raw() et raw_to_normalized().
    """

    def __init__(self, convention_map: ConventionMap, verbose: bool = False):
        self._map     = convention_map
        self._verbose = verbose

    def normalized_to_raw(self, joint_id: str, normalized: float) -> float:
        """Angle normalisé -> angle brut (signe et décalage).

        Rejette NaN et l'infini par InvalidAngleError.
        """
        _check_finite(joint_id, normalized, "normalized_to_raw")
        entry = self._get(joint_id)
        raw   = (normalized * entry.sign) + entry.offset_rad
        if self._verbose:
            print(f"  [Normalizer] {joint_id}: norm={normalized:+.4f} → raw={raw:+.4f}")
        return raw

    def raw_to_normalized(self, joint_id: str, raw: float) -> float:
        """Angle brut -> angle normalisé (décalage et signe).

        Rejette NaN et l'infini par InvalidAngleError.
        """
        _check_finite(joint_id, raw, "raw_to_normalized")
        entry      = self._get(joint_id)
        normalized = (raw - entry.offset_rad) * entry.sign
        if self._verbose:
            print(f"  [Normalizer] {joint_id}: raw={raw:+.4f} → norm={normalized:+.4f}")
        return normalized

    def command_to_raw(
        self, command: ActuatorCommand, joint_id: str
    ) -> ActuatorCommand:
        """Commande normalisée -> commande brute pour le joint joint_id.

        Chaque grandeur ne reçoit que la conversion qui a un sens physique
        pour elle : la position prend le signe et le décalage ; vitesse,
        couple et courant ne prennent que le signe. Les commentaires
        ci-dessous expliquent pourquoi.
        """
        entry = self._get(joint_id)
        raw = ActuatorCommand(
            mode=command.mode, position=command.position,
            velocity=command.velocity, torque=command.torque,
            current=command.current, stiffness=command.stiffness,
            damping=command.damping, timestamp_ns=command.timestamp_ns,
        )
        # Position : le signe et le décalage s'appliquent. Le décalage
        # compense un zéro mécanique différent entre la convention AXIS-OS
        # et le zéro brut du joint ; il n'a de sens que pour une position.
        if command.mode in (ControlMode.POSITION, ControlMode.IMPEDANCE):
            if command.position is not None:
                raw.position = self.normalized_to_raw(joint_id, command.position)

        # Vitesse : le signe seul, jamais le décalage. Un audit de
        # sécurité, mené en exécutant réellement le code, a trouvé ce
        # bug : la version précédente appelait normalized_to_raw() pour
        # la vitesse, ce qui ajoutait entry.offset_rad. Or un décalage de
        # position n'a aucun sens physique appliqué à une vitesse :
        # « vitesse 0 » aurait produit une consigne brute égale à
        # offset_rad, donc non nulle, et un joint qui continue de bouger
        # alors qu'on lui demande explicitement de s'arrêter. Le défaut
        # restait latent tant que tous les décalages du G1 (le Unitree G1,
        # robot de référence d'AXIS-OS) valent 0.0, ce qui est le cas
        # aujourd'hui ; c'était un piège pour le premier robot ou joint
        # qui en aurait un non nul.
        if command.mode in (ControlMode.POSITION, ControlMode.IMPEDANCE, ControlMode.VELOCITY):
            if command.velocity is not None:
                raw.velocity = command.velocity * entry.sign

        # Couple et courant : le signe seul. Même bug que pour la vitesse,
        # trouvé par le même audit, mais plus grave. La vitesse n'était
        # qu'un défaut sans effet tant que offset = 0 ; le couple et le
        # courant, eux, n'étaient ici pas convertis du tout, ni signe ni
        # décalage (le décalage n'a de toute façon aucun sens pour un
        # couple). Sur les 9 joints du G1 à sign = -1 (right_hip_*,
        # right_knee, right_ankle_*, right_shoulder_roll/yaw,
        # right_wrist_roll ; voir conventions.py), une commande de couple
        # arrivait donc au moteur réel avec le signe inversé par rapport à
        # la demande de l'app, alors que la lecture
        # (feedback_to_normalized ci-dessous) applique bien
        # « * entry.sign » au couple. La correction traite l'écriture
        # exactement comme la lecture le faisait déjà.
        if command.torque is not None:
            raw.torque = command.torque * entry.sign
        if command.current is not None:
            raw.current = command.current * entry.sign

        return raw

    def feedback_to_normalized(self, feedback: ActuatorFeedback) -> ActuatorFeedback:
        """Retour capteur (feedback) brut -> feedback normalisé.

        Position et vitesse passent par raw_to_normalized() (décalage et
        signe), le couple prend le signe seul ; courant, température,
        tension, état et code d'erreur sont recopiés tels quels.

        À savoir : la vitesse reçoit donc ici aussi le décalage de
        position, le piège corrigé côté commande dans command_to_raw(),
        sans effet tant que tous les décalages valent 0.0. Et le courant
        mesuré n'est pas multiplié par le signe, alors que le courant
        commandé l'est.
        """
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
        """command_to_raw() appliqué à {joint_id: commande}."""
        return {jid: self.command_to_raw(cmd, jid) for jid, cmd in commands.items()}

    def feedbacks_batch_to_normalized(
        self, feedbacks: Dict[str, ActuatorFeedback]
    ) -> Dict[str, ActuatorFeedback]:
        """feedback_to_normalized() appliqué à {joint_id: feedback}."""
        return {jid: self.feedback_to_normalized(fb) for jid, fb in feedbacks.items()}

    def get_normalized_limits(
        self, joint_id: str, raw_min: float, raw_max: float
    ) -> tuple[float, float]:
        """Limites brutes -> limites normalisées (min, max).

        Elles sont remises dans l'ordre, car un joint à sign = -1 échange
        le minimum et le maximum.
        """
        n_min = self.raw_to_normalized(joint_id, raw_min)
        n_max = self.raw_to_normalized(joint_id, raw_max)
        return (min(n_min, n_max), max(n_min, n_max))

    def is_inverted(self, joint_id: str) -> bool:
        """True si le joint tourne en sens inverse de la convention."""
        return self._get(joint_id).sign < 0

    def _get(self, joint_id: str) -> ConventionEntry:
        """Convention du joint ; KeyError s'il est absent de la table."""
        if joint_id not in self._map:
            raise KeyError(f"AXIS-OS: joint '{joint_id}' absent de la ConventionMap.")
        return self._map[joint_id]


def build_g1_normalizer(verbose: bool = False,
                        variante: str = "23dof") -> KinematicNormalizer:
    """Fabrique : normalizer configuré pour le Unitree G1.

    variante choisit la table de conventions du G1, "23dof" ou "29dof"
    (nombre de degrés de liberté, c'est-à-dire de joints) ; voir
    conventions_pour() dans axis_os/hal/drivers/unitree_g1/conventions.py.
    """
    from axis_os.hal.drivers.unitree_g1.conventions import conventions_pour
    convention_map: ConventionMap = {
        jid: ConventionEntry(sign=c.sign, offset_rad=c.offset_rad)
        for jid, c in conventions_pour(variante).items()
    }
    return KinematicNormalizer(convention_map, verbose=verbose)
