"""
AXIS-OS — HAL Core Types
=========================
Types universels partagés par toute la stack AXIS-OS.
Aucune dépendance externe — stdlib Python uniquement.

Règle absolue : ce fichier ne doit JAMAIS être modifié
sans mettre à jour tous ses implémenteurs.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional
import time


class JointType(Enum):
    REVOLUTE   = auto()
    PRISMATIC  = auto()
    FIXED      = auto()
    CONTINUOUS = auto()


class SensorType(Enum):
    IMU          = auto()
    FORCE_TORQUE = auto()
    ENCODER      = auto()
    CAMERA_RGB   = auto()
    CAMERA_DEPTH = auto()
    TACTILE      = auto()
    TEMPERATURE  = auto()
    MICROPHONE   = auto()
    GPS          = auto()


class ActuatorState(Enum):
    UNINITIALIZED  = auto()
    IDLE           = auto()
    ACTIVE         = auto()
    FAULT          = auto()
    EMERGENCY_STOP = auto()
    CALIBRATING    = auto()


class ControlMode(Enum):
    POSITION  = auto()
    VELOCITY  = auto()
    TORQUE    = auto()
    IMPEDANCE = auto()
    CURRENT   = auto()


class HALStatus(Enum):
    OK              = 0
    ERROR_COMM      = 1
    ERROR_TIMEOUT   = 2
    ERROR_LIMIT     = 3
    ERROR_FAULT     = 4
    ERROR_NOT_INIT  = 5
    NOT_SUPPORTED   = 6
    CALIBRATION_REQ = 7


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def magnitude(self) -> float:
        return (self.x**2 + self.y**2 + self.z**2) ** 0.5


@dataclass
class Quaternion:
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def is_normalized(self, tol: float = 1e-4) -> bool:
        norm = (self.w**2 + self.x**2 + self.y**2 + self.z**2) ** 0.5
        return abs(norm - 1.0) < tol


@dataclass
class JointLimits:
    pos_min:         float
    pos_max:         float
    vel_max:         float
    torque_max:      float
    current_max:     float
    temperature_max: float
    accel_max:       Optional[float] = None
    """
    Accélération maximale autorisée (rad/s² ou m/s²), dérivée du delta
    de vitesse entre deux commandes successives. None = non contrainte
    (comportement historique inchangé — rétrocompatible avec tout
    JointLimits existant qui ne spécifie pas ce champ).
    """
    position_delta_max: Optional[float] = None
    """
    Écart maximal autorisé, en une seule commande, entre la position
    consigne et la position mesurée actuelle (rad ou m) — ajouté suite
    à un audit de sécurité (C5) : en contrôle de position, le couple
    physique réellement produit n'est PAS command.torque (souvent None
    dans ce mode) mais celui du contrôleur PD embarqué du moteur,
    τ ≈ kp × (q_consigne − q_mesuré) + ... Une commande de position
    parfaitement dans [pos_min, pos_max] peut donc commander un couple
    bien au-delà de torque_max si l'écart avec la position actuelle
    est trop grand.

    Se calcule comme `torque_max / kp` (borne directement le couple
    produit par le terme proportionnel du PD, en ignorant le terme
    dérivé — approximation conservatrice qui suffit à éliminer le
    risque décrit ci-dessus) — voir g1_dds_actuator.py pour le calcul
    réel à partir des gains Kp utilisés. None = non contrainte
    (rétrocompatible : tout JointLimits existant continue de fonctionner
    à l'identique tant que ce champ n'est pas renseigné).
    """


@dataclass
class ActuatorCommand:
    mode:         ControlMode
    position:     Optional[float] = None
    velocity:     Optional[float] = None
    torque:       Optional[float] = None
    current:      Optional[float] = None
    stiffness:    Optional[float] = None
    damping:      Optional[float] = None
    timestamp_ns: int = field(default_factory=time.time_ns)


@dataclass
class ActuatorFeedback:
    joint_id:     str
    state:        ActuatorState
    position:     float
    velocity:     float
    torque:       float
    current:      float
    temperature:  float
    voltage:      float
    error_code:   int = 0
    timestamp_ns: int = field(default_factory=time.time_ns)


@dataclass
class IMUData:
    linear_acceleration: Vector3
    angular_velocity:    Vector3
    orientation:         Quaternion
    timestamp_ns: int = field(default_factory=time.time_ns)


@dataclass
class ForceTorqueData:
    force:        Vector3
    torque:       Vector3
    timestamp_ns: int = field(default_factory=time.time_ns)


@dataclass
class JointDefinition:
    joint_id:             str
    joint_type:           JointType
    limits:               JointLimits
    parent_link:          str
    child_link:           str
    default_control_mode: ControlMode = ControlMode.POSITION
