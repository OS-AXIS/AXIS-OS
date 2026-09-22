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
    current_is_measured: bool = False
    """
    True si `feedback.current` provient d'une vraie mesure matérielle,
    False si c'est une valeur estimée ou codée en dur — trouvé
    nécessaire par un second audit externe (2026-09-14) : le check
    `CURRENT_OVERLOAD` (ajouté en M4) surveillait sur le G1 réel une
    valeur TOUJOURS égale à 0.0 (`MotorState_` n'expose pas le
    courant — voir `G1DDSActuator.get_feedback()`), donc un garde-fou
    qui ne peut STRUCTURELLEMENT jamais se déclencher — pire qu'absent,
    car il rassure sans jamais protéger. `False` par défaut
    (rétrocompatible) : `JointSafetyGuard.check_feedback_health()`
    n'émet `CURRENT_OVERLOAD` QUE si ce champ est explicitement `True`,
    ce qui n'est le cas qu'en simulation (`G1SimulatedActuator`, où le
    courant est dérivé du couple simulé — voir `robot.py`), jamais sur
    le driver matériel réel tant qu'aucun vrai capteur de courant n'y
    est branché.
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
    """Mesure d'une centrale inertielle.

    CONVENTION DE REPÈRE, désormais explicite :
      - angular_velocity    : dans le repère du CAPTEUR (du corps), comme
                              le rend TOUTE centrale inertielle réelle ;
      - linear_acceleration : idem, repère du capteur ;
      - orientation         : rotation monde <- capteur, (w, x, y, z).

    Un driver de SIMULATION qui lit la vitesse angulaire dans le repère du
    monde (Isaac Sim le fait) doit la CONVERTIR avant de remplir ce champ.
    La conversion appartient à la source, jamais au consommateur : sinon,
    le même consommateur branché sur une vraie centrale convertirait une
    seconde fois. Mesuré : une vitesse angulaire dans le mauvais repère
    fait tomber le G1 après 103° de virage (docs/validation_locomotion.md).

    Attention : sur le G1 réel, la centrale est dans le TORSE, au-dessus
    de l'articulation de taille. Une politique qui attend le repère du
    BASSIN doit en plus compenser l'angle et la vitesse de la taille
    (voir transform_imu_data dans unitree_rl_gym/deploy_real).
    """
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
class CameraFrame:
    """
    Une image capturée — largeur/hauteur/canaux explicites plutôt
    qu'implicites dans la forme d'un tableau, pour rester agnostique
    à toute bibliothèque de traitement d'image précise (numpy, PIL...)
    au niveau du contrat HAL lui-même. `data` : octets bruts, format
    et encodage définis par `encoding` (ex: "rgb8", "bgr8", "depth16")
    — même esprit que sensor_msgs/Image en ROS, pas réinventé
    différemment sans raison.
    """
    width:        int
    height:       int
    encoding:     str
    data:         bytes
    timestamp_ns: int = field(default_factory=time.time_ns)


@dataclass(frozen=True)
class CameraIntrinsics:
    """
    Calibration d'une caméra — ce qui permet de passer d'un PIXEL à une
    DIRECTION dans le repère de la caméra.

    Sans ces paramètres, une détection ne produit qu'une boîte en
    pixels : impossible de dire OÙ se trouve l'objet dans l'espace,
    donc impossible pour la préhension de savoir où tendre la main.
    C'est le chaînon qui relie perception et manipulation ; il est
    déclaré ici, au niveau du contrat HAL, plutôt que réinventé dans
    chaque service.

    fx/fy : distances focales en pixels. cx/cy : centre optique, qui
    n'est pas exactement le centre de l'image sur une vraie caméra.
    distortion : coefficients du modèle de distorsion, vides si l'image
    est déjà rectifiée — cas fréquent des caméras qui rectifient en
    interne, et cas systématique d'une caméra simulée, où l'optique est
    parfaite par construction.

    ATTENTION : ces valeurs sont PROPRES À CHAQUE EXEMPLAIRE de caméra,
    pas au modèle. Deux G1 sortis de la même chaîne n'ont pas la même
    calibration. Elles doivent venir du capteur ou d'un fichier de
    calibration, jamais être codées en dur dans le driver.
    """
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple = ()

    def pixel_to_ray(self, u: float, v: float) -> "Vector3":
        """Direction (non normalisée) dans le repère caméra, en
        convention optique : x vers la droite, y vers le bas, z vers
        l'avant. Suppose l'image rectifiée — la distorsion n'est PAS
        corrigée ici, ce serait le rôle d'un service de perception
        avec la bibliothèque adéquate, pas du contrat HAL."""
        return Vector3((u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0)


@dataclass
class AudioChunk:
    """
    Segment audio brut — sample_rate/channels/sample_width explicites
    plutôt qu'un format fichier (WAV, etc.), pour rester un vrai
    contrat HAL de flux temps réel, pas un format d'échange fichier.
    """
    sample_rate:  int
    channels:     int
    sample_width: int    # octets par échantillon (2 = 16-bit PCM)
    data:         bytes
    timestamp_ns: int = field(default_factory=time.time_ns)


class JointGroup(Enum):
    """Groupe fonctionnel d'une articulation, DÉCLARÉ PAR LE DRIVER.

    Remplace le devinement par mots-clés anglais ("shoulder", "knee"...)
    qui était codé en dur dans le SDK. Ce devinement supposait
    silencieusement un humanoïde nommé à l'anglaise :

    - un bras industriel aux axes nommés joint_1..joint_6 ne
      correspondait à AUCUN groupe : move_arms() retournait {} sans
      erreur, et l'app ne bougeait rien, en silence ;
    - un quadrupède aurait été classé "jambes" par accident, ce qui
      marche à moitié — pire qu'un échec franc ;
    - et sur le G1 lui-même, waist_yaw_joint ne contenait aucun des six
      mots-clés : le torse était inatteignable pour toute app, sauf via
      move_all.

    C'est au driver de savoir ce que SON robot possède, exactement comme
    c'est lui qui connaît ses conventions de signe. Le SDK ne fait plus
    que router ce qui lui est déclaré.

    OTHER existe pour ne forcer personne à mentir : un joint qui n'est
    ni manipulation ni locomotion (torse, tête, mât) se déclare OTHER et
    reste accessible via move_all, sans être faussement rangé ailleurs.
    """
    MANIPULATION = auto()   # bras, pinces — ce que move_arms() pilote
    LOCOMOTION   = auto()   # jambes, pattes, roues — move_legs()
    OTHER        = auto()   # torse, tête, mât... move_all() uniquement


# Mots-clés de REPLI, utilisés seulement si un driver ne déclare pas
# encore ses groupes. Reproduit l'ancien comportement pour ne casser
# aucun driver existant, mais reste un pis-aller : tout driver devrait
# renseigner JointDefinition.group.
_FALLBACK_MANIPULATION = ("shoulder", "elbow", "wrist")
_FALLBACK_LOCOMOTION   = ("hip", "knee", "ankle")


def infer_joint_group(joint_id: str) -> JointGroup:
    """Repli par mots-clés pour un joint sans groupe déclaré.

    Ne jamais appeler ça dans un driver neuf : déclarer le groupe sur
    JointDefinition. Cette fonction n'existe que pour la transition.
    """
    low = joint_id.lower()
    if any(k in low for k in _FALLBACK_MANIPULATION):
        return JointGroup.MANIPULATION
    if any(k in low for k in _FALLBACK_LOCOMOTION):
        return JointGroup.LOCOMOTION
    return JointGroup.OTHER


@dataclass
class JointDefinition:
    joint_id:             str
    joint_type:           JointType
    limits:               JointLimits
    parent_link:          str
    child_link:           str
    default_control_mode: ControlMode = ControlMode.POSITION
    group:                Optional[JointGroup] = None
