"""
AXIS-OS — HAL Core Types
=========================
Types universels partagés par toute la pile logicielle d'AXIS-OS : états,
modes de contrôle et codes de retour des moteurs, mesures des capteurs
(centrale inertielle, force, caméra, son, lidar, c'est-à-dire le laser
qui mesure les distances autour du robot) et description des
articulations. Ce sont les données qui traversent le HAL (Hardware
Abstraction Layer, la couche d'abstraction matérielle : le contrat commun
entre AXIS-OS et le matériel de chaque robot, défini dans interfaces.py).

Aucune dépendance externe : bibliothèque standard de Python uniquement.

Règle absolue : ce fichier ne doit jamais être modifié sans mettre à jour
tous ses implémenteurs, c'est-à-dire tous les drivers (le code qui pilote
un modèle de robot précis) et tout le code qui s'appuie sur ces types.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional
import time


class JointType(Enum):
    """Type mécanique d'une articulation. Ce sont les catégories des
    fichiers URDF (le format standard de description d'un robot, pièce
    par pièce et articulation par articulation)."""
    REVOLUTE   = auto()   # rotation autour d'un axe, entre deux butées
    PRISMATIC  = auto()   # translation le long d'un axe
    FIXED      = auto()   # liaison rigide, aucun mouvement
    CONTINUOUS = auto()   # rotation sans butée (une roue, par exemple)


class SensorType(Enum):
    """Sorte de capteur, déclarée par chaque ISensor (voir
    interfaces.py)."""
    IMU          = auto()   # centrale inertielle : accélérations, rotation
    FORCE_TORQUE = auto()   # capteur de force et de couple
    ENCODER      = auto()   # codeur : mesure la position d'un moteur
    CAMERA_RGB   = auto()   # caméra couleur
    CAMERA_DEPTH = auto()   # caméra de profondeur (distance par pixel)
    TACTILE      = auto()   # capteur de toucher
    TEMPERATURE  = auto()
    MICROPHONE   = auto()
    GPS          = auto()   # positionnement par satellite
    LIDAR        = auto()   # laser qui mesure les distances autour du robot


class ActuatorState(Enum):
    """État d'un actionneur (le moteur d'une articulation).

    IActuator.is_ready() n'est vrai qu'en IDLE, et is_faulted() est vrai
    en FAULT comme en EMERGENCY_STOP (voir interfaces.py).
    """
    UNINITIALIZED  = auto()   # pas encore initialisé
    IDLE           = auto()   # prêt, aucune consigne en cours
    ACTIVE         = auto()   # a accepté au moins une consigne
    FAULT          = auto()   # en panne
    EMERGENCY_STOP = auto()   # arrêt d'urgence (e-stop) : refuse tout
    CALIBRATING    = auto()   # calibration en cours


class ControlMode(Enum):
    """Mode de contrôle d'un moteur : la grandeur qu'il cherche à
    suivre."""
    POSITION  = auto()   # une position cible
    VELOCITY  = auto()   # une vitesse cible
    TORQUE    = auto()   # un couple cible
    IMPEDANCE = auto()   # comportement de ressort : raideur, amortissement
    CURRENT   = auto()   # un courant électrique cible


class HALStatus(Enum):
    """Code de retour de toutes les opérations du HAL. Seul OK signifie
    que l'opération a abouti."""
    OK              = 0
    ERROR_COMM      = 1   # échec de communication avec le matériel
    ERROR_TIMEOUT   = 2   # pas de réponse dans le délai imparti
    ERROR_LIMIT     = 3   # consigne refusée par le garde-fou de sécurité
    ERROR_FAULT     = 4   # matériel en panne ou en arrêt d'urgence
    ERROR_NOT_INIT  = 5   # composant pas encore initialisé
    NOT_SUPPORTED   = 6   # opération non prise en charge par ce matériel
    CALIBRATION_REQ = 7   # calibration nécessaire avant usage


@dataclass
class Vector3:
    """Vecteur à trois composantes (accélération, vitesse angulaire,
    force, couple ou direction, selon le champ qui le porte).
    magnitude() renvoie sa norme."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def magnitude(self) -> float:
        return (self.x**2 + self.y**2 + self.z**2) ** 0.5


@dataclass
class Quaternion:
    """Orientation sous forme de quaternion (w, x, y, z) : quatre nombres
    qui décrivent une rotation dans l'espace. La valeur par défaut est la
    rotation nulle.

    Seul un quaternion de norme 1 représente une rotation valide :
    is_normalized() le vérifie, à `tol` près.
    """
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def is_normalized(self, tol: float = 1e-4) -> bool:
        norm = (self.w**2 + self.x**2 + self.y**2 + self.z**2) ** 0.5
        return abs(norm - 1.0) < tol


@dataclass
class JointLimits:
    """Limites physiques d'une articulation : position (pos_min,
    pos_max), vitesse, couple, courant et température maximaux, plus
    deux limites facultatives (accel_max, position_delta_max) et un
    indicateur (current_is_measured), décrits champ par champ
    ci-dessous.

    JointSafetyGuard (axis_os/hal/safety/limits.py), le garde-fou de
    chaque articulation, s'en sert pour valider chaque commande
    (validate_command()) et pour surveiller les mesures renvoyées par le
    moteur (check_feedback_health()).
    """
    pos_min:         float
    pos_max:         float
    vel_max:         float
    torque_max:      float
    current_max:     float
    temperature_max: float
    accel_max:       Optional[float] = None
    """
    Accélération maximale autorisée (rad/s² ou m/s²), déduite de l'écart
    de vitesse entre deux commandes successives. None : pas de limite,
    comme avant l'ajout de ce champ ; tout JointLimits existant qui ne
    le renseigne pas garde donc le même comportement.
    """
    current_is_measured: bool = False
    """
    True si `feedback.current` est une vraie mesure matérielle, False
    si c'est une valeur estimée ou fixée dans le code.

    Pourquoi ce champ : un second audit externe du code a constaté que
    la détection de surintensité `CURRENT_OVERLOAD`, ajoutée après le
    premier audit pour vérifier aussi le courant mesuré (et plus
    seulement le courant commandé), surveillait sur le G1 réel
    (l'humanoïde Unitree G1, robot de référence d'AXIS-OS) une valeur
    toujours égale à 0.0. En effet, `MotorState_`, le message DDS (le
    système de messages par lequel on parle aux moteurs du G1) qui
    rapporte l'état de chaque moteur, ne fournit pas le courant (voir
    `G1DDSActuator.get_feedback()`). Ce garde-fou ne pouvait donc, par
    construction, jamais se déclencher : c'est pire que pas de
    garde-fou, car il rassure sans jamais protéger. Détails dans
    CHANGELOG.md, sections « audit externe, vague 3 » et « suite au
    deuxième audit externe ».

    False par défaut, pour rester compatible avec l'existant :
    `JointSafetyGuard.check_feedback_health()` n'émet `CURRENT_OVERLOAD`
    que si ce champ vaut explicitement True. Ce n'est le cas qu'en
    simulation (`G1SimulatedActuator`, où le courant est déduit du
    couple simulé, voir `robot.py`), jamais avec le pilote du robot réel
    tant qu'aucun vrai capteur de courant n'y est branché.
    """
    position_delta_max: Optional[float] = None
    """
    Écart maximal autorisé, en une seule commande, entre la position
    consigne et la position mesurée actuelle (rad ou m).

    Pourquoi : un audit de sécurité externe a classé ce risque comme
    critique (CHANGELOG.md, « audit externe, vague 2 »). En contrôle de
    position, le couple réellement produit n'est pas command.torque
    (souvent None dans ce mode), mais celui du contrôleur PD
    (proportionnel-dérivé) embarqué dans le moteur :
    τ ≈ kp × (q_consigne − q_mesuré) + ..., où τ (tau) est le couple,
    q la position de l'articulation et kp le gain proportionnel ; les
    termes omis dépendent de la vitesse (gain kd). Une commande de
    position bien comprise dans [pos_min, pos_max] peut donc demander
    un couple très supérieur à torque_max si elle s'éloigne trop de la
    position actuelle.

    La valeur se calcule comme `torque_max / kp` : elle borne
    directement le couple produit par le terme proportionnel du PD. Le
    terme dérivé est ignoré ; cette approximation prudente suffit à
    écarter le risque décrit ci-dessus. Le calcul réel, à partir des
    gains kp utilisés, se trouve dans g1_dds_actuator.py. None : pas de
    limite ; tout JointLimits existant qui ne renseigne pas ce champ
    fonctionne exactement comme avant.
    """

@dataclass
class ActuatorCommand:
    """Consigne envoyée à un actionneur (IActuator.send_command()).

    `mode` (ControlMode) indique la grandeur pilotée ; les champs qui ne
    servent pas à ce mode peuvent rester à None. stiffness et damping
    sont la raideur et l'amortissement demandés. timestamp_ns est
    l'instant de création de la consigne, en nanosecondes.
    """
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
    """Dernière mesure d'un actionneur (IActuator.get_feedback()).

    Attention : sur le G1 réel, `current` vaut toujours 0.0, car le
    message d'état des moteurs ne fournit pas le courant (voir
    JointLimits.current_is_measured ci-dessus). error_code est le code
    d'erreur rapporté pour le moteur (0 par défaut).
    """
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
    """Mesure d'une centrale inertielle, ou IMU (Inertial Measurement
    Unit : le capteur qui mesure accélérations, vitesses de rotation et
    orientation).

    Pourquoi la convention de repère est désormais écrite ici, alors
    qu'elle était implicite auparavant : une vitesse angulaire exprimée
    dans le mauvais repère fait tomber le robot. Mesuré dans Isaac Sim
    (le simulateur robotique de NVIDIA) : sans la conversion décrite
    plus bas, le G1 tombe après 103° de virage
    (docs/validation_locomotion.md).

    Convention :
      - angular_velocity    : dans le repère du capteur (donc du corps),
                              comme la rend toute centrale inertielle
                              réelle ;
      - linear_acceleration : de même, dans le repère du capteur ;
      - orientation         : rotation monde <- capteur, (w, x, y, z).

    Un driver de simulation qui lit la vitesse angulaire dans le repère
    du monde (Isaac Sim le fait) doit la convertir avant de remplir ce
    champ. La conversion appartient à la source, jamais au consommateur :
    sinon, le même consommateur branché sur une vraie centrale
    convertirait une seconde fois.

    Attention : sur le G1 réel, la centrale est dans le torse, au-dessus
    de l'articulation de la taille. Une politique (le réseau de neurones
    entraîné qui commande le robot, par exemple pour la marche) qui
    attend le repère du bassin doit en plus compenser l'angle et la
    vitesse de la taille (voir transform_imu_data dans
    unitree_rl_gym/deploy_real, le dépôt public d'apprentissage par
    renforcement d'Unitree).
    """
    linear_acceleration: Vector3
    angular_velocity:    Vector3
    orientation:         Quaternion
    timestamp_ns: int = field(default_factory=time.time_ns)


@dataclass
class ForceTorqueData:
    """Mesure d'un capteur de force et de couple : une force et un
    couple, chacun sur trois axes."""
    force:        Vector3
    torque:       Vector3
    timestamp_ns: int = field(default_factory=time.time_ns)


@dataclass
class CameraFrame:
    """
    Une image capturée.

    Largeur, hauteur et encodage sont des champs explicites, plutôt que
    déduits de la forme d'un tableau, pour que le contrat du HAL reste
    indépendant de toute bibliothèque de traitement d'image (numpy,
    PIL...). `data` contient les octets bruts, dont le format est donné
    par `encoding` (par exemple "rgb8", "bgr8", "depth16"). C'est le même
    principe que le message sensor_msgs/Image de ROS (Robot Operating
    System, l'intergiciel robotique le plus répandu), plutôt qu'un format
    réinventé sans raison.
    """
    width:        int
    height:       int
    encoding:     str
    data:         bytes
    timestamp_ns: int = field(default_factory=time.time_ns)


@dataclass(frozen=True)
class CameraIntrinsics:
    """
    Calibration d'une caméra : ce qui permet de passer d'un pixel à une
    direction dans le repère de la caméra.

    Pourquoi c'est déclaré ici : sans ces paramètres, une détection ne
    produit qu'une boîte en pixels. Impossible alors de dire où se trouve
    l'objet dans l'espace, donc impossible pour la préhension (la saisie
    d'objets) de savoir où tendre la main. C'est le chaînon qui relie
    perception et manipulation ; il est déclaré au niveau du contrat du
    HAL plutôt que réinventé dans chaque service.

    fx et fy : distances focales, en pixels. cx et cy : centre optique,
    qui n'est pas exactement le centre de l'image sur une vraie caméra.
    distortion : coefficients du modèle de distorsion de l'objectif,
    vides si l'image est déjà rectifiée (corrigée de cette distorsion).
    C'est le cas fréquent des caméras qui rectifient en interne, et le
    cas systématique d'une caméra simulée, où l'optique est parfaite par
    construction.

    Attention : ces valeurs sont propres à chaque exemplaire de caméra,
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
        """Direction (non normalisée) du pixel (u, v) dans le repère de
        la caméra, en convention optique : x vers la droite, y vers le
        bas, z vers l'avant.

        Suppose l'image rectifiée : la distorsion n'est pas corrigée ici.
        Ce serait le rôle d'un service de perception doté de la
        bibliothèque adéquate, pas du contrat du HAL."""
        return Vector3((u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0)


@dataclass
class AudioChunk:
    """
    Segment audio brut.

    La fréquence d'échantillonnage (sample_rate), le nombre de canaux
    (channels) et la taille d'un échantillon (sample_width) sont
    explicites, plutôt qu'un format de fichier (WAV, etc.) : c'est un
    contrat de flux en temps réel du HAL, pas un format d'échange de
    fichiers. PCM désigne le son numérique non compressé ; 2 octets par
    échantillon correspondent au PCM 16 bits.
    """
    sample_rate:  int
    channels:     int
    sample_width: int    # octets par échantillon (2 = PCM 16 bits)
    data:         bytes
    timestamp_ns: int = field(default_factory=time.time_ns)


class JointGroup(Enum):
    """Groupe fonctionnel d'une articulation, déclaré par le driver.

    Pourquoi : avant ce type, le SDK (le kit de développement des apps,
    axis_os/store/sdk/) devinait le groupe à partir de mots-clés anglais
    du nom ("shoulder", "knee"...), codés en dur. Ce devinement supposait
    en silence un humanoïde aux articulations nommées en anglais :

    - un bras industriel aux axes nommés joint_1..joint_6 ne
      correspondait à aucun groupe : move_arms() renvoyait {} sans
      erreur, et l'app ne bougeait rien, en silence ;
    - un quadrupède aurait été classé « jambes » par accident, ce qui
      marche à moitié, pire qu'un échec franc ;
    - et sur le G1 lui-même, waist_yaw_joint (la rotation de la taille)
      ne contenait aucun des six mots-clés : le torse était inatteignable
      pour toute app, sauf via move_all.

    (move_arms(), move_legs() et move_all() sont les méthodes de
    RobotContext, dans le SDK, par lesquelles une app fait bouger le
    robot.)

    C'est au driver de savoir ce que son robot possède, exactement comme
    c'est lui qui connaît ses conventions de signe. Le SDK ne fait plus
    que router ce qui lui est déclaré.

    OTHER existe pour ne forcer personne à mentir : une articulation qui
    ne sert ni à la manipulation ni à la locomotion (torse, tête, mât) se
    déclare OTHER et reste accessible via move_all, sans être faussement
    rangée ailleurs.
    """
    MANIPULATION = auto()   # bras, pinces — ce que move_arms() pilote
    LOCOMOTION   = auto()   # jambes, pattes, roues — move_legs()
    OTHER        = auto()   # torse, tête, mât... move_all() uniquement


# Mots-clés de repli, utilisés seulement si un driver ne déclare pas
# encore ses groupes. Ils reproduisent l'ancien comportement pour ne
# casser aucun driver existant, mais restent un pis-aller : tout driver
# devrait renseigner JointDefinition.group.
_FALLBACK_MANIPULATION = ("shoulder", "elbow", "wrist")
_FALLBACK_LOCOMOTION   = ("hip", "knee", "ankle")


def infer_joint_group(joint_id: str) -> JointGroup:
    """Repli par mots-clés pour une articulation sans groupe déclaré.

    Ne jamais l'appeler dans un nouveau driver : déclarer plutôt le
    groupe sur JointDefinition. Cette fonction n'existe que pour la
    transition.
    """
    low = joint_id.lower()
    if any(k in low for k in _FALLBACK_MANIPULATION):
        return JointGroup.MANIPULATION
    if any(k in low for k in _FALLBACK_LOCOMOTION):
        return JointGroup.LOCOMOTION
    return JointGroup.OTHER


@dataclass
class JointDefinition:
    """Description d'une articulation, fournie par le driver
    (IRobotHAL.get_joint_definitions()).

    parent_link et child_link sont les deux pièces du robot que
    l'articulation relie, comme dans un fichier URDF. group est le groupe
    fonctionnel déclaré (voir JointGroup) ; s'il vaut None, AXIS-OS se
    rabat sur infer_joint_group().
    """
    joint_id:             str
    joint_type:           JointType
    limits:               JointLimits
    parent_link:          str
    child_link:           str
    default_control_mode: ControlMode = ControlMode.POSITION
    group:                Optional[JointGroup] = None



@dataclass
class LidarScan:
    """Un balayage lidar (le capteur laser qui mesure les distances tout
    autour du robot) : un nuage de points dans le repère du capteur.

    points : tableau N x 3 (x, y, z), en mètres. Le repère du capteur est
        déclaré dans l'arbre des repères (FrameTree, voir frames.py),
        comme celui d'une caméra : il n'est jamais supposé.
    """
    points: object                      # numpy.ndarray (N, 3)
    frame_id: str = "lidar"
    timestamp_ns: int = field(default_factory=time.time_ns)
