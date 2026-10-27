"""
AXIS-OS — HAL Abstract Interfaces
===================================
Le contrat entre AXIS-OS et le matériel. Le HAL (Hardware Abstraction
Layer, la couche d'abstraction matérielle) est le seul chemin par lequel
le logiciel atteint les moteurs et les capteurs du robot. Toute
implémentation matérielle hérite des classes de ce fichier : c'est le
« driver » d'un modèle de robot, c'est-à-dire le code qui sait parler à
ce robot précis (par exemple le Unitree G1, l'humanoïde de référence
d'AXIS-OS, dans axis_os/hal/drivers/unitree_g1/).

Aucune logique métier ici : seulement des signatures, plus quelques
comportements par défaut volontairement prudents (état d'arrêt
d'urgence, suivi de mouvement absent, capacités vides).

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .motion_state import MotionStateTracker
from abc import ABC, abstractmethod
from typing import Optional, List, Dict

from .types import (
    ActuatorCommand, ActuatorFeedback, ActuatorState,
    ControlMode, HALStatus, IMUData, ForceTorqueData,
    JointDefinition, SensorType, CameraFrame, CameraIntrinsics, AudioChunk,
    BatteryState,
)
from .capabilities import CapabilitySet


class IActuator(ABC):
    """
    Un actionneur : le moteur qui fait bouger une articulation (un
    « joint ») du robot. Une instance par articulation.

    Les méthodes renvoient un HALStatus (voir types.py) qui dit si
    l'opération a abouti. initialize() et shutdown() démarrent et
    arrêtent le moteur, calibrate() le calibre, set_control_mode() choisit
    le mode de contrôle (position, vitesse, couple...), send_command()
    envoie une consigne, emergency_stop() déclenche son arrêt d'urgence
    (e-stop) et get_feedback() renvoie la dernière mesure du moteur, ou
    None s'il n'y en a pas.

    L'état courant (ActuatorState) est tenu par l'implémentation dans
    self._state : is_ready() n'est vrai qu'à l'état IDLE (prêt), et
    is_faulted() est vrai en cas de panne (FAULT) ou d'arrêt d'urgence
    (EMERGENCY_STOP).
    """
    def __init__(self, joint_id: str, definition: JointDefinition):
        self.joint_id   = joint_id
        self.definition = definition
        self._state     = ActuatorState.UNINITIALIZED

    @abstractmethod
    def initialize(self) -> HALStatus: ...

    @abstractmethod
    def shutdown(self) -> HALStatus: ...

    @abstractmethod
    def calibrate(self) -> HALStatus: ...

    @abstractmethod
    def set_control_mode(self, mode: ControlMode) -> HALStatus: ...

    @abstractmethod
    def send_command(self, command: ActuatorCommand, defer_publish: bool = False) -> HALStatus:
        """
        Envoie une consigne au moteur.

        Pourquoi `defer_publish` existe : sur le G1, un seul message DDS
        (le système de messages par lequel on parle aux moteurs du G1)
        transporte les consignes de tous les moteurs à la fois. Publier
        ce message après chaque articulation envoyait au robot, à chaque
        cycle, des poses intermédiaires incohérentes : une partie des
        articulations sur la nouvelle consigne, le reste encore sur
        l'ancienne. Le contrôleur embarqué du G1, plus rapide que la
        boucle de commande logicielle, peut figer l'une de ces poses
        incohérentes ; sur un robot debout sur ses deux jambes, cela
        crée un déséquilibre passager à chaque cycle. Ce paramètre a été
        ajouté par un correctif de sécurité ; la justification complète
        est dans G1DDSActuator.send_command() et dans CHANGELOG.md
        (section « suite au deuxième audit externe »).

        Contrat : toute implémentation qui publie sur un bus partagé,
        où un seul message porte plusieurs actionneurs (comme G1DDSHub),
        DOIT respecter ce paramètre. Quand il vaut True, elle écrit la
        commande dans le tampon sans publier ; c'est l'appelant qui
        publie une seule fois, après avoir rempli le tampon pour toutes
        les articulations (voir UnitreeG1HAL.send_joint_commands()).
        Une implémentation sans notion de publication groupée (par
        exemple G1SimulatedActuator) peut l'ignorer sans risque : la
        commande y prend effet immédiatement de toute façon.
        """
        ...

    @abstractmethod
    def emergency_stop(self) -> HALStatus: ...

    @abstractmethod
    def get_feedback(self) -> Optional[ActuatorFeedback]: ...

    @property
    def state(self) -> ActuatorState:
        return self._state

    def is_ready(self) -> bool:
        return self._state == ActuatorState.IDLE

    def is_faulted(self) -> bool:
        return self._state in (ActuatorState.FAULT, ActuatorState.EMERGENCY_STOP)


class ISensor(ABC):
    """
    Base commune des capteurs : un identifiant, un type (SensorType, voir
    types.py), un cycle de vie initialize() / shutdown() et un état de
    santé is_healthy(). Chaque sorte de capteur ajoute sa propre méthode
    de lecture dans les sous-classes ci-dessous.
    """
    def __init__(self, sensor_id: str, sensor_type: SensorType):
        self.sensor_id   = sensor_id
        self.sensor_type = sensor_type
        self._initialized = False

    @abstractmethod
    def initialize(self) -> HALStatus: ...

    @abstractmethod
    def shutdown(self) -> HALStatus: ...

    @abstractmethod
    def is_healthy(self) -> bool: ...


class ILidar(ISensor, ABC):
    """Lidar (le capteur laser qui mesure les distances autour du robot) :
    il fournit un nuage de points horodaté (LidarScan, voir types.py).

    C'est le capteur sur lequel repose la sécurité de proximité : une
    caméra ne couvre qu'environ 90° devant le robot, alors que le lidar
    voit tout autour. Un humain qui arrive de côté ou de dos reste donc
    visible.
    """

    @abstractmethod
    def scan(self) -> Optional["LidarScan"]:
        """Dernier balayage, ou None s'il n'y en a pas.

        Ne jamais renvoyer un nuage vide à la place : il serait lu comme
        « rien autour du robot », et l'absence de mesure passerait pour
        une zone libre."""


class IIMU(ISensor, ABC):
    """
    Centrale inertielle, ou IMU (Inertial Measurement Unit : le capteur
    qui mesure les accélérations, les vitesses de rotation et
    l'orientation du robot). read() renvoie la dernière mesure, ou None ;
    la convention de repère des mesures est décrite sur IMUData, dans
    types.py.
    """
    @abstractmethod
    def read(self) -> Optional[IMUData]: ...

    @abstractmethod
    def reset_orientation(self) -> HALStatus: ...


class IForceTorqueSensor(ISensor, ABC):
    """
    Capteur de force et de couple : il mesure les efforts qu'il subit.
    read() renvoie la dernière mesure, ou None ; tare() remet la mesure
    à zéro au repos, comme on tare une balance.
    """
    @abstractmethod
    def read(self) -> Optional[ForceTorqueData]: ...

    @abstractmethod
    def tare(self) -> HALStatus: ...


class ICamera(ISensor, ABC):
    """
    Une caméra. C'est un point de sécurité central de l'architecture
    (voir docs/architecture_services.md, section 2) : un service (un
    module système démarré avec AXIS-OS et partagé par toutes les apps,
    comme la perception) ne commande jamais un actionneur directement.
    Cet invariant de sécurité est documenté sur IService
    (axis_os/services/base.py), et il s'applique ici dès le niveau du
    HAL, avant même qu'un service n'existe.

    ICamera n'offre donc aucun moyen de commander un actionneur : c'est
    un pur capteur en lecture, exactement comme IIMU et
    IForceTorqueSensor ci-dessus. Aucun service ni aucune app ne peut
    « voir puis agir » en un seul appel à travers cette interface. On
    voit toujours par la caméra, et on agit toujours séparément, par
    NormalizedRobotHAL (la façade du HAL que les apps utilisent, jamais
    le driver brut).
    """
    @abstractmethod
    def capture(self) -> Optional[CameraFrame]: ...

    @abstractmethod
    def get_intrinsics(self) -> Optional[CameraIntrinsics]:
        """Calibration de CET exemplaire de caméra, ou None si elle
        n'est pas connue.

        Renvoyer None est un cas légitime et fréquent (caméra non
        calibrée, driver qui ne fournit pas la calibration) : le contrat
        impose de le dire, pas d'inventer des valeurs plausibles. Un
        service de perception qui reçoit None doit se dégrader
        honnêtement, en rendant une détection sans position 3D, plutôt
        que de produire des coordonnées fausses qu'une préhension
        prendrait au sérieux.
        """


class IMicrophone(ISensor, ABC):
    """Microphone : capture() enregistre `duration_s` secondes de son et
    renvoie un AudioChunk (voir types.py), ou None."""
    @abstractmethod
    def capture(self, duration_s: float) -> Optional[AudioChunk]: ...


class ISpeaker(ABC):
    """
    Haut-parleur : play() joue un AudioChunk (voir types.py).

    Ce n'est volontairement pas une sous-classe de ISensor : un
    haut-parleur agit sur le monde physique (il émet du son), il ne le
    perçoit pas, contrairement à ICamera, IMicrophone, IIMU et
    IForceTorqueSensor ci-dessus. Le champ `speaker_id` suit la même
    convention de nommage que le `sensor_id` des autres interfaces, mais
    ISpeaker ne prétend pas être le capteur qu'il n'est pas.
    """
    def __init__(self, speaker_id: str):
        self.speaker_id = speaker_id
        self._initialized = False

    @abstractmethod
    def initialize(self) -> HALStatus: ...

    @abstractmethod
    def shutdown(self) -> HALStatus: ...

    @abstractmethod
    def play(self, audio: AudioChunk) -> HALStatus: ...


class IRobotHAL(ABC):
    """
    Point d'entrée unique d'AXIS-OS vers le matériel d'un robot.
    Chaque constructeur livre UN fichier qui implémente cette interface
    (pour le G1 : UnitreeG1HAL, dans axis_os/hal/drivers/unitree_g1/
    robot.py).

    On y trouve le cycle de vie du robot entier (initialize_all(),
    shutdown_all()), l'arrêt d'urgence global (emergency_stop_all()),
    l'accès aux actionneurs et aux capteurs, l'envoi groupé de consignes
    (send_joint_commands()), la lecture de l'état complet et la
    description du robot (articulations, informations, capacités).
    """

    def __init__(self, robot_id: str):
        self.robot_id     = robot_id
        self._initialized = False
        # État d'arrêt d'urgence (e-stop) partagé par tout le système.
        #
        # Pourquoi il est ici : un audit de sécurité externe a montré
        # qu'avant ce champ, seul WatchdogSystem._estop_fired existait.
        # Le WatchdogSystem (le « chien de garde » logiciel qui surveille
        # en continu l'état du robot) le positionne quand il
        # déclenche lui-même un e-stop. Mais le Supervisor (le composant
        # qui surveille le watchdog de l'extérieur) appelle
        # emergency_stop_all() précisément quand le watchdog est mort ou
        # bloqué, et ne positionnait jamais ce drapeau.
        # PlayStoreRuntime.run_mission() (qui lance les missions des apps
        # installées par le Play Store, la boutique d'apps d'AXIS-OS)
        # et RobotContext._require_no_estop() (le contrôle fait avant
        # chaque mouvement demandé par une app) lisaient
        # watchdog.estop_fired, et continuaient donc d'accepter des
        # missions après un e-stop du Supervisor. Le matériel restait
        # protégé, car chaque actionneur, une fois lui-même en
        # EMERGENCY_STOP, refuse individuellement toute commande avec
        # ERROR_FAULT. Mais les trois couches au-dessus (watchdog, Play
        # Store, apps) affichaient un état faux.
        #
        # L'état est porté par le HAL plutôt que par le watchdog : c'est
        # le seul composant dont le WatchdogSystem ET le Supervisor
        # détiennent tous deux une référence. Détail du correctif dans
        # CHANGELOG.md (« audit externe, vague 2 »).
        self._estop_active = False

    @property
    def estop_active(self) -> bool:
        """
        État d'e-stop (arrêt d'urgence) global, positionné par
        emergency_stop_all().

        Il est indépendant de WatchdogSystem.estop_fired, qui ne reflète
        qu'un e-stop déclenché par le watchdog lui-même (il reste ensuite
        à True) et suit son propre cycle de vie. Toute couche qui décide
        d'autoriser ou non une action doit donc consulter CELUI-CI en plus
        de watchdog.estop_fired : voir RobotContext._require_no_estop() et
        PlayStoreRuntime.run_mission().
        """
        return self._estop_active

    def _mark_estop_active(self) -> None:
        """
        Positionne estop_active. Chaque implémentation concrète doit
        l'appeler dans son emergency_stop_all() (voir
        UnitreeG1HAL.emergency_stop_all()).

        Ce n'est jamais automatique, pour que ce soit un geste explicite
        et visible dans chaque driver plutôt qu'un comportement caché
        dans la classe de base.

        Il n'existe volontairement aucune méthode publique inverse pour
        effacer cet état. Réarmer après un e-stop, si cela doit un jour
        exister, est une décision de sécurité à formaliser à part, pas
        un effet de bord discret de ce correctif. Le choix de ne jamais
        réarmer automatiquement est documenté dans CHANGELOG.md
        (« audit externe, vague 3 ») : un réarmement automatique
        risquerait de relancer un mouvement sur un robot dont l'état
        physique réel n'a pas été vérifié.
        """
        self._estop_active = True

    @abstractmethod
    def initialize_all(self) -> HALStatus: ...

    @abstractmethod
    def shutdown_all(self) -> HALStatus: ...

    @abstractmethod
    def emergency_stop_all(self) -> HALStatus:
        """Arrêt d'urgence global de tous les actionneurs. Doit se
        terminer en moins de 10 ms."""
        ...

    @abstractmethod
    def get_actuator(self, joint_id: str) -> Optional[IActuator]: ...

    @abstractmethod
    def get_all_actuators(self) -> Dict[str, IActuator]: ...

    @abstractmethod
    def get_imu(self, imu_id: str = "base_imu") -> Optional[IIMU]: ...

    @abstractmethod
    def get_force_torque(self, sensor_id: str) -> Optional[IForceTorqueSensor]: ...

    @abstractmethod
    def send_joint_commands(
        self, commands: Dict[str, ActuatorCommand]
    ) -> Dict[str, HALStatus]: ...

    @abstractmethod
    def get_full_state(self) -> Dict[str, ActuatorFeedback]: ...

    @abstractmethod
    def get_joint_definitions(self) -> List[JointDefinition]: ...

    @abstractmethod
    def get_robot_info(self) -> Dict[str, str]:
        """Doit renvoyer au moins les clés : manufacturer (fabricant),
        model (modèle), serial_number (numéro de série), hal_version et
        dof (nombre de degrés de liberté, c'est-à-dire d'articulations
        pilotables)."""
        ...

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    def get_motion_tracker(self) -> Optional["MotionStateTracker"]:
        """
        Renvoie le MotionStateTracker de ce robot, si le driver en
        possède un. Ce composant mémorise les commandes et mesures
        précédentes de chaque articulation, pour calculer ce qui a changé
        d'une commande à la suivante.

        Implémentation par défaut : None. Un backend qui n'a pas encore
        ce composant (par exemple un ancien driver de simulateur) reste
        valide : JointSafetyGuard (le garde-fou de chaque articulation)
        et WatchdogSystem traitent l'absence de tracker comme « limitation
        de la variation entre deux commandes (rate limiting) et détection
        d'écart entre commande et mesure désactivées », et non comme une
        erreur.

        Décision de conception arrêtée : ce composant est générique
        (hal/core/motion_state.py) et vit au niveau d'IRobotHAL, jamais
        dans un driver particulier. Le raisonnement complet est dans la
        documentation de ce module.
        """
        return None

    def get_capabilities(self) -> CapabilitySet:
        """
        Renvoie l'ensemble des capacités physiques de ce robot (LEGS,
        ARMS, WHEELS, CAMERA, etc.). Le raisonnement complet est dans
        capabilities.py.

        Implémentation par défaut : un ensemble vide, sur le même
        principe que get_motion_tracker() ci-dessus. Un driver qui n'a
        pas encore déclaré ses capacités reste valide, et le défaut
        (une capacité est absente tant qu'elle n'est pas déclarée
        explicitement) est le choix prudent : mieux vaut qu'une
        vérification de capacité échoue par excès de prudence sur un
        robot mal déclaré que l'inverse.
        """
        return CapabilitySet.empty()

    def get_battery(self) -> Optional[BatteryState]:
        """
        Charge de la batterie, mesurée par le driver, ou None si ce
        robot ne sait pas la lire (implémentation par défaut). None veut
        dire « inconnue » : jamais une valeur inventée, qui ferait
        continuer une app sur une batterie vide.
        """
        return None
