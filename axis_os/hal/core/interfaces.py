"""
AXIS-OS — HAL Abstract Interfaces
===================================
LE CONTRAT. Toute implémentation hardware hérite de ces classes.
Aucune logique métier ici — que des signatures.

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
    JointDefinition, SensorType, CameraFrame, CameraIntrinsics, AudioChunk
)
from .capabilities import CapabilitySet


class IActuator(ABC):
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
        `defer_publish` (ajouté par correctif de sécurité, voir
        G1DDSActuator.send_command() pour la justification complète) :
        toute implémentation qui publie sur un bus partagé transportant
        plusieurs actionneurs dans un seul message (comme G1DDSHub)
        DOIT le respecter — écrire dans le buffer sans publier quand
        True. Les implémentations sans notion de "publication groupée"
        (ex: G1SimulatedActuator) peuvent l'ignorer sans risque, la
        commande étant de toute façon immédiatement effective.
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


class IIMU(ISensor, ABC):
    @abstractmethod
    def read(self) -> Optional[IMUData]: ...

    @abstractmethod
    def reset_orientation(self) -> HALStatus: ...


class IForceTorqueSensor(ISensor, ABC):
    @abstractmethod
    def read(self) -> Optional[ForceTorqueData]: ...

    @abstractmethod
    def tare(self) -> HALStatus: ...


class ICamera(ISensor, ABC):
    """
    Point de sécurité central (voir docs/architecture_services*.md
    §2) : ICamera n'expose AUCUN moyen de commander un actionneur —
    c'est un pur capteur en lecture, exactement comme IIMU/
    IForceTorqueSensor ci-dessus. Aucun service ou app ne peut
    « voir puis agir » en un seul appel à travers cette interface ;
    voir toujours, agir toujours via NormalizedRobotHAL séparément —
    cohérent avec l'invariant de sécurité documenté sur IService
    (axis_os/services/base.py) : un service ne commande jamais un
    actionneur directement, et ce même principe s'applique déjà ici,
    au niveau HAL, avant même qu'un service existe.
    """
    @abstractmethod
    def capture(self) -> Optional[CameraFrame]: ...

    @abstractmethod
    def get_intrinsics(self) -> Optional[CameraIntrinsics]:
        """Calibration de CET exemplaire de caméra, ou None si elle
        n'est pas connue.

        Retourner None est un cas légitime et fréquent (caméra non
        calibrée, driver qui ne l'expose pas) : le contrat impose de
        le DIRE, pas d'inventer des valeurs plausibles. Un service de
        perception qui reçoit None doit dégrader honnêtement — rendre
        une détection sans position 3D — plutôt que de produire des
        coordonnées fausses qu'une préhension prendrait au sérieux.
        """


class IMicrophone(ISensor, ABC):
    @abstractmethod
    def capture(self, duration_s: float) -> Optional[AudioChunk]: ...


class ISpeaker(ABC):
    """
    Pas de sous-classe de ISensor — un haut-parleur EXPOSE le monde
    physique (son), il ne le PERÇOIT pas, contrairement à ICamera/
    IMicrophone/IIMU/IForceTorqueSensor ci-dessus. `speaker_id` reste
    cohérent avec la convention de nommage `sensor_id` des autres
    interfaces, mais ISpeaker ne prétend pas être un capteur qu'il
    n'est pas.
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
    Point d'entrée unique de AXIS-OS vers le hardware.
    Chaque constructeur livre UN fichier qui implémente cette interface.
    """

    def __init__(self, robot_id: str):
        self.robot_id     = robot_id
        self._initialized = False
        # État e-stop partagé — correctif E10, audit de sécurité :
        # avant ce champ, seul WatchdogSystem._estop_fired existait, et
        # Supervisor.emergency_stop_all() (déclenché précisément quand
        # le watchdog est mort ou bloqué) ne le positionnait jamais.
        # PlayStoreRuntime.run_mission() et RobotContext._require_no_estop()
        # lisaient watchdog.estop_fired et continuaient d'accepter des
        # missions après un e-stop du Supervisor — le matériel restait
        # protégé (chaque actionneur refuse individuellement en
        # ERROR_FAULT une fois lui-même en EMERGENCY_STOP), mais les
        # trois couches au-dessus (Watchdog, PlayStore, apps) affichaient
        # un état faux. Porté par le HAL — le seul composant dont
        # WatchdogSystem ET Supervisor détiennent tous deux une
        # référence — plutôt que par le Watchdog lui-même.
        self._estop_active = False

    @property
    def estop_active(self) -> bool:
        """
        État e-stop global, positionné par emergency_stop_all() —
        indépendant de WatchdogSystem.estop_fired (qui persiste après un
        e-stop DÉCLENCHÉ PAR le watchdog lui-même, avec son propre cycle
        de vie). Toute couche qui décide d'autoriser ou non une action
        doit consulter CELUI-CI en plus de watchdog.estop_fired — voir
        RobotContext._require_no_estop() et PlayStoreRuntime.run_mission().
        """
        return self._estop_active

    def _mark_estop_active(self) -> None:
        """
        Appelé par emergency_stop_all() dans CHAQUE implémentation
        concrète (voir UnitreeG1HAL.emergency_stop_all()) — jamais
        automatique, pour que ce soit un geste explicite et visible dans
        chaque driver plutôt qu'un comportement caché dans la classe de
        base. Volontairement pas de méthode symétrique "unmark" publique
        ici : le réarmement, s'il doit exister, est une décision de
        sécurité à formaliser séparément (voir M9 dans l'audit), pas un
        effet de bord silencieux de ce correctif.
        """
        self._estop_active = True

    @abstractmethod
    def initialize_all(self) -> HALStatus: ...

    @abstractmethod
    def shutdown_all(self) -> HALStatus: ...

    @abstractmethod
    def emergency_stop_all(self) -> HALStatus:
        """URGENCE GLOBALE — complète en < 10ms."""
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
        """Doit retourner : manufacturer, model, serial_number, hal_version, dof"""
        ...

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    def get_motion_tracker(self) -> Optional["MotionStateTracker"]:
        """
        Retourne le MotionStateTracker de ce robot, si le driver en
        possède un. Implémentation par défaut : None — un backend qui
        n'a pas encore ce composant (ex: un driver simulateur ancien)
        reste valide, SafetyGuard et WatchdogSystem traitent l'absence
        de tracker comme "rate limiting / détection d'écart désactivés"
        plutôt que comme une erreur.

        Décision de conception actée : ce composant est générique
        (hal/core/motion_state.py) et vit au niveau IRobotHAL, jamais
        dans un driver spécifique — voir sa documentation pour le
        raisonnement complet.
        """
        return None

    def get_capabilities(self) -> CapabilitySet:
        """
        Retourne l'ensemble des capacités physiques de ce robot
        (LEGS, ARMS, WHEELS, CAMERA, etc.) — voir capabilities.py pour
        le raisonnement complet. Implémentation par défaut : ensemble
        vide, même principe que get_motion_tracker() ci-dessus — un
        driver qui n'a pas encore déclaré ses capacités reste valide,
        et le comportement par défaut (capacité absente tant qu'elle
        n'est pas explicitement déclarée) est le choix prudent :
        mieux vaut qu'une vérification de capacité échoue par excès
        de prudence sur un robot mal déclaré, que l'inverse.
        """
        return CapabilitySet.empty()
