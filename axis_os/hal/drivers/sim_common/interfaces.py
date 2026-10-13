"""
AXIS-OS — ISimBackend
=======================
Contrat commun à tous les simulateurs physiques branchés sur AXIS-OS
(aujourd'hui Isaac Sim de NVIDIA et Webots). Un « backend » est le pont
entre AXIS-OS et un simulateur donné.

Pourquoi une interface unique : la HAL simulée (IsaacSimHAL ; HAL pour
Hardware Abstraction Layer, la couche d'abstraction matérielle qui
présente le robot au reste d'AXIS-OS) ne parle qu'à un ISimBackend.
Changer de simulateur revient donc à changer d'objet backend, sans
toucher au reste du système (voir axis_os/hal/drivers/isaac_sim/hal.py).

Les structures ci-dessous restent volontairement simples : un nom
d'articulation en texte, des valeurs en nombres flottants. SimActuator et
SimIMU (sim_actuator.py) les traduisent vers les types de la HAL. Le
contrat est pensé pour être compatible avec IRobotHAL (l'interface d'un
robot complet), ActuatorFeedback (le retour d'état d'un moteur) et
NormalizedRobotHAL (l'enveloppe qui présente le robot aux applications
sous une forme normalisée).

Licence : Apache 2.0
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Dict


@dataclass
class SimJointState:
    """État d'une articulation tel que le simulateur le rapporte :
    position, vitesse, couple, température et tension d'alimentation.
    Un simulateur ne mesure en général ni la température ni la tension :
    les backends existants y placent une valeur fixe ou une estimation
    (voir axis_os/hal/drivers/webots/backend.py)."""
    name:        str
    position:    float
    velocity:    float
    torque:      float
    temperature: float
    voltage:     float


@dataclass
class SimIMUState:
    """État de la centrale inertielle (IMU, Inertial Measurement Unit :
    le capteur qui mesure les accélérations et les rotations du robot).
    acc_* : accélération linéaire (au repos, acc_z vaut 9,81 m/s², la
    gravité) ; gyr_* : vitesse angulaire ; quat_* : orientation sous
    forme de quaternion (quatre nombres w, x, y, z)."""
    acc_x: float; acc_y: float; acc_z: float
    gyr_x: float; gyr_y: float; gyr_z: float
    quat_w: float; quat_x: float; quat_y: float; quat_z: float


@dataclass
class SimCommand:
    """Consigne envoyée à une articulation du simulateur. value est la
    position, la vitesse ou le couple visé, selon mode. velocity est une
    vitesse facultative qui accompagne la consigne : en mode position,
    WebotsBackend s'en sert comme vitesse de déplacement vers la cible."""
    name:     str
    mode:     str      # "position" | "velocity" | "torque"
    value:    float
    velocity: Optional[float] = None


class ISimBackend(ABC):
    """Contrat minimal que tout simulateur doit implémenter.

    Les deux backends existants (IsaacSimBackend et WebotsBackend)
    basculent sur un stub, une physique simplifiée intégrée, quand le
    simulateur est absent : connect() réussit alors quand même, et
    get_backend_info() indique le mode réellement utilisé.
    """

    # Établit la liaison avec le simulateur. Rend True si le backend est
    # prêt à recevoir des commandes.
    @abstractmethod
    def connect(self) -> bool: ...

    # Coupe la liaison et libère les ressources du simulateur.
    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def is_connected(self) -> bool: ...

    # État d'une articulation, ou None si elle est inconnue ou illisible.
    @abstractmethod
    def get_joint_state(self, joint_name: str) -> Optional[SimJointState]: ...

    # États de toutes les articulations lisibles, indexés par nom.
    @abstractmethod
    def get_all_joint_states(self) -> Dict[str, SimJointState]: ...

    # Envoie une consigne. Rend True si le simulateur l'a acceptée.
    @abstractmethod
    def send_command(self, command: SimCommand) -> bool: ...

    # Envoie plusieurs consignes d'un coup, pour qu'elles s'appliquent au
    # même instant simulé. Rend True seulement si toutes sont acceptées.
    @abstractmethod
    def send_commands_batch(self, commands: list[SimCommand]) -> bool: ...

    # État de la centrale inertielle, ou None si elle est indisponible.
    @abstractmethod
    def get_imu_state(self, imu_name: str = "imu") -> Optional[SimIMUState]: ...

    # Remet la simulation dans son état initial. Rend True en cas de
    # succès.
    @abstractmethod
    def reset(self) -> bool: ...

    # Temps simulé écoulé, en secondes (et non le temps réel de la
    # machine).
    @abstractmethod
    def get_sim_time(self) -> float: ...

    # Description du backend sous forme de textes : nom, version, mode
    # (« stub » ou « production »), modèle de robot, etc.
    @abstractmethod
    def get_backend_info(self) -> Dict[str, str]: ...
