"""
AXIS-OS — SimActuator & SimIMU
================================
Implémentations génériques d'un moteur (IActuator) et d'une centrale
inertielle (IIMU ; IMU pour Inertial Measurement Unit, le capteur qui
mesure les accélérations et les rotations du robot), partagées par tous
les simulateurs.

Pourquoi : la logique côté HAL (HAL pour Hardware Abstraction Layer, la
couche d'abstraction matérielle d'AXIS-OS), notamment le contrôle de
sécurité de chaque commande, n'est ainsi écrite qu'une fois. Seul le
backend change d'un simulateur à l'autre : l'ISimBackend (le pont vers
un simulateur donné, voir interfaces.py) est injecté à la construction,
et ces classes lui délèguent toute lecture et toute écriture.

Licence : Apache 2.0
"""

from __future__ import annotations
from typing import Optional
import time

from axis_os.hal.core.interfaces import IActuator, IIMU
from axis_os.hal.core.types import (
    ActuatorCommand, ActuatorFeedback, ActuatorState,
    ControlMode, HALStatus, IMUData, SensorType,
    JointDefinition, Vector3, Quaternion
)
from axis_os.hal.safety.limits import JointSafetyGuard
from .interfaces import ISimBackend, SimCommand


class SimActuator(IActuator):
    """Moteur générique (IActuator) relié à un ISimBackend.

    Chaque commande reçue par send_command passe d'abord par un
    JointSafetyGuard, le garde-fou de sécurité d'une articulation
    (axis_os/hal/safety/limits.py), comme pour un pilote de moteur réel :
    une commande hors limites est refusée et n'atteint jamais le
    simulateur. Seules les consignes internes de shutdown (retour à zéro)
    et d'emergency_stop (couple nul) sont envoyées sans ce contrôle.

    Le simulateur ne fournit ni vraie température ni vrai courant : les
    deux sont estimés à partir du couple (voir send_command et
    get_feedback). Ce sont des approximations, pas des mesures.
    """

    # Température de départ simulée, en degrés Celsius.
    BASE_TEMP_C = 25.0

    def __init__(self, definition: JointDefinition, backend: ISimBackend,
                 sim_joint_name: Optional[str] = None):
        """sim_joint_name : nom de l'articulation côté simulateur, s'il
        diffère de l'identifiant AXIS-OS (definition.joint_id)."""
        super().__init__(definition.joint_id, definition)
        self._backend      = backend
        self._sim_name     = sim_joint_name or definition.joint_id
        self._guard        = JointSafetyGuard(definition.joint_id, definition.limits)
        self._control_mode = definition.default_control_mode
        self._sim_temp     = self.BASE_TEMP_C
        self._last_torque  = 0.0

    def initialize(self) -> HALStatus:
        # Refuse de démarrer si le backend n'est pas connecté ou si le
        # simulateur ne connaît pas cette articulation (aucun état lu).
        if not self._backend.is_connected():
            return HALStatus.ERROR_COMM
        state = self._backend.get_joint_state(self._sim_name)
        if state is None:
            return HALStatus.ERROR_COMM
        self._state = ActuatorState.IDLE
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        # Ramène l'articulation en position zéro, à vitesse réduite (0,5),
        # avant de la marquer comme non initialisée.
        self._backend.send_command(SimCommand(self._sim_name, "position", 0.0, 0.5))
        self._state = ActuatorState.UNINITIALIZED
        return HALStatus.OK

    def calibrate(self) -> HALStatus:
        # Rien à calibrer en simulation : l'articulation passe directement
        # à l'état IDLE (au repos, prête à recevoir des commandes).
        self._state = ActuatorState.IDLE
        return HALStatus.OK

    def set_control_mode(self, mode: ControlMode) -> HALStatus:
        # Le mode est mémorisé, mais c'est le champ mode de chaque commande
        # qui décide de sa traduction pour le simulateur (_to_sim_command).
        self._control_mode = mode
        return HALStatus.OK

    def send_command(self, command: ActuatorCommand) -> HALStatus:
        # Tant que l'articulation est en arrêt d'urgence (e-stop), aucune
        # commande n'est acceptée.
        if self._state == ActuatorState.EMERGENCY_STOP:
            return HALStatus.ERROR_FAULT
        if not self._backend.is_connected():
            self._state = ActuatorState.FAULT
            return HALStatus.ERROR_COMM
        # Contrôle de sécurité avant tout envoi, avec le dernier retour du
        # moteur (position, vitesse, couple…). Un refus est affiché sur la
        # sortie standard (préfixe [SAFETY/SIM]) et la commande n'est pas
        # transmise.
        fb = self.get_feedback()
        ok, violation = self._guard.validate_command(command, fb)
        if not ok:
            print(f"  [SAFETY/SIM] {violation}")
            return HALStatus.ERROR_LIMIT
        sim_cmd = self._to_sim_command(command)
        if sim_cmd is None:
            return HALStatus.NOT_SUPPORTED
        if not self._backend.send_command(sim_cmd):
            self._state = ActuatorState.FAULT
            return HALStatus.ERROR_COMM
        # Estimation de température, faute de capteur simulé : elle monte
        # avec le dernier couple commandé (0,002 °C par unité de couple
        # au-dessus de BASE_TEMP_C) et plafonne à la température maximale
        # de l'articulation. Seules les commandes qui portent un couple
        # mettent à jour cette estimation.
        if command.torque is not None:
            self._last_torque = command.torque
        self._sim_temp = min(
            self.BASE_TEMP_C + abs(self._last_torque) * 0.002,
            self.definition.limits.temperature_max
        )
        self._state = ActuatorState.ACTIVE
        return HALStatus.OK

    def emergency_stop(self) -> HALStatus:
        # Arrêt d'urgence : couple nul, le moteur cesse tout effort.
        # Le résultat de l'envoi n'est pas vérifié : l'articulation passe
        # en EMERGENCY_STOP dans tous les cas, ce qui bloque toute
        # commande ultérieure (voir send_command).
        self._backend.send_command(SimCommand(self._sim_name, "torque", 0.0))
        self._state = ActuatorState.EMERGENCY_STOP
        return HALStatus.OK

    def get_feedback(self) -> Optional[ActuatorFeedback]:
        # Retour d'état au format de la HAL. Valeurs qui ne viennent pas
        # directement du simulateur :
        #   - courant : estimé à partir du couple (couple / 10, le même
        #     facteur que _to_sim_command applique dans l'autre sens) ;
        #   - température : l'estimation calculée dans send_command, et
        #     non SimJointState.temperature fourni par le backend ;
        #   - horodatage : temps réel de la machine (time.time_ns), pas
        #     le temps simulé.
        if not self._backend.is_connected():
            return None
        s = self._backend.get_joint_state(self._sim_name)
        if s is None:
            return None
        return ActuatorFeedback(
            joint_id=    self.joint_id,
            state=       self._state,
            position=    s.position,
            velocity=    s.velocity,
            torque=      s.torque,
            current=     abs(s.torque) / 10.0,
            temperature= self._sim_temp,
            voltage=     s.voltage,
            timestamp_ns=time.time_ns(),
        )

    def _to_sim_command(self, command: ActuatorCommand) -> Optional[SimCommand]:
        """Traduit une commande HAL en consigne simulateur.

        Le simulateur ne connaît que trois modes (position, vitesse,
        couple). Le mode impédance devient une simple consigne de
        position : la raideur et l'amortissement demandés ne sont pas
        transmis. Le mode courant devient une consigne de couple
        (courant × 10) ; un courant absent y vaut 0, soit une consigne de
        couple nul. Rend None, d'où NOT_SUPPORTED, si le mode est inconnu
        ou, pour les autres modes, si la valeur correspondante manque.
        """
        mode_map = {
            ControlMode.POSITION:  ("position", command.position),
            ControlMode.VELOCITY:  ("velocity", command.velocity),
            ControlMode.TORQUE:    ("torque",   command.torque),
            ControlMode.IMPEDANCE: ("position", command.position),
            ControlMode.CURRENT:   ("torque",   (command.current or 0.0) * 10.0),
        }
        entry = mode_map.get(command.mode)
        if entry is None or entry[1] is None:
            return None
        return SimCommand(name=self._sim_name, mode=entry[0],
                          value=entry[1], velocity=command.velocity)


class SimIMU(IIMU):
    """Centrale inertielle générique (IIMU) reliée à un ISimBackend,
    exposée sous l'identifiant « base_imu »."""

    def __init__(self, backend: ISimBackend, sim_imu_name: str = "imu"):
        super().__init__("base_imu", SensorType.IMU)
        self._backend  = backend
        self._sim_name = sim_imu_name

    def initialize(self) -> HALStatus:
        if not self._backend.is_connected():
            return HALStatus.ERROR_COMM
        self._initialized = True
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._initialized = False
        return HALStatus.OK

    def is_healthy(self) -> bool:
        return self._initialized and self._backend.is_connected()

    def read(self) -> Optional[IMUData]:
        # None si la centrale n'est pas initialisée ou si le backend ne
        # rend rien.
        if not self._initialized:
            return None
        s = self._backend.get_imu_state(self._sim_name)
        if s is None:
            return None
        return IMUData(
            linear_acceleration=Vector3(s.acc_x, s.acc_y, s.acc_z),
            angular_velocity=   Vector3(s.gyr_x, s.gyr_y, s.gyr_z),
            orientation=        Quaternion(s.quat_w, s.quat_x, s.quat_y, s.quat_z),
        )

    def reset_orientation(self) -> HALStatus:
        # Sans effet en simulation : rend OK sans rien faire.
        return HALStatus.OK
