"""
AXIS-OS — HAL Abstract Interfaces
===================================
LE CONTRAT. Toute implémentation hardware hérite de ces classes.
Aucune logique métier ici — que des signatures.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Optional, List, Dict

from .types import (
    ActuatorCommand, ActuatorFeedback, ActuatorState,
    ControlMode, HALStatus, IMUData, ForceTorqueData,
    JointDefinition, SensorType
)


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
    def send_command(self, command: ActuatorCommand) -> HALStatus: ...

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


class IRobotHAL(ABC):
    """
    Point d'entrée unique de AXIS-OS vers le hardware.
    Chaque constructeur livre UN fichier qui implémente cette interface.
    """

    def __init__(self, robot_id: str):
        self.robot_id     = robot_id
        self._initialized = False

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
