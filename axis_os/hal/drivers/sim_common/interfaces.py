"""
AXIS-OS — ISimBackend
=======================
Contrat universel pour tout simulateur.
Compatible avec IRobotHAL, ActuatorFeedback, NormalizedRobotHAL.

Licence : Apache 2.0
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Dict


@dataclass
class SimJointState:
    name:        str
    position:    float
    velocity:    float
    torque:      float
    temperature: float
    voltage:     float


@dataclass
class SimIMUState:
    acc_x: float; acc_y: float; acc_z: float
    gyr_x: float; gyr_y: float; gyr_z: float
    quat_w: float; quat_x: float; quat_y: float; quat_z: float


@dataclass
class SimCommand:
    name:     str
    mode:     str      # "position" | "velocity" | "torque"
    value:    float
    velocity: Optional[float] = None


class ISimBackend(ABC):
    """Contrat minimal que tout simulateur doit implémenter."""

    @abstractmethod
    def connect(self) -> bool: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def is_connected(self) -> bool: ...

    @abstractmethod
    def get_joint_state(self, joint_name: str) -> Optional[SimJointState]: ...

    @abstractmethod
    def get_all_joint_states(self) -> Dict[str, SimJointState]: ...

    @abstractmethod
    def send_command(self, command: SimCommand) -> bool: ...

    @abstractmethod
    def send_commands_batch(self, commands: list[SimCommand]) -> bool: ...

    @abstractmethod
    def get_imu_state(self, imu_name: str = "imu") -> Optional[SimIMUState]: ...

    @abstractmethod
    def reset(self) -> bool: ...

    @abstractmethod
    def get_sim_time(self) -> float: ...

    @abstractmethod
    def get_backend_info(self) -> Dict[str, str]: ...
