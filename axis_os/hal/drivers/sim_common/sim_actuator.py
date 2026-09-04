"""
AXIS-OS — SimActuator & SimIMU
================================
Implémentations génériques partagées entre tous les simulateurs.
Délèguent à ISimBackend injecté.

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
    """IActuator générique connecté à un ISimBackend."""

    BASE_TEMP_C = 25.0

    def __init__(self, definition: JointDefinition, backend: ISimBackend,
                 sim_joint_name: Optional[str] = None):
        super().__init__(definition.joint_id, definition)
        self._backend      = backend
        self._sim_name     = sim_joint_name or definition.joint_id
        self._guard        = JointSafetyGuard(definition.joint_id, definition.limits)
        self._control_mode = definition.default_control_mode
        self._sim_temp     = self.BASE_TEMP_C
        self._last_torque  = 0.0

    def initialize(self) -> HALStatus:
        if not self._backend.is_connected():
            return HALStatus.ERROR_COMM
        state = self._backend.get_joint_state(self._sim_name)
        if state is None:
            return HALStatus.ERROR_COMM
        self._state = ActuatorState.IDLE
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._backend.send_command(SimCommand(self._sim_name, "position", 0.0, 0.5))
        self._state = ActuatorState.UNINITIALIZED
        return HALStatus.OK

    def calibrate(self) -> HALStatus:
        self._state = ActuatorState.IDLE
        return HALStatus.OK

    def set_control_mode(self, mode: ControlMode) -> HALStatus:
        self._control_mode = mode
        return HALStatus.OK

    def send_command(self, command: ActuatorCommand) -> HALStatus:
        if self._state == ActuatorState.EMERGENCY_STOP:
            return HALStatus.ERROR_FAULT
        if not self._backend.is_connected():
            self._state = ActuatorState.FAULT
            return HALStatus.ERROR_COMM
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
        if command.torque is not None:
            self._last_torque = command.torque
        self._sim_temp = min(
            self.BASE_TEMP_C + abs(self._last_torque) * 0.002,
            self.definition.limits.temperature_max
        )
        self._state = ActuatorState.ACTIVE
        return HALStatus.OK

    def emergency_stop(self) -> HALStatus:
        self._backend.send_command(SimCommand(self._sim_name, "torque", 0.0))
        self._state = ActuatorState.EMERGENCY_STOP
        return HALStatus.OK

    def get_feedback(self) -> Optional[ActuatorFeedback]:
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
    """IIMU générique connectée à un ISimBackend."""

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
        return HALStatus.OK
