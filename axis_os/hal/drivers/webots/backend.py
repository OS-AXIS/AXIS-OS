"""
AXIS-OS — WebotsBackend
=========================
ISimBackend pour Webots. 100% interchangeable avec IsaacSimBackend.

Usage :
    hal = IsaacSimHAL(backend=WebotsBackend(timestep_ms=32))

Licence : Apache 2.0
"""

from __future__ import annotations
from typing import Optional, Dict

from axis_os.hal.drivers.sim_common.interfaces import (
    ISimBackend, SimJointState, SimIMUState, SimCommand
)
from axis_os.hal.drivers.isaac_sim.backend import _StubPhysics

WEBOTS_G1_JOINT_MAP: Dict[str, str] = {
    "left_hip_roll": "LeftHipRoll", "left_hip_yaw": "LeftHipYaw",
    "left_hip_pitch": "LeftHipPitch", "left_knee": "LeftKnee",
    "left_ankle_pitch": "LeftAnklePitch", "left_ankle_roll": "LeftAnkleRoll",
    "right_hip_roll": "RightHipRoll", "right_hip_yaw": "RightHipYaw",
    "right_hip_pitch": "RightHipPitch", "right_knee": "RightKnee",
    "right_ankle_pitch": "RightAnklePitch", "right_ankle_roll": "RightAnkleRoll",
    "waist_yaw": "WaistYaw", "waist_roll": "WaistRoll",
    "left_shoulder_pitch": "LeftShoulderPitch", "left_shoulder_roll": "LeftShoulderRoll",
    "left_shoulder_yaw": "LeftShoulderYaw", "left_elbow": "LeftElbow",
    "left_wrist_roll": "LeftWristRoll",
    "right_shoulder_pitch": "RightShoulderPitch", "right_shoulder_roll": "RightShoulderRoll",
    "right_shoulder_yaw": "RightShoulderYaw", "right_elbow": "RightElbow",
    "right_wrist_roll": "RightWristRoll",
}


class WebotsBackend(ISimBackend):
    STUB_VOLTAGE = 48.0; STUB_TEMP = 25.0

    def __init__(self, timestep_ms: int = 32, force_stub: bool = False):
        self._timestep_ms = timestep_ms
        self._force_stub  = force_stub
        self._connected   = False
        self._stub_mode   = False
        self._joint_map   = WEBOTS_G1_JOINT_MAP
        self._robot_node  = None
        self._motors: dict = {}
        self._sensors: dict = {}
        self._imu_device  = None
        self._stub: Optional[_StubPhysics] = None

    def connect(self) -> bool:
        if self._force_stub:
            return self._activate_stub("Stub forcé.")
        try:
            from controller import Robot, Motor, PositionSensor, InertialUnit
            self._robot_node = Robot()
            for hal_name, wb_name in self._joint_map.items():
                try:
                    motor = self._robot_node.getDevice(wb_name)
                    if motor:
                        motor.setPosition(float("inf")); motor.setVelocity(0.0)
                        sensor = self._robot_node.getDevice(f"{wb_name}S")
                        if sensor:
                            sensor.enable(self._timestep_ms)
                            self._sensors[hal_name] = sensor
                        self._motors[hal_name] = motor
                except Exception: pass
            try:
                imu = self._robot_node.getDevice("inertial_unit")
                if imu: imu.enable(self._timestep_ms); self._imu_device = imu
            except Exception: pass
            self._connected = True; self._stub_mode = False
            print(f"[WebotsBackend] Connecté | timestep={self._timestep_ms}ms")
            return True
        except ImportError:
            return self._activate_stub("module 'controller' non trouvé.")
        except Exception as e:
            return self._activate_stub(f"Erreur: {e}")

    def disconnect(self) -> None:
        if self._stub: self._stub.stop(); self._stub = None
        self._connected = False; self._robot_node = None
        self._motors.clear(); self._sensors.clear()

    def is_connected(self) -> bool: return self._connected

    def get_joint_state(self, joint_name: str) -> Optional[SimJointState]:
        if not self._connected: return None
        if self._stub_mode:
            s = self._stub.get(joint_name)
            if s is None: return None
            return SimJointState(name=joint_name, position=s["pos"], velocity=s["vel"],
                                 torque=s["torque"],
                                 temperature=self.STUB_TEMP + abs(s["torque"]) * 0.02,
                                 voltage=self.STUB_VOLTAGE)
        sensor = self._sensors.get(joint_name)
        motor  = self._motors.get(joint_name)
        if sensor is None or motor is None: return None
        try:
            return SimJointState(name=joint_name, position=sensor.getValue(),
                                 velocity=motor.getVelocity(),
                                 torque=motor.getTorqueFeedback()
                                        if hasattr(motor, "getTorqueFeedback") else 0.0,
                                 temperature=self.STUB_TEMP, voltage=self.STUB_VOLTAGE)
        except Exception: return None

    def get_all_joint_states(self) -> Dict[str, SimJointState]:
        if not self._stub_mode and self._robot_node:
            try: self._robot_node.step(self._timestep_ms)
            except Exception: pass
        return {n: s for n in self._joint_map
                if (s := self.get_joint_state(n)) is not None}

    def send_command(self, command: SimCommand) -> bool:
        if not self._connected: return False
        if self._stub_mode:
            return self._stub.set(command.name, command.mode,
                                  command.value, command.velocity)
        motor = self._motors.get(command.name)
        if motor is None: return False
        try:
            if command.mode == "position":
                motor.setPosition(command.value)
                if command.velocity: motor.setVelocity(abs(command.velocity))
            elif command.mode == "velocity":
                motor.setPosition(float("inf")); motor.setVelocity(command.value)
            elif command.mode == "torque":
                motor.setTorque(command.value)
            return True
        except Exception: return False

    def send_commands_batch(self, commands: list[SimCommand]) -> bool:
        results = [self.send_command(c) for c in commands]
        if not self._stub_mode and self._robot_node:
            try: self._robot_node.step(self._timestep_ms)
            except Exception: pass
        return all(results)

    def get_imu_state(self, imu_name: str = "imu") -> Optional[SimIMUState]:
        if not self._connected: return None
        if self._stub_mode:
            return SimIMUState(acc_x=0.0, acc_y=0.0, acc_z=9.81,
                               gyr_x=0.0, gyr_y=0.0, gyr_z=0.0,
                               quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0)
        if self._imu_device is None: return None
        try:
            import math
            r, p, y = self._imu_device.getRollPitchYaw()
            cy, sy = math.cos(y/2), math.sin(y/2)
            cp, sp = math.cos(p/2), math.sin(p/2)
            cr, sr = math.cos(r/2), math.sin(r/2)
            return SimIMUState(acc_x=0.0, acc_y=0.0, acc_z=9.81,
                               gyr_x=0.0, gyr_y=0.0, gyr_z=0.0,
                               quat_w=cr*cp*cy + sr*sp*sy,
                               quat_x=sr*cp*cy - cr*sp*sy,
                               quat_y=cr*sp*cy + sr*cp*sy,
                               quat_z=cr*cp*sy - sr*sp*cy)
        except Exception: return None

    def reset(self) -> bool:
        if self._stub_mode and self._stub: self._stub.reset(); return True
        if self._robot_node:
            try: self._robot_node.simulationReset(); return True
            except Exception: pass
        return False

    def get_sim_time(self) -> float:
        if self._stub_mode and self._stub: return self._stub.get_time()
        if self._robot_node:
            try: return self._robot_node.getTime()
            except Exception: pass
        return 0.0

    def get_backend_info(self) -> Dict[str, str]:
        return {"name": "WebotsBackend", "version": "0.1.0",
                "mode": "stub" if self._stub_mode else "production",
                "robot_model": "Unitree G1",
                "timestep_ms": str(self._timestep_ms),
                "joints": str(len(self._joint_map))}

    def _activate_stub(self, reason: str) -> bool:
        print(f"[WebotsBackend] Stub — {reason}")
        self._stub_mode = True; self._connected = True
        self._stub = _StubPhysics(list(self._joint_map.keys()))
        return True
