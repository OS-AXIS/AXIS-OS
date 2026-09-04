"""
AXIS-OS — IsaacSimBackend
===========================
ISimBackend pour NVIDIA Isaac Sim.
Auto-détecte Isaac Sim — stub si absent.

Licence : Apache 2.0
"""

from __future__ import annotations
import time
import threading
from typing import Optional, Dict

from axis_os.hal.drivers.sim_common.interfaces import (
    ISimBackend, SimJointState, SimIMUState, SimCommand
)

ISAAC_G1_JOINT_MAP: Dict[str, str] = {
    "left_hip_roll": "left_hip_roll_joint",
    "left_hip_yaw": "left_hip_yaw_joint",
    "left_hip_pitch": "left_hip_pitch_joint",
    "left_knee": "left_knee_joint",
    "left_ankle_pitch": "left_ankle_pitch_joint",
    "left_ankle_roll": "left_ankle_roll_joint",
    "right_hip_roll": "right_hip_roll_joint",
    "right_hip_yaw": "right_hip_yaw_joint",
    "right_hip_pitch": "right_hip_pitch_joint",
    "right_knee": "right_knee_joint",
    "right_ankle_pitch": "right_ankle_pitch_joint",
    "right_ankle_roll": "right_ankle_roll_joint",
    "waist_yaw": "waist_yaw_joint",
    "waist_roll": "waist_roll_joint",
    "left_shoulder_pitch": "left_shoulder_pitch_joint",
    "left_shoulder_roll": "left_shoulder_roll_joint",
    "left_shoulder_yaw": "left_shoulder_yaw_joint",
    "left_elbow": "left_elbow_joint",
    "left_wrist_roll": "left_wrist_roll_joint",
    "right_shoulder_pitch": "right_shoulder_pitch_joint",
    "right_shoulder_roll": "right_shoulder_roll_joint",
    "right_shoulder_yaw": "right_shoulder_yaw_joint",
    "right_elbow": "right_elbow_joint",
    "right_wrist_roll": "right_wrist_roll_joint",
}


class _StubPhysics:
    """Moteur PD simplifié — stub sans Isaac Sim."""
    INERTIA = 0.05; DAMPING = 2.0; STIFFNESS = 50.0; DT = 0.004

    def __init__(self, joint_names: list[str]):
        self._joints = {n: {"pos": 0.0, "vel": 0.0, "torque": 0.0,
                            "target_pos": 0.0, "target_vel": 0.0,
                            "mode": "position"} for n in joint_names}
        self._time   = 0.0
        self._lock   = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True, name="stub_physics").start()

    def _loop(self):
        while self._running:
            t0 = time.monotonic()
            with self._lock:
                for s in self._joints.values():
                    if s["mode"] == "position":
                        torque = self.STIFFNESS * (s["target_pos"] - s["pos"]) - self.DAMPING * s["vel"]
                    elif s["mode"] == "velocity":
                        torque = self.STIFFNESS * 0.1 * (s["target_vel"] - s["vel"])
                    else:
                        torque = s["torque"]
                    acc = (torque - self.DAMPING * s["vel"]) / self.INERTIA
                    s["vel"] += acc * self.DT
                    s["pos"] += s["vel"] * self.DT
                    s["torque"] = torque
                self._time += self.DT
            sleep = self.DT - (time.monotonic() - t0)
            if sleep > 0: time.sleep(sleep)

    def set(self, name: str, mode: str, value: float, velocity: Optional[float] = None) -> bool:
        with self._lock:
            if name not in self._joints: return False
            s = self._joints[name]; s["mode"] = mode
            if mode == "position": s["target_pos"] = value
            elif mode == "velocity": s["target_vel"] = value
            else: s["torque"] = value
            return True

    def get(self, name: str) -> Optional[dict]:
        with self._lock:
            return dict(self._joints[name]) if name in self._joints else None

    def get_time(self) -> float:
        with self._lock: return self._time

    def reset(self):
        with self._lock:
            for s in self._joints.values():
                s["pos"] = s["vel"] = s["torque"] = s["target_pos"] = s["target_vel"] = 0.0

    def stop(self): self._running = False


class IsaacSimBackend(ISimBackend):
    STUB_VOLTAGE = 48.0; STUB_TEMP = 25.0

    def __init__(self, robot_prim_path: str = "/World/G1", force_stub: bool = False):
        self._prim_path  = robot_prim_path
        self._force_stub = force_stub
        self._connected  = False
        self._stub_mode  = False
        self._stub: Optional[_StubPhysics] = None
        self._joint_map  = ISAAC_G1_JOINT_MAP

    def connect(self) -> bool:
        if self._force_stub:
            return self._activate_stub("Stub forcé.")
        try:
            import omni.isaac.core as _
            # Production : connexion omni.isaac.core
            self._connected = True; self._stub_mode = False
            print(f"[IsaacBackend] Connecté | prim={self._prim_path}")
            return True
        except ImportError:
            return self._activate_stub("omni.isaac.core non trouvé.")

    def disconnect(self) -> None:
        if self._stub: self._stub.stop(); self._stub = None
        self._connected = False

    def is_connected(self) -> bool: return self._connected

    def get_joint_state(self, joint_name: str) -> Optional[SimJointState]:
        if not self._connected or not self._stub_mode: return None
        s = self._stub.get(joint_name)
        if s is None: return None
        return SimJointState(name=joint_name, position=s["pos"], velocity=s["vel"],
                             torque=s["torque"],
                             temperature=self.STUB_TEMP + abs(s["torque"]) * 0.02,
                             voltage=self.STUB_VOLTAGE)

    def get_all_joint_states(self) -> Dict[str, SimJointState]:
        return {n: self.get_joint_state(n) for n in self._joint_map
                if self.get_joint_state(n) is not None}

    def send_command(self, command: SimCommand) -> bool:
        if not self._connected: return False
        if self._stub_mode:
            return self._stub.set(command.name, command.mode, command.value, command.velocity)
        return False

    def send_commands_batch(self, commands: list[SimCommand]) -> bool:
        return all(self.send_command(c) for c in commands)

    def get_imu_state(self, imu_name: str = "imu") -> Optional[SimIMUState]:
        if not self._connected: return None
        return SimIMUState(acc_x=0.0, acc_y=0.0, acc_z=9.81,
                           gyr_x=0.0, gyr_y=0.0, gyr_z=0.0,
                           quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0)

    def reset(self) -> bool:
        if self._stub: self._stub.reset(); return True
        return False

    def get_sim_time(self) -> float:
        return self._stub.get_time() if self._stub else 0.0

    def get_backend_info(self) -> Dict[str, str]:
        return {"name": "IsaacSimBackend", "version": "0.1.0",
                "mode": "stub" if self._stub_mode else "production",
                "robot_model": "Unitree G1", "joints": str(len(self._joint_map))}

    def _activate_stub(self, reason: str) -> bool:
        print(f"[IsaacBackend] Stub — {reason}")
        self._stub_mode = True; self._connected = True
        self._stub = _StubPhysics(list(self._joint_map.keys()))
        return True
