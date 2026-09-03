"""
AXIS-OS Robotics Play Store — App SDK Base
============================================
Classe de base que tout développeur d'app doit étendre.
Le RobotContext est le sandbox — l'app n'accède au robot qu'à travers lui.

Licence : Apache 2.0 (open-source — SDK public)
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Dict, Any
import time

from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.types import ActuatorCommand, ActuatorFeedback, HALStatus, ControlMode
from axis_os.hal.watchdog.system import WatchdogSystem
from .manifest import AppManifest, Permission


class PermissionError(Exception):
    pass


class RobotContext:
    """Interface sécurisée entre une app et le robot."""

    def __init__(self, robot: NormalizedRobotHAL, watchdog: WatchdogSystem, manifest: AppManifest):
        self._robot    = robot
        self._watchdog = watchdog
        self._manifest = manifest
        self._log_entries: list[str] = []
        self._t0 = time.monotonic()

    def get_joint_state(self, joint_id: str) -> Optional[ActuatorFeedback]:
        self._require(Permission.ACTUATORS_READ)
        return self._robot.get_full_state().get(joint_id)

    def get_full_state(self) -> Dict[str, ActuatorFeedback]:
        self._require(Permission.ACTUATORS_READ)
        return self._robot.get_full_state()

    def get_imu(self):
        self._require(Permission.SENSORS_IMU)
        return self._robot.get_imu()

    def move_arms(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        self._require(Permission.ACTUATORS_WRITE_ARMS)
        arm_joints = {jid for jid in commands
                      if any(k in jid for k in ["shoulder", "elbow", "wrist"])}
        filtered = {jid: cmd for jid, cmd in commands.items() if jid in arm_joints}
        self._apply_velocity_cap(filtered)
        return self._robot.send_joint_commands(filtered)

    def move_legs(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        self._require(Permission.ACTUATORS_WRITE_LEGS)
        leg_joints = {jid for jid in commands
                      if any(k in jid for k in ["hip", "knee", "ankle"])}
        filtered = {jid: cmd for jid, cmd in commands.items() if jid in leg_joints}
        self._apply_velocity_cap(filtered)
        return self._robot.send_joint_commands(filtered)

    def move_all(self, commands: Dict[str, ActuatorCommand]) -> Dict[str, HALStatus]:
        self._require(Permission.ACTUATORS_WRITE_ALL)
        self._apply_velocity_cap(commands)
        return self._robot.send_joint_commands(commands)

    def subscribe_alerts(self, callback) -> None:
        self._require(Permission.WATCHDOG_READ)
        self._watchdog.subscribe(callback)

    def log(self, message: str) -> None:
        elapsed = time.monotonic() - self._t0
        entry   = f"[T+{elapsed:06.2f}s] [{self._manifest.app_id}] {message}"
        self._log_entries.append(entry)
        print(f"  {entry}")

    def get_mission_log(self) -> list[str]:
        return list(self._log_entries)

    def _require(self, permission: Permission) -> None:
        if not self._manifest.has_permission(permission):
            raise PermissionError(
                f"Permission refusée : {permission.name} "
                f"non déclarée dans '{self._manifest.app_id}'"
            )

    def _apply_velocity_cap(self, commands: Dict[str, ActuatorCommand]) -> None:
        ratio = self._manifest.max_velocity_ratio
        for cmd in commands.values():
            if cmd.velocity is not None:
                cmd.velocity *= ratio


@dataclass
class MissionResult:
    success:          bool
    mission_id:       str
    app_id:           str
    duration_s:       float
    items_processed:  int              = 0
    error_message:    Optional[str]    = None
    metrics:          Dict[str, Any]   = field(default_factory=dict)
    log:              list[str]        = field(default_factory=list)

    def __str__(self) -> str:
        status = "✓ SUCCESS" if self.success else "✗ FAILED"
        return (f"[Mission {self.mission_id}] {status} | "
                f"app={self.app_id} | durée={self.duration_s:.2f}s | "
                f"items={self.items_processed}")


class RobotApp(ABC):
    """Classe de base pour toute application du Robotics Play Store."""

    @abstractmethod
    def manifest(self) -> AppManifest: ...

    @abstractmethod
    def on_start(self, ctx: RobotContext) -> bool: ...

    @abstractmethod
    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult: ...

    @abstractmethod
    def on_stop(self, ctx: RobotContext) -> None: ...
