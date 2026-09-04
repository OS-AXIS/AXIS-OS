"""
AXIS-OS — IsaacSimHAL
=======================
IRobotHAL pour simulateurs. Accepte n'importe quel ISimBackend.
Swap Isaac→Webots : 1 ligne.

Licence : Apache 2.0
"""

from __future__ import annotations
from typing import Optional, Dict, List
import time

from axis_os.hal.core.interfaces import IRobotHAL, IActuator, IIMU, IForceTorqueSensor
from axis_os.hal.core.types import (
    ActuatorCommand, ActuatorFeedback, HALStatus, JointDefinition
)
from axis_os.hal.drivers.unitree_g1.robot import _build_g1_joint_definitions
from axis_os.hal.drivers.sim_common.interfaces import ISimBackend, SimCommand
from axis_os.hal.drivers.sim_common.sim_actuator import SimActuator, SimIMU
from axis_os.hal.drivers.isaac_sim.backend import IsaacSimBackend


class IsaacSimHAL(IRobotHAL):
    """
    IRobotHAL pour Isaac Sim ou Webots.

    Usage :
        hal = IsaacSimHAL()                                    # stub auto
        hal = IsaacSimHAL(backend=WebotsBackend(...))          # Webots
        hal = IsaacSimHAL(backend=IsaacSimBackend(force_stub=False))  # Isaac prod
    """

    HAL_VERSION = "0.1.0"

    def __init__(self, robot_id: str = "isaac_g1",
                 backend: Optional[ISimBackend] = None):
        super().__init__(robot_id)
        self._backend    = backend or IsaacSimBackend(force_stub=True)
        self._joint_defs = _build_g1_joint_definitions()
        self._actuators: Dict[str, IActuator] = {}
        self._imu: Optional[IIMU] = None

    def initialize_all(self) -> HALStatus:
        print(f"\n[AXIS-OS] IsaacSimHAL — id={self.robot_id}")
        if not self._backend.connect():
            return HALStatus.ERROR_COMM
        info = self._backend.get_backend_info()
        print(f"  Backend : {info['name']} | mode={info['mode']}")

        for jdef in self._joint_defs:
            act = SimActuator(jdef, self._backend, jdef.joint_id)
            act.initialize()
            self._actuators[jdef.joint_id] = act

        self._imu = SimIMU(self._backend)
        self._imu.initialize()
        self._initialized = True
        print(f"  ✓ {len(self._actuators)} joints | HAL v{self.HAL_VERSION}\n")
        return HALStatus.OK

    def shutdown_all(self) -> HALStatus:
        for a in self._actuators.values(): a.shutdown()
        if self._imu: self._imu.shutdown()
        self._backend.disconnect()
        self._initialized = False
        return HALStatus.OK

    def emergency_stop_all(self) -> HALStatus:
        print(f"[AXIS-OS] ⚠️  E-STOP — {self.robot_id}")
        for a in self._actuators.values(): a.emergency_stop()
        return HALStatus.OK

    def get_actuator(self, joint_id: str) -> Optional[IActuator]:
        return self._actuators.get(joint_id)

    def get_all_actuators(self) -> Dict[str, IActuator]:
        return dict(self._actuators)

    def get_imu(self, imu_id: str = "base_imu") -> Optional[IIMU]:
        return self._imu

    def get_force_torque(self, sensor_id: str) -> Optional[IForceTorqueSensor]:
        return None

    def send_joint_commands(
        self, commands: Dict[str, ActuatorCommand]
    ) -> Dict[str, HALStatus]:
        results: Dict[str, HALStatus] = {}
        batch: list[SimCommand] = []

        for jid, cmd in commands.items():
            act = self._actuators.get(jid)
            if act is None:
                results[jid] = HALStatus.ERROR_NOT_INIT; continue
            if not isinstance(act, SimActuator):
                results[jid] = act.send_command(cmd); continue
            fb = act.get_feedback()
            ok, violation = act._guard.validate_command(cmd, fb)
            if not ok:
                print(f"  [SAFETY/SIM] {violation}")
                results[jid] = HALStatus.ERROR_LIMIT; continue
            sim_cmd = act._to_sim_command(cmd)
            if sim_cmd is None:
                results[jid] = HALStatus.NOT_SUPPORTED; continue
            batch.append(sim_cmd)
            results[jid] = HALStatus.OK

        if batch:
            if not self._backend.send_commands_batch(batch):
                for jid in commands:
                    if results.get(jid) == HALStatus.OK:
                        results[jid] = HALStatus.ERROR_COMM
        return results

    def get_full_state(self) -> Dict[str, ActuatorFeedback]:
        all_sim = self._backend.get_all_joint_states()
        feedbacks = {}
        for jid, act in self._actuators.items():
            if not isinstance(act, SimActuator):
                fb = act.get_feedback()
                if fb: feedbacks[jid] = fb
                continue
            s = all_sim.get(jid)
            if s is None: continue
            feedbacks[jid] = ActuatorFeedback(
                joint_id=jid, state=act.state,
                position=s.position, velocity=s.velocity,
                torque=s.torque, current=abs(s.torque) / 10.0,
                temperature=act._sim_temp, voltage=s.voltage,
                timestamp_ns=time.time_ns(),
            )
        return feedbacks

    def get_joint_definitions(self) -> List[JointDefinition]:
        return list(self._joint_defs)

    def get_robot_info(self) -> Dict[str, str]:
        info = self._backend.get_backend_info()
        return {
            "manufacturer": "Unitree Robotics", "model": "G1",
            "serial_number": self.robot_id, "hal_version": self.HAL_VERSION,
            "firmware_version": f"sim-{info.get('version', '?')}",
            "simulator": info.get("name", "unknown"),
            "sim_mode": info.get("mode", "unknown"),
            "dof": str(len(self._joint_defs)),
        }
