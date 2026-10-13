"""
AXIS-OS — IsaacSimHAL
=======================
Implémentation de IRobotHAL pour les simulateurs. La HAL (Hardware
Abstraction Layer, couche d'abstraction matérielle) est l'interface par
laquelle le reste d'AXIS-OS commande le robot, qu'il soit réel ou simulé.

Cette classe accepte n'importe quel ISimBackend, le pont vers un
simulateur précis : passer d'Isaac Sim (le simulateur de NVIDIA) à Webots
(un autre simulateur robotique) ne demande de changer qu'une ligne. Sans
backend fourni, elle utilise IsaacSimBackend en mode stub, une physique
simplifiée qui fonctionne sans simulateur installé (voir backend.py).

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
    IRobotHAL pour Isaac Sim ou Webots, avec les articulations du robot
    Unitree G1 (le robot humanoïde d'Unitree Robotics).

    Usage :
        hal = IsaacSimHAL()                            # stub automatique
        hal = IsaacSimHAL(backend=WebotsBackend(...))  # Webots
        # Vrai Isaac Sim :
        hal = IsaacSimHAL(backend=IsaacSimBackend(force_stub=False))

    Avec force_stub=False, le backend utilise Isaac Sim s'il est installé,
    et ne se rabat sur le stub que s'il est absent ou ne se charge pas.

    Limite connue, non corrigée : les noms d'articulations ne
    correspondent pas entre cette classe et IsaacSimBackend. Cette classe
    nomme les articulations comme l'URDF officiel du G1 (Unified Robot
    Description Format, le fichier standard qui décrit les articulations
    d'un robot), par exemple "left_knee_joint", via
    _build_g1_joint_definitions(). Le stub d'IsaacSimBackend, lui,
    connaît les clés de ISAAC_G1_JOINT_MAP, par exemple "left_knee" (voir
    la docstring de cette table dans backend.py). Constaté en exécution
    avec le backend par défaut : get_full_state() rend 0 articulation,
    get_feedback() d'un actionneur rend None, et une commande en
    position acceptée par le garde-fou revient en ERROR_COMM, car le
    backend refuse le lot.
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
        """Arrêt d'urgence (e-stop) : chaque actionneur simulé reçoit son
        propre ordre d'arrêt d'urgence."""
        print(f"[AXIS-OS] ⚠️  E-STOP — {self.robot_id}")
        for a in self._actuators.values(): a.emergency_stop()
        return HALStatus.OK

    def get_actuator(self, joint_id: str) -> Optional[IActuator]:
        return self._actuators.get(joint_id)

    def get_all_actuators(self) -> Dict[str, IActuator]:
        return dict(self._actuators)

    def get_imu(self, imu_id: str = "base_imu") -> Optional[IIMU]:
        # Une seule centrale inertielle (IMU : accéléromètre et gyroscope)
        # simulée : imu_id est ignoré.
        return self._imu

    def get_force_torque(self, sensor_id: str) -> Optional[IForceTorqueSensor]:
        # Aucun capteur d'effort (force et couple) n'est simulé.
        return None

    def send_joint_commands(
        self, commands: Dict[str, ActuatorCommand]
    ) -> Dict[str, HALStatus]:
        """Envoie un ensemble de commandes d'articulations, et rend un
        statut par articulation.

        Chaque commande passe d'abord par le garde-fou de sécurité de son
        actionneur ; une commande refusée rend ERROR_LIMIT et n'est pas
        envoyée. Une articulation inconnue rend ERROR_NOT_INIT, et une
        commande que l'actionneur ne sait pas traduire pour le simulateur
        rend NOT_SUPPORTED. Les commandes acceptées partent ensuite
        ensemble, en un seul lot, vers le backend. Si l'envoi du lot
        échoue, toutes celles qui avaient été acceptées passent en
        ERROR_COMM. Un actionneur qui n'est pas un SimActuator traite
        lui-même sa commande.
        """
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
        """État de toutes les articulations, lu en une fois sur le backend.

        Le simulateur ne mesure pas de courant électrique : il est estimé
        à partir du couple (abs(couple) / 10), comme dans SimActuator
        (sim_common/sim_actuator.py). Une articulation que le backend ne
        connaît pas est absente du résultat : avec le backend par défaut,
        c'est le cas de toutes (voir la limite connue décrite dans la
        docstring de la classe). Voir aussi les limites connues décrites
        en tête de backend.py.
        """
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
