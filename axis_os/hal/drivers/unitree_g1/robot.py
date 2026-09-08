"""
AXIS-OS — Driver Unitree G1
==============================
Implémentation concrète de IRobotHAL pour le Unitree G1.
En simulation : actionneurs mockés.
En production  : remplacer G1SimulatedActuator par G1CANActuator.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from pathlib import Path
from typing import Optional, Dict, List

from axis_os.hal.core.interfaces import IRobotHAL, IActuator, IIMU, IForceTorqueSensor
from axis_os.hal.core.capabilities import CapabilitySet, RobotCapability
from axis_os.hal.core.types import (
    ActuatorCommand, ActuatorFeedback, ActuatorState,
    ControlMode, HALStatus, IMUData, SensorType,
    JointDefinition, JointLimits, JointType,
    Vector3, Quaternion
)
from axis_os.hal.core.urdf_loader import load_urdf, URDFParseError
from axis_os.hal.safety.limits import JointSafetyGuard
from axis_os.hal.core.motion_state import MotionStateTracker
import time


# Fichier URDF officiel Unitree (23 DOF) — voir urdf/README.md pour la
# provenance, la licence (BSD 3-Clause), et le détail des écarts que
# ce fichier a corrigés par rapport à l'ancien codage manuel (torque_max
# sous-estimé jusqu'à 37% sur hanches/genoux, waist_roll inexistant
# dans cette variante, coude déjà correct en un seul joint contrairement
# à ce que la session précédente supposait).
_G1_URDF_PATH = Path(__file__).parent / "urdf" / "g1_23dof_rev_1_0.urdf"


def _build_g1_joint_definitions() -> List[JointDefinition]:
    """
    23 joints du Unitree G1, chargés depuis le fichier URDF officiel
    du fabricant (voir _G1_URDF_PATH) plutôt que recodés à la main.

    Remplace l'ancienne fonction qui construisait chaque JointDefinition
    manuellement — approche abandonnée après la découverte (session du
    2026-09-08) que le codage manuel comportait de vraies erreurs de
    sécurité (limites de couple sous-estimées, pas seulement des noms
    approximatifs) en plus des écarts déjà documentés sur le nombre de
    joints du torse et du coude. Le fichier URDF du fabricant est
    désormais l'unique source de vérité pour la géométrie et les
    limites du G1 — voir urdf_loader.py pour le parseur et ses garanties
    (rejet strict de tout joint actionné sans limites, par exemple).

    Lève URDFParseError si le fichier est absent/corrompu — volontaire :
    un HAL qui ne peut pas charger sa propre définition de joints ne
    doit jamais démarrer avec des valeurs par défaut silencieuses.
    """
    model = load_urdf(_G1_URDF_PATH)
    return model.get_joints()


class G1SimulatedActuator(IActuator):
    """Mock actionneur G1 — pour simulation sans SDK physique."""

    def __init__(self, definition: JointDefinition):
        super().__init__(definition.joint_id, definition)
        self._guard       = JointSafetyGuard(definition.joint_id, definition.limits)
        self._position    = 0.0
        self._velocity    = 0.0
        self._torque      = 0.0
        self._current     = 0.0
        self._temperature = 25.0

    def initialize(self) -> HALStatus:
        self._state = ActuatorState.IDLE
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._state = ActuatorState.UNINITIALIZED
        return HALStatus.OK

    def calibrate(self) -> HALStatus:
        self._state    = ActuatorState.CALIBRATING
        self._position = 0.0
        self._state    = ActuatorState.IDLE
        return HALStatus.OK

    def set_control_mode(self, mode: ControlMode) -> HALStatus:
        self.definition.default_control_mode = mode
        return HALStatus.OK

    def send_command(self, command: ActuatorCommand) -> HALStatus:
        if self._state == ActuatorState.EMERGENCY_STOP:
            return HALStatus.ERROR_FAULT
        ok, violation = self._guard.validate_command(command, self.get_feedback())
        if not ok:
            print(f"  [SAFETY] {violation}")
            return HALStatus.ERROR_LIMIT
        if command.position is not None: self._position = command.position
        if command.velocity is not None: self._velocity = command.velocity
        if command.torque   is not None: self._torque   = command.torque
        self._state = ActuatorState.ACTIVE
        return HALStatus.OK

    def emergency_stop(self) -> HALStatus:
        self._torque = self._velocity = 0.0
        self._state  = ActuatorState.EMERGENCY_STOP
        return HALStatus.OK

    def get_feedback(self) -> Optional[ActuatorFeedback]:
        return ActuatorFeedback(
            joint_id=    self.joint_id,
            state=       self._state,
            position=    self._position,
            velocity=    self._velocity,
            torque=      self._torque,
            current=     self._current if self._current != 0.0 else self._torque / 10.0,
            temperature= self._temperature,
            voltage=     48.0,
        )


class G1SimulatedIMU(IIMU):
    def __init__(self):
        super().__init__("base_imu", SensorType.IMU)

    def initialize(self) -> HALStatus:
        self._initialized = True
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._initialized = False
        return HALStatus.OK

    def is_healthy(self) -> bool:
        return self._initialized

    def read(self) -> Optional[IMUData]:
        return IMUData(
            linear_acceleration=Vector3(0.0, 0.0, 9.81),
            angular_velocity=Vector3(0.0, 0.0, 0.0),
            orientation=Quaternion(1.0, 0.0, 0.0, 0.0),
        )

    def reset_orientation(self) -> HALStatus:
        return HALStatus.OK


class UnitreeG1HAL(IRobotHAL):
    """IRobotHAL complet pour le Unitree G1."""

    HAL_VERSION = "0.1.0"

    def __init__(self, robot_id: str = "unitree_g1"):
        super().__init__(robot_id)
        self._joint_defs: List[JointDefinition] = _build_g1_joint_definitions()
        self._actuators:  Dict[str, IActuator]  = {}
        self._imu:        Optional[IIMU]         = None

        # MotionStateTracker branché ici — décision de conception actée :
        # IRobotHAL le possède et l'instancie (composant générique, voir
        # hal/core/motion_state.py), jamais un driver spécifique qui le
        # créerait pour lui-même. Exposé via get_motion_tracker() pour
        # que SafetyGuard et WatchdogSystem le consultent sans le dupliquer.
        self._motion_tracker = MotionStateTracker(
            [jdef.joint_id for jdef in self._joint_defs]
        )

    def initialize_all(self) -> HALStatus:
        print(f"\n[AXIS-OS] Initialisation Unitree G1 — id={self.robot_id}")

        # Validation de la table de conventions AVANT tout mouvement possible.
        # Trou identifié par revue externe : sans ce check, un joint manquant
        # ou un sign invalide (ex: 0.5 au lieu de ±1.0) ne serait détecté
        # qu'au premier appel du normalizer sur ce joint précis — potentiellement
        # jamais en test, potentiellement en plein mouvement en production.
        from .conventions import G1_JOINT_CONVENTIONS
        from .convention_validator import validate_convention_table

        expected_ids = [jdef.joint_id for jdef in self._joint_defs]
        validation = validate_convention_table(G1_JOINT_CONVENTIONS, expected_ids)
        print(f"  {validation}")
        if not validation.valid:
            print("  [ERREUR] Table de conventions invalide — initialisation refusée.")
            return HALStatus.ERROR_FAULT

        for jdef in self._joint_defs:
            act = G1SimulatedActuator(jdef)
            if act.initialize() != HALStatus.OK:
                return HALStatus.ERROR_COMM
            self._actuators[jdef.joint_id] = act
        self._imu = G1SimulatedIMU()
        self._imu.initialize()
        self._initialized = True
        print(f"  ✓ {len(self._actuators)} joints | HAL v{self.HAL_VERSION}\n")
        return HALStatus.OK

    def shutdown_all(self) -> HALStatus:
        for a in self._actuators.values(): a.shutdown()
        if self._imu: self._imu.shutdown()
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
        results = {}
        for jid, cmd in commands.items():
            if jid in self._actuators:
                status = self._actuators[jid].send_command(cmd)
                # Enregistrement dans le tracker APRÈS l'envoi réel —
                # reflète ce qui a effectivement été transmis au HAL,
                # pas une commande hypothétique qui aurait pu être
                # rejetée plus tôt dans le pipeline (SafetyGuard).
                self._motion_tracker.record_command(jid, cmd)
                results[jid] = status
            else:
                results[jid] = HALStatus.ERROR_NOT_INIT
        return results

    def get_full_state(self) -> Dict[str, ActuatorFeedback]:
        state = {}
        for jid, a in self._actuators.items():
            fb = a.get_feedback()
            if fb is not None:
                state[jid] = fb
                self._motion_tracker.record_feedback(jid, fb)
        return state

    def get_motion_tracker(self) -> MotionStateTracker:
        """
        Expose le tracker à qui doit le consulter — SafetyGuard (rate
        limiting avant envoi) et WatchdogSystem (détection d'anomalie
        après coup). Ni l'un ni l'autre ne le crée : IRobotHAL en reste
        l'unique propriétaire, conformément à la décision de conception
        actée (voir hal/core/motion_state.py).
        """
        return self._motion_tracker

    def get_joint_definitions(self) -> List[JointDefinition]:
        return list(self._joint_defs)

    def get_robot_info(self) -> Dict[str, str]:
        return {
            "manufacturer":     "Unitree Robotics",
            "model":            "G1",
            "serial_number":    self.robot_id,
            "hal_version":      self.HAL_VERSION,
            "firmware_version": "sim-0.1.0",
            "dof":              str(len(self._joint_defs)),
        }

    def get_capabilities(self) -> CapabilitySet:
        """
        Capacités réelles du G1 — humanoïde bipède avec bras
        articulés, pas de roues ni de contrôle de vol. GRIPPER inclus
        car le G1 dispose de mains capables de préhension (même si
        AXIS-OS ne contrôle actuellement que 24 joints principaux,
        sans le détail des doigts — voir backend.py IsaacSimBackend
        pour cette limite déjà documentée ; déclarer GRIPPER ici
        reste correct au niveau du ROBOT, indépendamment de ce
        qu'AXIS-OS pilote effectivement aujourd'hui). CAMERA ajoutée
        (oubli corrigé) : le G1 est équipé de série d'une caméra
        frontale, indépendamment du fait qu'AXIS-OS n'expose pas
        encore de driver de vision dédié pour l'utiliser.
        """
        return CapabilitySet.from_iterable([
            RobotCapability.LEGS,
            RobotCapability.ARMS,
            RobotCapability.GRIPPER,
            RobotCapability.IMU,
            RobotCapability.FORCE_TORQUE,
            RobotCapability.CAMERA,
        ])
