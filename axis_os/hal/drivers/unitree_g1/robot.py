"""
AXIS-OS — Driver Unitree G1
==============================
Implémentation concrète de IRobotHAL pour le Unitree G1.
En simulation : actionneurs mockés.
En production  : remplacer G1SimulatedActuator par G1CANActuator.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from typing import Optional, Dict, List

from axis_os.hal.core.interfaces import IRobotHAL, IActuator, IIMU, IForceTorqueSensor
from axis_os.hal.core.capabilities import CapabilitySet, RobotCapability
from axis_os.hal.core.types import (
    ActuatorCommand, ActuatorFeedback, ActuatorState,
    ControlMode, HALStatus, IMUData, SensorType,
    JointDefinition, JointLimits, JointType,
    Vector3, Quaternion
)
from axis_os.hal.safety.limits import JointSafetyGuard
from axis_os.hal.core.motion_state import MotionStateTracker
import time


def _build_g1_joint_definitions() -> List[JointDefinition]:
    """24 joints du Unitree G1 avec leurs limites hardware."""

    def leg(pos_min, pos_max) -> JointLimits:
        return JointLimits(pos_min=pos_min, pos_max=pos_max,
                           vel_max=20.0, torque_max=88.0,
                           current_max=20.0, temperature_max=80.0)

    def arm(pos_min, pos_max) -> JointLimits:
        return JointLimits(pos_min=pos_min, pos_max=pos_max,
                           vel_max=20.0, torque_max=25.0,
                           current_max=10.0, temperature_max=80.0)

    return [
        # Jambe gauche
        JointDefinition("left_hip_roll",    JointType.REVOLUTE, leg(-0.52, 0.52),  "pelvis",         "left_thigh"),
        JointDefinition("left_hip_yaw",     JointType.REVOLUTE, leg(-0.52, 0.52),  "pelvis",         "left_thigh"),
        JointDefinition("left_hip_pitch",   JointType.REVOLUTE, leg(-1.57, 2.09),  "left_thigh",     "left_upper_leg"),
        JointDefinition("left_knee",        JointType.REVOLUTE, leg(-0.09, 2.53),  "left_upper_leg", "left_lower_leg"),
        JointDefinition("left_ankle_pitch", JointType.REVOLUTE, leg(-0.87, 0.52),  "left_lower_leg", "left_foot"),
        JointDefinition("left_ankle_roll",  JointType.REVOLUTE, leg(-0.26, 0.26),  "left_foot",      "left_sole"),
        # Jambe droite
        JointDefinition("right_hip_roll",    JointType.REVOLUTE, leg(-0.52, 0.52), "pelvis",          "right_thigh"),
        JointDefinition("right_hip_yaw",     JointType.REVOLUTE, leg(-0.52, 0.52), "pelvis",          "right_thigh"),
        JointDefinition("right_hip_pitch",   JointType.REVOLUTE, leg(-2.09, 1.57), "right_thigh",     "right_upper_leg"),
        JointDefinition("right_knee",        JointType.REVOLUTE, leg(-2.53, 0.09), "right_upper_leg", "right_lower_leg"),
        JointDefinition("right_ankle_pitch", JointType.REVOLUTE, leg(-0.52, 0.87), "right_lower_leg", "right_foot"),
        JointDefinition("right_ankle_roll",  JointType.REVOLUTE, leg(-0.26, 0.26), "right_foot",      "right_sole"),
        # Torse — ⚠️ ÉCART CONNU NON CORRIGÉ ICI (2026-09-06) : le vrai
        # G1 dans Isaac Sim n'a qu'un seul joint de torse ("torso_joint"),
        # pas deux (waist_yaw + waist_roll) comme supposé ci-dessous.
        # conventions.py a déjà été corrigé pour utiliser "torso" à la
        # place — mais PAS ce fichier, car fusionner waist_yaw+waist_roll
        # en un seul joint change la CHAÎNE CINÉMATIQUE (pelvis→torso→
        # chest) définie par les 3e/4e arguments de JointDefinition,
        # pas juste un nom. Une correction ici doit être testée sur le
        # vrai G1 avant d'être faite — non fait dans cette session pour
        # éviter un changement structurel non vérifiable sans serveur
        # Isaac Sim actif. Ce fichier (robot.py, notre stub interne
        # UnitreeG1HAL) et backend.py (IsaacSimBackend, le vrai G1) sont
        # donc actuellement DÉSYNCHRONISÉS sur ce point précis — le stub
        # utilise encore waist_yaw/waist_roll, le vrai G1 utilise torso.
        JointDefinition("waist_yaw",  JointType.REVOLUTE, leg(-1.05, 1.05), "pelvis", "torso"),
        JointDefinition("waist_roll", JointType.REVOLUTE, leg(-0.52, 0.52), "torso",  "chest"),
        # Bras gauche — ⚠️ MÊME ÉCART CONNU pour le coude : le vrai G1 a
        # left_elbow_pitch_joint + left_elbow_roll_joint (2 joints),
        # pas un seul left_elbow comme ci-dessous. Même raisonnement
        # que pour le torse — non corrigé ici, désynchronisé avec
        # backend.py volontairement plutôt que de deviner la bonne
        # chaîne cinématique sans pouvoir tester.
        JointDefinition("left_shoulder_pitch", JointType.REVOLUTE, arm(-3.14, 3.14), "chest",           "left_upper_arm"),
        JointDefinition("left_shoulder_roll",  JointType.REVOLUTE, arm(-1.57, 1.57), "left_upper_arm",  "left_forearm"),
        JointDefinition("left_shoulder_yaw",   JointType.REVOLUTE, arm(-1.57, 1.57), "left_forearm",    "left_elbow"),
        JointDefinition("left_elbow",          JointType.REVOLUTE, arm(-1.57, 1.57), "left_elbow",      "left_lower_arm"),
        JointDefinition("left_wrist_roll",     JointType.REVOLUTE, arm(-1.57, 1.57), "left_lower_arm",  "left_hand"),
        # Bras droit — même écart connu
        JointDefinition("right_shoulder_pitch", JointType.REVOLUTE, arm(-3.14, 3.14), "chest",            "right_upper_arm"),
        JointDefinition("right_shoulder_roll",  JointType.REVOLUTE, arm(-1.57, 1.57), "right_upper_arm",  "right_forearm"),
        JointDefinition("right_shoulder_yaw",   JointType.REVOLUTE, arm(-1.57, 1.57), "right_forearm",    "right_elbow"),
        JointDefinition("right_elbow",          JointType.REVOLUTE, arm(-1.57, 1.57), "right_elbow",      "right_lower_arm"),
        JointDefinition("right_wrist_roll",     JointType.REVOLUTE, arm(-1.57, 1.57), "right_lower_arm",  "right_hand"),
    ]


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
