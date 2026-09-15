"""
AXIS-OS — Driver Unitree G1
==============================
Implémentation concrète de IRobotHAL pour le Unitree G1.
En simulation : actionneurs mockés (G1SimulatedActuator, ce fichier).
En production : G1DDSActuator (voir g1_dds_actuator.py) — driver
réel via le SDK officiel Unitree (unitree_sdk2py, protocole DDS),
⚠️ non testé sur matériel physique, voir l'avertissement en tête de
ce module.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from pathlib import Path
from typing import Optional, Dict, List
from dataclasses import replace

from axis_os.hal.core.interfaces import IRobotHAL, IActuator, IIMU, IForceTorqueSensor
from axis_os.hal.core.capabilities import CapabilitySet, RobotCapability
from axis_os.hal.core.types import (
    ActuatorCommand, ActuatorFeedback, ActuatorState,
    ControlMode, HALStatus, IMUData, SensorType,
    JointDefinition, JointLimits, JointType,
    Vector3, Quaternion
)
from axis_os.hal.core.urdf_loader import load_urdf, URDFParseError
from axis_os.hal.safety.limits import JointSafetyGuard, SafetyErrorKind
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
        # Borne sur le delta de position (correctif C5, audit de
        # sécurité) — même logique que G1DDSActuator, avec un kp de
        # référence conservateur (40.0, cohérent avec le défaut de
        # _DEFAULT_KP côté driver réel) puisque le simulé n'a pas de
        # vrai contrôleur PD dont dériver cette valeur.
        limits_with_delta = replace(
            definition.limits,
            position_delta_max=definition.limits.torque_max / 40.0,
            # current_is_measured=True UNIQUEMENT ici — ce simulé dérive
            # bien son courant du couple simulé (voir get_feedback()
            # ci-dessous), à la différence de G1DDSActuator où
            # feedback.current vaut toujours 0.0 sur le G1 réel (voir
            # JointLimits.current_is_measured pour le contexte complet).
            current_is_measured=True,
        )
        self._guard       = JointSafetyGuard(definition.joint_id, limits_with_delta)
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

    def send_command(self, command: ActuatorCommand, defer_publish: bool = False) -> HALStatus:
        # defer_publish ignoré : pas de notion de publication groupée
        # sur un bus partagé en simulation pure — voir IActuator.
        # send_command() pour le contexte complet.
        if self._state == ActuatorState.EMERGENCY_STOP:
            return HALStatus.ERROR_FAULT

        current_feedback = self.get_feedback()

        # Trouvé par un audit de sécurité : check_feedback_health()
        # existait déjà (détecte notamment TORQUE_OVERLOAD — couple
        # mesuré > 110% du max, signe qu'un moteur force contre un
        # obstacle, ex: un bras coincé) mais n'était appelée nulle
        # part en dehors des tests. Le watchdog fait un contrôle
        # voisin mais le classe seulement CRITICAL (pas FATAL, qui
        # seul déclenche un e-stop) — un bras qui pousse contre un
        # humain continuait donc de pousser. Ici, un TORQUE_OVERLOAD
        # déclenche un arrêt d'urgence immédiat de CE joint avant même
        # de traiter la nouvelle commande.
        if current_feedback is not None:
            health_violations = self._guard.check_feedback_health(current_feedback)
            for v in health_violations:
                if v.kind in (SafetyErrorKind.TORQUE_OVERLOAD,
                              SafetyErrorKind.CURRENT_OVERLOAD):
                    print(f"  [SAFETY] Surcharge détectée, arrêt d'urgence : {v}")
                    self.emergency_stop()
                    return HALStatus.ERROR_FAULT

        ok, violation = self._guard.validate_command(command, current_feedback)
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

    def __init__(
        self,
        robot_id: str = "unitree_g1",
        backend: str = "sim",
        network_interface: Optional[str] = None,
        domain_id: int = 0,
    ):
        """
        `backend` : "sim" (défaut, rétrocompatible — G1SimulatedActuator,
        aucune dépendance externe) ou "dds" (matériel réel, G1DDSActuator
        via unitree_sdk2py — voir g1_dds_actuator.py, ⚠️ non testé sur
        matériel physique). Trou identifié par un audit de sécurité :
        jusqu'ici AUCUN chemin de code ne permettait de construire un
        UnitreeG1HAL avec le driver réel — il était instancié en dur sur
        G1SimulatedActuator, sans option, rendant G1DDSActuator
        totalement inatteignable malgré son existence.

        `network_interface` et `domain_id` : requis uniquement si
        `backend="dds"` — transmis tels quels à G1DDSHub (voir sa
        docstring). Ignorés pour `backend="sim"`.
        """
        super().__init__(robot_id)
        if backend not in ("sim", "dds"):
            raise ValueError(
                f"UnitreeG1HAL: backend doit être 'sim' ou 'dds', reçu '{backend}'."
            )
        if backend == "dds" and not network_interface:
            raise ValueError(
                "UnitreeG1HAL(backend='dds') requiert network_interface "
                "(ex: 'eth0') — jamais deviné, voir G1DDSHub."
            )
        self._backend = backend
        self._network_interface = network_interface
        self._domain_id = domain_id
        self._dds_hub = None  # créé dans initialize_all() si backend == "dds"

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
        print(f"\n[AXIS-OS] Initialisation Unitree G1 — id={self.robot_id} (backend={self._backend})")

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

        if self._backend == "dds":
            from .g1_dds_actuator import G1DDSHub, G1DDSActuator
            self._dds_hub = G1DDSHub(self._network_interface, self._domain_id)
            hub_status = self._dds_hub.initialize()
            if hub_status != HALStatus.OK:
                print(f"  [ERREUR] Échec initialisation G1DDSHub : {hub_status}")
                return hub_status

        for jdef in self._joint_defs:
            if self._backend == "dds":
                act = G1DDSActuator(jdef, self._dds_hub)
            else:
                act = G1SimulatedActuator(jdef)
            if act.initialize() != HALStatus.OK:
                return HALStatus.ERROR_COMM
            self._actuators[jdef.joint_id] = act

        if self._backend == "dds":
            # Pas d'IMU simulée en mode matériel réel — le vrai G1
            # expose son IMU via G1DDSHub.read_imu_state(), pas via
            # une classe IIMU séparée dans cette première version du
            # driver. Documenté explicitement plutôt que de faire
            # semblant avec une IMU simulée en contexte "réel".
            self._imu = None
        else:
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
        self._mark_estop_active()
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
                # defer_publish=True en mode dds — correctif d'un
                # second audit de sécurité (après la correction
                # initiale de C2) : publier après CHAQUE joint plutôt
                # qu'une fois pour les 23 produisait des messages
                # LowCmd_ intermédiaires incohérents (une partie des
                # articulations sur la nouvelle consigne, l'autre sur
                # l'ancienne) — voir G1DDSActuator.send_command() pour
                # la justification physique complète. En mode "sim",
                # G1SimulatedActuator ignore ce paramètre sans risque
                # (pas de notion de publication groupée).
                status = self._actuators[jid].send_command(cmd, defer_publish=True)
                # Enregistrement dans le tracker UNIQUEMENT si la
                # commande a vraiment été acceptée — bug trouvé par un
                # audit de sécurité (exécution réelle) : la version
                # précédente enregistrait INCONDITIONNELLEMENT, malgré
                # un commentaire affirmant le contraire ("reflète ce
                # qui a effectivement été transmis... pas une commande
                # hypothétique qui aurait pu être rejetée"). Une
                # commande rejetée par JointSafetyGuard (ERROR_LIMIT)
                # était quand même enregistrée comme dernière commande
                # du joint — un garde-fou de sécurité qui utiliserait
                # cet historique (limite d'accélération basée sur le
                # delta entre commandes successives) raisonnerait alors
                # sur une référence falsifiée, pouvant soit déclencher
                # un faux positif sur la commande légitime suivante,
                # soit masquer une vraie violation.
                if status == HALStatus.OK:
                    self._motion_tracker.record_command(jid, cmd)
                results[jid] = status
            else:
                results[jid] = HALStatus.ERROR_NOT_INIT

        # Publication groupée UNIQUE, après avoir rempli le buffer pour
        # les 23 joints — voir defer_publish ci-dessus. Aucune commande
        # normale n'atteint le réseau DDS avant cette ligne ; seul
        # emergency_stop() (sur chaque actionneur) publie immédiatement
        # et indépendamment de ce chemin.
        if self._backend == "dds" and self._dds_hub is not None:
            self._dds_hub.publish()

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
