"""
AXIS-OS — Driver Unitree G1
==============================
Implémentation concrète de IRobotHAL pour le robot humanoïde Unitree G1.
Le HAL (Hardware Abstraction Layer, couche d'abstraction matérielle) est
l'interface commune par laquelle le reste d'AXIS-OS commande un robot
sans connaître son matériel ; ce fichier en fournit la version propre au
G1.

Deux modes de fonctionnement :
  - en simulation, des actionneurs simulés (G1SimulatedActuator, défini
    dans ce fichier) remplacent les moteurs ;
  - sur le robot réel, G1DDSActuator (voir g1_dds_actuator.py) passe par
    le SDK officiel Unitree (unitree_sdk2py, le kit de développement
    logiciel du fabricant) et le protocole DDS (Data
    Distribution Service, la messagerie temps réel par laquelle on
    commande le G1). Attention : ce driver réel n'a encore jamais été
    testé sur un robot physique (voir l'avertissement en tête de
    g1_dds_actuator.py).

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
    JointDefinition, JointLimits, JointType, JointGroup,
    Vector3, Quaternion
)
from axis_os.hal.core.urdf_loader import load_urdf, URDFParseError
from axis_os.hal.safety.limits import JointSafetyGuard, SafetyErrorKind
from axis_os.hal.core.motion_state import MotionStateTracker
import time


# Fichier URDF officiel du fabricant pour la variante 23 DOF (23 degrés
# de liberté, c'est-à-dire 23 articulations motorisées). Un URDF (Unified
# Robot Description Format) est le fichier XML standard qui décrit les
# articulations d'un robot, leurs axes, leurs limites et sa géométrie.
# Voir urdf/README.md pour sa provenance, sa licence (BSD 3-Clause) et le
# détail des erreurs qu'il a permis de corriger par rapport à l'ancienne
# description écrite à la main : couple maximal (torque_max) sous-estimé
# jusqu'à 37 % sur les hanches et les genoux, articulation waist_roll qui
# n'existe pas dans cette variante, et coude qui était déjà correct en une
# seule articulation, contrairement à ce que supposait une version
# antérieure du code.
_G1_URDF_PATH = Path(__file__).parent / "urdf" / "g1_23dof_rev_1_0.urdf"


# URDF officiel de la variante 29 DOF, de même provenance (voir
# urdf/README.md).
_G1_29DOF_URDF_PATH = Path(__file__).parent / "urdf" / "g1_29dof_rev_1_0.urdf"


def build_g1_joint_definitions(variante: str = "23dof") -> List[JointDefinition]:
    """Articulations d'une variante du G1, lues dans l'URDF officiel.

    23dof : une articulation de taille, cinq par bras, mains en caoutchouc.
    29dof : trois articulations de taille, sept par bras. C'est la
            variante que visent GR00T Whole-Body Control (la politique de
            contrôle du corps entier de NVIDIA), les modèles GR00T et
            LeRobot (bibliothèque de robotique de Hugging Face) ; voir
            docs/integration_gr00t_wbc.md.

    Toute autre valeur de `variante` lève ValueError.
    """
    if variante == "23dof":
        return _build_g1_joint_definitions()
    if variante == "29dof":
        model = load_urdf(_G1_29DOF_URDF_PATH)
        return [replace(j, group=_G1_29DOF_JOINT_GROUPS[j.joint_id])
                for j in model.get_joints()]
    raise ValueError(f"variante G1 inconnue : {variante!r} (23dof ou 29dof)")


def _build_g1_joint_definitions() -> List[JointDefinition]:
    """
    Les 23 articulations du G1 23 DOF, chargées depuis le fichier URDF
    officiel du fabricant (voir _G1_URDF_PATH) plutôt que recodées à la
    main.

    Pourquoi : une version antérieure construisait chaque JointDefinition
    à la main. Cette approche a été abandonnée le 2026-09-08, quand on a
    découvert que ce codage manuel contenait de vraies erreurs de
    sécurité (limites de couple sous-estimées, et pas seulement des noms
    approximatifs), en plus des écarts déjà connus sur le nombre
    d'articulations du torse et du coude. Le fichier URDF du fabricant est
    désormais l'unique source de vérité pour la géométrie et les limites
    du G1. Voir urdf_loader.py pour le parseur et ses garanties (par
    exemple, le rejet strict de toute articulation motorisée sans
    limites).

    Lève URDFParseError si le fichier est absent ou corrompu. C'est
    voulu : un HAL qui ne peut pas charger la définition de ses propres
    articulations ne doit jamais démarrer avec des valeurs par défaut
    silencieuses.
    """
    model = load_urdf(_G1_URDF_PATH)
    return [replace(j, group=_G1_JOINT_GROUPS[j.joint_id])
            for j in model.get_joints()]


# Groupe fonctionnel de chaque articulation du G1, déclaré ici par le
# driver. L'URDF décrit la géométrie et les limites, jamais le rôle
# fonctionnel d'une articulation : ce rôle est une connaissance du robot,
# au même titre que les conventions de signe de conventions.py.
#
# Pourquoi le déclarer explicitement : le SDK des apps d'AXIS-OS
# devinait auparavant ce rôle à partir de mots-clés anglais du nom
# ("shoulder", "knee"...). Conséquence constatée sur le G1 :
# waist_yaw_joint (rotation du torse) ne contenait aucun de ces
# mots-clés, si bien qu'aucune app ne pouvait faire pivoter le torse,
# sauf via move_all(). Il est désormais déclaré OTHER : il reste hors de
# move_arms() et de move_legs(), ce qui est correct (ce n'est ni un bras
# ni une jambe), mais de façon explicite, et move_to_pose() le signale
# au lieu de l'ignorer.
_G1_JOINT_GROUPS: Dict[str, JointGroup] = {
    # Jambes : 12 articulations
    "left_hip_pitch_joint":       JointGroup.LOCOMOTION,
    "left_hip_roll_joint":        JointGroup.LOCOMOTION,
    "left_hip_yaw_joint":         JointGroup.LOCOMOTION,
    "left_knee_joint":            JointGroup.LOCOMOTION,
    "left_ankle_pitch_joint":     JointGroup.LOCOMOTION,
    "left_ankle_roll_joint":      JointGroup.LOCOMOTION,
    "right_hip_pitch_joint":      JointGroup.LOCOMOTION,
    "right_hip_roll_joint":       JointGroup.LOCOMOTION,
    "right_hip_yaw_joint":        JointGroup.LOCOMOTION,
    "right_knee_joint":           JointGroup.LOCOMOTION,
    "right_ankle_pitch_joint":    JointGroup.LOCOMOTION,
    "right_ankle_roll_joint":     JointGroup.LOCOMOTION,
    # Torse — ni bras ni jambe
    "waist_yaw_joint":            JointGroup.OTHER,
    # Bras : 10 articulations
    "left_shoulder_pitch_joint":  JointGroup.MANIPULATION,
    "left_shoulder_roll_joint":   JointGroup.MANIPULATION,
    "left_shoulder_yaw_joint":    JointGroup.MANIPULATION,
    "left_elbow_joint":           JointGroup.MANIPULATION,
    "left_wrist_roll_joint":      JointGroup.MANIPULATION,
    "right_shoulder_pitch_joint": JointGroup.MANIPULATION,
    "right_shoulder_roll_joint":  JointGroup.MANIPULATION,
    "right_shoulder_yaw_joint":   JointGroup.MANIPULATION,
    "right_elbow_joint":          JointGroup.MANIPULATION,
    "right_wrist_roll_joint":     JointGroup.MANIPULATION,
}


# Variante 29 DOF : les groupes du 23 DOF, plus les six articulations
# propres à cette variante. Les nouvelles articulations de la taille
# sont classées OTHER, comme waist_yaw : ni membre locomoteur, ni bras.
# C'est le groupe que la marche et les apps devront se partager
# explicitement.
_G1_29DOF_JOINT_GROUPS: Dict[str, JointGroup] = {
    **_G1_JOINT_GROUPS,
    "waist_roll_joint":        JointGroup.OTHER,
    "waist_pitch_joint":       JointGroup.OTHER,
    "left_wrist_pitch_joint":  JointGroup.MANIPULATION,
    "left_wrist_yaw_joint":    JointGroup.MANIPULATION,
    "right_wrist_pitch_joint": JointGroup.MANIPULATION,
    "right_wrist_yaw_joint":   JointGroup.MANIPULATION,
}


class G1SimulatedActuator(IActuator):
    """Actionneur simulé d'une articulation du G1 (un « mock », c'est-à-dire
    une imitation logicielle du moteur), pour la simulation sans le SDK
    ni le robot physique."""

    def __init__(self, definition: JointDefinition):
        super().__init__(definition.joint_id, definition)
        # Borne sur l'écart entre la position de consigne et la position
        # mesurée. Pourquoi : un audit de sécurité a montré qu'en
        # contrôle de position, le couple réellement produit par le
        # moteur n'était borné par rien. Ce couple vaut à peu près l'écart
        # de position multiplié par le gain kp du contrôleur PD
        # (proportionnel-dérivé) intégré au moteur : un saut de consigne
        # pourtant dans les limites pouvait donc dépasser le couple
        # maximal. D'où position_delta_max = torque_max / kp, même logique
        # que G1DDSActuator (détail : docs/audit_externe_2026-09-14.md).
        # Le simulé n'ayant pas de vrai contrôleur PD dont tirer kp, il
        # prend une valeur de référence prudente, 40.0, identique à la
        # valeur par défaut de _DEFAULT_KP côté driver réel.
        limits_with_delta = replace(
            definition.limits,
            position_delta_max=definition.limits.torque_max / 40.0,
            # current_is_measured=True uniquement ici : ce simulé dérive
            # réellement son courant du couple simulé (voir
            # get_feedback() ci-dessous), alors que sur le G1 réel,
            # G1DDSActuator reçoit toujours feedback.current = 0.0
            # (contexte complet dans JointLimits.current_is_measured).
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
        # defer_publish est ignoré : en simulation pure, il n'y a pas de
        # bus partagé, donc pas de publication groupée à différer. Voir
        # IActuator.send_command() pour le contexte complet.
        if self._state == ActuatorState.EMERGENCY_STOP:
            return HALStatus.ERROR_FAULT

        current_feedback = self.get_feedback()

        # Pourquoi contrôler la santé du moteur avant chaque commande : un
        # audit de sécurité a montré que check_feedback_health() existait
        # déjà (elle détecte notamment TORQUE_OVERLOAD, un couple mesuré
        # supérieur à 110 % du maximum, signe qu'un moteur force contre
        # un obstacle, par exemple un bras coincé), mais qu'elle n'était
        # appelée nulle part en dehors des tests. Le watchdog (le
        # superviseur qui surveille l'état du robot en continu) fait un
        # contrôle voisin, mais le classe seulement CRITICAL, et non
        # FATAL, le seul niveau qui déclenche un arrêt d'urgence
        # (e-stop) : un bras qui poussait contre un humain continuait
        # donc de pousser. Ici, une surcharge de couple (ou de courant)
        # déclenche un arrêt d'urgence immédiat de cette articulation,
        # avant même de traiter la nouvelle commande.
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
    """Centrale inertielle (IMU, le capteur qui mesure accélérations,
    vitesses de rotation et orientation) simulée : elle décrit toujours un
    robot immobile et droit (gravité seule, orientation neutre)."""

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
    """IRobotHAL complet pour le Unitree G1, en simulation ("sim") ou sur
    le robot réel ("dds")."""

    HAL_VERSION = "0.1.0"

    def __init__(
        self,
        robot_id: str = "unitree_g1",
        backend: str = "sim",
        network_interface: Optional[str] = None,
        domain_id: int = 0,
        variante: str = "23dof",
    ):
        """
        `backend` : "sim" (par défaut, compatible avec l'existant :
        G1SimulatedActuator, aucune dépendance externe) ou "dds" (robot
        réel : G1DDSActuator via unitree_sdk2py, voir g1_dds_actuator.py ;
        attention, jamais testé sur un robot physique).

        Pourquoi ce paramètre : un audit de sécurité a relevé qu'aucun
        chemin de code ne permettait de construire un UnitreeG1HAL avec le
        driver réel. La classe instanciait toujours G1SimulatedActuator,
        sans option, si bien que G1DDSActuator existait mais restait
        totalement inatteignable.

        `network_interface` et `domain_id` : ne servent qu'avec
        `backend="dds"` (network_interface y est alors obligatoire) et
        sont transmis tels quels à G1DDSHub (voir sa docstring). Ignorés
        avec `backend="sim"`.

        `variante` : "23dof" (par défaut) ou "29dof", voir
        build_g1_joint_definitions().
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

        self._variante = variante
        self._joint_defs: List[JointDefinition] = build_g1_joint_definitions(variante)
        self._actuators:  Dict[str, IActuator]  = {}
        self._imu:        Optional[IIMU]         = None

        # Le MotionStateTracker (qui garde, pour chaque articulation, la
        # dernière commande envoyée et la dernière mesure reçue) est
        # branché ici. Décision de conception : c'est IRobotHAL qui le
        # possède et l'instancie (le composant est générique, voir
        # hal/core/motion_state.py), jamais un driver particulier qui le
        # créerait pour lui-même. Il est
        # exposé par get_motion_tracker() pour que SafetyGuard et
        # WatchdogSystem le consultent sans le dupliquer.
        self._motion_tracker = MotionStateTracker(
            [jdef.joint_id for jdef in self._joint_defs]
        )

    def initialize_all(self) -> HALStatus:
        print(f"\n[AXIS-OS] Initialisation Unitree G1 — id={self.robot_id} (backend={self._backend})")

        # Validation de la table de conventions (sens et origine de chaque
        # articulation, voir conventions.py) avant tout mouvement possible.
        # Pourquoi : une revue extérieure du code a relevé que, sans ce
        # contrôle, une articulation absente de la table ou un signe
        # invalide (par exemple 0.5 au lieu de ±1.0) ne serait détecté
        # qu'au premier appel du normaliseur (KinematicNormalizer, qui
        # convertit les angles bruts du robot en angles normalisés) sur
        # cette articulation précise : peut-être jamais en test, peut-être
        # en plein mouvement en production.
        from .conventions import conventions_pour
        from .convention_validator import validate_convention_table

        expected_ids = [jdef.joint_id for jdef in self._joint_defs]
        validation = validate_convention_table(conventions_pour(self._variante),
                                               expected_ids)
        print(f"  {validation}")
        if not validation.valid:
            print("  [ERREUR] Table de conventions invalide — initialisation refusée.")
            return HALStatus.ERROR_FAULT
        if validation.non_verifiees and self._backend != "sim":
            # Sur un robot réel, jamais : un signe non vérifié peut envoyer
            # une articulation dans le sens opposé à celui demandé. Ce
            # signe se vérifie en simulation, avec la suite de validation
            # S2 (voir docs/campagne_validation_isaac.md).
            print(f"  [ERREUR] Conventions NON VÉRIFIÉES sur un robot réel : "
                  f"{validation.non_verifiees} — initialisation refusée. "
                  f"Lancer la suite S2 en simulation d'abord.")
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
            # Pas d'IMU simulée en mode matériel réel : dans cette
            # première version du driver, le vrai G1 expose sa centrale
            # inertielle par G1DDSHub.read_imu_state(), et non par une
            # classe IIMU séparée. Ce choix est documenté explicitement
            # plutôt que de faire semblant avec une IMU simulée dans un
            # contexte réel.
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
                # defer_publish=True : en mode "dds", l'actionneur se
                # contente de remplir le message commun, publié une seule
                # fois plus bas. Pourquoi : un premier audit de sécurité
                # avait montré qu'aucune commande n'atteignait jamais le
                # robot, faute de publication sur le réseau DDS. Le
                # premier correctif publiait après chaque articulation,
                # plutôt qu'une seule fois pour les 23 ; un second audit
                # a montré que cela produisait des messages LowCmd_
                # intermédiaires incohérents. LowCmd_ est le message
                # unique de commande bas niveau du G1, qui porte les
                # consignes de tous les moteurs : publié en cours de
                # remplissage, il mêle des articulations déjà sur la
                # nouvelle consigne et d'autres encore sur l'ancienne.
                # Voir G1DDSActuator.send_command() pour la justification
                # physique complète, et CHANGELOG.md. En mode "sim",
                # G1SimulatedActuator ignore ce paramètre sans risque (il
                # n'y a pas de publication groupée).
                status = self._actuators[jid].send_command(cmd, defer_publish=True)
                # La commande n'est enregistrée dans le tracker que si
                # elle a vraiment été acceptée. Pourquoi : un audit de
                # sécurité, en exécutant le code, a trouvé que la version
                # précédente l'enregistrait sans condition, malgré un
                # commentaire qui affirmait le contraire (« reflète ce
                # qui a effectivement été transmis... pas une commande
                # hypothétique qui aurait pu être rejetée »). Une
                # commande rejetée par JointSafetyGuard (ERROR_LIMIT)
                # était donc quand même enregistrée comme dernière
                # commande de l'articulation. Un garde-fou qui s'appuie
                # sur cet historique (par exemple une limite
                # d'accélération calculée sur l'écart entre commandes
                # successives) raisonnait alors sur une référence
                # faussée : il pouvait soit refuser à tort la commande
                # légitime suivante, soit laisser passer une vraie
                # violation.
                if status == HALStatus.OK:
                    self._motion_tracker.record_command(jid, cmd)
                results[jid] = status
            else:
                results[jid] = HALStatus.ERROR_NOT_INIT

        # Publication groupée unique, une fois le message (buffer) rempli
        # pour toutes les articulations (voir defer_publish ci-dessus).
        # Aucune commande normale n'atteint le réseau DDS avant cette
        # ligne ; seul emergency_stop(), sur chaque actionneur, publie
        # immédiatement et indépendamment de ce chemin.
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
        Donne accès au tracker aux composants qui doivent le consulter :
        SafetyGuard (limitation du rythme de variation des commandes, ou
        « rate limiting », avant envoi) et WatchdogSystem (détection
        d'anomalies après coup). Ni l'un ni l'autre ne le crée : IRobotHAL
        en reste l'unique propriétaire, conformément à la décision de
        conception décrite dans hal/core/motion_state.py.
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
        Capacités réelles du G1 : humanoïde bipède avec bras articulés,
        sans roues ni contrôle de vol.

        Règle suivie : on décrit ce que le robot possède, et non ce
        qu'AXIS-OS en pilote aujourd'hui. Ainsi :
          - GRIPPER (préhension) est déclaré, car le G1 dispose de mains
            capables de saisir, même si AXIS-OS ne pilote actuellement
            que les articulations principales des membres (24 quand
            cette règle a été écrite, 23 ou 29 aujourd'hui selon la
            variante), sans le détail des doigts (limite déjà documentée
            dans IsaacSimBackend, backend.py) ;
          - CAMERA est déclarée (un oubli corrigé) : le G1 est équipé
            de série d'une caméra frontale, même si AXIS-OS n'expose pas
            encore de driver de vision dédié pour l'utiliser.
        """
        return CapabilitySet.from_iterable([
            RobotCapability.LEGS,
            RobotCapability.ARMS,
            RobotCapability.GRIPPER,
            RobotCapability.IMU,
            RobotCapability.FORCE_TORQUE,
            RobotCapability.CAMERA,
            # MICROPHONE / SPEAKER : même règle que CAMERA (voir la
            # docstring) : on décrit ce que le robot possède, et non ce
            # qu'AXIS-OS en pilote aujourd'hui. Le G1 est équipé d'un
            # micro et d'un haut-parleur intégrés.
            #
            # Conséquence concrète, constatée : sans cette déclaration,
            # le Play Store (la boutique d'apps d'AXIS-OS) refuse
            # d'installer toute app qui exige SPEECH_IN (l'écoute de la
            # parole par le micro). Ce refus était d'ailleurs le bon
            # comportement, puisque le robot ne déclarait aucun micro.
            #
            # À vérifier sur l'exemplaire réel avant mise en service :
            # l'équipement varie d'une version du robot à l'autre, et une
            # capacité déclarée à tort fait installer une app qui ne
            # pourra pas fonctionner.
            RobotCapability.MICROPHONE,
            RobotCapability.SPEAKER,
        ])
