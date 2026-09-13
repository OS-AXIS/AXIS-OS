"""
AXIS-OS — Driver Unitree G1 (matériel réel, via unitree_sdk2py)
====================================================================
Implémentation de IActuator qui parle au VRAI G1 physique, en
utilisant le SDK officiel Unitree (unitree_sdk2py), plutôt que le
G1SimulatedActuator utilisé en simulation (voir robot.py).

Comble un trou identifié lors d'un audit du projet : jusqu'ici,
AUCUN chemin technique n'existait pour faire fonctionner AXIS-OS sur
un G1 physique — seul G1SimulatedActuator existait, et le commentaire
laissé dans robot.py ("remplacer G1SimulatedActuator par
G1CANActuator") était lui-même erroné : le G1 ne communique PAS par
CAN direct, mais par DDS (Data Distribution Service) via Ethernet —
confirmé par le SDK officiel unitree_sdk2py
(github.com/unitreerobotics/unitree_sdk2_python, licence BSD-3-Clause).

⚠️ NON TESTÉ SUR MATÉRIEL RÉEL — ce module a été écrit en lisant le
code source officiel de unitree_sdk2py (structures LowCmd_/LowState_/
MotorCmd_/MotorState_, exemple officiel
example/g1/low_level/g1_low_level_example.py) et en suivant
exactement ses patterns documentés, mais n'a jamais été exécuté
contre un vrai G1. À valider sur du matériel physique avant tout
déploiement client — voir la section "Points de sécurité obligatoires"
ci-dessous, directement issus des avertissements du SDK officiel.

Dépendances (non incluses dans requirements.txt — optionnelles,
nécessaires uniquement pour ce driver précis) :
    pip install unitree_sdk2py
    (nécessite aussi cyclonedds==0.10.2 — voir le README officiel du
    SDK pour les instructions d'installation, notamment en cas
    d'erreur "Could not locate cyclonedds")

## Points de sécurité obligatoires (issus du SDK officiel, jamais à contourner)

1. **Désactiver le mode sport intégré AVANT tout contrôle bas niveau.**
   Le G1 a un "high-level motion service" (sport_mode) actif par
   défaut qui gère lui-même la marche/l'équilibre. Envoyer des
   commandes bas niveau SANS le désactiver crée un conflit de
   commandes directement sur les moteurs — dangereux. Le SDK officiel
   fournit `MotionSwitcherClient` pour ça (voir `_ensure_low_level_mode()`
   ci-dessous), utilisé exactement comme dans l'exemple officiel.

2. **CRC obligatoire sur chaque commande.** Le champ `crc` de
   `LowCmd_` doit être calculé via `unitree_sdk2py.utils.crc.CRC`
   avant chaque envoi — une commande sans CRC valide est silencieusement
   ignorée par le firmware du robot (protection déjà intégrée par
   Unitree, pas quelque chose qu'AXIS-OS ajoute).

3. **`mode_machine` doit être lu depuis le premier LowState_ reçu**,
   jamais deviné ou codé en dur — varie selon la configuration exacte
   du robot.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from typing import Optional, Dict
import time

from axis_os.hal.core.interfaces import IActuator
from axis_os.hal.core.types import (
    ActuatorCommand, ActuatorFeedback, ActuatorState,
    ControlMode, HALStatus, JointDefinition
)
from axis_os.hal.safety.limits import JointSafetyGuard


# Table de correspondance joint_id AXIS-OS -> index officiel dans le
# tableau motor_cmd[35]/motor_state[35] du protocole unitree_hg.
# Source : example/g1/low_level/g1_low_level_example.py du SDK
# officiel (classe G1JointIndex), copiée à l'identique — ne jamais
# deviner ces index, une erreur ici enverrait une commande au MAUVAIS
# moteur physique. Seuls les 23 joints de la variante 23dof sont
# mappés (les index invalides pour cette variante — poignets pitch/
# yaw, waist roll/pitch — sont documentés comme tels dans le SDK
# officiel et volontairement absents ici).
G1_JOINT_ID_TO_MOTOR_INDEX: Dict[str, int] = {
    "left_hip_pitch_joint":      0,
    "left_hip_roll_joint":       1,
    "left_hip_yaw_joint":        2,
    "left_knee_joint":           3,
    "left_ankle_pitch_joint":    4,
    "left_ankle_roll_joint":     5,
    "right_hip_pitch_joint":     6,
    "right_hip_roll_joint":      7,
    "right_hip_yaw_joint":       8,
    "right_knee_joint":          9,
    "right_ankle_pitch_joint":   10,
    "right_ankle_roll_joint":    11,
    "waist_yaw_joint":           12,
    "left_shoulder_pitch_joint": 15,
    "left_shoulder_roll_joint":  16,
    "left_shoulder_yaw_joint":   17,
    "left_elbow_joint":          18,
    "left_wrist_roll_joint":     19,
    "right_shoulder_pitch_joint": 22,
    "right_shoulder_roll_joint":  23,
    "right_shoulder_yaw_joint":   24,
    "right_elbow_joint":          25,
    "right_wrist_roll_joint":     26,
}

# Gains PD par défaut, copiés de l'exemple officiel (Kp/Kd indexés
# 0-28 dans le SDK, ici restreints aux 23 index réellement utilisés
# par la variante 23dof). Un développeur intégrateur DOIT les
# retuner pour son robot précis — ce sont des valeurs de démarrage
# sûres issues de l'exemple officiel, pas des valeurs optimisées.
_DEFAULT_KP: Dict[int, float] = {
    0: 60, 1: 60, 2: 60, 3: 100, 4: 40, 5: 40,        # jambe gauche
    6: 60, 7: 60, 8: 60, 9: 100, 10: 40, 11: 40,       # jambe droite
    12: 60,                                             # taille
    15: 40, 16: 40, 17: 40, 18: 40, 19: 40,             # bras gauche
    22: 40, 23: 40, 24: 40, 25: 40, 26: 40,             # bras droit
}
_DEFAULT_KD: Dict[int, float] = {
    0: 1, 1: 1, 2: 1, 3: 2, 4: 1, 5: 1,
    6: 1, 7: 1, 8: 1, 9: 2, 10: 1, 11: 1,
    12: 1,
    15: 1, 16: 1, 17: 1, 18: 1, 19: 1,
    22: 1, 23: 1, 24: 1, 25: 1, 26: 1,
}


def _ensure_low_level_mode(timeout_s: float = 5.0) -> None:
    """
    Désactive le mode sport intégré du G1 avant tout contrôle bas
    niveau — voir "Points de sécurité obligatoires" en tête de module.
    Reproduit exactement le pattern de l'exemple officiel Unitree
    (boucle CheckMode/ReleaseMode jusqu'à confirmation).

    Importe unitree_sdk2py localement (pas en tête de module) : ce
    driver reste importable et son code lisible même sur une machine
    qui n'a pas le SDK installé (ex: CI, dev sans matériel réel) — le
    coût d'import n'est payé qu'au moment où ce driver est réellement
    instancié.
    """
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
        MotionSwitcherClient,
    )

    msc = MotionSwitcherClient()
    msc.SetTimeout(timeout_s)
    msc.Init()

    status, result = msc.CheckMode()
    while result.get("name"):
        msc.ReleaseMode()
        status, result = msc.CheckMode()
        time.sleep(1)


class G1DDSHub:
    """
    Point de communication DDS partagé pour TOUS les G1DDSActuator
    d'un même robot — un seul publisher/subscriber pour les 23
    joints, pas un par joint (le protocole LowCmd_/LowState_ du G1
    transporte déjà TOUS les moteurs dans un seul message, voir
    MotorCmd_[35]/MotorState_[35] en tête de module).

    Instancié UNE FOIS par UnitreeG1HAL réel (jamais par
    G1DDSActuator individuellement), puis passé à chaque actuateur
    à sa construction — pattern différent de G1SimulatedActuator
    (qui n'a pas besoin d'état partagé) mais nécessaire ici car le
    protocole DDS réel du G1 fonctionne par message unique multi-
    joints, pas par joint isolé.
    """

    def __init__(self, network_interface: str, domain_id: int = 0):
        self._network_interface = network_interface
        self._domain_id = domain_id
        self._low_cmd = None
        self._low_state = None
        self._mode_machine: Optional[int] = None
        self._publisher = None
        self._subscriber = None
        self._crc = None
        self._initialized = False

    def initialize(self) -> HALStatus:
        from unitree_sdk2py.core.channel import (
            ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize,
        )
        from unitree_sdk2py.idl.default import (
            unitree_hg_msg_dds__LowCmd_,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        ChannelFactoryInitialize(self._domain_id, self._network_interface)

        _ensure_low_level_mode()

        self._low_cmd = unitree_hg_msg_dds__LowCmd_()
        self._crc = CRC()

        self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._publisher.Init()

        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(self._on_low_state, 10)

        # Attend le premier LowState_ pour connaître le vrai
        # mode_machine — ne jamais le deviner (voir tête de module).
        waited_s = 0.0
        while self._mode_machine is None and waited_s < 5.0:
            time.sleep(0.1)
            waited_s += 0.1
        if self._mode_machine is None:
            return HALStatus.ERROR_TIMEOUT

        self._initialized = True
        return HALStatus.OK

    def _on_low_state(self, msg) -> None:
        self._low_state = msg
        if self._mode_machine is None:
            self._mode_machine = msg.mode_machine

    def send_motor_command(
        self, motor_index: int, mode: int, q: float, dq: float,
        tau: float, kp: float, kd: float,
    ) -> None:
        """Écrit dans le message LowCmd_ partagé — ne l'envoie PAS
        encore (voir publish()). Permet à plusieurs G1DDSActuator
        d'accumuler leurs commandes respectives dans le MÊME message
        avant un envoi groupé, cohérent avec le protocole réel du G1
        (un seul message transporte tous les moteurs)."""
        cmd = self._low_cmd.motor_cmd[motor_index]
        cmd.mode = mode
        cmd.q = q
        cmd.dq = dq
        cmd.tau = tau
        cmd.kp = kp
        cmd.kd = kd

    def publish(self, mode_pr: int = 0) -> None:
        """Calcule le CRC (obligatoire, voir tête de module) et
        envoie le message LowCmd_ accumulé sur le réseau DDS."""
        self._low_cmd.mode_pr = mode_pr
        self._low_cmd.mode_machine = self._mode_machine
        self._low_cmd.crc = self._crc.Crc(self._low_cmd)
        self._publisher.Write(self._low_cmd)

    def read_motor_state(self, motor_index: int):
        """Retourne le MotorState_ courant pour cet index, ou None
        si aucun LowState_ n'a encore été reçu."""
        if self._low_state is None:
            return None
        return self._low_state.motor_state[motor_index]

    def read_imu_state(self):
        if self._low_state is None:
            return None
        return self._low_state.imu_state

    @property
    def is_initialized(self) -> bool:
        return self._initialized


class G1DDSActuator(IActuator):
    """
    IActuator réel pour UN joint du G1 physique, via G1DDSHub partagé.

    ⚠️ NON TESTÉ SUR MATÉRIEL RÉEL — voir avertissement en tête de
    module. Suit exactement le contrat IActuator (interfaces.py) déjà
    validé par G1SimulatedActuator et par toute la suite de tests
    existante — aucune application ne devrait avoir besoin d'être
    modifiée pour utiliser ce driver à la place du simulé, seule la
    construction de UnitreeG1HAL change (voir robot.py).
    """

    def __init__(self, definition: JointDefinition, hub: G1DDSHub):
        super().__init__(definition.joint_id, definition)
        if definition.joint_id not in G1_JOINT_ID_TO_MOTOR_INDEX:
            raise ValueError(
                f"G1DDSActuator({definition.joint_id}) : aucun index moteur "
                f"connu pour ce joint_id — voir G1_JOINT_ID_TO_MOTOR_INDEX. "
                f"Un joint sans mapping ne doit JAMAIS être silencieusement "
                f"ignoré : mieux vaut échouer à la construction."
            )
        self._motor_index = G1_JOINT_ID_TO_MOTOR_INDEX[definition.joint_id]
        self._hub = hub
        self._guard = JointSafetyGuard(definition.joint_id, definition.limits)
        self._kp = _DEFAULT_KP.get(self._motor_index, 40.0)
        self._kd = _DEFAULT_KD.get(self._motor_index, 1.0)

    def initialize(self) -> HALStatus:
        if not self._hub.is_initialized:
            return HALStatus.ERROR_NOT_INIT
        self._state = ActuatorState.IDLE
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._state = ActuatorState.UNINITIALIZED
        return HALStatus.OK

    def calibrate(self) -> HALStatus:
        # Le G1 n'expose pas de calibration par joint via ce protocole
        # bas niveau — la calibration se fait au niveau firmware du
        # robot, hors de portée d'AXIS-OS. No-op documenté plutôt
        # qu'une fausse implémentation.
        return HALStatus.OK

    def set_control_mode(self, mode: ControlMode) -> HALStatus:
        self.definition.default_control_mode = mode
        return HALStatus.OK

    def send_command(self, command: ActuatorCommand) -> HALStatus:
        if self._state == ActuatorState.EMERGENCY_STOP:
            return HALStatus.ERROR_FAULT

        feedback = self.get_feedback()
        ok, violation = self._guard.validate_command(command, feedback)
        if not ok:
            print(f"  [SAFETY] {violation}")
            return HALStatus.ERROR_LIMIT

        # mode=1 : Enable (voir commentaire de l'exemple officiel :
        # "1:Enable, 0:Disable"). AXIS-OS n'utilise jamais mode=0 ici
        # — la désactivation d'un moteur individuel passe par
        # emergency_stop(), pas par ce chemin normal.
        self._hub.send_motor_command(
            motor_index=self._motor_index,
            mode=1,
            q=command.position if command.position is not None else 0.0,
            dq=command.velocity if command.velocity is not None else 0.0,
            tau=command.torque if command.torque is not None else 0.0,
            kp=self._kp,
            kd=self._kd,
        )
        self._state = ActuatorState.ACTIVE
        return HALStatus.OK

    def emergency_stop(self) -> HALStatus:
        # kp=kd=0, tau=0 : coupe le couple actif sur CE moteur tout en
        # gardant mode=1 (Enable) — le moteur cesse de pousser mais
        # reste "vu" par le firmware, plutôt que mode=0 qui pourrait
        # le faire retomber en roue libre de façon incontrôlée selon
        # la charge portée à cet instant précis (ex: un bras en l'air).
        self._hub.send_motor_command(
            motor_index=self._motor_index,
            mode=1, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0,
        )
        self._state = ActuatorState.EMERGENCY_STOP
        return HALStatus.OK

    def get_feedback(self) -> Optional[ActuatorFeedback]:
        motor_state = self._hub.read_motor_state(self._motor_index)
        if motor_state is None:
            return None
        return ActuatorFeedback(
            joint_id=self.joint_id,
            state=self._state,
            position=motor_state.q,
            velocity=motor_state.dq,
            torque=motor_state.tau_est,
            current=0.0,  # non exposé par MotorState_ — voir sa définition
            # temperature[0] = capteur moteur, temperature[1] = capteur
            # driver selon la structure officielle MotorState_ ; on
            # remonte le moteur, plus directement pertinent pour
            # JointSafetyGuard (seuils déjà pensés pour une température
            # de bobinage, pas de carte électronique).
            temperature=float(motor_state.temperature[0]),
            voltage=motor_state.vol,
        )
