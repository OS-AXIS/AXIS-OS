"""
AXIS-OS — Driver Unitree G1 pour le robot réel (via unitree_sdk2py)
====================================================================
Ce module commande le vrai G1 physique (l'humanoïde Unitree G1, robot de
référence d'AXIS-OS). Il implémente IActuator, l'interface commune à
tous les moteurs d'articulation dans la HAL (couche d'abstraction
matérielle : la partie d'AXIS-OS qui cache aux applications les détails
propres à chaque robot). Il s'appuie sur unitree_sdk2py, le SDK (kit de
développement logiciel) officiel d'Unitree. En simulation, c'est
G1SimulatedActuator (voir robot.py) qui joue ce rôle.

Pourquoi ce module existe : un audit du projet a montré que, jusque-là,
aucun chemin technique ne permettait de faire fonctionner AXIS-OS sur un
G1 physique. Seul G1SimulatedActuator existait, et le commentaire qui
figurait alors dans robot.py (« remplacer G1SimulatedActuator par
G1CANActuator ») était lui-même faux : le G1 ne se commande pas en CAN
direct (un bus de terrain courant en automobile et en robotique), mais
par DDS (Data Distribution Service, un standard d'échange de messages
entre programmes) sur Ethernet. C'est ce que confirme le SDK officiel
unitree_sdk2py (github.com/unitreerobotics/unitree_sdk2_python, licence
BSD-3-Clause).

Attention : ce module n'a JAMAIS été testé sur un G1 réel. Il a été
écrit en lisant le code source officiel de unitree_sdk2py et en
reprenant exactement les façons de faire qu'il documente : structures
de messages LowCmd_ (commande bas niveau envoyée aux moteurs), LowState_
(état bas niveau renvoyé par le robot), MotorCmd_ et MotorState_ (la
part de ces messages propre à chaque moteur), et exemple officiel
example/g1/low_level/g1_low_level_example.py. Il n'a cependant jamais
tourné face à un vrai G1 et doit être validé sur du matériel physique
avant tout déploiement chez un client. Voir la section « Points de
sécurité obligatoires » ci-dessous, tirée directement des avertissements
du SDK officiel.

Dépendances (absentes de requirements.txt : elles sont optionnelles et
ne servent qu'à ce driver) :
    pip install unitree_sdk2py
    (il faut aussi cyclonedds==0.10.2, une implémentation de DDS ; voir
    le README officiel du SDK pour l'installation, notamment en cas
    d'erreur "Could not locate cyclonedds")

## Points de sécurité obligatoires (issus du SDK officiel, jamais à contourner)

1. **Désactiver le mode sport intégré AVANT tout contrôle bas niveau.**
   Le G1 embarque un service de mouvement de haut niveau (« high-level
   motion service », appelé sport_mode), actif par défaut, qui gère
   lui-même la marche et l'équilibre. Le contrôle bas niveau, lui,
   commande directement chaque moteur. Envoyer des commandes bas niveau
   sans avoir désactivé ce service met deux sources de commandes en
   conflit directement sur les moteurs, ce qui est dangereux. Le SDK
   officiel fournit pour cela `MotionSwitcherClient`, utilisé ici
   exactement comme dans l'exemple officiel (voir
   `_ensure_low_level_mode()` ci-dessous).

2. **CRC obligatoire sur chaque commande.** Le CRC est une somme de
   contrôle qui permet de détecter un message altéré. Le champ `crc` de
   `LowCmd_` doit être calculé avec `unitree_sdk2py.utils.crc.CRC` avant
   chaque envoi : le firmware du robot (son logiciel interne) ignore,
   sans le signaler, toute commande dont le CRC est invalide. Cette
   protection est intégrée par Unitree ; ce n'est pas AXIS-OS qui
   l'ajoute.

3. **`mode_machine` doit être lu dans le premier LowState_ reçu**,
   jamais deviné ni codé en dur : sa valeur dépend de la configuration
   exacte du robot. Ce champ de l'état renvoyé par le robot est ensuite
   recopié dans chaque commande LowCmd_ envoyée (voir publish()).

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from typing import Optional, Dict
from dataclasses import replace
import threading
import time

from axis_os.hal.core.interfaces import IActuator
from axis_os.hal.core.types import (
    ActuatorCommand, ActuatorFeedback, ActuatorState,
    ControlMode, HALStatus, JointDefinition
)
from axis_os.hal.safety.limits import JointSafetyGuard, SafetyErrorKind


# Correspondance entre le joint_id d'AXIS-OS (le nom de l'articulation)
# et l'index officiel du moteur dans les tableaux motor_cmd[35] et
# motor_state[35] du protocole unitree_hg (le format des messages DDS du
# G1).
# Ces index ne doivent jamais être devinés : une erreur ici enverrait
# une commande au MAUVAIS moteur physique. Ils sont donc copiés à
# l'identique de la classe G1JointIndex de l'exemple officiel du SDK
# (example/g1/low_level/g1_low_level_example.py).
# Seuls les 23 joints de la variante à 23 degrés de liberté (« 23dof »)
# figurent ici. Les index sans objet pour cette variante (poignets en
# tangage et en lacet, « pitch » et « yaw » ; taille en roulis et en
# tangage, « roll » et « pitch ») sont signalés comme tels dans le SDK
# officiel et volontairement absents.
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

# Gains PD par défaut. Le moteur applique un régulateur PD
# (proportionnel-dérivé) : le couple dépend de l'écart de position,
# pondéré par le gain Kp (la « raideur »), et de l'écart de vitesse,
# pondéré par le gain Kd (l'« amortissement »).
# Valeurs copiées de l'exemple officiel, où Kp et Kd sont indexés de 0 à
# 28, et restreintes ici aux 23 index réellement utilisés par la
# variante 23dof. Ce sont des valeurs de départ sûres, pas des valeurs
# optimisées : un intégrateur doit les réajuster pour son robot précis.
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


def _ensure_low_level_mode(timeout_s: float = 5.0, max_attempts: int = 10) -> bool:
    """
    Désactive le mode sport intégré du G1 avant tout contrôle bas niveau
    (voir « Points de sécurité obligatoires » en tête de module).

    Reprend la méthode de l'exemple officiel Unitree : appeler CheckMode
    puis ReleaseMode en boucle jusqu'à confirmation. Un audit de sécurité
    y a apporté deux corrections :
      - Le code de retour `status` de CheckMode et de ReleaseMode est
        désormais vérifié. Un `status` d'erreur (robot injoignable,
        service indisponible) ne doit jamais être pris pour « mode sport
        déjà désactivé ». L'exemple officiel l'ignore ; AXIS-OS ne peut
        pas se permettre cette hypothèse optimiste juste avant de
        prendre le contrôle direct des moteurs.
      - La boucle est bornée par `max_attempts` (10 tentatives par
        défaut, une seconde d'attente entre deux). Une boucle infinie
        bloquerait `initialize()` indéfiniment, en silence, sans rien
        rendre à l'appelant.

    Retourne True si le mode sport est confirmé désactivé, False sinon :
    délai dépassé, erreur de communication, statut d'erreur non nul, ou
    mode sport toujours actif au bout des `max_attempts` tentatives.
    L'appelant doit refuser de continuer sur un False : voir son usage
    dans G1DDSHub.initialize().

    unitree_sdk2py est importé ici, dans la fonction, et non en tête de
    module : le driver reste ainsi importable et lisible sur une machine
    où le SDK n'est pas installé (par exemple la CI, l'intégration
    continue qui lance les tests, ou un poste de développement sans
    robot). Le coût de l'import n'est payé qu'au moment où le driver est
    réellement instancié.
    """
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
        MotionSwitcherClient,
    )

    msc = MotionSwitcherClient()
    msc.SetTimeout(timeout_s)
    msc.Init()

    for _ in range(max_attempts):
        status, result = msc.CheckMode()
        if status != 0:
            # Un statut d'erreur ne dit rien de l'état réel du mode
            # sport : ne jamais le prendre pour « déjà désactivé ».
            return False
        if not result.get("name"):
            return True
        release_status, _ = msc.ReleaseMode()
        if release_status != 0:
            return False
        time.sleep(1)

    return False


class G1DDSHub:
    """
    Point de communication DDS unique, partagé par tous les
    G1DDSActuator d'un même robot : un seul publisher (émetteur de
    messages) et un seul subscriber (abonné qui les reçoit) pour les 23
    joints, et non un par joint.

    Pourquoi : le protocole réel du G1 fonctionne par message unique
    multi-joints, pas par joint isolé. LowCmd_ et LowState_ transportent
    déjà tous les moteurs dans un seul message (tableaux MotorCmd_[35] et
    MotorState_[35], voir en tête de module).

    Le hub est donc créé une seule fois par UnitreeG1HAL en mode réel
    (backend="dds", voir robot.py), jamais par un G1DDSActuator, puis
    transmis à chaque actionneur à sa construction. Cette organisation
    diffère de celle de G1SimulatedActuator, qui n'a pas besoin d'état
    partagé.
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
        # Verrou dont un audit de sécurité a montré la nécessité. Ce hub
        # est partagé par les 23 G1DDSActuator d'un même robot (voir la
        # docstring de la classe), avec en pratique au moins trois
        # écrivains concurrents possibles : le thread de mission, qui
        # remplit le message joint par joint ; le watchdog (le chien de
        # garde qui surveille le système), qui peut déclencher un arrêt
        # d'urgence sur les 23 joints à la fois ; et le callback DDS
        # _on_low_state(), appelé à chaque message d'état reçu.
        # Sans verrou, publish() pourrait calculer un CRC valide sur un
        # message dont une moitié porte une commande de mouvement et
        # l'autre un arrêt d'urgence intercalé : un message parfaitement
        # valide au sens du CRC, mais incohérent au sens de la sécurité.
        # RLock est un verrou réentrant : le thread qui le détient peut
        # le reprendre sans se bloquer lui-même.
        self._lock = threading.RLock()

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

        # Le mode sport intégré doit être confirmé désactivé avant toute
        # autre étape. Un audit de sécurité a montré que la version
        # précédente ignorait le code de retour de cette vérification et
        # pouvait continuer malgré un doute. Mieux vaut refuser
        # d'initialiser que risquer un conflit de commandes directement
        # sur les moteurs (voir « Points de sécurité obligatoires » en
        # tête de module).
        if not _ensure_low_level_mode():
            return HALStatus.ERROR_COMM

        self._low_cmd = unitree_hg_msg_dds__LowCmd_()
        self._crc = CRC()

        # Canaux DDS (« topics ») : rt/lowcmd pour envoyer les
        # commandes, rt/lowstate pour recevoir l'état du robot, chaque
        # LowState_ reçu étant transmis à _on_low_state().
        self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self._publisher.Init()

        self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self._subscriber.Init(self._on_low_state, 10)

        # Attend le premier LowState_ pour connaître la vraie valeur de
        # mode_machine, qui ne doit jamais être devinée (voir en tête de
        # module). Abandon si rien n'arrive en 5 secondes.
        waited_s = 0.0
        while self._mode_machine is None and waited_s < 5.0:
            time.sleep(0.1)
            waited_s += 0.1
        if self._mode_machine is None:
            return HALStatus.ERROR_TIMEOUT

        self._initialized = True
        return HALStatus.OK

    def _on_low_state(self, msg) -> None:
        """Callback DDS : mémorise le dernier état reçu du robot et, la
        première fois, la valeur de mode_machine qu'il contient."""
        with self._lock:
            self._low_state = msg
            if self._mode_machine is None:
                self._mode_machine = msg.mode_machine

    def send_motor_command(
        self, motor_index: int, mode: int, q: float, dq: float,
        tau: float, kp: float, kd: float,
    ) -> None:
        """Écrit la commande d'un moteur dans le message LowCmd_ partagé,
        sans l'envoyer : l'envoi se fait dans publish().

        Pourquoi : le protocole réel du G1 transporte tous les moteurs
        dans un seul message. Plusieurs G1DDSActuator peuvent ainsi
        accumuler leurs commandes respectives dans le même message avant
        un envoi groupé. Protégé par le verrou (voir __init__)."""
        with self._lock:
            cmd = self._low_cmd.motor_cmd[motor_index]
            cmd.mode = mode
            cmd.q = q
            cmd.dq = dq
            cmd.tau = tau
            cmd.kp = kp
            cmd.kd = kd

    def publish(self, mode_pr: int = 0) -> None:
        """Calcule le CRC (obligatoire, voir en tête de module) puis
        envoie sur le réseau DDS le message LowCmd_ accumulé.

        Le verrou couvre à la fois le calcul du CRC et l'envoi sur le
        réseau : aucune écriture concurrente de send_motor_command() ne
        peut ainsi se glisser entre le calcul du CRC et l'envoi effectif
        du message qu'il est censé valider."""
        with self._lock:
            self._low_cmd.mode_pr = mode_pr
            self._low_cmd.mode_machine = self._mode_machine
            self._low_cmd.crc = self._crc.Crc(self._low_cmd)
            self._publisher.Write(self._low_cmd)

    def read_motor_state(self, motor_index: int):
        """Retourne le MotorState_ courant (l'état mesuré) de ce moteur,
        ou None si aucun LowState_ n'a encore été reçu."""
        with self._lock:
            if self._low_state is None:
                return None
            return self._low_state.motor_state[motor_index]

    def read_imu_state(self):
        """Retourne l'état de l'IMU (centrale inertielle : le capteur
        d'orientation et d'accélération du robot), ou None si aucun
        LowState_ n'a encore été reçu."""
        with self._lock:
            if self._low_state is None:
                return None
            return self._low_state.imu_state

    @property
    def is_initialized(self) -> bool:
        return self._initialized


class G1DDSActuator(IActuator):
    """
    IActuator réel pour un seul joint du G1 physique, via le G1DDSHub
    partagé.

    Attention : jamais testé sur matériel réel, voir l'avertissement en
    tête de module. Respecte exactement le contrat IActuator
    (interfaces.py), déjà validé par G1SimulatedActuator et par toute la
    suite de tests existante. Aucune application ne devrait donc avoir à
    être modifiée pour utiliser ce driver à la place du simulé : seule la
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
        self._kp = _DEFAULT_KP.get(self._motor_index, 40.0)
        self._kd = _DEFAULT_KD.get(self._motor_index, 1.0)

        # Borne sur l'écart entre la position commandée et la position
        # actuelle, ajoutée à la suite d'un audit de sécurité externe
        # (voir docs/audit_externe_2026-09-14.md et CHANGELOG.md). Sans
        # elle, une consigne pourtant comprise dans les limites de
        # l'articulation, mais loin de la position actuelle, faisait
        # produire au régulateur PD du moteur un couple bien au-delà de
        # torque_max, le couple maximal autorisé pour l'articulation.
        # La borne vaut torque_max / kp ; la justification
        # physique complète est dans JointLimits.position_delta_max
        # (axis_os/hal/core/types.py). Elle est vérifiée par
        # JointSafetyGuard, le garde-fou de sécurité de chaque
        # articulation (axis_os/hal/safety/limits.py).
        # Calculée ici, car kp est propre à ce driver et non à
        # JointLimits, puis injectée par dataclasses.replace() plutôt
        # qu'en modifiant definition.limits, qui peut être partagé.
        limits_with_delta = replace(
            definition.limits,
            position_delta_max=definition.limits.torque_max / self._kp,
            # current_is_measured garde sa valeur par défaut, False :
            # feedback.current vaut toujours 0.0 sur ce driver (voir
            # get_feedback() ci-dessous). Un second audit externe a
            # relevé que surveiller la surintensité (CURRENT_OVERLOAD)
            # sur une valeur toujours nulle rassurerait sans jamais
            # protéger ; contexte complet dans
            # JointLimits.current_is_measured.
        )
        self._guard = JointSafetyGuard(definition.joint_id, limits_with_delta)

    def initialize(self) -> HALStatus:
        if not self._hub.is_initialized:
            return HALStatus.ERROR_NOT_INIT
        self._state = ActuatorState.IDLE
        return HALStatus.OK

    def shutdown(self) -> HALStatus:
        self._state = ActuatorState.UNINITIALIZED
        return HALStatus.OK

    def calibrate(self) -> HALStatus:
        # Le G1 ne propose pas de calibration articulation par
        # articulation dans ce protocole bas niveau : elle se fait dans
        # le firmware du robot, hors de portée d'AXIS-OS. D'où cette
        # fonction volontairement sans effet, et documentée comme telle,
        # plutôt qu'une fausse implémentation.
        return HALStatus.OK

    def set_control_mode(self, mode: ControlMode) -> HALStatus:
        self.definition.default_control_mode = mode
        return HALStatus.OK

    def send_command(self, command: ActuatorCommand, defer_publish: bool = False) -> HALStatus:
        """
        Valide la commande de ce joint, l'écrit dans le message LowCmd_
        partagé et l'envoie, sauf si `defer_publish` est vrai.

        `defer_publish` : si True, la commande est écrite dans le message
        partagé sans appel à hub.publish(). L'appelant doit alors publier
        lui-même, une seule fois, après avoir rempli les 23 joints (voir
        UnitreeG1HAL.send_joint_commands()). Vaut False par défaut pour
        qu'un appel isolé à send_command(), hors du chemin normal qui
        passe par send_joint_commands (un test ou un script de mise au
        point, par exemple), reste immédiatement effectif, comme avant.

        Pourquoi ce paramètre : un premier audit de sécurité avait
        relevé qu'aucune commande n'était jamais envoyée sur le réseau
        DDS, faute d'appel à publish() (voir CHANGELOG.md). Le premier
        correctif publiait après chaque joint ; un second audit de
        sécurité a montré que cela produisait 22 messages LowCmd_
        intermédiaires par cycle de commande du corps entier, chacun avec
        une partie des articulations déjà sur la nouvelle consigne et
        l'autre encore sur l'ancienne. Or LowCmd_ transporte les 35
        moteurs dans un seul message (voir sa structure officielle en
        tête de module), et le contrôleur du G1, plus rapide que la
        boucle de commande logicielle, peut figer l'une de ces poses
        incohérentes avec un gain kp=100 : sur un bipède en appui, cela
        crée un déséquilibre passager à chaque cycle de commande complet.

        Retourne ERROR_FAULT si le joint est en arrêt d'urgence ou si une
        surcharge est détectée (l'arrêt d'urgence du joint est alors
        déclenché), ERROR_LIMIT si la commande viole une limite de
        sécurité, OK sinon.
        """
        if self._state == ActuatorState.EMERGENCY_STOP:
            return HALStatus.ERROR_FAULT

        feedback = self.get_feedback()

        # Surveillance de la santé du moteur avant chaque commande. Un
        # audit de sécurité a constaté que check_feedback_health(), qui
        # détecte notamment une surcharge de couple (signe qu'un moteur
        # force contre un obstacle), n'était appelée nulle part en
        # production. Même correctif que dans robot.py (voir
        # G1SimulatedActuator.send_command pour le contexte complet),
        # appliqué ici au matériel réel, où une surcharge de couple non
        # détectée a des conséquences physiques réelles, et pas
        # seulement simulées.
        if feedback is not None:
            health_violations = self._guard.check_feedback_health(feedback)
            for v in health_violations:
                if v.kind in (SafetyErrorKind.TORQUE_OVERLOAD,
                              SafetyErrorKind.CURRENT_OVERLOAD):
                    print(f"  [SAFETY] Surcharge détectée, arrêt d'urgence : {v}")
                    self.emergency_stop()
                    return HALStatus.ERROR_FAULT

        ok, violation = self._guard.validate_command(command, feedback)
        if not ok:
            print(f"  [SAFETY] {violation}")
            return HALStatus.ERROR_LIMIT

        # q (la position de consigne) ne doit JAMAIS être transmis dans
        # un mode où il n'a pas de sens physique. Un audit de sécurité,
        # en exécutant réellement le code, a montré le problème : à
        # l'époque, JointSafetyGuard ne validait command.position qu'en
        # mode POSITION, alors que q était transmis au moteur quel que
        # soit le mode. Une commande en mode TORQUE ou VELOCITY dont le
        # champ position gardait une valeur aberrante (objet
        # ActuatorCommand réutilisé, sortie brute d'un modèle VLA,
        # c'est-à-dire un modèle d'IA vision-langage-action qui produit
        # directement des commandes) atteignait le moteur sans jamais
        # être vérifiée. Avec kp≈100, une erreur de position de
        # plusieurs centaines de radians sature instantanément le couple
        # du régulateur PD embarqué dans le moteur.
        # Désormais, JointSafetyGuard vérifie la position dès qu'elle est
        # renseignée, quel que soit le mode (voir
        # axis_os/hal/safety/limits.py), et q n'est transmis qu'en mode
        # POSITION ou IMPEDANCE, les seuls où il a un sens.
        send_position = command.mode in (ControlMode.POSITION, ControlMode.IMPEDANCE)

        self._hub.send_motor_command(
            motor_index=self._motor_index,
            mode=1,
            q=command.position if (send_position and command.position is not None) else 0.0,
            dq=command.velocity if command.velocity is not None else 0.0,
            tau=command.torque if command.torque is not None else 0.0,
            kp=self._kp,
            kd=self._kd,
        )
        if not defer_publish:
            self._hub.publish()
        self._state = ActuatorState.ACTIVE
        return HALStatus.OK

    def emergency_stop(self) -> HALStatus:
        # Arrêt d'urgence (e-stop) de ce moteur.
        # kp = kd = 0 et tau = 0 (tau est le couple de consigne) coupent
        # le couple actif de ce moteur-ci, tout en gardant mode=1 (moteur
        # activé, « Enable ») : le moteur cesse de pousser mais reste
        # actif du point de vue du firmware.
        # mode=0 pourrait au contraire le laisser retomber en roue libre
        # de façon incontrôlée, selon la charge qu'il porte à cet instant
        # précis (un bras levé, par exemple).
        #
        # publish() est appelé immédiatement et sans condition ; un audit
        # de sécurité a jugé ce point critique. send_motor_command()
        # n'écrit que dans un message local partagé (voir sa docstring)
        # et ne transmet rien sur le réseau DDS tant que publish() n'est
        # pas appelé. L'arrêt d'urgence est le SEUL chemin du système qui
        # ne doit jamais dépendre d'un envoi groupé ultérieur.
        # send_command() (ci-dessus) publie aussi par défaut, mais un
        # e-stop ne doit pas attendre le prochain appel normal pour
        # partir.
        self._hub.send_motor_command(
            motor_index=self._motor_index,
            mode=1, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0,
        )
        self._hub.publish()
        self._state = ActuatorState.EMERGENCY_STOP
        return HALStatus.OK

    def get_feedback(self) -> Optional[ActuatorFeedback]:
        """Dernier état mesuré du moteur, converti en ActuatorFeedback,
        ou None si le robot n'a encore envoyé aucun LowState_."""
        motor_state = self._hub.read_motor_state(self._motor_index)
        if motor_state is None:
            return None
        return ActuatorFeedback(
            joint_id=self.joint_id,
            state=self._state,
            position=motor_state.q,
            velocity=motor_state.dq,
            torque=motor_state.tau_est,
            current=0.0,  # absent de MotorState_ (voir sa définition)
            # Selon la structure officielle MotorState_, temperature[0]
            # vient du capteur du moteur et temperature[1] de celui du
            # driver (la carte électronique qui l'alimente). On remonte
            # celle du moteur, plus directement utile à JointSafetyGuard,
            # dont les seuils visent une température de bobinage et non
            # de carte électronique.
            temperature=float(motor_state.temperature[0]),
            voltage=motor_state.vol,
        )
