"""
AXIS-OS — Tests G1DDSActuator
=================================
Valide le driver du vrai matériel G1 (axis_os/hal/drivers/unitree_g1/
g1_dds_actuator.py), écrit en suivant le code source officiel de
unitree_sdk2py (structures LowCmd_/LowState_, exemple officiel
g1_low_level_example.py) sans jamais avoir été exécuté contre un
vrai robot — voir l'avertissement en tête de ce module source.

Ces tests utilisent un G1DDSHub MOCKÉ (unittest.mock) : ils valident
la logique côté AXIS-OS (mapping joint->index moteur, garde de
sécurité, construction correcte des commandes DDS) SANS nécessiter
le SDK unitree_sdk2py ni un vrai robot — ce ne sont PAS des tests
d'intégration matérielle, qui restent à faire sur du vrai G1 avant
tout déploiement client.
"""

import sys
from unittest.mock import MagicMock
sys.path.insert(0, ".")

from axis_os.hal.drivers.unitree_g1.g1_dds_actuator import (
    G1_JOINT_ID_TO_MOTOR_INDEX, G1DDSActuator,
)
from axis_os.hal.core.types import (
    JointDefinition, JointLimits, JointType, ActuatorCommand,
    ActuatorState, ControlMode, HALStatus,
)
from axis_os.hal.core.urdf_loader import load_urdf

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def make_actuator(joint_id="right_knee_joint", pos_min=-0.087, pos_max=2.880,
                  torque_max=139.0):
    definition = JointDefinition(
        joint_id=joint_id, joint_type=JointType.REVOLUTE,
        limits=JointLimits(
            pos_min=pos_min, pos_max=pos_max, vel_max=20.0,
            torque_max=torque_max, current_max=1000.0, temperature_max=80.0,
        ),
        parent_link="p", child_link="c",
        default_control_mode=ControlMode.POSITION,
    )
    hub = MagicMock()
    hub.is_initialized = True
    hub.read_motor_state.return_value = None
    return G1DDSActuator(definition, hub), hub


# ══════════════════════════════════════════════════════════════════

section("1 — Mapping joint_id -> index moteur : couverture exacte des 23 joints réels")

model = load_urdf("axis_os/hal/drivers/unitree_g1/urdf/g1_23dof_rev_1_0.urdf")
real_joint_ids = {j.joint_id for j in model.get_joints()}

check("23 joints mappés au total", len(G1_JOINT_ID_TO_MOTOR_INDEX) == 23)
check("Aucun joint réel (urdf_loader) sans mapping DDS",
      real_joint_ids - set(G1_JOINT_ID_TO_MOTOR_INDEX.keys()) == set())
check("Aucun joint mappé qui n'existe pas réellement dans l'URDF",
      set(G1_JOINT_ID_TO_MOTOR_INDEX.keys()) - real_joint_ids == set())

indices = list(G1_JOINT_ID_TO_MOTOR_INDEX.values())
check("CRITIQUE : aucun index moteur utilisé deux fois (enverrait au mauvais moteur)",
      len(indices) == len(set(indices)))

check("right_knee_joint -> index 9 (confirmé contre l'exemple officiel Unitree)",
      G1_JOINT_ID_TO_MOTOR_INDEX["right_knee_joint"] == 9)
check("waist_yaw_joint -> index 12", G1_JOINT_ID_TO_MOTOR_INDEX["waist_yaw_joint"] == 12)
check("left_elbow_joint -> index 18", G1_JOINT_ID_TO_MOTOR_INDEX["left_elbow_joint"] == 18)

# ══════════════════════════════════════════════════════════════════

section("2 — Construction : un joint_id sans mapping échoue explicitement")

bad_definition = JointDefinition(
    joint_id="joint_qui_nexiste_pas", joint_type=JointType.REVOLUTE,
    limits=JointLimits(pos_min=-1.0, pos_max=1.0, vel_max=1.0,
                       torque_max=1.0, current_max=1.0, temperature_max=80.0),
    parent_link="p", child_link="c", default_control_mode=ControlMode.POSITION,
)
try:
    G1DDSActuator(bad_definition, MagicMock())
    check("joint_id inconnu lève ValueError à la construction", False)
except ValueError:
    check("joint_id inconnu lève ValueError à la construction", True)

# ══════════════════════════════════════════════════════════════════

section("3 — Commande valide : atteint bien le hub avec les bons paramètres")

actuator, hub = make_actuator()
result = actuator.send_command(ActuatorCommand(mode=ControlMode.POSITION, position=1.0))
check("Commande dans les limites -> OK", result == HALStatus.OK)
check("send_motor_command() du hub a bien été appelé", hub.send_motor_command.called)
# Trouvé manquant par un second audit externe (2026-09-14) : aucun test
# n'assertait publish(), le correctif le plus critique de la série
# (C2 — sans lui, aucune commande n'atteint jamais le réseau DDS réel,
# y compris l'arrêt d'urgence) n'était gardé par rien.
check("publish() appelé par défaut (defer_publish=False)", hub.publish.called)

call_kwargs = hub.send_motor_command.call_args.kwargs
check("motor_index correct transmis (9 pour right_knee_joint)",
      call_kwargs["motor_index"] == 9)
check("Position (q) correctement transmise", call_kwargs["q"] == 1.0)
check("mode=1 (Enable) transmis, jamais 0", call_kwargs["mode"] == 1)

# ══════════════════════════════════════════════════════════════════

section("3bis — defer_publish=True : écrit dans le buffer sans publier")

actuator_defer, hub_defer = make_actuator()
result_defer = actuator_defer.send_command(
    ActuatorCommand(mode=ControlMode.POSITION, position=1.0), defer_publish=True)
check("Commande avec defer_publish=True -> OK", result_defer == HALStatus.OK)
check("send_motor_command() appelé même avec defer_publish=True",
      hub_defer.send_motor_command.called)
check("publish() JAMAIS appelé quand defer_publish=True",
      not hub_defer.publish.called)

# ══════════════════════════════════════════════════════════════════

section("4 — CRITIQUE : une commande dangereuse n'atteint JAMAIS le hub DDS")

actuator2, hub2 = make_actuator()
result_bad = actuator2.send_command(
    ActuatorCommand(mode=ControlMode.POSITION, position=5.0)  # hors limite (max=2.88)
)
check("Commande hors limites -> ERROR_LIMIT", result_bad == HALStatus.ERROR_LIMIT)
check("CRITIQUE : send_motor_command() JAMAIS appelé pour une commande dangereuse",
      not hub2.send_motor_command.called)
check("CRITIQUE : publish() non plus JAMAIS appelé pour une commande dangereuse",
      not hub2.publish.called)

actuator3, hub3 = make_actuator()
result_torque = actuator3.send_command(
    ActuatorCommand(mode=ControlMode.TORQUE, torque=500.0)  # hors limite (max=139)
)
check("Couple hors limite -> ERROR_LIMIT", result_torque == HALStatus.ERROR_LIMIT)
check("CRITIQUE : couple dangereux n'atteint jamais le hub",
      not hub3.send_motor_command.called)

# ══════════════════════════════════════════════════════════════════

section("5 — Arrêt d'urgence : coupe le couple actif (kp=kd=0), pas mode=0")

actuator4, hub4 = make_actuator()
result_estop = actuator4.emergency_stop()
check("emergency_stop() -> OK", result_estop == HALStatus.OK)
check("État passe bien à EMERGENCY_STOP", actuator4.state == ActuatorState.EMERGENCY_STOP)
check("CRITIQUE : publish() appelé immédiatement et inconditionnellement par emergency_stop()",
      hub4.publish.called)

estop_kwargs = hub4.send_motor_command.call_args.kwargs
check("kp=0.0 lors de l'e-stop (coupe le couple actif)", estop_kwargs["kp"] == 0.0)
check("kd=0.0 lors de l'e-stop", estop_kwargs["kd"] == 0.0)
check("mode reste 1 (Enable) même en e-stop, jamais 0 (évite la roue libre incontrôlée)",
      estop_kwargs["mode"] == 1)

# ══════════════════════════════════════════════════════════════════

section("6 — Après un e-stop, toute nouvelle commande est bloquée")

hub4.reset_mock()
result_after = actuator4.send_command(
    ActuatorCommand(mode=ControlMode.POSITION, position=0.5)
)
check("Commande après e-stop -> ERROR_FAULT", result_after == HALStatus.ERROR_FAULT)
check("Le hub n'est JAMAIS sollicité après un e-stop tant qu'aucun reset explicite",
      not hub4.send_motor_command.called)

# ══════════════════════════════════════════════════════════════════

section("7 — get_feedback() : None tant qu'aucun LowState_ n'a été reçu")

actuator5, hub5 = make_actuator()
hub5.read_motor_state.return_value = None
check("Feedback None si le hub n'a pas encore reçu de LowState_",
      actuator5.get_feedback() is None)

# ══════════════════════════════════════════════════════════════════

section("8 — CRITIQUE : un cycle corps entier publie UNE SEULE FOIS, pas 23")

# Trouvé manquant par un second audit externe : le correctif initial de
# C2 (publish() ajouté partout où une commande atteint le hub) avait
# lui-même introduit un nouveau problème — publier après CHAQUE joint
# produit 22 messages LowCmd_ intermédiaires incohérents sur un cycle
# de 23 joints, alors que LowCmd_ transporte les 35 moteurs dans UN
# SEUL message. Ce test valide directement UnitreeG1HAL.
# send_joint_commands() (pas G1DDSActuator seul) avec un hub mocké au
# niveau du module, pour vérifier le VRAI nombre d'appels à publish()
# sur un cycle réaliste.

import unittest.mock as _mock
import axis_os.hal.drivers.unitree_g1.g1_dds_actuator as _g1_mod
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.core.types import HALStatus as _HALStatus


class _CountingMockHub:
    def __init__(self, *a, **k):
        self.publish_calls = 0
        self.send_calls = 0
        self.is_initialized = True

    def initialize(self):
        return _HALStatus.OK

    def send_motor_command(self, **kwargs):
        self.send_calls += 1

    def publish(self, mode_pr=0):
        self.publish_calls += 1

    def read_motor_state(self, i):
        return None


_original_hub_cls = _g1_mod.G1DDSHub
_mock_hub_instance = _CountingMockHub()
_g1_mod.G1DDSHub = lambda *a, **k: _mock_hub_instance

try:
    hal_fullbody = UnitreeG1HAL("g1_wave2_fix", backend="dds", network_interface="eth0")
    hal_fullbody.initialize_all()

    fullbody_commands = {
        jid: ActuatorCommand(mode=ControlMode.POSITION, position=0.1)
        for jid in hal_fullbody._actuators.keys()
    }
    hal_fullbody.send_joint_commands(fullbody_commands)

    check("23 joints commandés -> 23 appels à send_motor_command",
          _mock_hub_instance.send_calls == 23)
    check("CRITIQUE : 23 joints commandés -> UN SEUL appel à publish() (pas 23)",
          _mock_hub_instance.publish_calls == 1)
finally:
    _g1_mod.G1DDSHub = _original_hub_cls

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ Mapping des 23 joints réels vers les bons index moteur DDS")
    print("  ✓ Aucun doublon d'index (erreur qui enverrait au mauvais moteur)")
    print("  ✓ Commande dangereuse : JAMAIS transmise au hub DDS réel")
    print("  ✓ Arrêt d'urgence coupe le couple actif sans risque de roue libre")
    print("  ✓ publish() explicitement vérifié à chaque point critique")
    print("  ✓ Un cycle corps entier publie UNE SEULE FOIS, pas par joint")
    print()
    print("  ⚠️  Rappel : ces tests valident la LOGIQUE (hub mocké), pas")
    print("     l'intégration matérielle réelle — à valider sur un vrai G1")
    print("     avant tout déploiement client.")

sys.exit(0 if FAIL == 0 else 1)
