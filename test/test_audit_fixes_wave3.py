"""
AXIS-OS — Tests audit externe, vague 3 (M1, M4, M5, M6, M7, M9, M12)
========================================================================
Complète test_audit_fixes_wave2.py avec le reste de la liste MOYEN
priorité du même audit externe (Claude Opus, 2026-09-14). M2, M3, M8,
M10, M11 déjà couverts ailleurs (voir CHANGELOG.md pour le détail
complet par point).
"""

import sys
import math
import os
sys.path.insert(0, ".")

from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.core.types import ActuatorCommand, ControlMode, ActuatorState
from axis_os.hal.safety.limits import JointSafetyGuard, SafetyErrorKind
from axis_os.hal.core.types import JointLimits
from axis_os.store.sdk.manifest import Permission
from axis_os.store.runtime.hal_bridge import HALBridgeClient, HALBridgeServer

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


# ══════════════════════════════════════════════════════════════════

section("M1 — permissions non appliquées documentées explicitement")

names = [p.name for p in Permission]
check("Les 13 permissions existent toujours (enum rétrocompatible)",
      len(names) == 13)
check("NETWORK_CLOUD et NETWORK_LOCAL toujours présentes",
      "NETWORK_CLOUD" in names and "NETWORK_LOCAL" in names)
check("SENSORS_FORCE_TORQUE toujours présente",
      "SENSORS_FORCE_TORQUE" in names)
check("La docstring de la classe documente explicitement les permissions non appliquées",
      "AUCUNE méthode de RobotContext ne" in (Permission.__doc__ or ""))
check("Chaque permission non appliquée porte un commentaire 'NON APPLIQUÉE' dans le code source",
      "NON APPLIQUÉE" in open("axis_os/store/sdk/manifest.py", encoding="utf-8").read())

# ══════════════════════════════════════════════════════════════════

section("M4 — le courant mesuré (feedback) est vérifié, pas seulement command.current")

hal_m4 = UnitreeG1HAL("g1_m1_m4_w3")
hal_m4.initialize_all()
act_m4 = hal_m4.get_actuator("left_knee_joint")
check("current_max par défaut == 15.0 A", act_m4._guard.limits.current_max == 15.0)

act_m4._current = 20.0  # > 15 * 1.1 = 16.5
result_m4 = act_m4.send_command(ActuatorCommand(mode=ControlMode.POSITION, position=0.5))
check("Commande avec current mesuré en surcharge → ERROR_FAULT",
      result_m4.name == "ERROR_FAULT")
check("État actuateur → EMERGENCY_STOP", act_m4.state == ActuatorState.EMERGENCY_STOP)

# check_feedback_health() seul, sans passer par send_command
from axis_os.hal.core.types import ActuatorFeedback
guard_m4 = JointSafetyGuard("test", JointLimits(
    pos_min=-1, pos_max=1, vel_max=10, torque_max=50,
    current_max=15.0, temperature_max=80.0,
    # current_is_measured=True requis explicitement depuis le correctif
    # du point 3 (second audit externe) : CURRENT_OVERLOAD ne se
    # déclenche plus que si la source du courant est documentée comme
    # une vraie mesure — voir JointLimits.current_is_measured.
    current_is_measured=True,
))
fb_over = ActuatorFeedback(
    joint_id="test", state=ActuatorState.ACTIVE, position=0.0, velocity=0.0,
    torque=0.0, current=20.0, temperature=25.0, voltage=48.0,
)
violations_m4 = guard_m4.check_feedback_health(fb_over)
check("check_feedback_health() détecte CURRENT_OVERLOAD isolément (current_is_measured=True)",
      any(v.kind == SafetyErrorKind.CURRENT_OVERLOAD for v in violations_m4))

# Non-régression : sans current_is_measured=True (défaut False), le
# check ne se déclenche JAMAIS, même avec un courant en surcharge —
# c'est précisément le comportement voulu pour G1DDSActuator, où
# feedback.current vaut toujours 0.0 et ne doit jamais faussement
# rassurer.
guard_m4_unmeasured = JointSafetyGuard("test2", JointLimits(
    pos_min=-1, pos_max=1, vel_max=10, torque_max=50,
    current_max=15.0, temperature_max=80.0,
))  # current_is_measured=False par défaut
violations_m4_unmeasured = guard_m4_unmeasured.check_feedback_health(fb_over)
check("Sans current_is_measured=True, CURRENT_OVERLOAD ne se déclenche jamais (comportement voulu pour DDS réel)",
      not any(v.kind == SafetyErrorKind.CURRENT_OVERLOAD for v in violations_m4_unmeasured))

# ══════════════════════════════════════════════════════════════════

section("M5 — le backend Webots ne renvoie plus une température constante en mode production")

webots_path = "axis_os/hal/drivers/webots/backend.py"
with open(webots_path, "r", encoding="utf-8") as f:
    webots_source = f.read()
check("Le mode production calcule désormais temperature = STUB_TEMP + f(torque), pas une constante",
      "temperature = self.STUB_TEMP + abs(torque)" in webots_source)

# ══════════════════════════════════════════════════════════════════

section("M6 — le backend Webots ne masque plus silencieusement un joint non lié")

check("connect() journalise désormais chaque joint qui échoue à se lier",
      "liaison du joint" in webots_source)
check("Aucune occurrence de 'except Exception: pass' silencieuse dans connect()",
      "except Exception: pass" not in webots_source.split("def disconnect")[0])

# Import + fallback stub toujours fonctionnel (non-régression) —
# le module 'controller' (Webots) n'est jamais disponible dans cet
# environnement de test, donc ce chemin est le seul réellement exercé.
from axis_os.hal.drivers.webots.backend import WebotsBackend
backend_m6 = WebotsBackend()
result_m6 = backend_m6.connect()
check("Non-régression : fallback stub (sans module controller) fonctionne toujours",
      result_m6 is True and backend_m6._stub_mode is True)

# ══════════════════════════════════════════════════════════════════

section("M7 — stiffness/damping validés et transportés par le pont HAL")

guard_m7 = JointSafetyGuard("test", JointLimits(
    pos_min=-1, pos_max=1, vel_max=10, torque_max=50,
    current_max=15.0, temperature_max=80.0,
))

valid_nan, _ = guard_m7.validate_command(
    ActuatorCommand(mode=ControlMode.IMPEDANCE, position=0.0, stiffness=float("nan")))
check("stiffness=NaN rejeté", not valid_nan)

valid_neg, _ = guard_m7.validate_command(
    ActuatorCommand(mode=ControlMode.IMPEDANCE, position=0.0, damping=-5.0))
check("damping négatif rejeté", not valid_neg)

valid_ok, _ = guard_m7.validate_command(
    ActuatorCommand(mode=ControlMode.IMPEDANCE, position=0.0, stiffness=80.0, damping=3.0))
check("stiffness/damping valides acceptés", valid_ok)

cmd_m7 = {"left_elbow_joint": ActuatorCommand(
    mode=ControlMode.IMPEDANCE, position=0.5, stiffness=80.0, damping=3.0)}
serialized_m7 = HALBridgeClient._serialize(cmd_m7)
check("stiffness transporté par _serialize()",
      serialized_m7["left_elbow_joint"]["stiffness"] == 80.0)
check("damping transporté par _serialize()",
      serialized_m7["left_elbow_joint"]["damping"] == 3.0)
check("current transporté par _serialize() (même trou que stiffness/damping)",
      "current" in serialized_m7["left_elbow_joint"])

deserialized_m7 = HALBridgeServer._deserialize_commands(serialized_m7)
check("stiffness/damping restitués par _deserialize_commands()",
      deserialized_m7["left_elbow_joint"].stiffness == 80.0
      and deserialized_m7["left_elbow_joint"].damping == 3.0)

# ══════════════════════════════════════════════════════════════════

section("M9 — l'e-stop non-réarmable est documenté comme choix de sécurité")

watchdog_path = "axis_os/hal/watchdog/system.py"
with open(watchdog_path, "r", encoding="utf-8") as f:
    watchdog_source = f.read()
check("La propriété estop_fired documente explicitement l'absence de réarmement automatique",
      "JAMAIS remis à False" in watchdog_source or "jamais remis à False" in watchdog_source.lower())

# ══════════════════════════════════════════════════════════════════

section("M12 — fichier requirments.txt (faute de frappe) supprimé")

check("requirments.txt n'existe plus", not os.path.exists("requirments.txt"))
check("requirements.txt (orthographe correcte) existe toujours",
      os.path.exists("requirements.txt"))

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

sys.exit(0 if FAIL == 0 else 1)
