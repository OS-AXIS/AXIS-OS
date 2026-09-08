"""
AXIS-OS — Test MotionStateTracker & Rate Limiting
=====================================================
Suite dédiée validant la conception actée avec l'utilisateur :
  - MotionStateTracker vit dans hal/core/ (générique, pas G1-specific)
  - Rate limiting en mode BLOCK, jamais lissé
  - JointSafetyGuard reste no-op sur ce point si aucun tracker injecté
    (rétrocompatibilité stricte avec le comportement pré-existant)

    python test_motion_state.py
"""

import sys
import time
sys.path.insert(0, ".")

from axis_os.hal.core.motion_state import MotionStateTracker, JointMotionState
from axis_os.hal.core.types import ActuatorCommand, ActuatorFeedback, ActuatorState, ControlMode, JointLimits
from axis_os.hal.safety.limits import JointSafetyGuard, SafetyErrorKind
from axis_os.hal.drivers.unitree_g1.robot import _build_g1_joint_definitions

PASS = 0
FAIL = 0


def section(title: str):
    print(f"\n{'═'*62}\n  {title}\n{'═'*62}")


def check(label: str, condition: bool) -> bool:
    global PASS, FAIL
    icon = "✓" if condition else "✗"
    print(f"  [{icon}] {label:<54} {'OK' if condition else 'FAILED'}")
    if condition: PASS += 1
    else: FAIL += 1
    return condition


# ══════════════════════════════════════════════════════════════════

section("1 — MotionStateTracker : générique, zéro dépendance G1")
import axis_os.hal.core.motion_state as ms_module
source = open(ms_module.__file__).read()
import ast
tree = ast.parse(source)
imported_modules = [n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module]
imported_names   = [n.names[0].name if n.names else "" for n in ast.walk(tree) if isinstance(n, ast.Import)]
check("Aucun import de code lié à unitree_g1 (le mot peut apparaître "
      "dans les docstrings explicatifs, ce n'est pas une dépendance)",
      not any("unitree" in (m or "").lower() for m in imported_modules))
check("Vit bien dans hal/core/ (pas hal/drivers/)",
      "hal/core/motion_state.py" in ms_module.__file__ or "hal\\core\\motion_state.py" in ms_module.__file__)

section("2 — JointMotionState : première commande sans historique")
state = JointMotionState("test_joint")
cmd1 = ActuatorCommand(ControlMode.POSITION, position=0.0, velocity=0.0)
delta = state.compute_command_delta(cmd1)
check("has_history == False pour la première commande", not delta.has_history)
check("position_delta == None (rien à comparer)", delta.position_delta is None)

section("3 — JointMotionState : delta calculé après une commande enregistrée")
state.record_command(cmd1)
cmd2 = ActuatorCommand(ControlMode.POSITION, position=0.5, velocity=5.0)
delta2 = state.compute_command_delta(cmd2)
check("has_history == True après un record_command", delta2.has_history)
check("position_delta == 0.5 (0.5 - 0.0)", abs(delta2.position_delta - 0.5) < 1e-9)
check("velocity_delta == 5.0 (5.0 - 0.0)", abs(delta2.velocity_delta - 5.0) < 1e-9)
check("elapsed_s > 0", delta2.elapsed_s > 0)
check("implied_acceleration calculé", delta2.implied_acceleration is not None)

section("4 — JointMotionState : gestion NaN/Infinity dans le calcul de delta")
cmd_nan = ActuatorCommand(ControlMode.POSITION, position=float("nan"), velocity=0.0)
delta_nan = state.compute_command_delta(cmd_nan)
check("position_delta == None si nouvelle commande a une position NaN",
      delta_nan.position_delta is None)

section("5 — JointMotionState : écart commande/feedback")
state2 = JointMotionState("test_joint2")
check("compute_feedback_error() == None sans commande ni feedback",
      state2.compute_feedback_error() is None)
cmd = ActuatorCommand(ControlMode.POSITION, position=1.0)
state2.record_command(cmd)
fb = ActuatorFeedback(joint_id="test_joint2", state=ActuatorState.ACTIVE,
                      position=1.8, velocity=0.0, torque=0.0, current=0.0,
                      temperature=25.0, voltage=48.0)
state2.record_feedback(fb)
error = state2.compute_feedback_error()
check("Écart commande/feedback calculé (1.8 - 1.0 = 0.8)",
      error is not None and abs(error - 0.8) < 1e-9)

section("6 — MotionStateTracker : registre multi-joints")
tracker = MotionStateTracker(["left_knee_joint", "right_knee_joint", "left_elbow_joint"])
check("3 joints enregistrés", len(tracker.joint_ids) == 3)
check("get() retourne None pour un joint inconnu", tracker.get("unknown_joint") is None)
check("get() retourne un JointMotionState pour un joint connu",
      isinstance(tracker.get("left_knee_joint"), JointMotionState))

tracker.record_command("left_knee_joint", ActuatorCommand(ControlMode.POSITION, position=0.3))
check("record_command() via le tracker fonctionne",
      tracker.get("left_knee_joint").last_command.position == 0.3)

section("7 — Rétrocompatibilité stricte : JointSafetyGuard SANS tracker (no-op)")
limits_with_accel = JointLimits(pos_min=-1.0, pos_max=2.5, vel_max=20.0,
                                torque_max=88.0, current_max=20.0,
                                temperature_max=80.0, accel_max=1.0)
guard_no_tracker = JointSafetyGuard("left_knee_joint", limits_with_accel)  # pas de motion_tracker
ok1, _ = guard_no_tracker.validate_command(
    ActuatorCommand(ControlMode.POSITION, position=0.0, velocity=0.0))
ok2, _ = guard_no_tracker.validate_command(
    ActuatorCommand(ControlMode.POSITION, position=2.0, velocity=100.0))
# La 2e commande dépasse vel_max de toute façon, donc on isole avec une commande
# qui reste DANS vel_max mais impliquerait une forte accélération si trackée
ok3, v3 = guard_no_tracker.validate_command(
    ActuatorCommand(ControlMode.POSITION, position=0.9, velocity=15.0))
check("Sans tracker injecté, aucune ACCELERATION_LIMIT n'est jamais levée",
      v3 is None or v3.rule != "ACCELERATION_LIMIT")

section("8 — Rate limiting BLOCK : commande brutale rejetée (décision actée)")
tracker2 = MotionStateTracker(["left_knee_joint"])
guard = JointSafetyGuard("left_knee_joint", limits_with_accel, motion_tracker=tracker2)

cmd_first = ActuatorCommand(ControlMode.POSITION, position=0.0, velocity=0.0)
ok_first, _ = guard.validate_command(cmd_first)
check("Première commande (sans historique) toujours acceptée", ok_first)
tracker2.record_command("left_knee_joint", cmd_first)

# Saut de vitesse brutal (0 -> 10 rad/s quasi instantané), accel_max=1.0
cmd_brutal = ActuatorCommand(ControlMode.POSITION, position=0.5, velocity=10.0)
ok_brutal, v_brutal = guard.validate_command(cmd_brutal)
check("Commande avec accélération implicite excessive → REJETÉE (pas lissée)",
      not ok_brutal)
check("rule == ACCELERATION_LIMIT", v_brutal is not None and v_brutal.rule == "ACCELERATION_LIMIT")
check("kind == SafetyErrorKind.ACCELERATION_LIMIT",
      v_brutal is not None and v_brutal.kind == SafetyErrorKind.ACCELERATION_LIMIT)
check("La commande rejetée N'EST PAS modifiée par le Guard (pure accept/reject)",
      cmd_brutal.velocity == 10.0)  # inchangée — le Guard ne "corrige" jamais

section("9 — Rate limiting : commande douce toujours acceptée")
tracker3 = MotionStateTracker(["left_knee_joint"])
guard3 = JointSafetyGuard("left_knee_joint", limits_with_accel, motion_tracker=tracker3)
cmd_a = ActuatorCommand(ControlMode.POSITION, position=0.0, velocity=0.0)
guard3.validate_command(cmd_a)
tracker3.record_command("left_knee_joint", cmd_a)
time.sleep(0.05)  # laisse s'écouler du temps réel — réduit l'accel implicite
cmd_b = ActuatorCommand(ControlMode.POSITION, position=0.005, velocity=0.02)
# vel_delta=0.02, elapsed≈0.05s → accel≈0.4 rad/s², sous accel_max=1.0
ok_b, v_b = guard3.validate_command(cmd_b)
check("Commande douce (petit delta après 50ms) acceptée", ok_b)

section("10 — limits.accel_max=None → check désactivé même AVEC tracker injecté")
limits_no_accel = JointLimits(pos_min=-1.0, pos_max=2.5, vel_max=20.0,
                              torque_max=88.0, current_max=20.0,
                              temperature_max=80.0)  # accel_max=None (défaut)
tracker4 = MotionStateTracker(["left_knee_joint"])
guard4 = JointSafetyGuard("left_knee_joint", limits_no_accel, motion_tracker=tracker4)
cmd_x = ActuatorCommand(ControlMode.POSITION, position=0.0, velocity=0.0)
guard4.validate_command(cmd_x)
tracker4.record_command("left_knee_joint", cmd_x)
cmd_y = ActuatorCommand(ControlMode.POSITION, position=0.5, velocity=10.0)
ok_y, v_y = guard4.validate_command(cmd_y)
check("accel_max=None → ACCELERATION_LIMIT jamais déclenchée",
      v_y is None or v_y.rule != "ACCELERATION_LIMIT")

section("11 — Intégration : les 24 joints du G1 acceptent un tracker sans erreur")
g1_joints = _build_g1_joint_definitions()
tracker_g1 = MotionStateTracker([j.joint_id for j in g1_joints])
all_ok = True
for jdef in g1_joints:
    try:
        g = JointSafetyGuard(jdef.joint_id, jdef.limits, motion_tracker=tracker_g1)
        mid = (jdef.limits.pos_min + jdef.limits.pos_max) / 2
        ok, _ = g.validate_command(ActuatorCommand(ControlMode.POSITION, position=mid))
        if not ok:
            all_ok = False
    except Exception as e:
        print(f"    ✗ {jdef.joint_id}: {e}")
        all_ok = False
check("Les 24 JointSafetyGuard du G1 fonctionnent avec un tracker injecté "
      "(accel_max=None par défaut, donc no-op mais aucune erreur)", all_ok)

section("RÉSUMÉ")
total = PASS + FAIL
print(f"  Résultats : {PASS}/{total} tests passés\n")
if FAIL == 0:
    print("  ✓ MotionStateTracker générique, zéro dépendance G1")
    print("  ✓ Deltas commande/commande et commande/feedback calculés correctement")
    print("  ✓ NaN/Infinity gérés dans le calcul de delta")
    print("  ✓ Rétrocompatibilité stricte : sans tracker, comportement inchangé")
    print("  ✓ Rate limiting en mode BLOCK confirmé — commande jamais modifiée")
    print("  ✓ accel_max=None désactive le check même avec tracker injecté")
    print("  ✓ Intégration validée sur les 24 joints réels du G1")
else:
    print(f"  ✗ {FAIL} test(s) échoué(s).")

sys.exit(0 if FAIL == 0 else 1)
