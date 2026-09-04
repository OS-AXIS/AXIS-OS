"""
AXIS-OS — Test JointSafetyGuard v2
=====================================
Suite dédiée couvrant les points #9 et #10 de la revue externe :
tests exhaustifs sur les 24 joints réels du G1, injection de pannes
(feedback incohérent, capteur défaillant, valeurs extrêmes, violations
simultanées), et validation des JointLimits elles-mêmes.

    python test_safety_v2.py
"""

import sys
import math
sys.path.insert(0, ".")

from axis_os.hal.safety.limits import (
    JointSafetyGuard, SafetyErrorKind, validate_limits, LimitsValidationResult
)
from axis_os.hal.core.types import (
    ActuatorCommand, ActuatorFeedback, ActuatorState,
    ControlMode, JointLimits
)
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


def make_feedback(joint_id: str, position=0.0, velocity=0.0, torque=0.0,
                  current=0.0, temperature=25.0, voltage=48.0,
                  state=ActuatorState.IDLE) -> ActuatorFeedback:
    return ActuatorFeedback(
        joint_id=joint_id, state=state, position=position, velocity=velocity,
        torque=torque, current=current, temperature=temperature, voltage=voltage
    )


G1_JOINTS = _build_g1_joint_definitions()

# ══════════════════════════════════════════════════════════════════

section("1 — Validation des JointLimits au démarrage (#5)")

valid_limits = JointLimits(pos_min=-1.0, pos_max=1.0, vel_max=20.0,
                           torque_max=88.0, current_max=20.0, temperature_max=80.0)
r = validate_limits("test_joint", valid_limits)
check("Limites cohérentes → valid=True", r.valid)

broken_minmax = JointLimits(pos_min=1.0, pos_max=-1.0, vel_max=20.0,
                            torque_max=88.0, current_max=20.0, temperature_max=80.0)
r2 = validate_limits("test_joint", broken_minmax)
check("pos_min >= pos_max → détecté invalide", not r2.valid)

broken_nan = JointLimits(pos_min=-1.0, pos_max=float("nan"), vel_max=20.0,
                         torque_max=88.0, current_max=20.0, temperature_max=80.0)
r3 = validate_limits("test_joint", broken_nan)
check("pos_max=NaN → détecté invalide", not r3.valid)

broken_neg_vel = JointLimits(pos_min=-1.0, pos_max=1.0, vel_max=-5.0,
                             torque_max=88.0, current_max=20.0, temperature_max=80.0)
r4 = validate_limits("test_joint", broken_neg_vel)
check("vel_max négatif (amplitude doit être positive) → détecté", not r4.valid)

section("2 — JointSafetyGuard refuse une construction avec limites invalides")
try:
    JointSafetyGuard("bad_joint", broken_minmax)
    check("Construction refusée si limites incohérentes", False)
except ValueError:
    check("Construction refusée si limites incohérentes", True)

# strict_init=False doit permettre de contourner (cas de test/debug explicite)
try:
    guard = JointSafetyGuard("bad_joint", broken_minmax, strict_init=False)
    check("strict_init=False permet la construction malgré tout", True)
except ValueError:
    check("strict_init=False permet la construction malgré tout", False)

section("3 — Toutes les 24 JointLimits du G1 sont valides (croisement avec conventions)")
all_valid = True
for jdef in G1_JOINTS:
    r = validate_limits(jdef.joint_id, jdef.limits)
    if not r.valid:
        all_valid = False
        print(f"    ✗ {jdef.joint_id}: {r}")
check(f"Les {len(G1_JOINTS)} JointLimits réelles du G1 passent validate_limits()",
      all_valid)

section("4 — NaN/Infinity explicitement rejetés avec le bon diagnostic (#1)")
guard = JointSafetyGuard("left_knee", G1_JOINTS[3].limits)  # left_knee

ok, v = guard.validate_command(ActuatorCommand(ControlMode.POSITION, position=float("nan")))
check("position=NaN rejetée", not ok)
check("rule == INVALID_VALUE (pas POSITION_LIMIT trompeur)", v.rule == "INVALID_VALUE")
check("kind == SafetyErrorKind.INVALID_VALUE", v.kind == SafetyErrorKind.INVALID_VALUE)

ok2, v2 = guard.validate_command(ActuatorCommand(ControlMode.VELOCITY, velocity=float("inf")))
check("velocity=Infinity rejetée avec INVALID_VALUE", not ok2 and v2.rule == "INVALID_VALUE")

ok3, v3 = guard.validate_command(ActuatorCommand(ControlMode.TORQUE, torque=float("-inf")))
check("torque=-Infinity rejeté avec INVALID_VALUE", not ok3 and v3.rule == "INVALID_VALUE")

ok4, v4 = guard.validate_command(ActuatorCommand(ControlMode.POSITION, position=0.3))
check("Commande normale toujours acceptée après ajout du check", ok4)

section("5 — Tests exhaustifs sur les 24 joints réels du G1 (#9)")
results_per_joint = {}
for jdef in G1_JOINTS:
    g = JointSafetyGuard(jdef.joint_id, jdef.limits)

    # Position hors limite
    over_pos = jdef.limits.pos_max + 10.0
    ok_pos, v_pos = g.validate_command(ActuatorCommand(ControlMode.POSITION, position=over_pos))

    # Vitesse hors limite
    over_vel = jdef.limits.vel_max * 5
    ok_vel, v_vel = g.validate_command(ActuatorCommand(ControlMode.VELOCITY, velocity=over_vel))

    # Couple hors limite
    over_torque = jdef.limits.torque_max * 5
    ok_torque, v_torque = g.validate_command(ActuatorCommand(ControlMode.TORQUE, torque=over_torque))

    # Commande dans les limites — milieu de la plage
    mid_pos = (jdef.limits.pos_min + jdef.limits.pos_max) / 2
    ok_normal, _ = g.validate_command(ActuatorCommand(ControlMode.POSITION, position=mid_pos))

    results_per_joint[jdef.joint_id] = (
        not ok_pos and v_pos.rule == "POSITION_LIMIT" and
        not ok_vel and v_vel.rule == "VELOCITY_LIMIT" and
        not ok_torque and v_torque.rule == "TORQUE_LIMIT" and
        ok_normal
    )

all_joints_ok = all(results_per_joint.values())
check(f"Position/vitesse/couple hors-limite rejetés + commande médiane "
      f"acceptée sur les {len(G1_JOINTS)} joints",
      all_joints_ok)
if not all_joints_ok:
    for jid, ok in results_per_joint.items():
        if not ok:
            print(f"    ✗ {jid}")

section("6 — Fault injection : feedback incohérent (#10)")
guard = JointSafetyGuard("right_knee", next(j.limits for j in G1_JOINTS if j.joint_id == "right_knee"))

bad_fb = make_feedback("right_knee", temperature=float("nan"))
violations = guard.check_feedback_health(bad_fb)
check("Feedback avec température NaN détecté comme violation",
      len(violations) == 1 and violations[0].rule == "INVALID_VALUE")

bad_fb2 = make_feedback("right_knee", torque=float("inf"))
violations2 = guard.check_feedback_health(bad_fb2)
check("Feedback avec couple Infinity détecté",
      len(violations2) == 1 and violations2[0].kind == SafetyErrorKind.INVALID_VALUE)

section("7 — Fault injection : capteur bloqué (valeur figée extrême) (#10)")
stuck_fb = make_feedback("right_knee", temperature=999.0)  # capteur qui délire
violations3 = guard.check_feedback_health(stuck_fb)
check("Température=999°C (capteur défaillant) → TEMPERATURE_CRITICAL détectée",
      any(v.rule == "TEMPERATURE_CRITICAL" for v in violations3))

section("8 — Fault injection : commande extrême dans un contexte normal (#10)")
guard = JointSafetyGuard("left_elbow", next(j.limits for j in G1_JOINTS if j.joint_id == "left_elbow"))
extreme_cmd = ActuatorCommand(ControlMode.POSITION, position=1e10)
ok, v = guard.validate_command(extreme_cmd)
check("Commande absurdement grande (1e10) rejetée proprement",
      not ok and v.rule == "POSITION_LIMIT")

section("9 — Fault injection : plusieurs violations simultanées sur des joints différents (#10)")
guards = {jid: JointSafetyGuard(jid, jdef.limits)
         for jid, jdef in [(j.joint_id, j) for j in G1_JOINTS[:5]]}
simultaneous_results = {}
for jid, g in guards.items():
    ok, v = g.validate_command(ActuatorCommand(ControlMode.TORQUE,
                                               torque=g.limits.torque_max * 10))
    simultaneous_results[jid] = not ok
check("5 joints en violation simultanée, tous correctement détectés indépendamment",
      all(simultaneous_results.values()))

section("10 — Rétrocompatibilité : signature validate_command() inchangée")
guard = JointSafetyGuard("left_knee", next(j.limits for j in G1_JOINTS if j.joint_id == "left_knee"))
ok, v = guard.validate_command(ActuatorCommand(ControlMode.POSITION, position=0.5))
check("Retourne toujours un tuple (bool, Optional[SafetyViolation])",
      isinstance(ok, bool) and (v is None or hasattr(v, "rule")))
check("last_violation toujours accessible", guard.last_violation is None)  # dernière cmd OK

section("RÉSUMÉ")
total = PASS + FAIL
print(f"  Résultats : {PASS}/{total} tests passés\n")
if FAIL == 0:
    print("  ✓ validate_limits() : min<max, valeurs finies, amplitudes positives")
    print("  ✓ Les 24 JointLimits réelles du G1 sont cohérentes")
    print("  ✓ NaN/Infinity rejetés avec diagnostic correct (INVALID_VALUE)")
    print("  ✓ Position/vitesse/couple testés sur les 24 joints réels")
    print("  ✓ Fault injection : feedback NaN, capteur figé, commande extrême,")
    print("    violations simultanées multi-joints — tous détectés")
else:
    print(f"  ✗ {FAIL} test(s) échoué(s).")

sys.exit(0 if FAIL == 0 else 1)
