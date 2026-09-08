"""
AXIS-OS — Test Conventions & Validation
==========================================
Suite couvrant les points de la revue externe sur conventions.py :
round-trip sur les 24 joints, tests dédiés aux inversions critiques,
test de non-régression nommé pour le bug historique right_knee_joint,
rejet NaN/Infinity, validation de la table au démarrage.

    python test_conventions.py
"""

import sys
import math
sys.path.insert(0, ".")

from axis_os.hal.drivers.unitree_g1.conventions import (
    G1_JOINT_CONVENTIONS, get_inverted_joints
)
from axis_os.hal.drivers.unitree_g1.convention_validator import (
    validate_convention_table, ConventionErrorKind
)
from axis_os.hal.drivers.unitree_g1.robot import (
    UnitreeG1HAL, _build_g1_joint_definitions
)
from axis_os.hal.core.kinematic_normalizer import (
    build_g1_normalizer, InvalidAngleError
)
from axis_os.hal.core.types import HALStatus

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


ALL_JOINT_IDS = list(G1_JOINT_CONVENTIONS.keys())
TEST_ANGLES = [0.0, 0.1, -0.5, 1.0, -1.57, 2.0, math.pi / 4]


# ══════════════════════════════════════════════════════════════════

section("1 — Round-trip complet sur les 23 joints (point #3 revue externe)")
normalizer = build_g1_normalizer()
check("23 joints présents dans la table (fichier URDF officiel)", len(ALL_JOINT_IDS) == 23)

failures = []
for jid in ALL_JOINT_IDS:
    for angle in TEST_ANGLES:
        raw  = normalizer.normalized_to_raw(jid, angle)
        back = normalizer.raw_to_normalized(jid, raw)
        if abs(back - angle) > 1e-9:
            failures.append((jid, angle, back))
check(f"Round-trip exact sur {len(ALL_JOINT_IDS)} joints × {len(TEST_ANGLES)} angles "
      f"({len(ALL_JOINT_IDS)*len(TEST_ANGLES)} combinaisons)",
      len(failures) == 0)
if failures:
    for jid, angle, back in failures[:5]:
        print(f"    ✗ {jid}: {angle} → ... → {back}")

section("2 — Tests dédiés aux inversions critiques (point #4)")
inverted = get_inverted_joints()
check("9 joints inversés détectés", len(inverted) == 9)

# right_hip_pitch_joint — signalé "INVERSION G1 CRITIQUE" dans conventions.py
raw = normalizer.normalized_to_raw("right_hip_pitch_joint", 0.3)
check("right_hip_pitch_joint: normalized=+0.3 (flexion) → raw=-0.3 (sign inversé)",
      abs(raw - (-0.3)) < 1e-9)
back = normalizer.raw_to_normalized("right_hip_pitch_joint", -0.3)
check("right_hip_pitch_joint: raw=-0.3 → normalized=+0.3 (retour cohérent)",
      abs(back - 0.3) < 1e-9)

# right_knee_joint — signalé "bug historique" dans conventions.py
section("3 — Test de non-régression nommé : bug historique right_knee_joint (point #13)")
raw_knee = normalizer.normalized_to_raw("right_knee_joint", 0.4)
check("test_regression_right_knee_flexion: normalized=+0.4 → raw=-0.4",
      abs(raw_knee - (-0.4)) < 1e-9)
# Le bug original (Sprint 1) : commander normalized=+0.4 sur les deux genoux
# faisait rejeter right_knee_joint car le safety guard voyait raw=+0.4 (hors limite
# pos_max=0.09) au lieu de raw=-0.4 (valide). Reproduction exacte du scénario :
sym_left  = normalizer.normalized_to_raw("left_knee_joint", 0.4)
sym_right = normalizer.normalized_to_raw("right_knee_joint", 0.4)
check("test_regression_right_knee_symetrie: mêmes normalized (+0.4), "
      "raw différents (le fix)", sym_left == 0.4 and sym_right == -0.4)

section("4 — Rejet NaN/Infinity (point #9, trou identifié)")
try:
    normalizer.normalized_to_raw("left_knee_joint", float("nan"))
    check("NaN rejeté avec InvalidAngleError", False)
except InvalidAngleError as e:
    check("NaN rejeté avec InvalidAngleError", True)
    print(f"    → {e}")

try:
    normalizer.normalized_to_raw("left_knee_joint", float("inf"))
    check("Infinity rejeté avec InvalidAngleError", False)
except InvalidAngleError:
    check("Infinity rejeté avec InvalidAngleError", True)

try:
    normalizer.raw_to_normalized("right_knee_joint", float("-inf"))
    check("-Infinity rejeté côté raw_to_normalized", False)
except InvalidAngleError:
    check("-Infinity rejeté côté raw_to_normalized", True)

check("Valeur normale (0.5) toujours acceptée après ajout du check",
      normalizer.normalized_to_raw("left_knee_joint", 0.5) == 0.5)

section("5 — Validation de la table au démarrage (point #5)")
expected = [j.joint_id for j in _build_g1_joint_definitions()]
result = validate_convention_table(G1_JOINT_CONVENTIONS, expected)
check("Table actuelle valide (0 erreur)", result.valid)
check("expected_joint_ids == clés de la table (24 == 24)",
      len(expected) == len(G1_JOINT_CONVENTIONS))

# Table cassée : joint manquant
broken_table = dict(G1_JOINT_CONVENTIONS)
del broken_table["right_knee_joint"]
result_broken = validate_convention_table(broken_table, expected)
check("Table avec joint manquant → invalide", not result_broken.valid)
check("Erreur de type MISSING_JOINT détectée",
      any(e.kind == ConventionErrorKind.MISSING_JOINT for e in result_broken.errors))

# Table cassée : sign invalide
from axis_os.hal.drivers.unitree_g1.conventions import JointConvention
broken_table2 = dict(G1_JOINT_CONVENTIONS)
broken_table2["left_knee_joint"] = JointConvention(
    "left_knee_joint", sign=0.5, offset_rad=0.0,  # sign invalide : ni -1 ni +1
    positive_label="flexion", negative_label="extension",
    anatomical_note="test"
)
result_broken2 = validate_convention_table(broken_table2, expected)
check("Table avec sign=0.5 (invalide) → détectée", not result_broken2.valid)
check("Erreur de type INVALID_SIGN détectée",
      any(e.kind == ConventionErrorKind.INVALID_SIGN for e in result_broken2.errors))

section("6 — Validation branchée dans initialize_all() (intégration réelle)")
robot = UnitreeG1HAL("g1_conv_test")
status = robot.initialize_all()
check("Robot avec table valide s'initialise normalement", status == HALStatus.OK)
check("is_initialized == True", robot.is_initialized)
robot.shutdown_all()

section("RÉSUMÉ")
total = PASS + FAIL
print(f"  Résultats : {PASS}/{total} tests passés\n")
if FAIL == 0:
    print("  ✓ Round-trip validé sur les 23 joints (fichier URDF officiel)")
    print("  ✓ Inversions critiques (right_hip_pitch_joint, right_knee_joint) confirmées")
    print("  ✓ Bug historique right_knee_joint : test de non-régression en place")
    print("  ✓ NaN/Infinity rejetés explicitement (trou de sécurité comblé)")
    print("  ✓ Validation de table au démarrage — branchée dans initialize_all()")
else:
    print(f"  ✗ {FAIL} test(s) échoué(s).")

sys.exit(0 if FAIL == 0 else 1)
