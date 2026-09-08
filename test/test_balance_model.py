"""
AXIS-OS — Test IBalanceModel
================================
Suite dédiée validant l'interface universelle d'équilibre et son stub
déterministe (NullBalanceModel), sur le même principe que
OpenVLAConnector : l'interface a de la valeur avant même qu'un vrai
modèle entraîné existe, car elle clarifie où l'équilibre doit se
brancher dans le pipeline de sécurité.

    python test_balance_model.py
"""

import sys
import math
sys.path.insert(0, ".")

from axis_os.hal.core.balance_model import (
    IBalanceModel, NullBalanceModel, BalanceCorrection
)
from axis_os.hal.core.types import IMUData, Vector3, Quaternion, ActuatorFeedback, ActuatorState

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


def make_imu(ax=0.0, ay=0.0, az=9.81) -> IMUData:
    return IMUData(
        linear_acceleration=Vector3(ax, ay, az),
        angular_velocity=Vector3(0.0, 0.0, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


def make_feedback(joint_id: str) -> ActuatorFeedback:
    return ActuatorFeedback(
        joint_id=joint_id, state=ActuatorState.ACTIVE, position=0.0,
        velocity=0.0, torque=0.0, current=0.0, temperature=25.0, voltage=48.0,
    )


# ══════════════════════════════════════════════════════════════════

section("1 — IBalanceModel est bien une interface abstraite")
try:
    IBalanceModel()
    check("IBalanceModel ne peut pas être instanciée directement", False)
except TypeError:
    check("IBalanceModel ne peut pas être instanciée directement", True)

section("2 — NullBalanceModel : cycle de vie load/is_loaded")
model = NullBalanceModel()
check("is_loaded() == False avant tout appel à load()", not model.is_loaded())
check("load() retourne True (le stub charge toujours 'avec succès')",
      model.load("fake_model.onnx"))
check("is_loaded() == True après load()", model.is_loaded())

section("3 — Sans modèle chargé : correction sûre par défaut, jamais de crash")
fresh_model = NullBalanceModel()
correction = fresh_model.compute_correction(make_imu(), {})
check("is_stable == False si aucun modèle chargé (signal de sécurité honnête)",
      not correction.is_stable)
check("joint_corrections vide (aucune correction hallucinée)",
      correction.joint_corrections == {})
check("confidence == 0.0 (aucune confiance sans modèle)",
      correction.confidence == 0.0)

section("4 — Avec modèle chargé : comportement stable par défaut")
model2 = NullBalanceModel()
model2.load("fake_model.onnx")
correction2 = model2.compute_correction(make_imu(), {})
check("is_stable == True avec IMU normale et modèle chargé", correction2.is_stable)
check("confidence == 1.0 (stub toujours confiant sur données valides)",
      correction2.confidence == 1.0)
check("joint_corrections toujours vide (stub ne corrige jamais réellement — "
      "documenté comme non fonctionnel)", correction2.joint_corrections == {})

section("5 — Robustesse : IMU avec NaN ne fait jamais crasher le modèle")
model3 = NullBalanceModel()
model3.load("fake_model.onnx")
bad_imu = make_imu(ax=float("nan"))
correction3 = model3.compute_correction(bad_imu, {})
check("Aucune exception levée sur IMU avec NaN (contrat respecté)", True)
check("is_stable == False sur IMU corrompue (signal de sécurité correct)",
      not correction3.is_stable)

bad_imu2 = make_imu(az=float("inf"))
correction4 = model3.compute_correction(bad_imu2, {})
check("is_stable == False sur IMU avec Infinity", not correction4.is_stable)

section("6 — Robustesse : feedback joints vide ou partiel accepté")
model4 = NullBalanceModel()
model4.load("fake_model.onnx")
correction5 = model4.compute_correction(make_imu(), {})
check("Dict de feedback vide accepté sans erreur", correction5.is_stable)

partial_feedback = {"left_knee_joint": make_feedback("left_knee_joint")}
correction6 = model4.compute_correction(make_imu(), partial_feedback)
check("Feedback partiel (1 seul joint) accepté sans erreur",
      correction6.is_stable)

section("7 — BalanceCorrection : structure conforme au contrat de sécurité")
c = BalanceCorrection(joint_corrections={"left_hip_pitch_joint": 0.05}, confidence=0.9)
check("joint_corrections accessible en dict {joint_id: angle}",
      c.joint_corrections["left_hip_pitch_joint"] == 0.05)
check("is_stable == True par défaut (valeur sûre)", c.is_stable == True)
check("confidence optionnel, None par défaut si non précisé",
      BalanceCorrection().confidence is None)

section("8 — Contrat respecté : n'importe quelle IBalanceModel est substituable")
def run_generic_balance_check(model: IBalanceModel) -> bool:
    """Simule comment le reste du HAL utiliserait N'IMPORTE QUELLE
    implémentation de IBalanceModel — vérifie le duck typing du contrat."""
    if not model.is_loaded():
        model.load("any_path.onnx")
    result = model.compute_correction(make_imu(), {})
    return isinstance(result, BalanceCorrection)

check("NullBalanceModel respecte le contrat générique attendu par le HAL",
      run_generic_balance_check(NullBalanceModel()))

section("RÉSUMÉ")
total = PASS + FAIL
print(f"  Résultats : {PASS}/{total} tests passés\n")
if FAIL == 0:
    print("  ✓ IBalanceModel : interface abstraite, non instanciable directement")
    print("  ✓ NullBalanceModel : cycle de vie load/is_loaded correct")
    print("  ✓ Comportement sûr par défaut : is_stable=False sans modèle chargé")
    print("  ✓ Robustesse NaN/Infinity confirmée sans aucune exception levée")
    print("  ✓ Contrat substituable — prêt pour qu'un futur fabricant l'implémente")
else:
    print(f"  ✗ {FAIL} test(s) échoué(s).")

sys.exit(0 if FAIL == 0 else 1)
