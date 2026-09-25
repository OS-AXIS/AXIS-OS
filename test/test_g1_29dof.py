"""
Variante G1 29 DOF — le robot de référence pour la loco-manipulation.

Ce que ce test protège :
  - l'ORDRE des 29 articulations est celui qu'attend GR00T Whole-Body
    Control (ordre moteur Unitree) : une permutation silencieuse ferait
    tomber le robot sans aucun message ;
  - les 6 articulations nouvelles sont marquées NON VÉRIFIÉES ;
  - un vrai robot REFUSE de démarrer avec une convention non vérifiée ;
  - le 23 DOF est strictement inchangé.
"""
import contextlib
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.types import HALStatus, JointGroup
from axis_os.hal.drivers.unitree_g1.conventions import (
    G1_29DOF_NOUVELLES, G1_JOINT_CONVENTIONS, conventions_pour,
)
from axis_os.hal.drivers.unitree_g1.robot import (
    UnitreeG1HAL, build_g1_joint_definitions,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


# Ordre moteur Unitree, tel que l'utilise le code de DÉPLOIEMENT de
# GR00T WBC (decoupled_wbc/control/policy/g1_gear_wbc_policy.py).
ORDRE_GR00T = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

section("1 — Les 29 articulations, dans l'ordre qu'attend GR00T WBC")

d29 = build_g1_joint_definitions("29dof")
check("29 articulations chargées depuis l'URDF officiel", len(d29) == 29)
check("ORDRE identique à l'ordre moteur de GR00T WBC — aucune table de "
      "réordonnancement, donc aucun risque de permutation",
      [d.joint_id for d in d29] == ORDRE_GR00T)
check("Chaque articulation a des limites issues de l'URDF",
      all(d.limits.pos_min < d.limits.pos_max and d.limits.torque_max > 0
          for d in d29))

section("2 — Groupes d'articulations")

g = {d.joint_id: d.group for d in d29}
check("12 articulations de LOCOMOTION (les jambes)",
      sum(v is JointGroup.LOCOMOTION for v in g.values()) == 12)
check("14 de MANIPULATION (deux bras de sept)",
      sum(v is JointGroup.MANIPULATION for v in g.values()) == 14)
check("3 articulations de taille en OTHER — le groupe que la marche et les "
      "apps devront se partager explicitement",
      [j for j, v in g.items() if v is JointGroup.OTHER]
      == ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"])

section("3 — Conventions : héritées, ou NON VÉRIFIÉES")

t29 = conventions_pour("29dof")
check("Table complète : 29 entrées", len(t29) == 29)
check("Les 23 articulations communes héritent de la table validée du "
      "23 DOF (S2, 23/23)",
      all(t29[j] == G1_JOINT_CONVENTIONS[j] for j in G1_JOINT_CONVENTIONS))
nv = [j for j, c in t29.items() if not c.verifiee]
check(f"Les 6 nouvelles sont marquées NON VÉRIFIÉES ({len(nv)})",
      sorted(nv) == sorted(G1_29DOF_NOUVELLES))
check("Leurs étiquettes ne prétendent à AUCUN sens anatomique — les nommer "
      "sans mesure serait inventer",
      all("urdf" in t29[j].positive_label for j in G1_29DOF_NOUVELLES))

section("4 — Simulation tolérée, robot RÉEL refusé")

buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    sim = UnitreeG1HAL(backend="sim", variante="29dof")
    st_sim = sim.initialize_all()
check("En simulation : démarrage autorisé", st_sim is HALStatus.OK)
check("… avec un avertissement qui NOMME les articulations non vérifiées",
      "NON VÉRIFIÉE" in buf.getvalue() and "waist_roll_joint" in buf.getvalue())

buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    reel = UnitreeG1HAL(backend="dds", network_interface="lo", variante="29dof")
    st_reel = reel.initialize_all()
check("Sur un robot RÉEL : démarrage REFUSÉ — on ne commande jamais un "
      "moteur dont on ignore le sens", st_reel is HALStatus.ERROR_FAULT)
check("… AVANT toute ouverture de communication avec le robot",
      reel._dds_hub is None)
check("… et le message dit quoi faire (lancer S2)",
      "S2" in buf.getvalue())

section("5 — Le 23 DOF est strictement inchangé")

with contextlib.redirect_stdout(io.StringIO()):
    h23 = UnitreeG1HAL(backend="sim")
    st23 = h23.initialize_all()
check("Variante par défaut : 23 DOF, 23 articulations, démarrage OK",
      len(h23._joint_defs) == 23 and st23 is HALStatus.OK)
check("Aucune convention non vérifiée dans le 23 DOF",
      all(c.verifiee for c in conventions_pour("23dof").values()))

try:
    UnitreeG1HAL(backend="sim", variante="31dof")
    ok = False
except ValueError:
    ok = True
check("Variante inconnue : refusée, jamais devinée", ok)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
