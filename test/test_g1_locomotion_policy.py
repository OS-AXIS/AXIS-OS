"""
G1LocomotionPolicy — tout ce qui se vérifie SANS le modèle ni le simulateur.

Une politique entraînée ne pardonne aucun écart d'observation, et un
écart ne lève jamais d'exception : le robot tombe, c'est tout. Ce test
vérifie donc l'assemblage des observations contre le code d'Unitree
lui-même, avant qu'une seule image ne soit rendue.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
    G1LocomotionConfig, G1LocomotionPolicy, body_angular_velocity,
    projected_gravity, quat_wxyz_to_matrix,
)
from axis_os.hal.drivers.unitree_g1.robot import _build_g1_joint_definitions

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def unitree_gravity(q):
    """Copie TEXTUELLE de get_gravity_orientation(), deploy_mujoco.py,
    unitree_rl_gym @ 276801e. Sert de référence."""
    qw, qx, qy, qz = q
    return np.array([
        2 * (-qz * qx + qw * qy),
        -2 * (qz * qy + qw * qx),
        1 - 2 * (qw * qw + qz * qz),
    ])


def random_quat(rng):
    q = rng.normal(size=4)
    return q / np.linalg.norm(q)


cfg = G1LocomotionConfig()
rng = np.random.default_rng(0)

section("1 — Paramètres d'entraînement")

check("12 articulations de jambe", cfg.num_actions == 12)
check("47 observations = 3+3+3+12+12+12+2", cfg.num_obs == 3 + 3 + 3 + 12 * 3 + 2)
check("Décimation 10 : commande 50 Hz sur physique 500 Hz", cfg.decimation == 10)
check("Gains et poses de même longueur que les articulations",
      len(cfg.kps) == len(cfg.kds) == len(cfg.default_angles) == 12)

g1_joints = {d.joint_id for d in _build_g1_joint_definitions()}
check("Toutes les articulations de la politique existent sur le G1 23 DOF",
      set(cfg.leg_joints) <= g1_joints)
held = {j for j, *_ in cfg.upper_body_hold}
check("Haut du corps tenu = exactement les 11 articulations restantes",
      held == g1_joints - set(cfg.leg_joints))

section("2 — Gravité projetée : identique au code d'Unitree")

identity = (1.0, 0.0, 0.0, 0.0)
g = projected_gravity(identity)
check(f"Robot droit -> gravité (0, 0, -1) (obtenu {np.round(g, 6)})",
      np.allclose(g, [0, 0, -1]))

ecart = max(np.abs(projected_gravity(q) - unitree_gravity(q)).max()
            for q in (random_quat(rng) for _ in range(1000)))
check(f"Égale à la formule d'Unitree sur 1000 orientations aléatoires "
      f"(écart max {ecart:.1e})", ecart < 1e-12)

section("3 — Vitesse angulaire : du repère MONDE au repère ROBOT")

# Robot tourné de 90° autour de z. Une rotation du monde autour de x
# doit apparaître, dans le repère du robot, autour de -y.
yaw90 = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
w_body = body_angular_velocity(yaw90, (1.0, 0.0, 0.0))
check(f"Robot tourné de 90° : ω_monde=(1,0,0) -> ω_robot=(0,-1,0) "
      f"(obtenu {np.round(w_body, 6)})",
      np.allclose(w_body, [0, -1, 0]))

check("Robot droit : repères confondus, ω inchangé",
      np.allclose(body_angular_velocity(identity, (0.3, -0.2, 0.7)), [0.3, -0.2, 0.7]))

# Le piège documenté : sans conversion, l'erreur est nulle face à la
# marche et maximale dès que le robot a tourné.
w_brut = np.array([1.0, 0.0, 0.0])
check("SANS conversion, l'erreur serait totale après un quart de tour — "
      "c'est pourquoi elle est obligatoire",
      np.linalg.norm(w_brut - w_body) > 1.0)

section("4 — Assemblage des 47 observations")

pol = G1LocomotionPolicy(policy_path=None)
pos = {j: d + 0.01 * i for i, (j, d) in enumerate(zip(cfg.leg_joints, cfg.default_angles))}
vel = {j: 0.5 for j in cfg.leg_joints}
obs = pol.build_observation(identity, (0.4, 0.0, 0.0), pos, vel, (0.5, 0.0, 0.0))

check("Vecteur de 47 valeurs", obs.shape == (47,))
check("[0:3] vitesse angulaire x 0.25", np.allclose(obs[0:3], [0.1, 0, 0]))
check("[3:6] gravité projetée", np.allclose(obs[3:6], [0, 0, -1]))
check("[6:9] consigne x (2, 2, 0.25)", np.allclose(obs[6:9], [1.0, 0, 0]))
check("[9:21] écart à la pose par défaut, dans l'ORDRE d'entraînement",
      np.allclose(obs[9:21], [0.01 * i for i in range(12)], atol=1e-6))
check("[21:33] vitesses x 0.05", np.allclose(obs[21:33], 0.025))
check("[33:45] action précédente, nulle au départ", np.allclose(obs[33:45], 0.0))
check("[45:47] phase à t=0 : sin=0, cos=1", np.allclose(obs[45:47], [0.0, 1.0]))

check("Consigne hors domaine bornée à max_cmd (5 m/s -> 0.8)",
      np.allclose(pol.clamp_command((5.0, -9.0, 9.0)), [0.8, -0.5, 1.57]))

try:
    pol.build_observation(identity, (0, 0, 0), {}, {}, (0, 0, 0))
    ok = False
except KeyError:
    ok = True
check("État incomplet : erreur explicite, jamais une observation à trous", ok)

section("5 — Action -> consignes")

t = pol.action_to_targets(np.zeros(12))
check("Action nulle -> pose par défaut exacte",
      all(abs(t[j] - d) < 1e-12 for j, d in zip(cfg.leg_joints, cfg.default_angles)))
t = pol.action_to_targets(np.ones(12))
check("Action 1 -> défaut + 0.25 (action_scale)",
      abs(t["left_knee_joint"] - (0.3 + 0.25)) < 1e-12)

limits = {d.joint_id: d.limits for d in _build_g1_joint_definitions()}
hors = [j for j, v in pol.default_pose().items()
        if not (limits[j].pos_min <= v <= limits[j].pos_max)]
check(f"Pose par défaut dans les limites de l'URDF (hors: {hors or 'aucune'})",
      not hors)

section("5b — Bornage aux butées, COMPTÉ, jamais silencieux")

lims = {d.joint_id: (d.limits.pos_min, d.limits.pos_max)
        for d in _build_g1_joint_definitions()}
pb = G1LocomotionPolicy(policy_path=None, joint_limits=lims)
big = np.zeros(12)
big[cfg.leg_joints.index("left_ankle_roll_joint")] = 4.0   # 0 + 4*0.25 = 1.0 rad
tb = pb.action_to_targets(big)
lo, hi = lims["left_ankle_roll_joint"]
check(f"Consigne hors butée ramenée exactement à la butée ({tb['left_ankle_roll_joint']:.4f} = {hi})",
      abs(tb["left_ankle_roll_joint"] - hi) < 1e-12)
check("Le bornage est COMPTÉ", pb.clamp_counts.get("left_ankle_roll_joint") == 1)
check("Les articulations non bornées ne sont pas comptées",
      set(pb.clamp_counts) == {"left_ankle_roll_joint"})

sans = G1LocomotionPolicy(policy_path=None)
check("Sans joint_limits : aucune modification (politique pure)",
      abs(sans.action_to_targets(big)["left_ankle_roll_joint"] - 1.0) < 1e-12)

# LE point subtil : le réseau doit continuer à voir son action BRUTE.
# À l'entraînement, c'est la physique qui bornait, pas la politique : la
# sortie non bornée était réinjectée telle quelle dans l'observation.
pb.last_action = big.copy()
obs_b = pb.build_observation((1.0, 0, 0, 0), (0, 0, 0),
                             {j: 0.0 for j in cfg.leg_joints},
                             {j: 0.0 for j in cfg.leg_joints}, (0, 0, 0))
check("L'action réinjectée dans l'observation reste BRUTE (4.0), pas bornée",
      abs(obs_b[33 + cfg.leg_joints.index("left_ankle_roll_joint")] - 4.0) < 1e-6)

pb.reset()
check("reset() remet les compteurs de bornage à zéro", pb.clamp_counts == {})

section("6 — La politique non chargée ne peut pas commander")

try:
    pol.step(identity, (0, 0, 0), pos, vel, (0, 0, 0))
    ok = False
except RuntimeError:
    ok = True
check("step() sans load() refuse plutôt que d'inventer une action", ok)

try:
    G1LocomotionPolicy(policy_path=Path("/tmp/axisos_inexistant.pt")).load()
    ok = False
except FileNotFoundError:
    ok = True
check("Politique absente : erreur explicite avec la source officielle", ok)

import tempfile
with tempfile.NamedTemporaryFile(suffix=".pt", delete=False) as f:
    f.write(b"pas la bonne politique")
try:
    G1LocomotionPolicy(policy_path=Path(f.name)).load()
    ok = False
except ValueError:
    ok = True
check("Politique d'empreinte différente : REFUSÉE avant même d'importer torch", ok)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
