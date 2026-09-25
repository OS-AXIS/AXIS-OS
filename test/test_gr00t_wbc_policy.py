"""
Politique GR00T WBC — l'observation, l'historique, la bascule, sans GPU.

Chaque piège de docs/integration_gr00t_wbc.md est vérifié ici, avec des
réseaux FACTICES injectés : aucun fichier ONNX nécessaire.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.unitree_g1.gr00t_wbc_policy import (
    G1_29_BODY_JOINTS, GR00TWBCAdapter, GR00TWBCConfig, GR00TWBCPolicy,
)
from axis_os.hal.drivers.unitree_g1.robot import build_g1_joint_definitions
from axis_os.services.locomotion.service import BaseState, Twist

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


cfg = GR00TWBCConfig()
DROIT = (1.0, 0.0, 0.0, 0.0)


def pose_defaut():
    q = {j: 0.0 for j in cfg.body_joints}
    for j, v in zip(cfg.controlled_joints, cfg.default_angles):
        q[j] = v
    return q


def zeros():
    return {j: 0.0 for j in cfg.body_joints}


section("0 — Accord avec le driver 29 DOF")

check("Les 29 articulations observées sont exactement celles, dans l'ordre, "
      "que charge le driver",
      tuple(d.joint_id for d in build_g1_joint_definitions("29dof"))
      == G1_29_BODY_JOINTS)
check("15 commandées : les 12 jambes puis les 3 de taille",
      cfg.controlled_joints[-3:] == ("waist_yaw_joint", "waist_roll_joint",
                                     "waist_pitch_joint"))
check("Entrée complète : 86 × 6 = 516", cfg.num_obs == 516)
check("Commande à 50 Hz (200 Hz ÷ 4)", abs(cfg.control_dt - 0.02) < 1e-12)

section("1 — Observation unitaire de 86 valeurs")

pol = GR00TWBCPolicy()
q = pose_defaut()
q["left_shoulder_pitch_joint"] = 0.3
dq = zeros()
dq["left_knee_joint"] = 2.0
obs = pol.build_observation(DROIT, (0.2, 0.0, 0.0), q, dq, (0.5, 0.0, 0.4),
                            0.70, (0.1, 0.0, 0.0), ang_vel_frame="body")
check("86 valeurs", obs.shape == (86,))
check("Consigne mise à l'échelle [2, 2, 0.5]",
      np.allclose(obs[0:3], [1.0, 0.0, 0.2]))
check("Hauteur et orientation du torse, BRUTES", abs(obs[3] - 0.70) < 1e-6
      and np.allclose(obs[4:7], [0.1, 0.0, 0.0]))
check("Vitesse angulaire × 0.5", np.allclose(obs[7:10], [0.1, 0.0, 0.0]))
check("Gravité projetée : robot droit -> (0, 0, -1)",
      np.allclose(obs[10:13], [0.0, 0.0, -1.0]))
check("Jambes à la pose par défaut -> 0 : centrées sur la pose d'entraînement",
      np.allclose(obs[13:13 + 15], 0.0))
i_epaule = 13 + G1_29_BODY_JOINTS.index("left_shoulder_pitch_joint")
check("Bras centrés sur ZÉRO, pas sur une pose par défaut — épaule à 0.3 "
      "-> 0.3 (piège 3)", abs(obs[i_epaule] - 0.3) < 1e-6)
i_genou = 13 + 29 + G1_29_BODY_JOINTS.index("left_knee_joint")
check("Vitesses articulaires × 0.05", abs(obs[i_genou] - 0.1) < 1e-6)
check("Action précédente dans les 15 dernières valeurs",
      np.allclose(obs[71:86], 0.0))

try:
    pol.build_observation(DROIT, (0, 0, 0), q, dq, (0, 0, 0), 0.74, (0, 0, 0))
    ok = False
except TypeError:
    ok = True
check("Repère de la vitesse angulaire OBLIGATOIRE (piège 4)", ok)

try:
    incomplet = dict(q); del incomplet["right_wrist_yaw_joint"]
    pol.build_observation(DROIT, (0, 0, 0), incomplet, dq, (0, 0, 0), 0.74,
                          (0, 0, 0), ang_vel_frame="body")
    ok = False
except KeyError:
    ok = True
check("Une articulation de bras manquante est REFUSÉE — les 29 sont "
      "observées", ok)

section("2 — Historique : plus ancienne en premier (piège 1)")

h = GR00TWBCPolicy()
a, b, c = (np.full(86, v, dtype=np.float32) for v in (1.0, 2.0, 3.0))
e1 = h.empiler(a)
check("Premier pas : 516 valeurs, zéros À GAUCHE, observation à droite",
      e1.shape == (516,) and np.all(e1[:430] == 0) and np.all(e1[430:] == 1.0))
h.empiler(b)
e3 = h.empiler(c)
blocs = e3.reshape(6, 86)[:, 0].tolist()
check(f"Après trois pas : {blocs} — la plus récente en DERNIER",
      blocs == [0.0, 0.0, 0.0, 1.0, 2.0, 3.0])
for v in range(4, 10):
    e = h.empiler(np.full(86, float(v), dtype=np.float32))
check("Au-delà de six, la plus ancienne sort",
      e.reshape(6, 86)[:, 0].tolist() == [4.0, 5.0, 6.0, 7.0, 8.0, 9.0])
h.reset()
check("reset() vide l'historique — une reprise ne reçoit pas le passé "
      "d'une autre situation (piège 2)",
      np.all(h.empiler(a)[:430] == 0))

section("3 — Bascule équilibre / marche, et cibles")

vus = []


def faux(nom):
    def net(x):
        vus.append((nom, x.shape))
        return np.zeros(15, dtype=np.float32)
    return net


p = GR00TWBCPolicy()
p.utiliser_reseaux(faux("balance"), faux("walk"))
cibles = p.step(DROIT, (0, 0, 0), pose_defaut(), zeros(), (0.0, 0.0, 0.0),
                ang_vel_frame="body")
check("Consigne nulle -> politique d'ÉQUILIBRE", vus[-1][0] == "balance")
check("Le réseau reçoit l'entrée complète de 516 valeurs", vus[-1][1] == (516,))
p.step(DROIT, (0, 0, 0), pose_defaut(), zeros(), (0.3, 0.0, 0.0),
       ang_vel_frame="body")
check("Consigne de marche -> politique de MARCHE", vus[-1][0] == "walk")
p.step(DROIT, (0, 0, 0), pose_defaut(), zeros(), (0.04, 0.0, 0.0),
       ang_vel_frame="body")
check("0.04 m/s : sous le seuil de 0.05, sur la norme BRUTE — équilibre",
      vus[-1][0] == "balance")
check("Action nulle -> pose par défaut, sur les 15 articulations commandées "
      "seulement", len(cibles) == 15 and all(
          abs(cibles[j] - v) < 1e-9
          for j, v in zip(cfg.controlled_joints, cfg.default_angles)))

try:
    GR00TWBCPolicy().step(DROIT, (0, 0, 0), pose_defaut(), zeros(), (0, 0, 0),
                          ang_vel_frame="body")
    ok = False
except RuntimeError:
    ok = True
check("Politique non chargée : refus de commander", ok)

section("4 — Adaptateur de locomotion")

ad = GR00TWBCAdapter(p, pose_bras={"left_elbow_joint": 0.6,
                                   "right_elbow_joint": 0.6})
st = BaseState(DROIT, (0.0, 0.0, 0.0), pose_defaut(), zeros())
t = ad.step(st, Twist())
check("Cibles pour les 29 articulations : 15 de la politique, 14 de bras",
      len(t) == 29)
check("La pose de bras demandée est transmise",
      t["left_elbow_joint"] == 0.6 and t["right_shoulder_pitch_joint"] == 0.0)
ad.posture(hauteur=0.62)
vus.clear()
ad.step(st, Twist())
# la hauteur figure en position 3 de la DERNIÈRE observation empilée
check("La posture commandée (hauteur 0.62) atteint la politique",
      abs(p._history[-1][3] - 0.62) < 1e-6)
try:
    GR00TWBCAdapter(p, pose_bras={"left_knee_joint": 1.0})
    ok = False
except KeyError:
    ok = True
check("Impossible de piloter une JAMBE par la pose de bras", ok)
couche = BaseState((math.cos(math.pi / 4), math.sin(math.pi / 4), 0, 0),
                   (0, 0, 0), {}, {})
check("Robot couché : chute détectée", ad.is_fallen(couche))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
