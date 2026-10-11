"""
Suite W1 — le test qui dit si le Graal est à portée.

G1 29 DOF, politique GR00T Whole-Body Control, dans Isaac Sim. Quatre
phases, dont chacune lève un risque avant la suivante :

  A. 0–4 s   debout, bras le long du corps     -> l'intégration est-elle juste ?
  B. 4–7 s   bras amenés en POSTURE DE PORT    -> tient-il en changeant de posture ?
  C. 7–13 s  marche à 0.3 m/s, bras en port    -> SAIT-IL MARCHER EN PORTANT ?
  D. 13–17 s arrêt, bras en port               -> vrai mode debout, sans piétiner ?

Lecture du verdict :
  chute en A      : erreur d'intégration (observation, ordre, repère) — à
                    corriger avant toute autre chose
  chute en B ou C : la politique ne supporte pas la posture de port — il
                    faudra l'entraîner, ou réduire la posture
  tout tient      : la voie du Graal est ouverte — saisie, port, dépose

Pas encore de colis : on sépare « porter la posture » de « porter la
charge ». Le colis viendra au test suivant.

PRÉREQUIS dans le conteneur :
  - /isaac-sim/g1_29dof_rev_1_0.urdf, et ses meshes (le pont les vérifie)
  - /isaac-sim/policies/gr00t_wbc_balance.onnx et gr00t_wbc_walk.onnx,
    récupérés par git lfs depuis NVlabs/GR00T-WholeBodyControl
    (decoupled_wbc/sim2mujoco/resources/robots/g1/policy/)
  - onnxruntime installé dans le Python d'Isaac Sim

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_w1_gr00t_port.py
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)

POLICIES = Path("/isaac-sim/policies")
FAIL = 0

# Pose de port en convention ANATOMIQUE, traduite par la table de
# conventions validée — pas devinée en angles bruts.
PORT_ANATOMIQUE = {
    "shoulder_pitch": 0.6,     # flexion antérieure : bras vers l'avant
    "shoulder_roll": -0.15,    # adduction : avant-bras rapprochés
    "elbow": 1.2,              # flexion du coude
}
# Gains des bras, relevés dans la configuration de déploiement NVIDIA
# (g1_29dof_gear_wbc.yaml, indices 15 à 28).
KP_BRAS = [90, 60, 20, 60, 4, 4, 4]
KD_BRAS = [2.0, 1.0, 0.4, 1.0, 0.2, 0.2, 0.2]


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    from isaacsim.core.utils.types import ArticulationAction

    from axis_os.hal.core.types import ActuatorCommand, ControlMode
    from axis_os.hal.drivers.unitree_g1.conventions import conventions_pour
    from axis_os.hal.drivers.unitree_g1.gr00t_wbc_policy import (
        GR00TWBCAdapter, GR00TWBCConfig, GR00TWBCPolicy,
    )
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        body_angular_velocity,
    )
    from axis_os.services.locomotion.service import (
        BaseState, LocomotionService, LocomotionState,
    )

    cfg = GR00TWBCConfig()
    pol = GR00TWBCPolicy(POLICIES / "gr00t_wbc_balance.onnx",
                         POLICIES / "gr00t_wbc_walk.onnx", cfg)
    pol.load()                      # affiche les empreintes, à épingler

    conv = conventions_pour("29dof")

    def brut(joint, anatomique):
        c = conv[joint]
        return c.sign * anatomique + c.offset_rad

    port = {}
    for cote in ("left", "right"):
        for art, v in PORT_ANATOMIQUE.items():
            j = f"{cote}_{art}_joint"
            port[j] = brut(j, v)
    print(f"  posture de port (angles bruts) : "
          f"{ {k: round(v, 2) for k, v in port.items()} }")

    bridge = IsaacSimBridge(variante="29dof")
    bridge.setup(fix_base=False, require_meshes=True,
                 physics_dt=cfg.physics_dt)
    robot, world = bridge._robot, bridge._world
    idx = bridge._joint_index
    check(f"29 articulations reliées à Isaac Sim ({len(idx)})", len(idx) == 29)
    names = sorted(idx, key=lambda j: idx[j])

    g = pol.gains()
    bras = cfg.body_joints[cfg.num_actions:]
    for i, j in enumerate(bras):
        g[j] = (KP_BRAS[i % 7], KD_BRAS[i % 7])
    world.reset()
    robot.get_articulation_controller().set_gains(
        kps=np.array([g[j][0] for j in names]),
        kds=np.array([g[j][1] for j in names]))

    q0 = {j: 0.0 for j in names}
    q0.update(dict(zip(cfg.controlled_joints, cfg.default_angles)))
    q0v = np.array([q0[j] for j in names])
    robot.set_joint_positions(q0v)
    robot.set_joint_velocities(np.zeros_like(q0v))
    robot.apply_action(ArticulationAction(joint_positions=q0v))

    hal = bridge.build_hal()
    hal.initialize_all()
    sim_t = [0.0]

    def read_state() -> BaseState:
        _p, quat = robot.get_world_pose()
        omega = body_angular_velocity(quat, robot.get_angular_velocity())
        jp, jv = robot.get_joint_positions(), robot.get_joint_velocities()
        return BaseState(tuple(float(v) for v in quat),
                         tuple(float(v) for v in omega),
                         {j: float(jp[idx[j]]) for j in names},
                         {j: float(jv[idx[j]]) for j in names})

    def send_targets(targets):
        return hal.send_joint_commands(
            {j: ActuatorCommand(mode=ControlMode.POSITION, position=v)
             for j, v in targets.items()})

    adapt = GR00TWBCAdapter(pol)
    loco = LocomotionService(adapt, read_state, send_targets,
                             command_timeout_s=0.5, clock=lambda: sim_t[0])
    loco.start()

    def lacet(q):
        w, x, y, z = (float(v) for v in q)
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    mesures = {}
    politiques = {"A": set(), "B": set(), "C": set(), "D": set()}
    chute = None
    k = 0
    t_mur = time.monotonic()
    while sim_t[0] < 17.0 and time.monotonic() - t_mur < 900:
        t = sim_t[0]
        phase = "A" if t < 4 else "B" if t < 7 else "C" if t < 13 else "D"

        if phase == "B":                        # posture de port en 1 s
            a = min(1.0, (t - 4.0) / 1.0)
            adapt.pose_bras.update({j: a * v for j, v in port.items()})
        if phase == "C":
            loco.set_velocity(0.3, 0.0, 0.0)
        if phase == "D":
            loco.halt()

        pos, quat = robot.get_world_pose()
        for nom, instant in (("A_debut", 2.0), ("A_fin", 4.0), ("C_debut", 7.0),
                             ("C_fin", 13.0), ("D_debut", 15.0), ("D_fin", 17.0)):
            if nom not in mesures and t >= instant - 1e-9:
                mesures[nom] = (float(pos[0]), float(pos[1]), lacet(quat))

        r = loco.tick()
        if loco.state is LocomotionState.FALLEN:
            chute = (t, phase)
            break
        politiques[phase].add(pol.derniere_politique)
        for _ in range(cfg.decimation - 1):
            world.step(render=False)
        sim_t[0] += cfg.control_dt
        k += 1

    print(f"\n{'=' * 66}\n  RÉSULTATS\n{'=' * 66}")
    print(f"  politiques utilisées : "
          f"{ {p: sorted(v) for p, v in politiques.items() if v} }")
    if chute:
        print(f"\n  CHUTE à t = {chute[0]:.2f} s, phase {chute[1]}")
    check("Aucune chute sur les 17 s", chute is None)

    def ecart(a, b):
        return math.hypot(mesures[b][0] - mesures[a][0],
                          mesures[b][1] - mesures[a][1])

    if "A_fin" in mesures:
        d = ecart("A_debut", "A_fin")
        check(f"A — debout bras le long du corps : dérive {d * 100:.1f} cm en "
              f"2 s (un vrai mode debout, pas de piétinement)", d < 0.10)
    if "C_fin" in mesures:
        d = mesures["C_fin"][0] - mesures["C_debut"][0]
        check(f"C — marche EN POSTURE DE PORT : {d:.2f} m en 6 s "
              f"(consigne 0.3 m/s, idéal 1.8 m)", d > 0.9)
    if "D_fin" in mesures:
        d = ecart("D_debut", "D_fin")
        dl = abs(math.degrees(mesures["D_fin"][2] - mesures["D_debut"][2]))
        check(f"D — arrêté en posture de port : dérive {d * 100:.1f} cm, "
              f"rotation {dl:.1f}° en 2 s", d < 0.10 and dl < 5.0)
    check("La politique d'ÉQUILIBRE est bien utilisée à l'arrêt",
          "balance" in politiques["A"] and "balance" in politiques["D"])
    check("La politique de MARCHE est bien utilisée en marche",
          "walk" in politiques["C"])
    print(f"\n  locomotion : {loco.stats}")
    check("Aucune consigne refusée par le garde-fou",
          loco.stats["refused"] == 0)

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    if chute is None and FAIL == 0:
        print("\n  ✓ Le G1 29 DOF marche en posture de port : la voie du Graal est ouverte")
    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
