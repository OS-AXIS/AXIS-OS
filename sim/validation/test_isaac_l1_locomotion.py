"""
Suite L1 — le G1 tient-il debout et marche-t-il avec la politique
officielle d'Unitree, dans Isaac Sim ?

La politique a été entraînée dans Isaac Gym et vérifiée dans MuJoCo.
Isaac Sim 6.1 a un autre moteur de contact : rien ne garantit qu'elle y
tienne. C'est précisément ce que ce test mesure.

DEUX PHASES, et leur comparaison est le vrai résultat — même méthode
différentielle que celle qui a débloqué la perception :

  A. BRUT     : consignes de la politique appliquées directement.
                Répond à : la politique fonctionne-t-elle dans Isaac Sim ?
  B. GARDÉ    : mêmes consignes, via JointSafetyGuard.
                Répond à : le garde-fou d'AXIS-OS la laisse-t-il marcher ?

  A tombe                 -> écart simulation/entraînement (physique,
                             gains, pas de temps…), pas un problème AXIS-OS
  A marche, B tombe       -> le garde-fou gêne la marche : à comprendre
                             AVANT tout essai réel (limites, chevilles…)
  A et B marchent         -> la marche passe par AXIS-OS sans raccourci

PRÉREQUIS :
  git clone --depth 1 https://github.com/unitreerobotics/unitree_rl_gym.git /tmp/rl_gym
  docker exec -u root isim-isaac-sim-1 mkdir -p /isaac-sim/policies
  docker cp /tmp/rl_gym/deploy/pre_train/g1/motion.pt \\
      isim-isaac-sim-1:/isaac-sim/policies/g1_motion.pt

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_l1_locomotion.py
"""
import sys
from pathlib import Path

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)

POLICY = Path("/isaac-sim/policies/g1_motion.pt")
STAND_S = 3.0
WALK_S = 5.0
WALK_CMD = (0.5, 0.0, 0.0)


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    from axis_os.hal.core.types import ActuatorCommand, ControlMode, HALStatus
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        G1LocomotionConfig, G1LocomotionPolicy, projected_gravity,
    )
    from isaacsim.core.utils.types import ArticulationAction

    cfg = G1LocomotionConfig()
    from axis_os.hal.drivers.unitree_g1.robot import _build_g1_joint_definitions
    # Bornage aux butées DANS L'ADAPTATEUR : premier essai, le garde-fou
    # a refusé 52 consignes de chevilles hors butée — sans effet sur la
    # marche (2.15 m contre 2.17 m) mais en noyant le journal de sécurité.
    limits = {d.joint_id: (d.limits.pos_min, d.limits.pos_max)
              for d in _build_g1_joint_definitions()}
    pol = G1LocomotionPolicy(POLICY, cfg, joint_limits=limits)
    pol.load()
    print(f"\n[politique] chargée, empreinte vérifiée, "
          f"physique {cfg.physics_dt*1000:.0f} ms, commande "
          f"{1/cfg.control_dt:.0f} Hz")

    bridge = IsaacSimBridge()
    # Base LIBRE (on marche !), meshes exigés (sans collision de pieds,
    # la marche n'a aucun sens), pas physique d'ENTRAÎNEMENT.
    bridge.setup(fix_base=False, require_meshes=True,
                 physics_dt=cfg.physics_dt)
    robot, world = bridge._robot, bridge._world
    idx = bridge._joint_index
    names = sorted(idx, key=lambda j: idx[j])

    # Gains de la politique — jamais des gains génériques : une
    # politique entraînée avec kp=150 au genou ne marche pas avec kp=80.
    g = pol.gains()
    kps = np.array([g[j][0] for j in names])
    kds = np.array([g[j][1] for j in names])
    robot.get_articulation_controller().set_gains(kps=kps, kds=kds)

    hal = bridge.build_hal()
    hal.initialize_all()

    def reset_pose():
        world.reset()
        robot.get_articulation_controller().set_gains(kps=kps, kds=kds)
        pose = pol.default_pose()
        q = np.array([pose[j] for j in names])
        robot.set_joint_positions(q)
        robot.set_joint_velocities(np.zeros_like(q))
        robot.apply_action(ArticulationAction(joint_positions=q))
        pol.reset()

    def read_state():
        pos, quat = robot.get_world_pose()
        omega = robot.get_angular_velocity()
        jp = robot.get_joint_positions()
        jv = robot.get_joint_velocities()
        return (np.asarray(pos), np.asarray(quat), np.asarray(omega),
                {j: float(jp[idx[j]]) for j in names},
                {j: float(jv[idx[j]]) for j in names})

    def run(label, guarded):
        reset_pose()
        upper = pol.upper_body_targets()
        rejected = {}
        start_pos = None
        fell_at = None
        min_z = 9.0
        steps_total = int((STAND_S + WALK_S) / cfg.control_dt)

        for k in range(steps_total):
            t = k * cfg.control_dt
            cmd = (0.0, 0.0, 0.0) if t < STAND_S else WALK_CMD
            pos, quat, omega, jp, jv = read_state()
            if start_pos is None:
                start_pos = pos.copy()
            if t >= STAND_S and k == int(STAND_S / cfg.control_dt):
                walk_start = pos.copy()

            # Chute : bassin trop bas, ou plus de 45° d'inclinaison.
            tilt_ok = projected_gravity(quat)[2] < -0.7
            min_z = min(min_z, float(pos[2]))
            if fell_at is None and (pos[2] < 0.45 or not tilt_ok):
                fell_at = t
                break

            targets = pol.step(quat, omega, jp, jv, cmd, ang_vel_frame="world")
            targets.update(upper)

            if guarded:
                cmds = {j: ActuatorCommand(mode=ControlMode.POSITION, position=v)
                        for j, v in targets.items()}
                st = hal.send_joint_commands(cmds)      # fait déjà 1 pas
                for j, s in st.items():
                    if s != HALStatus.OK:
                        rejected[j] = rejected.get(j, 0) + 1
                for _ in range(cfg.decimation - 1):
                    world.step(render=False)
            else:
                q = np.array([targets[j] for j in names])
                robot.apply_action(ArticulationAction(joint_positions=q))
                for _ in range(cfg.decimation):
                    world.step(render=False)

        pos, *_ = read_state()
        walked = float(pos[0] - walk_start[0]) if fell_at is None else None
        print(f"\n=== {label} ===")
        if fell_at is not None:
            print(f"  CHUTE à t={fell_at:.2f} s "
                  f"({'debout' if fell_at < STAND_S else 'en marchant'})")
        else:
            print(f"  debout {STAND_S:.0f} s puis marche {WALK_S:.0f} s : AUCUNE CHUTE")
            print(f"  distance parcourue en marche : {walked:+.2f} m "
                  f"(consigne {WALK_CMD[0]} m/s -> idéal {WALK_CMD[0]*WALK_S:.1f} m)")
        print(f"  hauteur minimale du bassin : {min_z:.3f} m")
        if pol.clamp_counts:
            print(f"  consignes BORNÉES par l'adaptateur : {pol.clamp_counts}")
        else:
            print(f"  consignes bornées par l'adaptateur : aucune")
        if guarded:
            if rejected:
                print(f"  commandes REFUSÉES par le garde-fou : {rejected}")
            else:
                print(f"  commandes refusées par le garde-fou : aucune")
        return fell_at, walked, rejected

    fa, wa, _ = run("A — BRUT (politique seule)", guarded=False)
    fb, wb, rb = run("B — GARDÉ (via JointSafetyGuard)", guarded=True)

    print(f"\n{'=' * 66}\n  VERDICT\n{'=' * 66}")
    ok_a = fa is None and wa is not None and wa > 1.0
    ok_b = fb is None and wb is not None and wb > 1.0
    if not ok_a:
        print("  A échoue : la politique ne tient pas dans Isaac Sim tel quel.")
        print("  -> écart simulation/entraînement (physique, contact, gains,")
        print("     pas de temps), PAS un problème d'AXIS-OS. À investiguer")
        print("     avant de conclure quoi que ce soit sur le garde-fou.")
    elif not ok_b:
        print("  A marche, B NON : le garde-fou empêche la marche.")
        print("  -> à comprendre AVANT tout essai réel. Voir les refus ci-dessus.")
    else:
        print("  A et B marchent : la marche passe par AXIS-OS, sans raccourci.")
        if rb:
            print(f"  ATTENTION : le garde-fou refuse encore {sum(rb.values())} "
                  f"consigne(s) malgré le bornage — une limite autre que la "
                  f"position est en cause.")
        else:
            print("  Aucune consigne refusée : le journal de sécurité ne contient")
            print("  plus que ce qui mérite d'y être.")

    simulation_app.close()
    sys.exit(0 if (ok_a and ok_b) else 1)


if __name__ == "__main__":
    main()
