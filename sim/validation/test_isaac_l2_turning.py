"""
Suite L2 — virage, et expérience TÉMOIN sur la conversion de repère.

L'affirmation à vérifier : « sans conversion du repère de la vitesse
angulaire (monde -> robot), le robot tombe dès qu'il tourne ». Elle a
motivé une correction faite AVANT le premier essai de marche — donc
jamais démontrée. La marche en ligne droite ne pouvait pas la tester :
tant que le robot regarde droit devant, les deux repères coïncident.

Deux phases, même consigne de virage (0.3 m/s en avançant, 0.6 rad/s) :

  AVEC conversion  : le code tel qu'il est
  SANS conversion  : on fournit délibérément à la politique la vitesse
                     angulaire dans le repère du MONDE

  AVEC tient, SANS tombe   -> la correction était indispensable : prouvé
  les deux tiennent         -> l'affirmation était exagérée, à corriger
                               dans docs/validation_locomotion.md ; la
                               correction reste juste mais n'était pas
                               critique ici
  AVEC tombe                -> problème de virage indépendant du repère

Un test qui ne peut pas détecter le bug qu'il prétend couvrir ne prouve
rien : c'est pourquoi la phase SANS existe.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_l2_turning.py
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)

POLICY = Path("/isaac-sim/policies/g1_motion.pt")
STAND_S = 2.0
TURN_S = 8.0
TURN_CMD = (0.3, 0.0, 0.6)          # ~4.8 rad : plus d'un demi-tour


def yaw_of(q):
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    from axis_os.hal.core.types import ActuatorCommand, ControlMode, HALStatus
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        G1LocomotionConfig, G1LocomotionPolicy, projected_gravity,
        quat_wxyz_to_matrix,
    )
    from axis_os.hal.drivers.unitree_g1.robot import _build_g1_joint_definitions
    from isaacsim.core.utils.types import ArticulationAction

    cfg = G1LocomotionConfig()
    limits = {d.joint_id: (d.limits.pos_min, d.limits.pos_max)
              for d in _build_g1_joint_definitions()}
    pol = G1LocomotionPolicy(POLICY, cfg, joint_limits=limits)
    pol.load()

    bridge = IsaacSimBridge()
    bridge.setup(fix_base=False, require_meshes=True, physics_dt=cfg.physics_dt)
    robot, world = bridge._robot, bridge._world
    idx = bridge._joint_index
    names = sorted(idx, key=lambda j: idx[j])
    g = pol.gains()
    kps = np.array([g[j][0] for j in names])
    kds = np.array([g[j][1] for j in names])
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

    def run(label, convert):
        reset_pose()
        upper = pol.upper_body_targets()
        fell_at, yaw_total, prev_yaw = None, 0.0, None
        rejected = 0
        steps = int((STAND_S + TURN_S) / cfg.control_dt)
        for k in range(steps):
            t = k * cfg.control_dt
            cmd = (0.0, 0.0, 0.0) if t < STAND_S else TURN_CMD
            pos, quat = robot.get_world_pose()
            omega = np.asarray(robot.get_angular_velocity())
            jp, jv = robot.get_joint_positions(), robot.get_joint_velocities()

            yaw = yaw_of(quat)
            if prev_yaw is not None:
                d = yaw - prev_yaw
                d = (d + math.pi) % (2 * math.pi) - math.pi   # déroulement
                if t >= STAND_S:
                    yaw_total += d
            prev_yaw = yaw

            if pos[2] < 0.45 or projected_gravity(quat)[2] > -0.7:
                fell_at = t
                break

            if not convert:
                # EXPÉRIENCE TÉMOIN : on annule la conversion en
                # pré-multipliant par R, pour que R^T·R·ω = ω_monde. La
                # politique reçoit alors la vitesse angulaire dans le
                # repère du MONDE — le bug qu'on prétend avoir évité.
                omega = quat_wxyz_to_matrix(quat) @ omega

            targets = pol.step(quat, omega,
                               {j: float(jp[idx[j]]) for j in names},
                               {j: float(jv[idx[j]]) for j in names}, cmd,
                               ang_vel_frame="world")
            targets.update(upper)
            cmds = {j: ActuatorCommand(mode=ControlMode.POSITION, position=v)
                    for j, v in targets.items()}
            st = hal.send_joint_commands(cmds)
            rejected += sum(1 for s in st.values() if s != HALStatus.OK)
            for _ in range(cfg.decimation - 1):
                world.step(render=False)

        print(f"\n=== {label} ===")
        if fell_at is not None:
            phase = "debout" if fell_at < STAND_S else "en tournant"
            print(f"  CHUTE à t={fell_at:.2f} s ({phase}), après "
                  f"{math.degrees(yaw_total):.0f}° de virage")
        else:
            print(f"  AUCUNE CHUTE — virage total {math.degrees(yaw_total):.0f}° "
                  f"(consigne ~{math.degrees(TURN_CMD[2]*TURN_S):.0f}°)")
        print(f"  commandes refusées par le garde-fou : {rejected}")
        return fell_at, yaw_total

    fa, ya = run("AVEC conversion de repère (code réel)", convert=True)
    fb, yb = run("SANS conversion (témoin : le bug évité)", convert=False)

    print(f"\n{'=' * 66}\n  VERDICT\n{'=' * 66}")
    avec_ok = fa is None and abs(ya) > math.radians(90)
    sans_ok = fb is None and abs(yb) > math.radians(90)
    if not avec_ok:
        print("  Le virage échoue MÊME avec la conversion : problème de")
        print("  virage indépendant du repère, à investiguer.")
    elif not sans_ok:
        print("  AVEC tient, SANS tombe : la conversion était INDISPENSABLE.")
        print("  L'affirmation est démontrée, plus seulement raisonnée.")
    else:
        print("  Les deux tiennent : mon affirmation « le robot tombe dès qu'il")
        print("  tourne » était EXAGÉRÉE pour ce virage. La conversion reste")
        print("  correcte, mais n'était pas critique ici — documentation à")
        print("  corriger en conséquence.")

    simulation_app.close()
    sys.exit(0 if avec_ok else 1)


if __name__ == "__main__":
    main()
