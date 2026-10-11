"""
Suite L3 — le service de locomotion, et une app qui PLANTE en pleine marche.

Scénario, comme une app réelle :
  1. 2 s debout (aucune consigne)
  2. 4 s de marche : l'app rafraîchit « 0.5 m/s » à chaque pas
  3. l'app PLANTE : plus aucun rafraîchissement

Attendu : après command_timeout_s (0.5 s), la consigne retombe à zéro,
le robot S'ARRÊTE ET RESTE DEBOUT. Il ne marche pas indéfiniment, et il
ne s'effondre pas non plus — deux échecs opposés, tous deux inacceptables.

Deux détails, chacun corrigeant un piège :
  - la source d'état convertit la vitesse angulaire monde -> robot : c'est
    désormais la SOURCE qui convertit, jamais le consommateur ;
  - l'horloge du service est celle de la SIMULATION : avec l'horloge
    murale, un simulateur plus lent que le temps réel ferait expirer les
    consignes trop tôt, et un plus rapide trop tard.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_l3_service.py
"""
import sys
from pathlib import Path

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)

POLICY = Path("/isaac-sim/policies/g1_motion.pt")
STAND_S, WALK_S, CRASH_S = 2.0, 4.0, 3.0
FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    from axis_os.hal.core.types import ActuatorCommand, ControlMode
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        G1LocomotionAdapter, G1LocomotionConfig, G1LocomotionPolicy,
        body_angular_velocity,
    )
    from axis_os.hal.drivers.unitree_g1.robot import _build_g1_joint_definitions
    from axis_os.services.locomotion.service import (
        BaseState, LocomotionService, LocomotionState,
    )
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

    world.reset()
    robot.get_articulation_controller().set_gains(kps=kps, kds=kds)
    q0 = np.array([pol.default_pose()[j] for j in names])
    robot.set_joint_positions(q0)
    robot.set_joint_velocities(np.zeros_like(q0))
    robot.apply_action(ArticulationAction(joint_positions=q0))

    sim_t = [0.0]

    def read_state() -> BaseState:
        _pos, quat = robot.get_world_pose()
        omega_world = robot.get_angular_velocity()
        jp, jv = robot.get_joint_positions(), robot.get_joint_velocities()
        # LA SOURCE CONVERTIT : Isaac Sim rend le repère du monde,
        # BaseState exige celui du corps.
        omega_body = body_angular_velocity(quat, omega_world)
        return BaseState(tuple(float(v) for v in quat),
                         tuple(float(v) for v in omega_body),
                         {j: float(jp[idx[j]]) for j in names},
                         {j: float(jv[idx[j]]) for j in names})

    def send_targets(targets):
        return hal.send_joint_commands(
            {j: ActuatorCommand(mode=ControlMode.POSITION, position=v)
             for j, v in targets.items()})

    svc = LocomotionService(G1LocomotionAdapter(pol), read_state, send_targets,
                            command_timeout_s=0.5, clock=lambda: sim_t[0])
    check("Service démarré", svc.start().name == "OK")

    x_at = {}
    fell = None
    total = STAND_S + WALK_S + CRASH_S
    k = 0
    while sim_t[0] < total:
        t = sim_t[0]
        # L'APP : rafraîchit sa consigne pendant la phase de marche
        # uniquement, puis « plante » et ne dit plus rien.
        if STAND_S <= t < STAND_S + WALK_S:
            svc.set_velocity(0.5, 0.0, 0.0)
        r = svc.tick()
        if svc.state is LocomotionState.FALLEN:
            fell = t
            break
        for _ in range(cfg.decimation - 1):      # send a déjà fait 1 pas
            world.step(render=False)
        sim_t[0] += cfg.control_dt
        k += 1
        pos, _ = robot.get_world_pose()
        for mark in (STAND_S, STAND_S + WALK_S, STAND_S + WALK_S + 1.0, total):
            if mark not in x_at and sim_t[0] >= mark - 1e-9:
                x_at[mark] = float(pos[0])

    print()
    check(f"Aucune chute sur {total:.0f} s (chute à {fell} s)" if fell
          else f"Aucune chute sur {total:.0f} s", fell is None)
    if fell is None:
        marche = x_at[STAND_S + WALK_S] - x_at[STAND_S]
        derive = x_at[total] - x_at[STAND_S + WALK_S + 1.0]
        print(f"\n  pendant la marche ({WALK_S:.0f} s) : {marche:+.2f} m")
        print(f"  après le plantage, dernières {CRASH_S - 1:.0f} s : {derive:+.2f} m")
        check(f"L'app marche : plus d'1 m en {WALK_S:.0f} s ({marche:.2f} m)",
              marche > 1.0)
        check(f"Après le plantage, le robot S'ARRÊTE ({derive:+.2f} m sur "
              f"{CRASH_S - 1:.0f} s) — il ne marche pas indéfiniment",
              abs(derive) < 0.3)
        check("Et il RESTE DEBOUT : un arrêt, pas un effondrement",
              svc.state is LocomotionState.RUNNING)
    check(f"L'expiration de la consigne a été détectée "
          f"(stale={svc.stats['stale']})", svc.stats["stale"] >= 1)
    check(f"Aucune consigne refusée par le garde-fou "
          f"(refused={svc.stats['refused']})", svc.stats["refused"] == 0)

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    if FAIL == 0:
        print("\n  ✓ Une app pilote la marche par consigne de vitesse")
        print("  ✓ Si elle plante, le robot s'arrête ET reste debout")

    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
