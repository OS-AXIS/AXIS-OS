"""
Mesure le coût RÉEL d'un pas de SetpointFollower contre Isaac Sim.

Répond à la question posée sans la supposer : le chemin normal
(interpolation -> move_arms/move_legs -> JointSafetyGuard) est-il
assez rapide pour servir une politique à haute fréquence, ou faut-il
vraiment un chemin séparé ?

Si le temps moyen par pas est nettement sous le budget de step_s, la
réponse est non : aucun chemin rapide n'est nécessaire, et l'arbitrage
de sécurité qu'il impliquerait n'a pas lieu d'être.

Teste aussi l'échéance de fraîcheur : on arrête d'alimenter le
suiveur et on vérifie qu'il passe HOLD puis DAMPING, sans jamais
continuer vers l'ancienne cible.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \
      /isaac-sim/axis_os/test_isaac_follower_latency.py
"""
import sys
import time

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)
from axis_os.hal.drivers.unitree_g1.robot import _build_g1_joint_definitions
from axis_os.services.policy.setpoint_follower import (
    SetpointFollower, FollowerState,
)

KP_SIM = 40.0        # robot.py:77 — position_delta_max = torque_max / 40
STEP_S = 0.02        # 50 Hz
N_TRACKING_TICKS = 200


class _DirectCtx:
    """Contexte minimal branché sur le HAL, sans sandbox ni permissions.

    On mesure ici le coût du chemin INTERPOLATION + GARDE. Le surcoût
    du sandbox (deux Queue inter-process) est mesuré séparément par la
    suite S5 — le mélanger ici masquerait ce qu'on cherche."""

    def __init__(self, hal, bridge):
        self._hal = hal
        self._bridge = bridge

    def get_full_state(self):
        return self._hal.get_full_state()

    def move_arms(self, cmds):
        return self._hal.send_joint_commands(cmds)

    def move_legs(self, cmds):
        return self._hal.send_joint_commands(cmds)

    def log(self, msg):
        print(f"    [ctx] {msg}")


def main():
    simulation_app = start_simulation_app(headless=True)
    bridge = IsaacSimBridge()
    bridge.setup(fix_base=True)
    bridge.set_holding_gains()
    hal = bridge.build_hal()
    hal.initialize_all()

    delta_max = {
        d.joint_id: d.limits.torque_max / KP_SIM
        for d in _build_g1_joint_definitions()
    }
    print(f"\ndelta_max le plus serré : {min(delta_max.values()):.3f} rad")

    ctx = _DirectCtx(hal, bridge)
    follower = SetpointFollower(ctx, delta_max, step_s=STEP_S,
                                max_age_s=0.5, damping_after_s=1.5)

    # --- 1. suivi normal : consigne rafraîchie en continu --------------
    print(f"\n=== 1. Suivi ({N_TRACKING_TICKS} pas à {1/STEP_S:.0f} Hz) ===")
    target = {j: 0.2 for j in delta_max if "shoulder" in j or "elbow" in j}
    for i in range(N_TRACKING_TICKS):
        follower.set_target(target, timestamp_ns=time.monotonic_ns())
        state = follower.tick()
        bridge.step(1)
        if state is FollowerState.FAULT:
            print(f"    FAULT au pas {i}")
            break

    print(f"\n  {follower.report()}")
    budget_ms = STEP_S * 1000
    mean = follower.stats.mean_tick_ms
    print(f"\n  VERDICT LATENCE : moyenne {mean:.2f} ms contre un budget "
          f"de {budget_ms:.1f} ms")
    if mean < budget_ms * 0.5:
        print("  -> le chemin normal tient largement. Aucun chemin rapide "
              "séparé n'est justifié.")
    elif mean < budget_ms:
        print("  -> ça tient, mais la marge est mince. Mesurer à nouveau "
              "avec le sandbox (S5) avant de conclure.")
    else:
        print("  -> le chemin normal ne tient PAS ce budget. C'est ici, et "
              "seulement ici, qu'un arbitrage devient nécessaire — et il "
              "devra être discuté, pas contourné.")

    # --- 2. échéance de fraîcheur : on arrête d'alimenter -------------
    print(f"\n=== 2. Échéance de fraîcheur (plus aucune consigne) ===")
    far_target = {j: 1.2 for j in target}   # cible ÉLOIGNÉE
    follower.set_target(far_target, timestamp_ns=time.monotonic_ns())
    follower.state = FollowerState.TRACKING

    seen = []
    pos_at_stale = None
    for i in range(150):     # 3 s à 50 Hz, sans jamais rafraîchir
        st = follower.tick()
        bridge.step(1)
        if st.name not in seen:
            seen.append(st.name)
            sample = next(iter(far_target))
            print(f"    t={i*STEP_S:.2f}s -> {st.name} "
                  f"(pos {bridge.read_position(sample):+.3f})")
            if st is FollowerState.HOLD:
                pos_at_stale = bridge.read_position(sample)
        time.sleep(0.001)

    sample = next(iter(far_target))
    final = bridge.read_position(sample)
    print(f"\n  États traversés : {' -> '.join(seen)}")
    print(f"  Position finale {final:+.3f} (cible périmée : +1.200)")

    ok = "HOLD" in seen and "DAMPING" in seen
    if pos_at_stale is not None:
        drift = abs(final - pos_at_stale)
        print(f"  Dérive après péremption : {drift:.4f} rad")
        ok = ok and drift < 0.05

    print(f"\n  VERDICT FRAÎCHEUR : {'OK' if ok else 'ÉCHEC'} — le suiveur "
          f"{'a bien cessé' if ok else 'a CONTINUÉ'} de progresser vers la "
          f"cible périmée.")

    simulation_app.close()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
