"""
Suite S5.3 — surcoût réel du sandbox sur une commande.

Même architecture de threads que test_s5_full_stack.py (voir ce
fichier pour la documentation complète) : pompe sur le thread
principal, mission dans un thread annexe, app dans un sous-process.

CE QU'ON COMPARE :

  chemin direct   (test_isaac_follower_latency.py) : 5.41 ms / cycle
    thread principal -> hal.send_joint_commands -> JointSafetyGuard
                     -> apply_action + step

  chemin sandbox  (ce script)                      : à mesurer
    sous-process -> Queue -> HALBridgeServer (thread) -> RobotContext
                 -> NormalizedRobotHAL -> JointSafetyGuard
                 -> pump.submit() -> attente du thread principal
                 -> apply_action + step -> Event -> Queue -> retour app

La différence EST le coût de l'isolation. La question à trancher :
reste-t-il assez de marge sous le budget de 20 ms (50 Hz) pour qu'une
politique apprise pilote une app sandboxée sans dégrader la
vérification de sécurité ?

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \
      /isaac-sim/axis_os/test_s5_latency_probe.py
"""
import sys

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, MainThreadSimPump, start_simulation_app,
    run_mission_in_thread,
)
from axis_os.apps.latency_probe.app import LatencyProbeApp

BUDGET_MS = 20.0       # 50 Hz
DIRECT_MS = 5.41       # mesuré par test_isaac_follower_latency.py


def _fmt(label, s):
    return (f"    {label:22} moyenne {s['mean']:6.2f} ms | p50 {s['p50']:6.2f} "
            f"| p95 {s['p95']:6.2f} | max {s['max']:6.2f}")


def main():
    simulation_app = start_simulation_app(headless=True)

    bridge = IsaacSimBridge()
    bridge.setup(fix_base=True)   # on mesure un chemin, pas l'équilibre
    bridge.set_holding_gains()
    hal = bridge.build_hal()
    hal.initialize_all()

    pump = MainThreadSimPump(bridge)
    bridge.attach_pump(pump)

    from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
    from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
    from axis_os.hal.watchdog.system import WatchdogSystem
    from axis_os.store.runtime.play_store import PlayStoreRuntime

    robot = NormalizedRobotHAL(hal, build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    runtime = PlayStoreRuntime(robot, wd)

    def mission():
        app = LatencyProbeApp()
        manifest = app.manifest()
        runtime.install(app)
        runtime.start_app(manifest.app_id)
        return runtime.run_mission(manifest.app_id, params={})

    thread, holder = run_mission_in_thread(mission)

    print("[S5.3] pompe démarrée sur le thread principal")
    pump.run_forever(should_stop=lambda: holder["done"], max_seconds=240.0)
    thread.join(timeout=5.0)

    print("\n=== RÉSULTAT S5.3 — surcoût du sandbox ===")

    if holder["error"] is not None:
        print("ÉCHEC — exception dans la mission :")
        import traceback
        traceback.print_exception(type(holder["error"]), holder["error"],
                                  holder["error"].__traceback__)
        simulation_app.close()
        sys.exit(1)

    result = holder["result"]
    if result is None or not getattr(result, "success", False):
        print("ÉCHEC — mission non réussie :", result)
        # error_message porte l'exception réelle levée DANS le sandbox
        # (play_store.py y recopie sandbox_result.error). Sans cet
        # affichage, un APPLICATION_ERROR ne dit rien de sa cause et on
        # en est réduit à deviner.
        msg = getattr(result, "error_message", None) if result else None
        if msg:
            print(f"\n  CAUSE RÉELLE : {msg}")
        simulation_app.close()
        sys.exit(1)

    m = result.metrics
    print(f"\n  {m['n_calls']} cycles, {m['joints_per_call']} joints par cycle, "
          f"{m['rejected']} commande(s) refusée(s)\n")
    print(_fmt("get_full_state()", m["get_full_state_ms"]))
    print(_fmt("move_arms()", m["move_arms_ms"]))
    print(_fmt("cycle complet", m["total_per_cycle_ms"]))

    sandbox_ms = m["total_per_cycle_ms"]["mean"]
    p95 = m["total_per_cycle_ms"]["p95"]
    overhead = sandbox_ms - DIRECT_MS

    print(f"\n  Chemin direct (mesuré précédemment) : {DIRECT_MS:.2f} ms")
    print(f"  Chemin sandbox (ce test)            : {sandbox_ms:.2f} ms")
    print(f"  SURCOÛT DE L'ISOLATION              : {overhead:+.2f} ms "
          f"({overhead / DIRECT_MS * 100:+.0f} %)")

    print(f"\n  VERDICT : {sandbox_ms:.2f} ms de moyenne, {p95:.2f} ms au p95, "
          f"contre un budget de {BUDGET_MS:.0f} ms")
    if p95 < BUDGET_MS * 0.5:
        print("  -> large marge même au p95. Une politique apprise peut "
              "piloter une app sandboxée à 50 Hz sans rien dégrader.")
    elif p95 < BUDGET_MS:
        print("  -> ça tient, mais la marge est mince au p95. Viser 25-30 Hz "
              "plutôt que 50 Hz pour une app sandboxée, ou mesurer à nouveau "
              "sous charge réelle avant de conclure.")
    else:
        print("  -> le p95 DÉPASSE le budget. C'est ici, et seulement ici, "
              "qu'un arbitrage devient nécessaire — et il devra porter sur la "
              "fréquence de commande, jamais sur le contournement du garde.")

    simulation_app.close()


if __name__ == "__main__":
    main()
