"""
Suite S5.1 — pile complète : PlayStoreRuntime -> sandbox -> HALBridgeServer
-> HAL -> Isaac Sim, avec une VRAIE app AXIS-OS.

Corrigé par rapport au squelette initial (signatures réelles vérifiées
dans le dépôt, pas supposées) :
  - PlayStoreRuntime(robot, watchdog) — pas PlayStoreRuntime(hal) seul,
    il faut un NormalizedRobotHAL + un WatchdogSystem autour du hal
    que bridge.build_hal() retourne.
  - app.manifest() est une MÉTHODE (pas un attribut) sur RobotApp —
    app.manifest().app_id, pas app.manifest.app_id.

STRUCTURE DE THREADS (le point critique de ce script) :

  thread principal  : MainThreadSimPump.run_forever()
                      seul endroit où world.step()/apply_action() sont
                      appelés. Isaac Sim bloque dans du C SANS relâcher
                      le GIL — appelé ailleurs, il gèle le process entier.

  thread "mission"  : PlayStoreRuntime.start_app() / run_mission()
                      lancé via run_mission_in_thread(). Le thread
                      principal NE DOIT PAS entrer là-dedans, sinon il
                      ne peut plus vider la queue de la pompe.

  thread du pont    : HALBridgeServer._loop() (créé par AXIS-OS)
                      -> send_joint_commands patché -> pump.submit()
                      dépose et attend. Ne touche jamais Isaac Sim.

  sous-process      : l'app sandboxée. Inchangée, elle ne sait rien de
                      tout ça.
"""
import sys
sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, MainThreadSimPump, start_simulation_app,
    run_mission_in_thread,
)


def main():
    simulation_app = start_simulation_app(headless=True)

    bridge = IsaacSimBridge()
    bridge.setup(fix_base=False)  # dynamique corps entier, comme S4/S5
    bridge.set_holding_gains()
    hal = bridge.build_hal()
    hal.initialize_all()

    # Activer le mode multi-thread AVANT de lancer quoi que ce soit.
    pump = MainThreadSimPump(bridge)
    bridge.attach_pump(pump)

    # --- construire la vraie pile AXIS-OS (signatures vérifiées) -----
    from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
    from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
    from axis_os.hal.watchdog.system import WatchdogSystem
    from axis_os.store.runtime.play_store import PlayStoreRuntime
    from axis_os.apps.package_sorting.app import PackageSortingApp

    robot = NormalizedRobotHAL(hal, build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    runtime = PlayStoreRuntime(robot, wd)

    def mission():
        app = PackageSortingApp()
        manifest = app.manifest()  # méthode, pas attribut
        runtime.install(app)
        runtime.start_app(manifest.app_id)
        return runtime.run_mission(manifest.app_id, params={})

    thread, holder = run_mission_in_thread(mission)

    # --- le thread principal fait tourner la simulation --------------
    print("[S5] pompe démarrée sur le thread principal")
    pump.run_forever(should_stop=lambda: holder["done"], max_seconds=180.0)
    thread.join(timeout=5.0)

    print("\n=== RÉSULTAT S5.1 ===")
    if holder["error"] is not None:
        print("ÉCHEC — exception dans la mission :")
        import traceback
        traceback.print_exception(type(holder["error"]), holder["error"],
                                  holder["error"].__traceback__)
        simulation_app.close()
        sys.exit(1)

    print("Mission terminée :", holder["result"])
    simulation_app.close()


if __name__ == "__main__":
    main()
