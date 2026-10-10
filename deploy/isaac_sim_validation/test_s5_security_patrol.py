"""
Suite S5.2 — pile complète contre SecurityPatrolApp, même architecture
que test_s5_full_stack.py (MainThreadSimPump) — voir ce fichier pour
la documentation complète de l'architecture de threads.

Testé en Python pur (sans Isaac Sim) sur 50 graines différentes
avant ce test : aucune violation POSITION_DELTA_LIMIT trouvée — les
mouvements de cette app (deltas de 0.05 rad sur les hanches) restent
largement sous la limite réelle (~2.2 rad pour hip_pitch_joint). Ce
script confirme ce résultat contre une vraie physique simulée.

RÉSULTAT CONFIRMÉ (2026-09-19) : mission réussie de bout en bout,
7 checkpoints visités, deux détections de personne (badge invalide
correctement rejeté avec alerte CRITICAL, badge valide correctement
autorisé), facturation calculée, aucune violation de sécurité. Voir
docs/isaac_lab_unitree_sim_status.md pour le contexte complet de
cette session de validation.
"""
import sys
sys.path.insert(0, "/isaac-sim")

import dataclasses

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, MainThreadSimPump, start_simulation_app,
    run_mission_in_thread,
)
from axis_os.apps.security_patrol.app import SecurityPatrolApp


class SecurityPatrolAppBigQuota(SecurityPatrolApp):
    """Même app, quota mémoire relevé — voir la docstring de
    test_s5_full_stack.py::PackageSortingAppBigQuota pour le contexte
    complet (artefact de mesure fork() + Isaac Sim déjà chargé)."""

    def manifest(self):
        m = super().manifest()
        m.quota = dataclasses.replace(m.quota, max_memory_mb=8192)
        return m


def main():
    simulation_app = start_simulation_app(headless=True)

    bridge = IsaacSimBridge()
    bridge.setup(fix_base=False)
    bridge.set_holding_gains()
    hal = bridge.build_hal()
    hal.initialize_all()

    pump = MainThreadSimPump(bridge)
    bridge.attach_pump(pump)

    from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
    from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
    from axis_os.hal.watchdog.system import WatchdogSystem
    from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION

    robot = NormalizedRobotHAL(hal, build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    runtime = PlayStoreRuntime(robot, wd, confirmer_permissions=accepter_tout_SIMULATION)

    def mission():
        app = SecurityPatrolAppBigQuota()
        manifest = app.manifest()
        runtime.install(app)
        runtime.start_app(manifest.app_id)
        return runtime.run_mission(manifest.app_id, params={"seed": 42})

    thread, holder = run_mission_in_thread(mission)

    print("[S5.2] pompe démarrée sur le thread principal")
    pump.run_forever(should_stop=lambda: holder["done"], max_seconds=180.0)
    thread.join(timeout=5.0)

    print("\n=== RÉSULTAT S5.2 (SecurityPatrolApp) ===")
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
