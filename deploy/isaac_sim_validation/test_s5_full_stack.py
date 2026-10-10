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

Quota mémoire relevé à 8192 Mo via une VRAIE sous-classe (jamais un
patch de méthode d'instance) — deux problèmes réels rencontrés en
exécution avant d'arriver à cette version :
  1. Le quota par défaut (256 Mo, ResourceQuota) déclenche
     systématiquement VIOLATED (MEMORY_QUOTA) dans ce contexte précis
     — psutil mesure memory_info().rss du sous-process, qui HÉRITE
     par fork() de toute la mémoire déjà chargée par Isaac Sim dans
     le process parent (plusieurs Go). Ce n'est pas un vrai
     dépassement causé par l'app elle-même, un artefact de mesure
     propre à ce contexte de test — 2048 Mo restait insuffisant,
     8192 Mo suffit.
  2. Un patch de méthode d'instance (app.manifest = lambda: ...)
     rend l'instance NON PICKLABLE — SandboxRunner sérialise l'app
     pour la transmettre au sous-process, et ce patch fait échouer
     silencieusement cette sérialisation (CRASHED/SANDBOX_ERROR,
     sans message clair). Une sous-classe normale, définie au niveau
     module (voir PackageSortingAppBigQuota ci-dessous), reste
     parfaitement picklable — même famille de problème que celle déjà
     rencontrée dans axis_os/store/runtime/packaging.py (classe créée
     par exec() dans un namespace transitoire).

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

RÉSULTAT CONFIRMÉ (2026-09-19) : mission réussie de bout en bout,
5/5 colis triés, 3 vraies violations POSITION_DELTA_LIMIT détectées
par JointSafetyGuard EN COURS DE MISSION (mission continue malgré la
violation, comportement voulu — voir docs/isaac_lab_unitree_sim_status.md),
facturation calculée correctement. Premier test de bout en bout d'une
vraie app AXIS-OS, à travers tout le sandboxing réel, contre un G1
physiquement simulé.
"""
import sys
sys.path.insert(0, "/isaac-sim")

import dataclasses

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, MainThreadSimPump, start_simulation_app,
    run_mission_in_thread,
)
from axis_os.apps.package_sorting.app import PackageSortingApp


class PackageSortingAppBigQuota(PackageSortingApp):
    """Même app, quota mémoire relevé pour ce test précis — voir la
    docstring de module pour la justification complète (artefact de
    mesure lié au fork() d'un sous-process depuis un process Isaac Sim
    déjà lourd, pas un vrai comportement gourmand de l'app). Définie
    au niveau module pour rester picklable — jamais un patch de
    méthode d'instance. Jamais utilisée en dehors de cette suite de
    validation."""

    def manifest(self):
        m = super().manifest()
        m.quota = dataclasses.replace(m.quota, max_memory_mb=8192)
        return m


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
    from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION

    robot = NormalizedRobotHAL(hal, build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    runtime = PlayStoreRuntime(robot, wd, confirmer_permissions=accepter_tout_SIMULATION)

    def mission():
        app = PackageSortingAppBigQuota()
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
