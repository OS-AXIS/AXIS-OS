"""
Suite S5.1 — pile complète : PlayStoreRuntime -> sandbox -> HALBridgeServer
-> HAL -> Isaac Sim, avec une VRAIE app AXIS-OS.

Dans docs/campagne_validation_isaac.md, la suite S5.1 fait passer les trois
apps de référence de bout en bout. Ce script couvre PackageSortingApp (tri
de colis) ; test_s5_security_patrol.py et test_s5_warehouse_ops.py couvrent
les deux autres.

Construction de la pile, telle que l'attend le code du dépôt :
  - PlayStoreRuntime(robot, watchdog) reçoit un NormalizedRobotHAL et un
    WatchdogSystem, construits autour du HAL que rend bridge.build_hal(),
    et non ce HAL seul ;
  - app.manifest() est une méthode de RobotApp, pas un attribut : on écrit
    app.manifest().app_id, et non app.manifest.app_id.

Pourquoi le quota mémoire est relevé à 8192 Mo, et par une vraie
sous-classe (PackageSortingAppBigQuota, plus bas) plutôt qu'en remplaçant
une méthode sur l'instance : deux problèmes rencontrés à l'exécution.
  1. Avec le quota par défaut (256 Mo, voir ResourceQuota), le sandbox
     signalait à chaque essai un dépassement (état VIOLATED, erreur
     MEMORY_QUOTA). psutil mesure la mémoire résidente
     (memory_info().rss) du sous-processus de l'app. Or ce sous-processus
     est créé par fork() : il hérite de toute la mémoire qu'Isaac Sim a
     déjà chargée dans le processus parent (plusieurs Go). Ce n'est donc
     pas l'app qui consomme trop, mais un artefact de mesure propre à ce
     test. 2048 Mo ne suffisaient pas ; 8192 Mo suffisent.
  2. Remplacer une méthode sur l'instance (app.manifest = lambda: …)
     empêche pickle (le format de Python qui transforme un objet en
     octets) de sérialiser l'app. Or SandboxRunner sérialise l'app pour
     la transmettre au sous-processus : la mission échouait alors (état
     CRASHED, erreur SANDBOX_ERROR) sans message clair. Depuis,
     SandboxRunner vérifie la sérialisation avant de lancer l'app et
     renvoie une erreur claire (voir axis_os/store/runtime/sandbox.py).
     Une sous-classe ordinaire, définie au niveau du module, se
     sérialise sans problème. C'est la
     même famille de problème que dans axis_os/store/runtime/packaging.py,
     où la classe créée par exec() vivait dans un espace de noms
     temporaire que pickle ne savait pas retrouver.

Répartition des threads (le point critique de ce script) :

  thread principal  : MainThreadSimPump.run_forever(), seul endroit où
                      world.step() et apply_action() sont appelés. Isaac
                      Sim bloque dans du code C SANS relâcher le GIL (le
                      verrou global de l'interpréteur Python) : appelé
                      depuis un autre thread, il gèle tout le processus.

  thread "mission"  : PlayStoreRuntime.start_app() puis run_mission(),
                      lancés par run_mission_in_thread(). Le thread
                      principal NE DOIT PAS exécuter ce code, sinon il ne
                      peut plus vider la file d'attente de la pompe.

  thread du pont    : HALBridgeServer._loop() (créé par AXIS-OS)
                      -> send_joint_commands(), remplacé par le pont
                      (voir IsaacSimBridge.build_hal()) -> pump.submit(),
                      qui dépose les commandes et attend. Ce thread ne
                      touche jamais Isaac Sim.

  sous-processus    : l'app, dans son sandbox. Elle n'est pas modifiée
                      et ne sait rien de tout cela.

RÉSULTAT CONFIRMÉ (2026-09-19) : mission réussie de bout en bout,
5/5 colis triés, facturation calculée correctement. Premier test de bout
en bout d'une vraie app AXIS-OS, à travers tout le sandboxing réel,
contre un G1 physiquement simulé.

Lors de ce premier essai, JointSafetyGuard a aussi détecté en cours de
mission 3 vraies violations POSITION_DELTA_LIMIT : des sauts de bras de
PackageSortingApp qui dépassaient, en un seul appel, l'écart permis
(position_delta_max, 0.625 rad pour les bras). Le garde-fou a refusé ces
commandes et la mission a continué malgré elles, comme voulu. L'app a été
corrigée depuis : ses mouvements de travail passent par move_to_pose(),
qui avance par petits pas (voir CHANGELOG.md, « fix(package_sorting) »,
et docs/isaac_lab_unitree_sim_status.md, « Point 1 »). Ce même document
annonce depuis les trois apps de référence sans aucune violation de
sécurité (« Point 2 »).
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
    """Même app, quota mémoire relevé pour ce test précis. La docstring du
    module donne la justification complète : un artefact de mesure, dû
    au fork() du sous-processus de l'app depuis un processus Isaac Sim
    déjà lourd, et non une app gourmande. Définie au niveau du module
    pour que pickle puisse la sérialiser, ce que ne permet pas une
    méthode remplacée sur l'instance. Jamais utilisée en dehors de cette
    suite de validation."""

    def manifest(self):
        m = super().manifest()
        m.quota = dataclasses.replace(m.quota, max_memory_mb=8192)
        return m


def main():
    simulation_app = start_simulation_app(headless=True)

    bridge = IsaacSimBridge()
    # Base libre (non épinglée au monde) : dynamique du corps entier,
    # comme pour les suites S4 (arrêt d'urgence) et S5 (pile complète et
    # endurance) de docs/campagne_validation_isaac.md.
    bridge.setup(fix_base=False)
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
    from axis_os.store.runtime.profil import Profil

    robot = NormalizedRobotHAL(hal, build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    runtime = PlayStoreRuntime(robot, wd, confirmer_permissions=accepter_tout_SIMULATION,
                               profil=Profil.SIMULATION)

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
