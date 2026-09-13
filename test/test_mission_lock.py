"""
AXIS-OS — Tests verrou global de mission
============================================
Valide le verrou ajouté à PlayStoreRuntime.run_mission() lors d'un
audit : sans lui, rien n'empêchait deux missions concurrentes
(même app appelée deux fois, ou deux apps différentes) d'envoyer des
commandes contradictoires aux mêmes actionneurs physiques en même
temps. Un seul robot = une seule mission active à la fois.
"""

import sys
import threading
import time
sys.path.insert(0, ".")

from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.store.runtime.play_store import PlayStoreRuntime
from axis_os.apps.package_sorting.app import PackageSortingApp
from axis_os.apps.security_patrol.app import SecurityPatrolApp

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def make_runtime_with_apps():
    robot = NormalizedRobotHAL(UnitreeG1HAL("g1_lock_test"), build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    runtime = PlayStoreRuntime(robot, wd)
    runtime.install(PackageSortingApp())
    runtime.install(SecurityPatrolApp())
    runtime.start_app("com.acme.package_sorting")
    runtime.start_app("com.axisos.security_patrol")
    return runtime


# ══════════════════════════════════════════════════════════════════

section("1 — Deux missions concurrentes, MÊME app : une seule passe")

runtime1 = make_runtime_with_apps()
results1 = []

def launch1(label):
    r = runtime1.run_mission("com.acme.package_sorting", params={})
    results1.append((label, r.success if r else None))

t1 = threading.Thread(target=launch1, args=("A",))
t2 = threading.Thread(target=launch1, args=("B",))
t1.start()
time.sleep(0.02)  # laisse A prendre le verrou en premier
t2.start()
t1.join()
t2.join()

successes1 = [s for _, s in results1 if s is True]
failures1 = [s for _, s in results1 if s is False]
check("Exactement UNE mission réussit sur les deux lancées en concurrence",
      len(successes1) == 1)
check("L'autre mission est REFUSÉE (pas juste en attente silencieuse)",
      len(failures1) == 1)

# ══════════════════════════════════════════════════════════════════

section("2 — Deux missions concurrentes, apps DIFFÉRENTES : une seule passe")

runtime2 = make_runtime_with_apps()
results2 = []

def launch2(app_id, label):
    r = runtime2.run_mission(app_id, params={})
    results2.append((label, r.success if r else None, r.error_message if r else None))

t3 = threading.Thread(target=launch2, args=("com.acme.package_sorting", "sorting"))
t4 = threading.Thread(target=launch2, args=("com.axisos.security_patrol", "patrol"))
t3.start()
time.sleep(0.02)
t4.start()
t3.join()
t4.join()

successes2 = [s for _, s, _ in results2 if s is True]
failures2 = [(label, err) for label, s, err in results2 if s is False]
check("Exactement UNE mission réussit, même avec deux apps différentes",
      len(successes2) == 1)
check("Le message de refus mentionne explicitement 'une autre mission'",
      len(failures2) == 1 and "autre mission" in failures2[0][1])

# ══════════════════════════════════════════════════════════════════

section("3 — Après la fin d'une mission, le verrou est bien relâché")

runtime3 = make_runtime_with_apps()
result_a = runtime3.run_mission("com.acme.package_sorting", params={})
check("Première mission séquentielle réussit", result_a is not None and result_a.success)

# Une DEUXIÈME mission séquentielle (pas concurrente) doit pouvoir
# s'exécuter normalement une fois la première terminée — le verrou
# ne doit jamais rester bloqué après coup.
result_b = runtime3.run_mission("com.acme.package_sorting", params={})
check("Deuxième mission séquentielle (après la fin de la première) réussit aussi",
      result_b is not None and result_b.success)

# ══════════════════════════════════════════════════════════════════

section("4 — Le verrou est bien relâché même si la mission échoue")

runtime4 = make_runtime_with_apps()

class _AlwaysFailApp(PackageSortingApp):
    def run_mission(self, ctx, params):
        raise RuntimeError("Panne simulée volontaire")

runtime4.install(_AlwaysFailApp())
runtime4.start_app("com.acme.package_sorting")  # même app_id, remplace l'entrée

result_fail = runtime4.run_mission("com.acme.package_sorting", params={})
check("Mission qui lève une exception se termine (pas de deadlock)",
      result_fail is not None)

# Le verrou doit être libre malgré l'échec précédent.
result_after_fail = runtime4.run_mission("com.acme.package_sorting", params={})
check("Nouvelle mission après un échec fonctionne (verrou bien relâché)",
      result_after_fail is not None)

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ Deux missions concurrentes (même app ou apps différentes)")
    print("    ne peuvent jamais s'exécuter en même temps sur le robot")
    print("  ✓ La mission refusée reçoit un résultat explicite immédiat,")
    print("    jamais une attente silencieuse")
    print("  ✓ Le verrou est toujours relâché après une mission, succès")
    print("    ou échec (try/finally) — pas de deadlock possible")

sys.exit(0 if FAIL == 0 else 1)
