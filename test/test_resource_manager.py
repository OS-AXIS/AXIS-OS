"""
AXIS-OS — Test Resource Manager
====================================
Referme la priorité 2 d'une revue externe du 2026-09-06 : rien
n'empêchait deux apps de contrôler le même actionneur avec des
commandes contradictoires. Ce test prouve le principe de partage
retenu : actionneurs exclusifs, capteurs librement partagés.

    python test_resource_manager.py
"""

import sys
import threading
import time
sys.path.insert(0, ".")

from axis_os.store.runtime.resource_manager import (
    ResourceManager, ResourceKind, ResourceConflictError
)

PASS = 0
FAIL = 0


def section(title: str):
    print(f"\n{'═'*66}\n  {title}\n{'═'*66}")


def check(label: str, condition: bool) -> bool:
    global PASS, FAIL
    icon = "✓" if condition else "✗"
    print(f"  [{icon}] {label:<58} {'OK' if condition else 'FAILED'}")
    if condition: PASS += 1
    else: FAIL += 1
    return condition


# ══════════════════════════════════════════════════════════════════

section("1 — LE VRAI TEST : deux apps se disputant le même bras")
rm1 = ResourceManager()
rm1.acquire(ResourceKind.ARMS, "com.app.a")
check("App A a réservé ARMS avec succès", rm1.get_holder(ResourceKind.ARMS) == "com.app.a")

conflict_raised = False
try:
    rm1.acquire(ResourceKind.ARMS, "com.app.b")
except ResourceConflictError:
    conflict_raised = True
check("App B est bloquée par ResourceConflictError sur le même bras",
      conflict_raised)
check("App A reste détentrice après la tentative échouée de App B",
      rm1.get_holder(ResourceKind.ARMS) == "com.app.a")

section("2 — Ressources différentes : aucun conflit inutile")
rm2 = ResourceManager()
rm2.acquire(ResourceKind.ARMS, "com.app.a")
try:
    rm2.acquire(ResourceKind.LEGS, "com.app.b")
    legs_ok = True
except ResourceConflictError:
    legs_ok = False
check("App B peut réserver LEGS pendant que App A tient ARMS "
      "(ressources différentes, aucun conflit)", legs_ok)
check("Les deux réservations coexistent correctement",
      rm2.get_all_reservations() == {"ARMS": "com.app.a", "LEGS": "com.app.b"})

section("3 — ALL_ACTUATORS bloque ARMS et LEGS, et réciproquement")
rm3 = ResourceManager()
rm3.acquire(ResourceKind.ALL_ACTUATORS, "com.app.a")
arms_blocked = False
legs_blocked = False
try:
    rm3.acquire(ResourceKind.ARMS, "com.app.b")
except ResourceConflictError:
    arms_blocked = True
try:
    rm3.acquire(ResourceKind.LEGS, "com.app.b")
except ResourceConflictError:
    legs_blocked = True
check("ALL_ACTUATORS bloque une tentative d'acquisition de ARMS seul",
      arms_blocked)
check("ALL_ACTUATORS bloque une tentative d'acquisition de LEGS seul",
      legs_blocked)

rm3b = ResourceManager()
rm3b.acquire(ResourceKind.ARMS, "com.app.a")
all_blocked = False
try:
    rm3b.acquire(ResourceKind.ALL_ACTUATORS, "com.app.b")
except ResourceConflictError:
    all_blocked = True
check("Réciproquement, ARMS détenu bloque une tentative de ALL_ACTUATORS",
      all_blocked)

section("4 — Libération et ré-acquisition")
rm4 = ResourceManager()
rm4.acquire(ResourceKind.ARMS, "com.app.a")
rm4.release(ResourceKind.ARMS, "com.app.a")
check("Ressource libre après release()", rm4.get_holder(ResourceKind.ARMS) is None)

reacquire_ok = False
try:
    rm4.acquire(ResourceKind.ARMS, "com.app.b")
    reacquire_ok = True
except ResourceConflictError:
    pass
check("Une autre app peut acquérir la ressource après libération",
      reacquire_ok)

section("5 — Une app ne peut pas libérer la ressource d'une autre app")
rm5 = ResourceManager()
rm5.acquire(ResourceKind.ARMS, "com.app.a")
rm5.release(ResourceKind.ARMS, "com.app.b")  # tentative malveillante/erronée
check("La ressource reste détenue par App A malgré la tentative de "
      "libération par App B (release() silencieux, pas d'effet)",
      rm5.get_holder(ResourceKind.ARMS) == "com.app.a")

section("6 — Idempotence : ré-acquérir sa propre ressource ne casse rien")
rm6 = ResourceManager()
rm6.acquire(ResourceKind.ARMS, "com.app.a")
reacquire_own_ok = False
try:
    rm6.acquire(ResourceKind.ARMS, "com.app.a")  # même app, même ressource
    reacquire_own_ok = True
except ResourceConflictError:
    pass
check("App A peut ré-acquérir sa propre ressource sans erreur "
      "(utile pour une mission qui acquiert à chaque étape de boucle)",
      reacquire_own_ok)

section("7 — LE FILET DE SÉCURITÉ : release_all_for_app après un crash")
rm7 = ResourceManager()
rm7.acquire(ResourceKind.ARMS, "com.app.crashed")
rm7.acquire(ResourceKind.LEGS, "com.app.crashed")
rm7.acquire(ResourceKind.ARMS if False else ResourceKind.ARMS, "com.app.crashed")  # no-op supplémentaire
check("App crashée détient bien ARMS et LEGS avant le nettoyage",
      len(rm7.get_all_reservations()) == 2)

rm7.release_all_for_app("com.app.crashed")
check("Après release_all_for_app(), plus aucune ressource détenue "
      "(empêche un blocage permanent si une app plante sans avoir "
      "appelé release() elle-même)",
      len(rm7.get_all_reservations()) == 0)

new_app_ok = False
try:
    rm7.acquire(ResourceKind.ARMS, "com.app.new")
    new_app_ok = True
except ResourceConflictError:
    pass
check("Une nouvelle app peut utiliser les ressources libérées par le nettoyage",
      new_app_ok)

section("8 — Thread-safety : accès concurrents réels, pas juste séquentiels")
rm8 = ResourceManager()
results = {"a_got_it": False, "b_got_it": False, "conflicts": 0}
results_lock = threading.Lock()

def try_acquire(app_id: str, key: str):
    try:
        rm8.acquire(ResourceKind.ARMS, app_id)
        with results_lock:
            results[key] = True
    except ResourceConflictError:
        with results_lock:
            results["conflicts"] += 1

t1 = threading.Thread(target=try_acquire, args=("com.app.a", "a_got_it"))
t2 = threading.Thread(target=try_acquire, args=("com.app.b", "b_got_it"))
t1.start(); t2.start()
t1.join(); t2.join()

check("Exactement une seule app a obtenu la ressource en accès concurrent "
      f"réel (a={results['a_got_it']}, b={results['b_got_it']}, "
      f"conflits={results['conflicts']})",
      (results["a_got_it"] != results["b_got_it"]) and results["conflicts"] == 1)

section("9 — Confirmation du principe : les capteurs n'ont pas besoin "
        "de ce système (ResourceKind ne les inclut pas du tout)")
sensor_kinds = [k for k in ResourceKind if "CAMERA" in k.name or "MIC" in k.name
               or "IMU" in k.name]
check("Aucun capteur (caméra, micro, IMU) n'existe dans ResourceKind — "
      "confirmé par conception : ils restent librement partagés, "
      "jamais réservables ni bloquants",
      len(sensor_kinds) == 0)

# ══════════════════════════════════════════════════════════════════

section("RÉSUMÉ")
total = PASS + FAIL
print(f"  Résultats : {PASS}/{total} tests passés\n")
if FAIL == 0:
    print("  ✓ LE VRAI TEST : deux apps ne peuvent plus se disputer le")
    print("    même bras avec des commandes contradictoires")
    print("  ✓ Les ressources indépendantes (bras vs jambes) restent")
    print("    utilisables en parallèle par des apps différentes")
    print("  ✓ ALL_ACTUATORS bloque/est bloqué correctement par ARMS et LEGS")
    print("  ✓ Filet de sécurité : une app qui plante sans libérer ne")
    print("    bloque jamais une ressource indéfiniment")
    print("  ✓ Thread-safe sous accès concurrent réel, pas juste théorique")
    print("  ✓ Confirmé : les capteurs restent hors de ce système, comme prévu")
    print("  → La priorité 2 de la revue externe est refermée.")
else:
    print(f"  ✗ {FAIL} test(s) échoué(s).")

sys.exit(0 if FAIL == 0 else 1)
