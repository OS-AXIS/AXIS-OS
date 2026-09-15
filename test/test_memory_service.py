"""
AXIS-OS — Tests MemoryService
==================================
Valide le premier service système réel (axis_os/services/memory/
service.py), écrit pour valider le contrat IService/ServiceResult
par l'usage plutôt que par supposition — voir axis_os/services/
base.py pour le contexte complet.

Critère de succès explicitement demandé : un lieu nommé est retrouvé
après un redémarrage COMPLET du process (pas une simulation en
mémoire) — voir section 2 ci-dessous, qui relance un vrai
sous-process Python pour le prouver honnêtement.
"""

import sys
import os
import subprocess
import tempfile
import shutil
sys.path.insert(0, ".")

from axis_os.services.base import IService, ServiceResult, ServiceHealth
from axis_os.services.memory.service import MemoryService
from axis_os.hal.core.types import HALStatus
from axis_os.store.sdk.manifest import Capability

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


TMP_DIR = tempfile.mkdtemp(prefix="axis_os_memory_test_")


def make_service() -> MemoryService:
    from pathlib import Path
    storage = Path(TMP_DIR) / f"store_{os.urandom(4).hex()}"
    svc = MemoryService(storage_dir=storage)
    svc.start()
    return svc


# ══════════════════════════════════════════════════════════════════

section("0 — MemoryService valide le contrat IService")

check("MemoryService est une vraie sous-classe de IService",
      issubclass(MemoryService, IService))

svc0 = make_service()
check("service_id == 'memory'", svc0.service_id == "memory")
check("capability == Capability.MEMORY_PERSISTENT",
      svc0.capability == Capability.MEMORY_PERSISTENT)

health0 = svc0.health()
check("health() retourne un ServiceHealth cohérent après start()",
      isinstance(health0, ServiceHealth) and health0.running and health0.healthy)

svc0.stop()
health0b = svc0.health()
check("Après stop(), health().running == False", health0b.running is False)

# start()/stop() idempotents
svc0.start()
svc0.start()  # deuxième appel, ne doit rien casser
check("start() est idempotent (deux appels consécutifs, pas d'exception)", True)
svc0.stop()
svc0.stop()
check("stop() est idempotent", True)

# ══════════════════════════════════════════════════════════════════

section("1 — Mémoire sémantique : remember_place / recall_place (même process)")

svc1 = make_service()
r1 = svc1.remember_place("cuisine", {"x": 3.5, "y": 1.2, "theta": 0.0})
check("remember_place() retourne ok=True", r1.ok is True)

r2 = svc1.recall_place("cuisine")
check("recall_place() retrouve exactement la pose stockée",
      r2.ok is True and r2.value == {"x": 3.5, "y": 1.2, "theta": 0.0})

r3 = svc1.recall_place("lieu_jamais_stocke")
check("recall_place() sur un lieu inconnu retourne ok=False avec une raison",
      r3.ok is False and r3.reason is not None)

r4 = svc1.remember_place("", {"x": 0, "y": 0, "theta": 0})
check("remember_place() avec un nom vide est rejeté", r4.ok is False)

# ══════════════════════════════════════════════════════════════════

section("2 — CRITÈRE DE SUCCÈS : un lieu survit à un VRAI redémarrage de process")

storage_dir = os.path.join(TMP_DIR, "restart_test")

script_write = f'''
import sys
sys.path.insert(0, {os.getcwd()!r})
from axis_os.services.memory.service import MemoryService
from pathlib import Path

svc = MemoryService(storage_dir=Path({storage_dir!r}))
svc.start()
result = svc.remember_place("salon", {{"x": 1.0, "y": 2.0, "theta": 1.57}})
print("WRITE_OK" if result.ok else "WRITE_FAIL")
'''

script_read = f'''
import sys
sys.path.insert(0, {os.getcwd()!r})
from axis_os.services.memory.service import MemoryService
from pathlib import Path

# Nouvelle instance, dans un process Python ENTIÈREMENT SÉPARÉ — pas
# le même interpréteur, pas d'état partagé en mémoire possible.
svc = MemoryService(storage_dir=Path({storage_dir!r}))
svc.start()
result = svc.recall_place("salon")
if result.ok and result.value == {{"x": 1.0, "y": 2.0, "theta": 1.57}}:
    print("READ_OK")
else:
    print("READ_FAIL", result)
'''

p1 = subprocess.run([sys.executable, "-c", script_write], capture_output=True, text=True)
check("Sous-process 1 (écriture) réussit", "WRITE_OK" in p1.stdout)

p2 = subprocess.run([sys.executable, "-c", script_read], capture_output=True, text=True)
check("Sous-process 2 (VRAI process séparé, lecture) retrouve le lieu "
      "exact stocké par le sous-process 1 — critère de succès explicite",
      "READ_OK" in p2.stdout)
if "READ_OK" not in p2.stdout:
    print("    stdout:", p2.stdout, "stderr:", p2.stderr[-500:])

# ══════════════════════════════════════════════════════════════════

section("3 — Mémoire épisodique")

svc3 = make_service()
svc3.log_episode({"event": "colis_livre", "lieu": "cuisine", "t": 123})
svc3.log_episode({"event": "porte_ouverte", "lieu": "entree", "t": 456})
svc3.log_episode({"event": "colis_livre", "lieu": "salon", "t": 789})

r_recall = svc3.recall_episodes("colis_livre")
check("recall_episodes() retrouve les épisodes correspondants (recherche naïve)",
      r_recall.ok is True and len(r_recall.value) == 2)

r_recall_limit = svc3.recall_episodes("colis_livre", limit=1)
check("recall_episodes() respecte la limite demandée",
      len(r_recall_limit.value) == 1)

# ══════════════════════════════════════════════════════════════════

section("4 — CRITIQUE : app_store/app_load cloisonné par app_id")

svc4 = make_service()
svc4.app_store("com.paid.app", "secret_data", "valeur_confidentielle")

r_leak = svc4.app_load("com.free.app", "secret_data")
check("CRITIQUE : une autre app_id ne peut PAS lire la donnée d'une app payante",
      r_leak.ok is False)

r_own = svc4.app_load("com.paid.app", "secret_data")
check("L'app propriétaire relit correctement sa propre donnée",
      r_own.ok is True and r_own.value == "valeur_confidentielle")

# Cas limite : deux app_id qui pourraient collisionner naïvement
svc4.app_store("a::b", "c", "valeur1")
svc4.app_store("a", "b::c", "valeur2")
r_a = svc4.app_load("a::b", "c")
r_b = svc4.app_load("a", "b::c")
check("Cas limite app_id contenant '::' : chaque app retrouve SA valeur, "
      "pas celle de l'autre",
      r_a.value == "valeur1" and r_b.value == "valeur2")

r_no_appid = svc4.app_store("", "key", "value")
check("app_store() avec app_id vide est rejeté", r_no_appid.ok is False)

# ══════════════════════════════════════════════════════════════════

section("5 — Robustesse : fichier corrompu ne fait jamais planter le service")

from pathlib import Path
corrupt_storage = Path(TMP_DIR) / "corrupt_test"
corrupt_storage.mkdir(parents=True, exist_ok=True)
with open(corrupt_storage / "places.json", "w") as f:
    f.write("{ceci n'est pas du JSON valide")

svc5 = MemoryService(storage_dir=corrupt_storage)
svc5.start()
r5 = svc5.recall_place("nimporte_quoi")
check("Fichier JSON corrompu : recall_place() échoue proprement, pas d'exception",
      r5.ok is False)

r5b = svc5.remember_place("nouveau_lieu", {"x": 0, "y": 0, "theta": 0})
check("Après un fichier corrompu, une nouvelle écriture réussit quand même "
      "(reprend sur un état vide plutôt que de rester bloqué)",
      r5b.ok is True)

# ══════════════════════════════════════════════════════════════════

shutil.rmtree(TMP_DIR, ignore_errors=True)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ MemoryService valide le contrat IService par l'usage réel")
    print("  ✓ Un lieu nommé survit à un VRAI redémarrage de process")
    print("    (vérifié via un sous-process Python entièrement séparé)")
    print("  ✓ Cloisonnement app_store/app_load garanti entre app_id")
    print("  ✓ Robuste à un fichier de stockage corrompu")

sys.exit(0 if FAIL == 0 else 1)
