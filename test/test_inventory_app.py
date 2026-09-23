"""
Warehouse Inventory — la logique métier, sans robot ni simulateur.

Ce que ce test protège : les deux décisions qui font l'app — quand deux
observations désignent le même colis, et combien de vues avant de
l'inscrire. Ni la perception ni un modèle ne peuvent les prendre.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.types import Vector3
from axis_os.apps.warehouse_inventory.app import (
    RAYON_FUSION_M, VUES_MINIMUM, WarehouseInventoryApp,
)
from axis_os.store.sdk.manifest import (
    Permission, ProtectionLevel, get_protection_level,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Det:
    def __init__(self, x, y, z=0.0, conf=0.3):
        self.position_3d = Vector3(x, y, z)
        self.direction = Vector3(1.0, 0.0, 0.0)
        self.confidence = conf
        self.label = "a cardboard box"


app = WarehouseInventoryApp()

section("1 — Fusion : deux mesures proches = UN colis")

obs = []
for x, y in [(2.0, 0.0), (2.1, 0.05), (1.95, -0.1)]:
    app._noter(obs, Det(x, y))
check("Trois mesures dans le rayon de fusion -> un seul colis", len(obs) == 1)
check("Compté trois fois", obs[0]["vues"] == 3)
check(f"La position est MOYENNÉE, pas remplacée "
      f"({obs[0]['x']:.2f}, {obs[0]['y']:.2f})",
      abs(obs[0]["x"] - 2.017) < 0.02 and abs(obs[0]["y"] + 0.017) < 0.02)

app._noter(obs, Det(2.0, 2.0))
check("Un colis plus loin que le rayon reste DISTINCT", len(obs) == 2)

# Une mesure aberrante ne doit pas déplacer un colis bien observé.
for _ in range(9):
    app._noter(obs, Det(2.0, 0.0))
avant = obs[0]["x"]
app._noter(obs, Det(2.0 + RAYON_FUSION_M * 0.9, 0.0))
check(f"Une mesure aberrante déplace peu un colis vu 12 fois "
      f"({abs(obs[0]['x'] - avant):.3f} m)",
      abs(obs[0]["x"] - avant) < 0.10)

section("2 — Le seuil de vues élimine les détections isolées")

obs2 = [{"x": 1.0, "y": 0.0, "vues": 1, "confiance": 0.13, "label": "x"},
        {"x": 3.0, "y": 0.0, "vues": 5, "confiance": 0.31, "label": "x"}]
retenus = [o for o in obs2 if o["vues"] >= VUES_MINIMUM]
check(f"Vue une seule fois -> écartée (seuil {VUES_MINIMUM})",
      len(retenus) == 1 and retenus[0]["x"] == 3.0)

section("3 — Comparaison avec l'inventaire précédent")

anciens = [{"x": 2.0, "y": 0.0}, {"x": -3.0, "y": 1.0}]
actuels = [{"x": 2.05, "y": 0.02}, {"x": 5.0, "y": 0.0}]
nouveaux, revus = app._comparer(anciens, actuels)
disparus = app._disparus(anciens, actuels)
check("Un colis retrouvé au même endroit est « déjà connu »",
      len(revus) == 1 and abs(revus[0]["x"] - 2.05) < 1e-9)
check("Un colis jamais vu est « nouveau »", len(nouveaux) == 1)
check("Un colis absent cette fois est « disparu »",
      len(disparus) == 1 and disparus[0]["x"] == -3.0)

check("Premier inventaire : tout est nouveau, rien n'a disparu",
      app._comparer([], actuels)[0] == actuels and app._disparus([], actuels) == [])

section("4 — Permissions et stockage")

m = app.manifest()
check("STORAGE_PERSISTENT est NORMAL : écrire dans son propre espace "
      "cloisonné ne met rien en danger physiquement",
      get_protection_level(Permission.STORAGE_PERSISTENT)
      is ProtectionLevel.NORMAL)
check("L'app déclare le stockage persistant",
      Permission.STORAGE_PERSISTENT in m.permissions)
check("Tourner sur place exige les permissions de déplacement",
      Permission.LOCOMOTION_WALK in m.permissions
      and Permission.ACTUATORS_WRITE_LEGS in m.permissions)

section("5 — Cloisonnement : une app ne lit pas la mémoire d'une autre")

import tempfile
from axis_os.services.memory.service import MemoryService

with tempfile.TemporaryDirectory() as d:
    mem = MemoryService(Path(d))
    mem.start()
    mem.app_store("com.axisos.warehouse_inventory", "inventaire", [{"x": 1.0}])
    check("L'app relit ses propres données",
          mem.app_load("com.axisos.warehouse_inventory", "inventaire").ok)
    autre = mem.app_load("com.pirate.app", "inventaire")
    check("Une AUTRE app, avec la MÊME clé, ne voit rien", not autre.ok)
    mem.app_store("com.pirate.app", "inventaire", [{"x": 99.0}])
    mien = mem.app_load("com.axisos.warehouse_inventory", "inventaire")
    check("Et elle n'a pas écrasé les données de la première",
          mien.ok and mien.value[0]["x"] == 1.0)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
