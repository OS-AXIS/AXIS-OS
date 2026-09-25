"""
Aléa dans le sandbox : chaque mission doit avoir le SIEN.

Un processus fils créé par duplication hérite de l'état des générateurs
aléatoires du parent. Sans réensemencement, toutes les missions tirent
les mêmes nombres — constaté : 20 échecs sur 20 au lieu de ~30 %.
"""
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.store.runtime.sandbox import SandboxRunner
from axis_os.store.sdk.manifest import ResourceQuota

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


random.seed(12345)            # état du parent FIGÉ avant les missions
tirages = []
for _ in range(8):
    r = SandboxRunner("t.alea", ResourceQuota(max_runtime_s=5.0)).run(random.random)
    tirages.append(r.result if hasattr(r, "result") else r.value)
print(f"  tirages : {[round(t, 3) for t in tirages if t is not None]}")
check("Chaque mission sandboxée tire un nombre DIFFÉRENT — l'aléa global "
      "est réensemencé dans le fils", len(set(tirages)) == len(tirages))
check("Et le parent n'a rien consommé : son état reste inchangé",
      random.random() == random.Random(12345).random())

print(f"\n  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
sys.exit(0 if FAIL == 0 else 1)
