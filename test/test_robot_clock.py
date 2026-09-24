"""
Horloge du robot — l'équivalent de use_sim_time dans ROS.

Ce que ce test protège : une app et ses quotas doivent vivre au rythme
du ROBOT, pas du mur. En simulation ralentie par le rendu vidéo, une app
qui comptait en temps mural perdait sa cible, et une mission était tuée
par un quota de 300 s murales après une trentaine de secondes simulées.
"""
import sys
import threading
import time
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


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class HorlogeSimulee:
    """Avance de `vitesse` secondes robot par seconde murale."""

    def __init__(self, vitesse):
        self.vitesse = vitesse
        self._t0 = time.monotonic()

    def __call__(self):
        return (time.monotonic() - self._t0) * self.vitesse


section("1 — Le quota du sandbox est compté en temps du ROBOT")

# Simulation LENTE : 0.1 s robot par seconde murale. L'app dort 3 s
# murales = 0.3 s robot, sous un quota de 1 s robot : elle doit ABOUTIR.
lent = SandboxRunner("test.lent", ResourceQuota(max_runtime_s=1.0),
                     clock=HorlogeSimulee(0.1))
t0 = time.monotonic()
r = lent.run(time.sleep, args=(3.0,))
# Exiger le SUCCÈS, pas seulement « pas TIMEOUT » : la première version
# de ce test était passée au vert alors que l'exécution avait PLANTÉ.
check(f"Simulation lente : 3 s murales = 0.3 s robot, sous un quota de "
      f"1 s -> menée à TERME (état {r.state.name}, erreur {r.error})",
      r.success)

# Simulation RAPIDE : 10 s robot par seconde murale. L'app dort 2 s
# murales = 20 s robot, au-delà du quota : elle doit être TUÉE.
rapide = SandboxRunner("test.rapide", ResourceQuota(max_runtime_s=1.0),
                       clock=HorlogeSimulee(10.0))
t0 = time.monotonic()
r2 = rapide.run(time.sleep, args=(2.0,))
mur = time.monotonic() - t0
# Le sandbox signale un dépassement de quota par l'état VIOLATED ; la
# CAUSE est portée par la violation. On teste la cause.
def _cause(res):
    v = getattr(res, "violation", None)
    return getattr(v, "rule", None)


check(f"Simulation rapide : quota atteint en temps robot -> TUÉE pour "
      f"dépassement de durée (cause {_cause(r2)})", _cause(r2) == "TIMEOUT")
check(f"… et bien avant les 2 s murales ({mur:.2f} s)", mur < 1.5)

sans = SandboxRunner("test.mur", ResourceQuota(max_runtime_s=0.5))
r3 = sans.run(time.sleep, args=(2.0,))
check("Sans horloge fournie : temps mural, comportement inchangé "
      f"(cause {_cause(r3)})", _cause(r3) == "TIMEOUT")

section("2 — ctx.now() et ctx.sleep() suivent l'horloge du robot")

import contextlib
import io
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.store.sdk.app_base import RobotContext
from axis_os.store.sdk.manifest import AppCategory, AppManifest

with contextlib.redirect_stdout(io.StringIO()):
    h = UnitreeG1HAL(backend="sim"); h.initialize_all()
    rb = NormalizedRobotHAL(h, build_g1_normalizer()); rb.initialize_all()
    wd = WatchdogSystem(rb)
man = AppManifest(app_id="com.test.horloge", name="T", version="1.0.0",
                  author="t", author_email="t@t.t", description="t",
                  category=AppCategory.RESEARCH, permissions=[])
ctx = RobotContext.for_testing(rb, wd, man)

t_sim = [100.0]
ctx._set_clock(lambda: t_sim[0])
check("now() rend l'heure INJECTÉE, pas l'heure murale", ctx.now() == 100.0)


def avancer():
    for _ in range(20):
        time.sleep(0.01)
        t_sim[0] += 0.1


th = threading.Thread(target=avancer)
t0 = time.monotonic()
th.start()
ctx.sleep(1.0)
mur = time.monotonic() - t0
th.join()
check(f"sleep(1.0) attend 1 s ROBOT (≈0.1 s murale ici : {mur:.2f} s)",
      mur < 0.5 and t_sim[0] >= 101.0)

ctx2 = RobotContext.for_testing(rb, wd, man)
a = ctx2.now(); time.sleep(0.05); b = ctx2.now()
check("Sans horloge injectée : heure murale, comportement inchangé",
      0.04 < b - a < 0.5)

section("3 — Le faux micro attend en temps du robot")

from axis_os.hal.drivers.sim_common.sim_perception import QueuedMicrophone
attentes = []
m = QueuedMicrophone("m", [], sleeper=attentes.append)
m.initialize()
m.capture(2.0)
check("File vide : l'attente passe par l'horloge fournie",
      attentes == [2.0])
from axis_os.hal.core.types import AudioChunk
attentes2 = []
m2 = QueuedMicrophone("m2", [AudioChunk(16000, 1, 2, b"\x01\x00" * 100)],
                      sleeper=attentes2.append)
m2.initialize()
check("Phrase en file : le micro prend AUSSI le temps d'écouter — sinon "
      "toutes les phrases sont « prononcées » à l'instant zéro",
      m2.capture(2.0) is not None and attentes2 == [2.0])

section("4 — Le pont tolère un appel bloquant en simulation lente")

from axis_os.store.runtime.hal_bridge import HALBridgeClient
cli = HALBridgeClient.__new__(HALBridgeClient)
cli._timeout = 5.0
check("Appel instantané : 5 s — détecte un noyau mort",
      cli._delai() == 5.0)
check(f"listen(2.0) : délai étendu à {cli._delai(2.0):.0f} s, pour une "
      f"simulation jusqu'à 60× plus lente — au lieu de planter à 5 s",
      cli._delai(2.0) >= 2.0 * 30)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
