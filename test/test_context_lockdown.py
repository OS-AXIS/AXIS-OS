"""
AXIS-OS — Tests du verrouillage RobotContext
=================================================
Valide qu'une app ne peut fonctionner QU'à travers un vrai
PlayStoreRuntime — demande explicite : une app écrite avec le SDK
AXIS-OS ne doit pouvoir s'exécuter que dans AXIS-OS, pas être
récupérée et lancée ailleurs (script autonome, ou avec un HAL
détourné hors de tout Play Store).

Limite assumée et documentée (voir _RuntimeSessionToken dans
axis_os/store/sdk/app_base.py) : en Python pur, sans extension
compilée, aucune barrière n'est absolue contre un développeur qui lit
activement le code source et écrit délibérément du code pour la
contourner. Ce que ce mécanisme garantit réellement : que
RobotContext(robot, watchdog, manifest) — l'appel évident, sans
connaissance du mécanisme interne — échoue explicitement, plutôt que
de sembler fonctionner et laisser croire qu'une app est utilisable
hors d'AXIS-OS.
"""

import sys
sys.path.insert(0, ".")

from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.store.sdk.app_base import (
    RobotContext, RobotApp, UnauthorizedContextError, _RuntimeSessionToken,
)
from axis_os.store.sdk.manifest import AppManifest, AppCategory, Permission
from axis_os.store.runtime.play_store import PlayStoreRuntime

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def make_robot(name):
    robot = NormalizedRobotHAL(UnitreeG1HAL(name), build_g1_normalizer())
    robot.initialize_all()
    return robot


def make_manifest(app_id, permissions=None):
    return AppManifest(
        app_id=app_id, name="T", version="1.0.0", author="A",
        author_email="a@a.com", description="d", category=AppCategory.UTILITIES,
        permissions=permissions or [],
    )


# ══════════════════════════════════════════════════════════════════

section("1 — CRITIQUE : RobotContext(robot, watchdog, manifest) échoue sans jeton")

robot1 = make_robot("g1_lockdown_1")
wd1 = WatchdogSystem(robot1)
manifest1 = make_manifest("com.attacker.direct", [Permission.ACTUATORS_WRITE_ALL])

try:
    ctx = RobotContext(robot1, wd1, manifest1)
    check("Construction directe (l'appel évident, sans connaître le "
          "mécanisme interne) échoue explicitement", False)
except UnauthorizedContextError:
    check("Construction directe (l'appel évident, sans connaître le "
          "mécanisme interne) échoue explicitement", True)

# ══════════════════════════════════════════════════════════════════

section("2 — CRITIQUE : un jeton forgé par l'attaquant ne passe jamais contre "
        "le vrai jeton d'un runtime existant")

robot2 = make_robot("g1_lockdown_2")
wd2 = WatchdogSystem(robot2)
manifest2 = make_manifest("com.attacker.forge", [Permission.ACTUATORS_WRITE_ALL])
runtime2 = PlayStoreRuntime(robot2, wd2)

forged_token = _RuntimeSessionToken()
try:
    ctx = RobotContext(
        robot2, wd2, manifest2,
        _session_token=forged_token,
        _runtime_token_holder=runtime2._session_token,
    )
    check("Jeton forgé comparé au VRAI jeton d'un runtime existant "
          "-> toujours refusé", False)
except UnauthorizedContextError:
    check("Jeton forgé comparé au VRAI jeton d'un runtime existant "
          "-> toujours refusé", True)

# ══════════════════════════════════════════════════════════════════

section("3 — Le chemin légitime (PlayStoreRuntime.install()) reste fonctionnel")


class _LegitApp(RobotApp):
    def manifest(self):
        return make_manifest("com.legit.app")
    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass


robot3 = make_robot("g1_lockdown_3")
wd3 = WatchdogSystem(robot3)
runtime3 = PlayStoreRuntime(robot3, wd3)
result3 = runtime3.install(_LegitApp())
check("install() légitime toujours fonctionnel", result3 is True)

started3 = runtime3.start_app("com.legit.app")
check("start_app() légitime (donc un vrai RobotContext utilisable en "
      "coulisse) toujours fonctionnel", started3 is True)

# ══════════════════════════════════════════════════════════════════

section("4 — RobotContext.for_testing() : chemin explicite réservé aux tests")

robot4 = make_robot("g1_lockdown_4")
wd4 = WatchdogSystem(robot4)
manifest4 = make_manifest("com.test.isolated")
ctx4 = RobotContext.for_testing(robot4, wd4, manifest4)
check("RobotContext.for_testing() construit un contexte utilisable "
      "sans PlayStoreRuntime complet (réservé aux tests unitaires)",
      isinstance(ctx4, RobotContext))

# ══════════════════════════════════════════════════════════════════

section("5 — Deux instances de PlayStoreRuntime ont des jetons distincts")

robot5a = make_robot("g1_lockdown_5a")
wd5a = WatchdogSystem(robot5a)
runtime5a = PlayStoreRuntime(robot5a, wd5a)

robot5b = make_robot("g1_lockdown_5b")
wd5b = WatchdogSystem(robot5b)
runtime5b = PlayStoreRuntime(robot5b, wd5b)

manifest5 = make_manifest("com.cross.runtime")
try:
    # Le jeton du runtime A ne doit JAMAIS être accepté par un
    # RobotContext qui prétend appartenir au runtime B — même entre
    # deux instances parfaitement légitimes.
    ctx5 = RobotContext(
        robot5a, wd5a, manifest5,
        _session_token=runtime5a._session_token,
        _runtime_token_holder=runtime5b._session_token,
    )
    check("Le jeton d'un runtime A n'est jamais accepté au nom d'un "
          "runtime B différent", False)
except UnauthorizedContextError:
    check("Le jeton d'un runtime A n'est jamais accepté au nom d'un "
          "runtime B différent", True)

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ RobotContext ne peut être construit que par un vrai")
    print("    PlayStoreRuntime — l'appel direct évident échoue")
    print("  ✓ Un jeton forgé par un attaquant ne passe jamais contre le")
    print("    vrai jeton d'un runtime existant")
    print("  ✓ Le chemin légitime (install/start_app) reste pleinement")
    print("    fonctionnel")

sys.exit(0 if FAIL == 0 else 1)
