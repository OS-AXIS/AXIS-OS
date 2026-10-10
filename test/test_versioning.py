"""
AXIS-OS — Tests versioning / compatibilité SDK
=================================================
Valide axis_os.SemVer (parsing, comparaison, règle de compatibilité
0.x stricte) et son intégration dans AppManifest.validate() /
PlayStoreRuntime.install() — le mécanisme qui aurait détecté et
signalé proprement le changement cassant de HardwareRequirements fait
plus tôt dans cette même session, au lieu de planter en silence chez
un développeur tiers.
"""

import sys
sys.path.insert(0, ".")

from axis_os import __version__, SemVer
from axis_os.store.sdk.manifest import AppManifest, AppCategory, PricingModel
from axis_os.store.sdk.app_base import RobotApp
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


class _VersionedApp(RobotApp):
    """App de test minimale, avec sdk_version_min paramétrable."""
    def __init__(self, sdk_version_min):
        self._sdk_version_min = sdk_version_min

    def manifest(self):
        return AppManifest(
            app_id="com.test.versioned", name="VersionedApp", version="1.0.0",
            author="T", author_email="t@t.com", description="d",
            category=AppCategory.UTILITIES,
            sdk_version_min=self._sdk_version_min,
        )

    def on_start(self, ctx): pass
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params): pass


def make_runtime() -> PlayStoreRuntime:
    robot = NormalizedRobotHAL(UnitreeG1HAL("g1_semver_test"), build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    return PlayStoreRuntime(robot, wd, confirmer_permissions=accepter_tout_SIMULATION)


# ══════════════════════════════════════════════════════════════════

section("1 — SemVer.parse() : cas valides et invalides")

v = SemVer.parse("0.2.0")
check("Parse '0.2.0' correctement", v == SemVer(0, 2, 0))
check("__str__ round-trip", str(v) == "0.2.0")

for bad in ["1.2", "1.2.3.4", "1.2.x", "", "a.b.c", "-1.0.0"]:
    try:
        SemVer.parse(bad)
        check(f"'{bad}' rejeté (ValueError)", False)
    except ValueError:
        check(f"'{bad}' rejeté (ValueError)", True)

# ══════════════════════════════════════════════════════════════════

section("2 — SemVer : comparaison et ordre")

check("0.1.0 < 0.2.0",  SemVer(0, 1, 0) < SemVer(0, 2, 0))
check("0.2.0 == 0.2.0", SemVer(0, 2, 0) == SemVer(0, 2, 0))
check("1.0.0 > 0.9.9",  SemVer(1, 0, 0) > SemVer(0, 9, 9))
check("0.2.1 >= 0.2.0", SemVer(0, 2, 1) >= SemVer(0, 2, 0))

# ══════════════════════════════════════════════════════════════════

section("3 — is_compatible_with() : règle stricte en 0.x")

current_0_2_0 = SemVer(0, 2, 0)
check("0.2.0 compatible avec 0.2.0 (identique)",
      SemVer(0, 2, 0).is_compatible_with(current_0_2_0))
check("0.2.0 compatible avec 0.2.1 (patch supérieur, même minor)",
      SemVer(0, 2, 0).is_compatible_with(SemVer(0, 2, 1)))
check("0.1.0 INCOMPATIBLE avec 0.2.0 (minor différent en 0.x)",
      not SemVer(0, 1, 0).is_compatible_with(current_0_2_0))
check("0.2.1 INCOMPATIBLE avec 0.2.0 (patch requis supérieur au courant)",
      not SemVer(0, 2, 1).is_compatible_with(current_0_2_0))
check("0.1.0 INCOMPATIBLE avec 1.0.0 (major différent)",
      not SemVer(0, 1, 0).is_compatible_with(SemVer(1, 0, 0)))

# Règle standard une fois >= 1.0 (comportement rétrocompatible normal)
current_1_5_0 = SemVer(1, 5, 0)
check("1.0.0 compatible avec 1.5.0 (minor supérieur, même major, >=1.0)",
      SemVer(1, 0, 0).is_compatible_with(current_1_5_0))
check("1.6.0 INCOMPATIBLE avec 1.5.0 (requiert plus récent que le courant)",
      not SemVer(1, 6, 0).is_compatible_with(current_1_5_0))
check("2.0.0 INCOMPATIBLE avec 1.5.0 (major supérieur)",
      not SemVer(2, 0, 0).is_compatible_with(current_1_5_0))

# ══════════════════════════════════════════════════════════════════

section("4 — AppManifest.validate() : sdk_version_min")

m_none = AppManifest(
    app_id="com.test.a", name="A", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
)
check("sdk_version_min=None (défaut) -> aucune erreur (rétrocompatible)",
      m_none.validate() == [])

m_valid = AppManifest(
    app_id="com.test.b", name="B", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
    sdk_version_min="0.2.0",
)
check("sdk_version_min='0.2.0' bien formé -> aucune erreur",
      m_valid.validate() == [])

m_bad = AppManifest(
    app_id="com.test.c", name="C", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
    sdk_version_min="not-a-version",
)
errors = m_bad.validate()
check("sdk_version_min mal formé -> erreur détectée",
      len(errors) == 1 and "sdk_version_min" in errors[0])

# ══════════════════════════════════════════════════════════════════

section("5 — PlayStoreRuntime.install() : refus si SDK incompatible")

runtime_ok = make_runtime()
check(f"App sans sdk_version_min s'installe normalement (AXIS-OS {__version__})",
      runtime_ok.install(_VersionedApp(None)))

runtime_compat = make_runtime()
check("App avec sdk_version_min == version courante s'installe",
      runtime_compat.install(_VersionedApp(__version__)))

runtime_incompat = make_runtime()
check("App avec sdk_version_min='0.1.0' (minor différent) REFUSÉE",
      not runtime_incompat.install(_VersionedApp("0.1.0")))

runtime_future = make_runtime()
check("App exigeant une version future ('9.9.9') REFUSÉE",
      not runtime_future.install(_VersionedApp("9.9.9")))


# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
total = FAIL
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ SemVer : parsing strict, comparaison correcte")
    print("  ✓ Règle 0.x : incompatibilité stricte même sur un minor différent")
    print("  ✓ sdk_version_min rétrocompatible (None = comportement d'avant)")
    print("  ✓ PlayStoreRuntime refuse proprement une app SDK-incompatible,")
    print("    avec un message clair plutôt qu'un échec silencieux ou confus.")

sys.exit(0 if FAIL == 0 else 1)
