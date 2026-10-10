"""
AXIS-OS — Tests persistance de la facturation via MemoryService
=====================================================================
Corrige le trou documenté depuis deux audits externes : l'état de
facturation (InstalledApp.one_time_billed, last_subscription_bill_time_s)
était tenu EN MÉMOIRE uniquement, perdu à tout redémarrage du process
— une app ONE_TIME refacturée comme neuve, une fenêtre SUBSCRIPTION
de 30 jours relancée à zéro.

PlayStoreRuntime accepte désormais un memory_service optionnel
(MemoryService, axis_os/services/memory/service.py, déjà écrit et
testé séparément) : quand fourni, l'état est lu à l'installation et
réécrit à chaque mise à jour. Sans lui (None, défaut), comportement
strictement inchangé — testé explicitement en section 3
(non-régression).

Critère de succès explicite : une app ONE_TIME/SUBSCRIPTION n'est
JAMAIS refacturée après un vrai redémarrage du process (pas une
simulation en mémoire — deux instances complètement séparées de
NormalizedRobotHAL/PlayStoreRuntime/MemoryService, partageant
uniquement le répertoire disque de MemoryService).
"""

import sys
import tempfile
import shutil
sys.path.insert(0, ".")

from pathlib import Path
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.core.types import ActuatorCommand, ControlMode
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
from axis_os.store.sdk.manifest import AppManifest, AppCategory, PricingModel, Permission
from axis_os.store.sdk.app_base import RobotApp, MissionResult
from axis_os.services.memory.service import MemoryService

FAIL = 0
TMP_DIR = tempfile.mkdtemp(prefix="axis_os_billing_persist_test_")


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


class _MovingApp(RobotApp):
    """
    App de test qui bouge RÉELLEMENT le robot — nécessaire depuis le
    correctif M3 (la facturation exige un vrai mouvement, pas juste
    result.success déclaré par l'app, voir CHANGELOG.md).
    """
    def __init__(self, app_id: str, pricing_model: PricingModel):
        self._app_id = app_id
        self._pricing_model = pricing_model

    def manifest(self):
        return AppManifest(
            app_id=self._app_id, name="T", version="1.0.0", author="T",
            author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
            pricing_model=self._pricing_model, base_price_eur=1.40,
            permissions=[Permission.ACTUATORS_WRITE_ARMS],
        )

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        ctx.move_arms({
            "left_elbow_joint": ActuatorCommand(mode=ControlMode.POSITION, position=0.1)
        })
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


def make_session(storage_dir: Path):
    robot = NormalizedRobotHAL(UnitreeG1HAL(f"g1_bp_{id(storage_dir)}"),
                               build_g1_normalizer())
    robot.initialize_all()
    wd = WatchdogSystem(robot)
    mem = MemoryService(storage_dir=storage_dir)
    mem.start()
    runtime = PlayStoreRuntime(robot, wd, memory_service=mem, confirmer_permissions=accepter_tout_SIMULATION)
    return runtime


# ══════════════════════════════════════════════════════════════════

section("1 — CRITÈRE DE SUCCÈS : ONE_TIME jamais refacturée après un vrai redémarrage")

storage1 = Path(TMP_DIR) / "onetime"
app_id1 = "com.test.onetime_w"

runtime1 = make_session(storage1)
app1 = _MovingApp(app_id1, PricingModel.ONE_TIME)
runtime1.install(app1)
runtime1.start_app(app_id1)
runtime1.run_mission(app_id1, params={})
check("Première session : facturée une fois", len(runtime1._billing) == 1)

# Nouvelle "session" — nouveau robot, nouveau watchdog, nouveau
# MemoryService, nouveau PlayStoreRuntime — seul le répertoire disque
# est partagé, exactement ce qu'un vrai redémarrage de process
# produirait.
runtime1b = make_session(storage1)
app1b = _MovingApp(app_id1, PricingModel.ONE_TIME)
runtime1b.install(app1b)
runtime1b.start_app(app_id1)
runtime1b.run_mission(app_id1, params={})
check("Après 'redémarrage' (nouvelles instances, même stockage disque) : "
      "AUCUNE nouvelle facture pour une app ONE_TIME déjà facturée",
      len(runtime1b._billing) == 0)

# ══════════════════════════════════════════════════════════════════

section("2 — CRITÈRE DE SUCCÈS : fenêtre SUBSCRIPTION survit au redémarrage")

storage2 = Path(TMP_DIR) / "subscription"
app_id2 = "com.test.sub_w"

runtime2 = make_session(storage2)
app2 = _MovingApp(app_id2, PricingModel.SUBSCRIPTION)
runtime2.install(app2)
runtime2.start_app(app_id2)
runtime2.run_mission(app_id2, params={})
check("Première session : facturée une fois", len(runtime2._billing) == 1)

runtime2b = make_session(storage2)
app2b = _MovingApp(app_id2, PricingModel.SUBSCRIPTION)
runtime2b.install(app2b)
runtime2b.start_app(app_id2)
runtime2b.run_mission(app_id2, params={})
check("Après 'redémarrage' le même jour : la fenêtre de 30 jours n'est "
      "PAS relancée à zéro — aucune nouvelle facture",
      len(runtime2b._billing) == 0)

# ══════════════════════════════════════════════════════════════════

section("3 — Non-régression : sans memory_service, comportement EN MÉMOIRE inchangé")

robot3 = NormalizedRobotHAL(UnitreeG1HAL("g1_bp_noreg"), build_g1_normalizer())
robot3.initialize_all()
wd3 = WatchdogSystem(robot3)
runtime3 = PlayStoreRuntime(robot3, wd3, confirmer_permissions=accepter_tout_SIMULATION)  # SANS memory_service (défaut None)
app3 = _MovingApp("com.test.noreg_w", PricingModel.ONE_TIME)
runtime3.install(app3)
runtime3.start_app("com.test.noreg_w")
for _ in range(3):
    runtime3.run_mission("com.test.noreg_w", params={})
check("Sans memory_service : ONE_TIME toujours facturée une seule fois "
      "MALGRÉ 3 missions dans la MÊME session (comportement en mémoire "
      "déjà correct depuis M2, non affecté par ce changement)",
      len(runtime3._billing) == 1)

# ══════════════════════════════════════════════════════════════════

section("4 — PAY_PER_MISSION : jamais affecté par la persistance (facturé à chaque fois)")

storage4 = Path(TMP_DIR) / "per_mission"
runtime4 = make_session(storage4)
app4 = _MovingApp("com.test.permission_w", PricingModel.PAY_PER_MISSION)
runtime4.install(app4)
runtime4.start_app("com.test.permission_w")
for _ in range(3):
    runtime4.run_mission("com.test.permission_w", params={})
check("PAY_PER_MISSION avec memory_service : toujours facturé à chaque "
      "mission (3 missions -> 3 factures, pas affecté par la persistance "
      "des DEUX autres modèles)",
      len(runtime4._billing) == 3)

# ══════════════════════════════════════════════════════════════════

shutil.rmtree(TMP_DIR, ignore_errors=True)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ ONE_TIME et SUBSCRIPTION : état de facturation survit à un")
    print("    vrai redémarrage (nouvelles instances séparées)")
    print("  ✓ Sans memory_service : comportement en mémoire strictement")
    print("    inchangé (rétrocompatible)")
    print("  ✓ PAY_PER_MISSION : jamais affecté, comme attendu")

sys.exit(0 if FAIL == 0 else 1)
