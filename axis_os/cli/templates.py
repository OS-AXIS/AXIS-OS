"""
AXIS-OS CLI — Templates
=========================
Contenu des fichiers générés par `axis-os init <app>`.

Licence : Apache 2.0
"""

APP_PY_TEMPLATE = '''"""
{app_name} — AXIS-OS Play Store App
=====================================
Généré par `axis-os init`.
"""

from axis_os.store.sdk.manifest import (
    AppManifest, Permission, AppCategory,
    PricingModel, HardwareRequirements
)
from axis_os.store.sdk.app_base import RobotApp, RobotContext, MissionResult


class {class_name}(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id="{app_id}",
            name="{app_name}",
            version="0.1.0",
            author="{author}",
            author_email="{author_email}",
            description="Décrivez votre app ici.",
            category=AppCategory.UTILITIES,
            permissions=[
                Permission.ACTUATORS_READ,
                # Ajoutez les permissions dont votre app a besoin
            ],
            hardware=HardwareRequirements(min_dof=0),
            pricing_model=PricingModel.FREE,
            max_velocity_ratio=0.5,
            requires_human_supervision=True,
        )

    def on_start(self, ctx: RobotContext) -> bool:
        ctx.log("Initialisation...")
        return True

    def run_mission(self, ctx: RobotContext, params: dict) -> MissionResult:
        ctx.log("Mission démarrée")

        # ── Votre logique métier ici ──────────────────────────

        return MissionResult(
            success=True,
            mission_id="",
            app_id=self.manifest().app_id,
            duration_s=0.0,
            items_processed=0,
        )

    def on_stop(self, ctx: RobotContext) -> None:
        ctx.log("Arrêt propre")
'''


MANIFEST_JSON_TEMPLATE = '''{{
  "app_id": "{app_id}",
  "name": "{app_name}",
  "version": "0.1.0",
  "author": "{author}",
  "author_email": "{author_email}",
  "category": "utilities",
  "pricing_model": "free",
  "base_price_eur": 0.0
}}
'''


TEST_APP_TEMPLATE = '''"""
Tests pour {app_name}
"""

import sys
sys.path.insert(0, ".")

from axis_os.hal.drivers.isaac_sim.hal import IsaacSimHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.watchdog.config import WatchdogConfig
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
from app import {class_name}


def test_app():
    hal = IsaacSimHAL(robot_id="test_{app_id_short}")
    hal.initialize_all()
    norm = NormalizedRobotHAL(hal, build_g1_normalizer())
    wd   = WatchdogSystem(hal, WatchdogConfig.simulation())
    wd.start()

    # Test en SIMULATION : permissions DANGEROUS accordées explicitement.
    # Sur un vrai robot, elles exigent une confirmation humaine.
    runtime = PlayStoreRuntime(norm, wd, confirmer_permissions=accepter_tout_SIMULATION)
    assert runtime.install({class_name}())
    assert runtime.start_app("{app_id}")

    result = runtime.run_mission("{app_id}", params={{}})
    assert result is not None
    print(f"Résultat : {{result}}")

    wd.stop()
    hal.shutdown_all()
    print("✓ Test passé")


if __name__ == "__main__":
    test_app()
'''


README_APP_TEMPLATE = '''# {app_name}

App AXIS-OS générée par `axis-os init`.

## Développement

```bash
axis-os simulate    # teste dans le simulateur
axis-os publish      # publie sur le Play Store
```

## Structure

```
{app_id_short}/
├── app.py           # logique de votre app
├── manifest.json    # métadonnées Play Store
├── test_app.py       # tests
└── README.md
```
'''
