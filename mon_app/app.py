"""
Mon App — AXIS-OS Play Store App
=====================================
Généré par `axis-os init`.
"""

from axis_os.store.sdk.manifest import (
    AppManifest, Permission, AppCategory,
    PricingModel, HardwareRequirements
)
from axis_os.store.sdk.app_base import RobotApp, RobotContext, MissionResult


class MonAppApp(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id="com.kenzo.mon_app",
            name="Mon App",
            version="0.1.0",
            author="Kenzo",
            author_email="axis.project.dev@gmail.com",
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
