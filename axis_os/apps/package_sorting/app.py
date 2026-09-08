"""
AXIS-OS Robotics Play Store
App : PackageSorting v1.0.0
================================
Tri automatisé de colis sur ligne logistique.
Première application tierce du Play Store AXIS-OS.

Auteur  : ACME Robotics Solutions
Prix    : 2.00€/mission client (1.40€ net dev, 0.60€ commission)
Licence : Propriétaire — distribué via AXIS-OS Play Store
"""

from __future__ import annotations
import time
import random
from typing import Dict, Any
from dataclasses import dataclass
from enum import Enum

from axis_os.store.sdk.manifest import (
    AppManifest, Permission, AppCategory,
    PricingModel, HardwareRequirements
)
from axis_os.store.sdk.app_base import RobotApp, RobotContext, MissionResult
from axis_os.hal.core.types import ActuatorCommand, ControlMode
from axis_os.hal.core.capabilities import RobotCapability


class Destination(Enum):
    CONVEYOR_A = "Convoyeur A (Local)"
    CONVEYOR_B = "Convoyeur B (Export)"
    CONVEYOR_C = "Retour expéditeur"


@dataclass
class Package:
    package_id:  str
    weight_kg:   float
    barcode:     str
    destination: Destination
    side:        str

    @classmethod
    def simulate(cls, index: int) -> "Package":
        return cls(
            package_id=  f"PKG-{index:04d}",
            weight_kg=   round(random.uniform(0.5, 15.0), 2),
            barcode=     f"PKG{index:04d}",
            destination= random.choice(list(Destination)),
            side=        random.choice(["left", "right"]),
        )


# Positions normalisées (positif = flexion — le normalizer gère les inversions G1)
HOME = {
    "left_shoulder_pitch_joint": 0.0, "left_shoulder_roll_joint": 0.0,
    "left_elbow_joint": 0.0, "left_wrist_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.0, "right_shoulder_roll_joint": 0.0,
    "right_elbow_joint": 0.0, "right_wrist_roll_joint": 0.0,
}
REACH_LEFT  = {"left_shoulder_pitch_joint": 0.8, "left_shoulder_roll_joint": 0.3,
               "left_elbow_joint": 0.4, "right_shoulder_pitch_joint": 0.2}
REACH_RIGHT = {"right_shoulder_pitch_joint": 0.8, "right_shoulder_roll_joint": 0.3,
               "right_elbow_joint": 0.4, "left_shoulder_pitch_joint": 0.2}
DEPOSIT_LEFT   = {"left_shoulder_pitch_joint": 0.5,  "left_shoulder_roll_joint": 0.8,  "left_elbow_joint": 0.6}
DEPOSIT_RIGHT  = {"right_shoulder_pitch_joint": 0.5, "right_shoulder_roll_joint": 0.8, "right_elbow_joint": 0.6}
DEPOSIT_CENTER = {"left_shoulder_pitch_joint": 0.6,  "left_elbow_joint": 0.5,
                  "right_shoulder_pitch_joint": 0.6, "right_elbow_joint": 0.5}


def _cmds(pos_dict: Dict, velocity: float = 1.5) -> Dict[str, ActuatorCommand]:
    return {jid: ActuatorCommand(ControlMode.POSITION, position=angle, velocity=velocity)
            for jid, angle in pos_dict.items()}


class PackageSortingApp(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id="com.acme.package_sorting",
            name="Package Sorting Pro",
            version="1.0.0",
            author="ACME Robotics Solutions",
            author_email="dev@acme-robotics.example.com",
            description=(
                "Triage automatisé de colis sur ligne logistique. "
                "3 destinations, cadence jusqu'à 120 colis/heure."
            ),
            category=AppCategory.LOGISTICS,
            permissions=[
                Permission.ACTUATORS_READ,
                Permission.ACTUATORS_WRITE_ARMS,
                Permission.SENSORS_IMU,
                Permission.WATCHDOG_READ,
                Permission.NETWORK_CLOUD,
            ],
            hardware=HardwareRequirements(
                required=[RobotCapability.ARMS, RobotCapability.IMU],
                min_dof=10,
                compatible_models=["G1", "Digit", "Optimus"],
            ),
            pricing_model=PricingModel.PAY_PER_MISSION,
            base_price_eur=1.40,
            max_velocity_ratio=0.7,
            requires_human_supervision=True,
        )

    def on_start(self, ctx: RobotContext) -> bool:
        ctx.log("Initialisation PackageSorting...")
        imu = ctx.get_imu()
        if imu is None:
            ctx.log("ERREUR : IMU non disponible.")
            return False
        imu_data = imu.read()
        if imu_data is None:
            ctx.log("ERREUR : IMU ne répond pas.")
            return False
        ctx.log(f"IMU OK — gravity_z={imu_data.linear_acceleration.z:.2f} m/s²")
        results = ctx.move_arms(_cmds(HOME))
        failed  = [jid for jid, s in results.items() if s.value != 0]
        if failed:
            ctx.log(f"ERREUR joints : {failed}")
            return False
        ctx.log("Position HOME — prêt.")
        return True

    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult:
        batch_size = int(params.get("batch_size", 5))
        speed_mode = params.get("speed_mode", "normal")
        delay = {"safe": 0.08, "normal": 0.04, "fast": 0.02}.get(speed_mode, 0.04)

        ctx.log(f"Mission | lot={batch_size} | mode={speed_mode}")

        sorted_count = 0
        errors       = []
        dest_counts  = {d: 0 for d in Destination}

        for i in range(batch_size):
            pkg = Package.simulate(i + 1)
            ctx.log(f"Colis {pkg.package_id} | {pkg.weight_kg}kg | "
                    f"→ {pkg.destination.value} | côté={pkg.side}")
            try:
                self._sort(ctx, pkg, delay)
                sorted_count += 1
                dest_counts[pkg.destination] += 1
            except Exception as exc:
                errors.append(str(exc))
                ctx.log(f"ERREUR {pkg.package_id}: {exc}")

        ctx.move_arms(_cmds(HOME))
        ctx.log(f"Terminé : {sorted_count}/{batch_size} triés")

        return MissionResult(
            success=         sorted_count == batch_size,
            mission_id=      "",
            app_id=          self.manifest().app_id,
            duration_s=      0.0,
            items_processed= sorted_count,
            error_message=   "; ".join(errors) if errors else None,
            metrics={
                "batch_size":       batch_size,
                "sorted_count":     sorted_count,
                "success_rate_pct": round(sorted_count / batch_size * 100, 1),
                "destinations": {d.name: c for d, c in dest_counts.items() if c > 0},
            }
        )

    def on_stop(self, ctx: RobotContext) -> None:
        ctx.log("Arrêt — retour HOME")
        ctx.move_arms(_cmds(HOME))

    def _sort(self, ctx: RobotContext, pkg: Package, delay: float) -> None:
        reach = REACH_LEFT if pkg.side == "left" else REACH_RIGHT
        ctx.move_arms(_cmds(reach))
        time.sleep(delay)
        ctx.log(f"  Saisie {pkg.package_id}")
        time.sleep(delay * 0.5)

        if pkg.destination == Destination.CONVEYOR_A:
            ctx.move_arms(_cmds(DEPOSIT_LEFT))
            ctx.log("  → Convoyeur A")
        elif pkg.destination == Destination.CONVEYOR_B:
            ctx.move_arms(_cmds(DEPOSIT_RIGHT))
            ctx.log("  → Convoyeur B")
        else:
            ctx.move_arms(_cmds(DEPOSIT_CENTER))
            ctx.log("  → Retour")

        time.sleep(delay)
        ctx.log(f"  Dépôt {pkg.package_id} OK")
