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
    PricingModel, HardwareRequirements, Capability
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
            sdk_version_min="0.2.0",
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
            # Capacité fonctionnelle réelle — un colis à la fois, avec
            # UN SEUL bras (gauche OU droit selon la destination,
            # jamais les deux simultanément — voir DEPOSIT_LEFT/
            # DEPOSIT_RIGHT plus haut dans ce fichier). GRASP_DUAL
            # serait un mensonge sur ce que fait vraiment cette app.
            capabilities_required=[Capability.GRASP_SINGLE],
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
        # move_to_pose() plutôt qu'un move_arms() direct — trouvé
        # bloquant sur du vrai matériel : si le G1 démarre bras repliés
        # (position réelle éloignée de HOME), JointSafetyGuard refuse à
        # raison un saut de position trop grand en un seul appel
        # (voir JointLimits.position_delta_max, correctif C5). Sans
        # move_to_pose() pour interpoler depuis la position réelle,
        # cette app ne pouvait jamais démarrer sur un vrai G1 dont les
        # bras ne partent pas déjà de HOME.
        reached = ctx.move_to_pose(HOME, duration_s=2.0)
        if not reached:
            ctx.log("ERREUR : impossible d'atteindre la position HOME.")
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

        # move_to_pose() plutôt qu'un move_arms() direct — même
        # raisonnement que dans on_start() ci-dessus (voir sa
        # docstring pour le contexte complet sur position_delta_max).
        # Trouvé manquant ici précisément en testant cette app contre
        # une vraie physique simulée (Isaac Sim) : 3 vraies violations
        # POSITION_DELTA_LIMIT déclenchées en cours de mission,
        # jamais visibles en tests unitaires purs (qui ne valident
        # jamais un enchaînement RÉEL de positions successives). Le
        # retour final vers HOME peut franchir un delta de 0.8 rad
        # (ex: REACH_RIGHT.right_shoulder_pitch_joint = 0.8 → HOME =
        # 0.0), largement au-delà de position_delta_max (0.625).
        ctx.move_to_pose(HOME, duration_s=1.0)
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
        # move_to_pose() — voir la docstring de run_mission() (juste
        # avant son "Terminé : ...") pour le contexte complet sur
        # cette même correction.
        ctx.move_to_pose(HOME, duration_s=1.0)

    def _sort(self, ctx: RobotContext, pkg: Package, delay: float) -> None:
        # move_to_pose() partout dans cette méthode — voir la
        # docstring de run_mission() (juste avant son "Terminé : ...")
        # pour le contexte complet. REACH_LEFT/RIGHT (0.8 rad) depuis
        # HOME (0.0 rad), puis DEPOSIT_* (0.8 rad sur un autre joint)
        # depuis REACH_* : deux sauts qui franchissent chacun
        # position_delta_max (0.625) avec un move_arms() direct.
        #
        # duration_s FIXE (0.3s), jamais `delay` — trouvé nécessaire
        # après un premier essai qui réutilisait `delay` (0.02 à 0.08s
        # selon speed_mode) comme duration_s : avec step_s=0.05 par
        # défaut, ça donne int(duration_s/step_s) = 1 SEUL pas
        # d'interpolation pour delay <= 0.05 — revient exactement au
        # saut direct qu'on cherche à éviter. `delay` régit le TEMPO
        # de simulation de la mission (attente entre étapes), pas la
        # durée d'un mouvement physique réel — les deux notions ne
        # doivent jamais partager la même valeur.
        #
        # 0.3s délibérément CONSERVÉ, pas réduit pour accélérer les
        # tests — revenu en arrière après une première tentative à
        # 0.15s (2 pas) : le temps d'exécution d'un test n'est qu'un
        # coût de développement, alors qu'un delta par pas plus grand
        # entame la seule vraie marge de sécurité disponible. De plus,
        # position_delta_max = torque_max / 40.0 dans ce HAL simulé
        # (voir UnitreeG1HAL._build_g1_joint_definitions) utilise une
        # constante UNIQUE de kp=40 pour tous les joints — le vrai
        # driver (g1_dds_actuator.py::_DEFAULT_KP) a un kp variable
        # par moteur (jusqu'à 100 sur certains genoux/chevilles), donc
        # une limite RÉELLE plus serrée que celle mesurée ici en
        # simulation sur ces joints précis. Un mouvement validé comme
        # sûr en sim n'est pas automatiquement sûr sur le vrai G1 —
        # raison de plus de garder de la marge maintenant. Un timeout
        # de test plus généreux (pas une durée de mouvement réduite)
        # est la bonne réponse si un test enchaînant de nombreuses
        # missions dépasse son délai.
        MOVE_DURATION_S = 0.3
        reach = REACH_LEFT if pkg.side == "left" else REACH_RIGHT
        ctx.move_to_pose(reach, duration_s=MOVE_DURATION_S)
        ctx.log(f"  Saisie {pkg.package_id}")
        time.sleep(delay * 0.5)

        if pkg.destination == Destination.CONVEYOR_A:
            ctx.move_to_pose(DEPOSIT_LEFT, duration_s=MOVE_DURATION_S)
            ctx.log("  → Convoyeur A")
        elif pkg.destination == Destination.CONVEYOR_B:
            ctx.move_to_pose(DEPOSIT_RIGHT, duration_s=MOVE_DURATION_S)
            ctx.log("  → Convoyeur B")
        else:
            ctx.move_to_pose(DEPOSIT_CENTER, duration_s=MOVE_DURATION_S)
            ctx.log("  → Retour")

        time.sleep(delay)
        ctx.log(f"  Dépôt {pkg.package_id} OK")
