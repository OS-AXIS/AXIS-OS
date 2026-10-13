"""
AXIS-OS Robotics Play Store
App : PackageSorting v1.0.0
================================
Tri automatisé de colis sur une ligne logistique. C'est la première app
tierce (publiée par un autre éditeur que le projet) du Play Store, la
boutique d'apps d'AXIS-OS : chaque colis est saisi, puis déposé vers
l'une de trois destinations (deux convoyeurs, ou retour à l'expéditeur).

Auteur  : ACME Robotics Solutions
Prix    : le client paie 2.00 € par mission, dont 1.40 € nets pour le
          développeur et 0.60 € de commission (30 %) pour la plateforme.
Licence : Apache 2.0, comme tout le dépôt (voir LICENSE). L'auteur ci-dessus
est fictif : cette app montre ce qu'un développeur tiers publierait sur le
Play Store, où une app peut avoir sa propre licence.
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
        """Fabrique un colis fictif, aux caractéristiques tirées au hasard
        (simulation : aucun vrai colis n'est lu)."""
        return cls(
            package_id=  f"PKG-{index:04d}",
            weight_kg=   round(random.uniform(0.5, 15.0), 2),
            barcode=     f"PKG{index:04d}",
            destination= random.choice(list(Destination)),
            side=        random.choice(["left", "right"]),
        )


# Poses des bras, en radians, dans l'espace normalisé d'AXIS-OS : positif =
# flexion, quel que soit le robot. Le KinematicNormalizer
# (axis_os/hal/core/kinematic_normalizer.py) se charge des articulations du
# G1 (le robot humanoïde Unitree G1) dont le sens est inversé par rapport à
# cette convention. HOME est la pose de départ et de repos (tout à 0).
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
    """Transforme une pose (articulation → angle) en commandes de position,
    toutes à la même vitesse."""
    return {jid: ActuatorCommand(ControlMode.POSITION, position=angle, velocity=velocity)
            for jid, angle in pos_dict.items()}


class PackageSortingApp(RobotApp):
    """App de tri de colis : saisit chaque colis d'un lot et le dépose
    vers sa destination."""

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
            # Capacité fonctionnelle réellement exercée (ce que l'app doit
            # savoir FAIRE) : un colis à la fois, saisi d'UN SEUL bras,
            # gauche ou droit, jamais les deux ensemble (voir REACH_LEFT/
            # REACH_RIGHT et DEPOSIT_LEFT/DEPOSIT_RIGHT plus haut dans ce
            # fichier). Déclarer GRASP_DUAL (saisie à deux bras) serait
            # mentir sur ce que fait vraiment cette app. Nuance :
            # REACH_LEFT/REACH_RIGHT inclinent aussi un peu l'épaule
            # opposée (0.2 rad), et la pose DEPOSIT_CENTER (retour
            # expéditeur) bouge les deux bras.
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
        # Sans IMU (la centrale inertielle, qui mesure l'orientation et les
        # accélérations du robot) disponible et qui répond, l'app refuse
        # de démarrer.
        imu = ctx.get_imu()
        if imu is None:
            ctx.log("ERREUR : IMU non disponible.")
            return False
        imu_data = imu.read()
        if imu_data is None:
            ctx.log("ERREUR : IMU ne répond pas.")
            return False
        ctx.log(f"IMU OK — gravity_z={imu_data.linear_acceleration.z:.2f} m/s²")
        # move_to_pose() plutôt qu'un move_arms() direct : le problème a
        # été trouvé bloquant sur du vrai matériel. Si le G1 démarre bras
        # repliés (position réelle éloignée de HOME), JointSafetyGuard (le
        # garde-fou de sécurité de chaque articulation) refuse, à raison,
        # un saut de position trop grand en un seul appel. C'est la borne
        # JointLimits.position_delta_max, ajoutée après un audit de
        # sécurité externe (voir CHANGELOG.md et
        # docs/audit_externe_2026-09-14.md). Sans move_to_pose(), qui
        # interpole par petits pas depuis la position réelle, cette app ne
        # pouvait jamais démarrer sur un vrai G1 dont les bras ne partent
        # pas déjà de HOME.
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

        # move_to_pose() plutôt qu'un move_arms() direct, pour la même
        # raison que dans on_start() ci-dessus (voir son commentaire sur
        # position_delta_max). Cet appel manquait ici : on l'a constaté
        # précisément en testant l'app contre une physique simulée
        # réaliste (Isaac Sim, le simulateur physique de NVIDIA). Trois
        # violations POSITION_DELTA_LIMIT réelles se déclenchaient en
        # cours de mission, jamais visibles dans les tests unitaires purs,
        # qui ne valident jamais un enchaînement RÉEL de positions
        # successives. Le retour final vers HOME peut franchir un écart de
        # 0.8 rad (par exemple REACH_RIGHT.right_shoulder_pitch_joint =
        # 0.8 → HOME = 0.0), bien au-delà de position_delta_max
        # (0.625 rad).
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
        # move_to_pose(), pour la même correction : contexte complet dans
        # le commentaire placé juste avant le « Terminé : ... » de
        # run_mission().
        ctx.move_to_pose(HOME, duration_s=1.0)

    def _sort(self, ctx: RobotContext, pkg: Package, delay: float) -> None:
        # move_to_pose() partout dans cette méthode (contexte complet dans
        # le commentaire placé juste avant le « Terminé : ... » de
        # run_mission()). Avec un move_arms() direct, deux sauts
        # dépasseraient chacun position_delta_max (0.625 rad) : vers
        # REACH_LEFT/RIGHT (0.8 rad) depuis HOME (0.0 rad), puis vers
        # DEPOSIT_* (0.8 rad sur une autre articulation) depuis REACH_*.
        #
        # duration_s est FIXE (0.3 s), jamais égale à `delay`. Un premier
        # essai réutilisait `delay` (0.02 à 0.08 s selon speed_mode) comme
        # duration_s. Or, avec le pas par défaut step_s = 0.05 s,
        # int(duration_s / step_s) ne donne qu'UN SEUL pas d'interpolation
        # dès que delay <= 0.05 : on retombe exactement sur le saut direct
        # qu'on cherche à éviter. `delay` règle le tempo simulé de la
        # mission (l'attente entre deux étapes), pas la durée d'un
        # mouvement physique réel : les deux notions ne doivent jamais
        # partager la même valeur.
        #
        # 0.3 s est délibérément CONSERVÉ, et non réduit pour accélérer
        # les tests : une première tentative à 0.15 s (2 pas) a été
        # annulée (voir CHANGELOG.md). La durée d'un test n'est qu'un coût
        # de développement, alors qu'un écart plus grand à chaque pas
        # entame la seule vraie marge de sécurité disponible. De plus,
        # dans le HAL simulé (le HAL est la couche d'abstraction matérielle,
        # seul chemin vers les moteurs et les capteurs),
        # position_delta_max = torque_max / 40.0 (voir
        # G1SimulatedActuator dans axis_os/hal/drivers/unitree_g1/robot.py)
        # repose sur une constante UNIQUE kp = 40 pour toutes les
        # articulations ; kp est la raideur du régulateur PD
        # (proportionnel-dérivé) intégré au moteur. Le pilote du vrai G1
        # (_DEFAULT_KP dans g1_dds_actuator.py) a un kp propre à chaque
        # moteur (jusqu'à 100 sur les genoux), donc une limite RÉELLE plus
        # serrée sur ces articulations que celle mesurée ici en
        # simulation. Un mouvement validé comme sûr en simulation n'est
        # pas automatiquement sûr sur le vrai G1 : raison de plus de
        # garder de la marge dès maintenant. Si un test qui enchaîne de
        # nombreuses missions dépasse son délai, la bonne réponse est un
        # délai de test plus généreux, pas une durée de mouvement réduite.
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
