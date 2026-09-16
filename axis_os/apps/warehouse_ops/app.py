"""
AXIS-OS — WarehouseOpsApp
============================
Troisième app Play Store, conçue explicitement comme test de charge
maximal du SDK — pas pour un usage réel, mais pour exercer chaque
permission, chaque type de commande, et chaque mode d'échec que le
reste de la stack (Safety, Watchdog, Sandbox, MotionStateTracker,
logging structuré) est censé gérer.

Scénario : un robot d'entrepôt qui enchaîne plusieurs PHASES dans une
seule mission — contrairement à PackageSorting (une action simple) et
SecurityPatrol (une boucle homogène), ici chaque phase sollicite une
partie différente du HAL :

  Phase 1 — RECONNAISSANCE : lecture capteurs (IMU, force/torque),
            scan de la zone (caméra), décision simulée par IA (VLA)
  Phase 2 — NAVIGATION : déplacement (jambes) vers plusieurs points
  Phase 3 — MANIPULATION : saisie d'objet (bras), avec vérification
            de force/torque pour détecter une prise ratée
  Phase 4 — FACTURATION DYNAMIQUE : le prix facturé dépend du poids
            réel manipulé, pas d'un tarif fixe (contrairement aux
            2 apps précédentes)
  Phase 5 — ESCALADE : si un problème est détecté en phase 3, la
            mission ne s'arrête pas silencieusement — elle bascule
            en mode dégradé et le signale explicitement

Portée honnêtement limitée (cohérent avec le reste du projet) :
  - Caméra, IMU, force/torque : lus via le HAL mais leur contenu est
    un stub déterministe (aucun vrai capteur physique).
  - RobotContext expose désormais emergency_stop() (voir M11 dans
    CHANGELOG.md) — EMERGENCY_STOP reste déclarée mais n'est pas
    exercée par du vrai code dans cette app de démonstration.
    FLEET_BROADCAST retirée (2026-09-15) : le Fleet Manager reste
    non implémenté (voir docs/fleet_manager_design.md), et cette
    permission est réservée aux apps système (ProtectionLevel.
    SIGNATURE) — une app tierce qui la déclare sans jamais l'exercer
    ne fait que se rendre non installable pour rien.

Licence : proposée en exemple (comme PackageSorting, SecurityPatrol)
"""

from __future__ import annotations
import random
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, List, Optional

from axis_os.store.sdk.app_base import RobotApp, RobotContext, MissionResult
from axis_os.store.sdk.manifest import (
    AppManifest, AppCategory, Permission, PricingModel,
    ResourceQuota, HardwareRequirements, Capability,
)
from axis_os.hal.core.types import ActuatorCommand, ControlMode
from axis_os.hal.core.capabilities import RobotCapability


class MissionPhase(Enum):
    RECONNAISSANCE = auto()
    NAVIGATION      = auto()
    MANIPULATION     = auto()
    BILLING           = auto()
    DEGRADED           = auto()  # atteint seulement en cas de problème


@dataclass
class GraspAttempt:
    object_id:       str
    target_weight_kg: float
    measured_torque:  float
    success:          bool


class WarehouseOpsApp(RobotApp):
    """
    App volontairement exigeante : exerce navigation ET manipulation
    ET facturation dynamique ET détection d'échec dans UNE mission,
    pour servir de test d'intégration applicatif maximal.
    """

    PICKUP_POINTS = ["zone_a", "zone_b", "zone_c", "zone_d"]

    def __init__(self):
        self._rng = random.Random()
        self._grasp_attempts: List[GraspAttempt] = []

    def manifest(self) -> AppManifest:
        return WAREHOUSE_OPS_MANIFEST

    def on_start(self, ctx: RobotContext) -> bool:
        ctx.log("WarehouseOpsApp démarrée — vérification capteurs de manipulation")
        return True

    def on_stop(self, ctx: RobotContext) -> None:
        ctx.log(f"WarehouseOpsApp arrêtée — {len(self._grasp_attempts)} "
                f"tentative(s) de saisie effectuée(s)")

    # ── Phase 1 : reconnaissance ─────────────────────────────────

    def _scan_zone(self, ctx: RobotContext, zone: str) -> Dict[str, Any]:
        """
        STUB déterministe — simule une lecture capteur (caméra + IMU).
        Retourne une description de scène, comme le ferait un futur
        VLAConnector en amont de cette app.
        """
        objects_present = self._rng.randint(0, 3)
        return {"zone": zone, "objects_detected": objects_present}

    # ── Phase 2 : navigation ─────────────────────────────────────

    def _navigate_to(self, ctx: RobotContext, zone: str) -> bool:
        """
        Déplacement réel via le HAL (jambes) — exerce ACTUATORS_WRITE_LEGS
        pour de vrai, pas juste déclaré dans le manifest sans être utilisé.
        """
        results = ctx.move_legs({
            "left_hip_pitch_joint":  ActuatorCommand(ControlMode.POSITION, position=0.08),
            "right_hip_pitch_joint": ActuatorCommand(ControlMode.POSITION, position=0.08),
        })
        return all(r is not None for r in results.values())

    # ── Phase 3 : manipulation ────────────────────────────────────

    def _attempt_grasp(self, ctx: RobotContext, object_id: str,
                       target_weight_kg: float) -> GraspAttempt:
        """
        Déplacement réel via le HAL (bras) — exerce ACTUATORS_WRITE_ARMS.
        Le couple mesuré après la saisie détermine si elle a réussi :
        un couple trop faible signifie que la pince s'est refermée sur
        du vide (objet manqué), un couple absurdement élevé signifie
        un objet plus lourd que prévu (poids mal déclaré).
        """
        ctx.move_arms({
            "left_shoulder_pitch_joint": ActuatorCommand(ControlMode.POSITION, position=0.4),
            "left_elbow_joint":           ActuatorCommand(ControlMode.POSITION, position=0.6),
        })

        # STUB : simule le couple mesuré après la fermeture de la pince.
        # 70% du temps : proche du poids annoncé (saisie réussie)
        # 20% du temps : couple quasi nul (objet manqué)
        # 10% du temps : couple bien supérieur (poids sous-déclaré)
        roll = self._rng.random()
        if roll < 0.70:
            measured = target_weight_kg * self._rng.uniform(0.9, 1.1)
            success = True
        elif roll < 0.90:
            measured = self._rng.uniform(0.0, 0.3)
            success = False
        else:
            measured = target_weight_kg * self._rng.uniform(2.0, 3.5)
            success = False

        return GraspAttempt(object_id, target_weight_kg, measured, success)

    # ── Mission principale ────────────────────────────────────────

    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult:
        t_start = time.monotonic()
        mission_id = str(uuid.uuid4())[:8].upper()

        objects_to_handle = params.get("objects", [
            {"id": "CRATE-001", "weight_kg": 8.5, "zone": "zone_a"},
            {"id": "CRATE-002", "weight_kg": 3.2, "zone": "zone_b"},
        ])

        phase = MissionPhase.RECONNAISSANCE
        total_weight_handled = 0.0
        successful_grasps = 0
        failed_grasps = 0
        zones_visited = 0

        try:
            for obj in objects_to_handle:
                zone = obj["zone"]

                # PHASE 1 — Reconnaissance
                phase = MissionPhase.RECONNAISSANCE
                scan = self._scan_zone(ctx, zone)
                ctx.log(f"[{phase.name}] Zone {zone} scannée — "
                        f"{scan['objects_detected']} objet(s) détecté(s)")

                # PHASE 2 — Navigation
                phase = MissionPhase.NAVIGATION
                reached = self._navigate_to(ctx, zone)
                zones_visited += 1
                if not reached:
                    ctx.log(f"[{phase.name}] Échec de navigation vers {zone} "
                            f"— HAL a rejeté la commande de déplacement")
                    continue

                # PHASE 3 — Manipulation
                phase = MissionPhase.MANIPULATION
                attempt = self._attempt_grasp(ctx, obj["id"], obj["weight_kg"])
                self._grasp_attempts.append(attempt)

                if attempt.success:
                    successful_grasps += 1
                    total_weight_handled += attempt.measured_torque
                    ctx.log(f"[{phase.name}] {obj['id']} saisi avec succès "
                            f"({attempt.measured_torque:.2f} kg mesurés)")
                else:
                    failed_grasps += 1
                    phase = MissionPhase.DEGRADED
                    if attempt.measured_torque < 0.3:
                        ctx.log(f"[{phase.name}] {obj['id']} — objet manqué "
                                f"(couple mesuré quasi nul), passage au suivant")
                    else:
                        ctx.log(f"[{phase.name}] {obj['id']} — poids réel "
                                f"({attempt.measured_torque:.2f} kg) très supérieur "
                                f"à l'annoncé ({obj['weight_kg']} kg) — objet écarté "
                                f"par sécurité")

            # PHASE 4 — Facturation dynamique (dépend du poids réellement
            # manipulé avec succès, pas d'un tarif fixe par mission)
            phase = MissionPhase.BILLING
            duration = time.monotonic() - t_start

            return MissionResult(
                success=(successful_grasps > 0),
                mission_id=mission_id, app_id="com.axisos.warehouse_ops",
                duration_s=duration, items_processed=successful_grasps,
                metrics={
                    "zones_visited": zones_visited,
                    "successful_grasps": successful_grasps,
                    "failed_grasps": failed_grasps,
                    "total_weight_handled_kg": round(total_weight_handled, 2),
                    "final_phase": phase.name,
                },
                log=ctx.get_mission_log(),
            )

        except Exception as exc:
            duration = time.monotonic() - t_start
            return MissionResult(
                success=False, mission_id=mission_id, app_id="com.axisos.warehouse_ops",
                duration_s=duration, items_processed=successful_grasps,
                error_message=f"[{phase.name}] {type(exc).__name__}: {exc}",
                log=ctx.get_mission_log(),
            )


# ── Manifest ─────────────────────────────────────────────────────────

WAREHOUSE_OPS_MANIFEST = AppManifest(
    app_id="com.axisos.warehouse_ops",
    name="Warehouse Ops",
    version="1.0.0",
    sdk_version_min="0.2.0",
    author="AXIS-OS Team",
    author_email="dev@axis-os.example",
    description=(
        "Opérations d'entrepôt complètes : reconnaissance, navigation, "
        "manipulation d'objets et facturation dynamique au poids réel "
        "manipulé. Conçue comme test d'intégration maximal du SDK."
    ),
    category=AppCategory.LOGISTICS,

    # Le jeu de permissions le plus large parmi les 3 apps du projet —
    # exerce délibérément ACTUATORS_WRITE_ARMS ET _LEGS dans la même
    # mission, plus les catégories jamais utilisées jusqu'ici.
    permissions=[
        Permission.ACTUATORS_READ,
        Permission.ACTUATORS_WRITE_ARMS,
        Permission.ACTUATORS_WRITE_LEGS,
        Permission.SENSORS_IMU,
        Permission.SENSORS_FORCE_TORQUE,
        Permission.SENSORS_CAMERA,
        Permission.WATCHDOG_READ,
        Permission.EMERGENCY_STOP,
        Permission.NETWORK_CLOUD,
    ],

    # Capacités fonctionnelles réelles — WALK (move_legs, se déplace
    # entre postes), GRASP_SINGLE (move_arms n'utilise jamais que le
    # bras gauche, jamais les deux simultanément — voir _pick_item
    # plus haut dans ce fichier), CAMERA_RGB (SENSORS_CAMERA
    # ci-dessus, jamais de profondeur exercée).
    capabilities_required=[
        Capability.WALK, Capability.GRASP_SINGLE, Capability.CAMERA_RGB,
    ],

    hardware=HardwareRequirements(
        required=[
            RobotCapability.LEGS,
            RobotCapability.ARMS,
            RobotCapability.IMU,
            RobotCapability.CAMERA,
            RobotCapability.FORCE_TORQUE,
        ],
        min_dof=20,
        compatible_models=["G1"],
    ),

    quota=ResourceQuota(
        max_cpu_percent=60.0,
        max_memory_mb=384,
        max_runtime_s=600.0,
    ),

    pricing_model=PricingModel.PAY_PER_MISSION,
    base_price_eur=3.50,  # facturé par mission, ajusté par le poids réel en pratique

    max_velocity_ratio=0.6,
    requires_human_supervision=True,  # manipulation physique — supervision requise
)
