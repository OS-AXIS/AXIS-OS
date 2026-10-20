"""
AXIS-OS — WarehouseOpsApp
============================
Troisième app de référence du Play Store (la boutique d'apps d'AXIS-OS),
conçue explicitement comme test de charge maximal du SDK (le kit de
développement des apps). Elle ne vise pas un usage réel : elle doit
exercer chaque permission, chaque type de commande et chaque mode
d'échec que le reste de la pile logicielle est censé gérer, à savoir
Safety (les garde-fous de sécurité), Watchdog (le superviseur qui
surveille l'état des moteurs), Sandbox (le processus isolé dans lequel
tourne chaque app), MotionStateTracker (le registre de l'état de
mouvement de chaque articulation) et la journalisation structurée.

Scénario : un robot d'entrepôt enchaîne plusieurs PHASES dans une seule
mission. PackageSortingApp répète une action simple et SecurityPatrolApp
une boucle homogène ; ici, chaque phase sollicite une partie différente
du HAL (la couche d'abstraction matérielle, seul chemin vers les moteurs
et les capteurs) :

  Phase 1 — RECONNAISSANCE : lecture des capteurs (IMU, la centrale
            inertielle ; capteurs de force et de couple), scan de la
            zone (caméra), décision simulée par une IA de type VLA
            (modèle vision-langage-action).
  Phase 2 — NAVIGATION : déplacement (jambes) vers plusieurs points.
  Phase 3 — MANIPULATION : saisie d'un objet (bras), avec contrôle du
            couple mesuré pour détecter une prise ratée.
  Phase 4 — FACTURATION DYNAMIQUE : le prix doit dépendre du poids
            réellement manipulé, et non d'un tarif fixe comme dans les
            deux apps précédentes. Limite actuelle : l'app calcule et
            rapporte ce poids (métrique total_weight_handled_kg), mais
            la facturation du Play Store ne s'en sert pas encore : elle
            facture le même prix fixe à chaque mission (base_price_eur,
            plus la commission de la plateforme).
  Phase 5 — ESCALADE : si un problème survient en phase 3, la mission
            ne s'arrête pas en silence ; elle bascule en mode dégradé
            (DEGRADED) et le signale explicitement.

Portée volontairement limitée, et dite comme telle, comme dans le reste
du projet :
  - Caméra, IMU, force et couple : aucun vrai capteur physique n'est lu.
    Le scan de zone et le couple mesuré après la saisie sont des stubs
    (des bouchons : simulations simples qui remplacent le vrai
    composant), tirés au hasard et reproductibles si une graine (seed,
    la valeur de départ du tirage) est fournie. RobotContext n'expose
    d'ailleurs pas encore de lecture de force et de couple :
    SENSORS_FORCE_TORQUE est marquée « NON APPLIQUÉE » dans manifest.py.
  - RobotContext expose désormais emergency_stop(), qui permet à une app
    de déclencher elle-même un e-stop (arrêt d'urgence). Cette méthode a
    été ajoutée à la suite d'un audit externe du code (septembre 2026,
    voir CHANGELOG.md). EMERGENCY_STOP reste déclarée, mais aucune ligne
    de cette app de démonstration ne l'exerce.
  - FLEET_BROADCAST (diffuser un message à toute une flotte de robots) a
    été retirée le 2026-09-15. Le Fleet Manager, futur gestionnaire de
    flotte, n'est toujours pas implémenté (voir
    docs/fleet_manager_design.md), et cette permission est réservée aux
    apps système (niveau de protection SIGNATURE : accordée seulement
    aux apps signées avec la clé de la plateforme). Une app tierce qui la
    déclare sans jamais l'exercer ne fait que se rendre non installable,
    pour rien.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE). Proposée en
exemple, comme PackageSortingApp et SecurityPatrolApp.
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
    """Phase en cours d'une mission (voir la docstring du module)."""
    RECONNAISSANCE = auto()
    NAVIGATION      = auto()
    MANIPULATION     = auto()
    BILLING           = auto()
    DEGRADED           = auto()  # atteinte seulement après une saisie ratée


@dataclass
class GraspAttempt:
    """Résultat d'une tentative de saisie. measured_torque est le couple
    simulé après fermeture de la pince ; l'app l'interprète comme un poids
    en kg (voir _attempt_grasp())."""
    object_id:       str
    target_weight_kg: float
    measured_torque:  float
    success:          bool


class WarehouseOpsApp(RobotApp):
    """
    App volontairement exigeante : navigation, manipulation, facturation
    dynamique et détection d'échec dans UNE seule mission, pour servir de
    test d'intégration applicatif maximal.
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
        Stub : simule une lecture des capteurs (caméra et IMU) en tirant
        au hasard le nombre d'objets présents (0 à 3). Rend une
        description de la scène, comme le ferait un futur VLAConnector
        (connecteur vers un modèle VLA) placé en amont de cette app.
        Aucun capteur réel n'est lu.
        """
        objects_present = self._rng.randint(0, 3)
        return {"zone": zone, "objects_detected": objects_present}

    # ── Phase 2 : navigation ─────────────────────────────────────

    def _navigate_to(self, ctx: RobotContext, zone: str) -> bool:
        """
        Déplacement réel par le HAL (jambes) : exerce vraiment la
        permission ACTUATORS_WRITE_LEGS, au lieu de la déclarer dans le
        manifeste (la fiche qui déclare permissions, capacités, quotas et
        prix de l'app) sans jamais s'en servir. Comme dans
        SecurityPatrolApp, ce n'est qu'une commande de posture symbolique
        des deux hanches, pas une vraie navigation. Rend False si l'une
        des commandes n'a pas de résultat (None), ce que run_mission()
        traite comme un rejet par le HAL.
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
        Mouvement réel par le HAL (bras gauche) : exerce la permission
        ACTUATORS_WRITE_ARMS.

        Le couple mesuré après la saisie dit si elle a réussi. Un couple
        trop faible signifie que la pince s'est refermée sur du vide
        (objet manqué) ; un couple anormalement élevé, que l'objet est
        plus lourd que prévu (poids mal déclaré).
        """
        ctx.move_arms({
            "left_shoulder_pitch_joint": ActuatorCommand(ControlMode.POSITION, position=0.4),
            "left_elbow_joint":           ActuatorCommand(ControlMode.POSITION, position=0.6),
        })

        # Stub : simule le couple mesuré après la fermeture de la pince.
        #   70 % du temps : proche du poids annoncé (saisie réussie) ;
        #   20 % du temps : couple quasi nul (objet manqué) ;
        #   10 % du temps : couple bien supérieur (poids sous-déclaré).
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
        # Générateur RECRÉÉ à chaque mission. Pourquoi : chaque mission
        # s'exécute dans un processus à part (la sandbox), qui repartait
        # autrefois d'une copie figée de l'app ; un générateur créé une
        # seule fois dans __init__ tirait alors les mêmes nombres à chaque
        # mission (20 échecs sur 20, ou 0 sur 20, au lieu d'environ 30 %).
        # Le recréer ici garde chaque mission indépendante, et une graine
        # (seed) peut la rendre reproductible.
        self._rng = random.Random(params.get("seed"))
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

            # PHASE 4 — Facturation dynamique : elle doit dépendre du poids
            # réellement manipulé avec succès, et non d'un tarif fixe par
            # mission. L'app se contente ici de rapporter ce poids
            # (total_weight_handled_kg) ; voir la docstring du module.
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


# ── Manifeste ────────────────────────────────────────────────────────

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

    # Le jeu de permissions le plus large des trois apps de référence (tri
    # de colis, ronde, entrepôt). Il exerce délibérément
    # ACTUATORS_WRITE_ARMS et ACTUATORS_WRITE_LEGS dans la même mission,
    # et ajoute des permissions qu'aucune des deux autres ne déclare
    # (SENSORS_FORCE_TORQUE, EMERGENCY_STOP).
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

    # Capacités fonctionnelles réellement exercées : WALK (move_legs, pour
    # se déplacer entre les postes), GRASP_SINGLE (move_arms ne commande
    # que le bras gauche, jamais les deux bras à la fois : voir
    # _attempt_grasp() plus haut dans ce fichier) et CAMERA_RGB (l'image
    # couleur, d'où SENSORS_CAMERA ci-dessus ; aucune image de profondeur
    # n'est utilisée).
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

    # Facturé à chaque mission. L'ajustement prévu au poids réellement
    # manipulé n'est pas encore appliqué (voir la docstring du module).
    pricing_model=PricingModel.PAY_PER_MISSION,
    base_price_eur=3.50,

    max_velocity_ratio=0.6,
    requires_human_supervision=True,  # manipulation physique : supervision requise
)
