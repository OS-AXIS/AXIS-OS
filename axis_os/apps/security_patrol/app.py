"""
AXIS-OS — SecurityPatrolApp
==============================
Deuxième app Play Store, pensée pour tester le SDK sur un cas d'usage
très différent de PackageSortingApp : planification de chemin autonome,
pauses d'observation, interaction avec une personne détectée, et
escalade d'alerte — plutôt que du tri d'objets déterministe.

Scénario (spécifié par l'utilisateur) :
  - Se déclenche à une heure planifiée (23h par défaut, configurable)
  - Génère un chemin de ronde ALÉATOIRE à travers des checkpoints connus
  - S'arrête à intervalles pour "observer" (pause + scan)
  - Si une personne est détectée : demande un badge
      → badge valide (base connue)  → laisse passer, continue la ronde
      → pas de badge / invalide      → déclenche une ALERTE
  - L'alerte est envoyée via NETWORK_CLOUD (vers un serveur de sécurité),
    PAS un appel police automatique — décision produit volontaire,
    voir note de sécurité en bas de fichier.

Portée volontairement simulée à ce stade (documentée honnêtement) :
  - La détection de personne et la lecture de badge sont des STUBS
    déterministes, exactement comme OpenVLAConnector — aucune caméra
    ni capteur biométrique réel n'existe encore dans le HAL.
  - Le "chemin aléatoire" est un ordre de checkpoints pré-définis,
    pas une planification de trajectoire réelle (pas de Nav2/SLAM
    branché à ce stade).

Licence : proposée en exemple public (comme PackageSortingApp)
"""

from __future__ import annotations
import random
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from axis_os.store.sdk.app_base import RobotApp, RobotContext, MissionResult
from axis_os.store.sdk.manifest import (
    AppManifest, AppCategory, Permission, PricingModel,
    ResourceQuota, HardwareRequirements,
)
from axis_os.hal.core.types import ActuatorCommand, ControlMode
from axis_os.hal.core.capabilities import RobotCapability


# ── Base de badges connue (stub — remplacée par une vraie base en prod) ──

KNOWN_BADGES = {
    "BADGE-4471": "Agent d'entretien de nuit",
    "BADGE-1092": "Responsable sécurité",
    "BADGE-7734": "Technicien maintenance",
}


class AlertSeverity:
    """Pas un Enum partagé volontairement : sévérité propre à CETTE app,
    distincte de AlertSeverity du Watchdog (qui concerne le robot lui-même,
    pas une alerte de sécurité de site)."""
    INFO    = "INFO"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"


@dataclass
class PatrolCheckpoint:
    checkpoint_id: str
    label:         str
    x: float
    y: float


@dataclass
class SecurityAlert:
    alert_id:    str
    severity:    str
    checkpoint:  str
    reason:      str
    timestamp:   float
    badge_seen:  Optional[str] = None

    def __str__(self) -> str:
        return (f"[ALERTE {self.severity}] {self.reason} | "
                f"checkpoint={self.checkpoint} | id={self.alert_id}")


class SecurityPatrolApp(RobotApp):
    """
    App de ronde de sécurité nocturne autonome.

    Le manifest déclare des permissions plus larges que PackageSortingApp
    (SENSORS_CAMERA, SENSORS_MICROPHONE, NETWORK_CLOUD) — c'est le premier
    vrai test du système de permissions granulaires sur un cas d'usage
    qui en a réellement besoin.
    """

    DEFAULT_CHECKPOINTS = [
        PatrolCheckpoint("entrance",   "Entrée principale",     0.0,  0.0),
        PatrolCheckpoint("hallway_a",  "Couloir A",              5.0,  2.0),
        PatrolCheckpoint("storage",    "Zone de stockage",       8.0,  6.0),
        PatrolCheckpoint("office_1",   "Bureau 1",              10.0,  3.0),
        PatrolCheckpoint("office_2",   "Bureau 2",              12.0,  3.0),
        PatrolCheckpoint("server_room","Salle serveurs",          9.0,  9.0),
        PatrolCheckpoint("exit_rear",  "Sortie arrière",          2.0, 10.0),
    ]

    def __init__(self):
        self._alerts: List[SecurityAlert] = []
        self._rng = random.Random()  # instance dédiée, jamais random global

    def on_start(self, ctx: RobotContext) -> bool:
        ctx.log("SecurityPatrolApp démarrée — vérification hardware")
        # Permissions déjà vérifiées par le Play Store à l'installation ;
        # on_start confirme juste que le contexte est opérationnel.
        return True

    def on_stop(self, ctx: RobotContext) -> None:
        ctx.log(f"SecurityPatrolApp arrêtée — {len(self._alerts)} alerte(s) émise(s) au total")

    def manifest(self) -> AppManifest:
        return SECURITY_PATROL_MANIFEST

    # ── Génération du chemin de ronde ──────────────────────────────

    def _generate_random_path(
        self, checkpoints: List[PatrolCheckpoint], seed: Optional[int] = None
    ) -> List[PatrolCheckpoint]:
        """
        Génère un ordre aléatoire de checkpoints. Toujours commence par
        'entrance' (point de départ logique du robot), le reste est
        mélangé — évite qu'un rôdeur apprenne un horaire de passage fixe.
        """
        rng = random.Random(seed) if seed is not None else self._rng
        start = [c for c in checkpoints if c.checkpoint_id == "entrance"]
        rest  = [c for c in checkpoints if c.checkpoint_id != "entrance"]
        rng.shuffle(rest)
        return start + rest

    # ── Détection et interaction (stub déterministe) ────────────────

    def _scan_for_person(self, ctx: RobotContext, checkpoint: PatrolCheckpoint) -> bool:
        """
        STUB : simule une détection de présence humaine.
        En production, ceci utiliserait SENSORS_CAMERA (vision) et/ou
        SENSORS_MICROPHONE. Ici, comportement déterministe simple pour
        rester testable sans dépendance externe.
        """
        # Détection simulée : 1 checkpoint sur 4 en moyenne a "quelqu'un"
        return self._rng.random() < 0.25

    def _request_badge(self, ctx: RobotContext, checkpoint: PatrolCheckpoint) -> Optional[str]:
        """
        STUB : simule la demande de badge à la personne détectée.
        Retourne le badge présenté, ou None si la personne ne répond pas.
        """
        # Simule 3 scénarios : badge valide (60%), pas de réponse (20%),
        # badge invalide (20%)
        roll = self._rng.random()
        if roll < 0.60:
            return self._rng.choice(list(KNOWN_BADGES.keys()))
        elif roll < 0.80:
            return None  # aucune réponse
        else:
            return "BADGE-0000"  # badge inconnu / invalide

    def _check_badge(self, badge: Optional[str]) -> bool:
        return badge is not None and badge in KNOWN_BADGES

    # ── Mission principale ────────────────────────────────────────

    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult:
        t_start = time.monotonic()
        mission_id = str(uuid.uuid4())[:8].upper()

        checkpoints = params.get("checkpoints", self.DEFAULT_CHECKPOINTS)
        seed        = params.get("seed", None)  # utile pour rejouer un test

        path = self._generate_random_path(checkpoints, seed=seed)
        ctx.log(f"Ronde {mission_id} — chemin généré : "
                f"{' → '.join(c.checkpoint_id for c in path)}")

        checkpoints_visited = 0
        persons_encountered  = 0
        alerts_triggered      = 0

        try:
            for checkpoint in path:
                ctx.log(f"→ Arrivée à {checkpoint.label} ({checkpoint.checkpoint_id})")
                # Déplacement réel vers le checkpoint — exerce vraiment
                # ACTUATORS_WRITE_LEGS (pas seulement déclarée dans le
                # manifest sans jamais être utilisée). La navigation
                # complète (chemin, évitement d'obstacle) reste hors-scope
                # sans Nav2 branché — ceci n'est qu'une commande de posture
                # symbolique par checkpoint pour exercer le chemin HAL réel.
                ctx.move_legs({
                    "left_hip_pitch": ActuatorCommand(ControlMode.POSITION, position=0.05),
                    "right_hip_pitch": ActuatorCommand(ControlMode.POSITION, position=0.05),
                })

                checkpoints_visited += 1

                # Pause d'observation — comportement demandé explicitement
                ctx.log(f"  Pause d'observation à {checkpoint.label}")

                if self._scan_for_person(ctx, checkpoint):
                    persons_encountered += 1
                    ctx.log(f"  ⚠ Personne détectée à {checkpoint.label}")
                    badge = self._request_badge(ctx, checkpoint)

                    if self._check_badge(badge):
                        role = KNOWN_BADGES[badge]
                        ctx.log(f"  ✓ Badge valide ({badge} — {role}) — accès autorisé")
                    else:
                        alert = SecurityAlert(
                            alert_id=str(uuid.uuid4())[:8].upper(),
                            severity=(AlertSeverity.WARNING if badge is None
                                     else AlertSeverity.CRITICAL),
                            checkpoint=checkpoint.checkpoint_id,
                            reason=("Aucune réponse au badge" if badge is None
                                   else f"Badge invalide présenté : {badge}"),
                            timestamp=time.time(),
                            badge_seen=badge,
                        )
                        self._alerts.append(alert)
                        alerts_triggered += 1
                        ctx.log(f"  {alert}")
                        self._send_alert(ctx, alert)

            duration = time.monotonic() - t_start
            return MissionResult(
                success=True, mission_id=mission_id, app_id="com.axisos.security_patrol",
                duration_s=duration, items_processed=checkpoints_visited,
                metrics={
                    "checkpoints_visited": checkpoints_visited,
                    "persons_encountered": persons_encountered,
                    "alerts_triggered": alerts_triggered,
                    "path": [c.checkpoint_id for c in path],
                },
                log=ctx.get_mission_log(),
            )

        except Exception as exc:
            duration = time.monotonic() - t_start
            return MissionResult(
                success=False, mission_id=mission_id, app_id="com.axisos.security_patrol",
                duration_s=duration, items_processed=checkpoints_visited,
                error_message=f"{type(exc).__name__}: {exc}",
                log=ctx.get_mission_log(),
            )

    def _send_alert(self, ctx: RobotContext, alert: SecurityAlert) -> None:
        """
        Envoie l'alerte vers le serveur de sécurité (NETWORK_CLOUD).

        Décision produit volontaire : cette app N'APPELLE JAMAIS
        automatiquement les secours ou la police elle-même. Elle
        transmet l'alerte à un serveur de sécurité qui, lui, décide
        de l'escalade (notification humaine, appel autorités, etc.).
        Un robot autonome qui déclenche directement une intervention
        d'urgence sans validation humaine est un risque de sécurité
        et de responsabilité légale trop grand pour cette version.
        """
        ctx.log(f"  → Alerte transmise au serveur de sécurité : {alert.alert_id}")
        # En production : ctx.send_cloud_event(alert) — méthode qui
        # n'existe pas encore dans RobotContext, voir notes de fin de session.

    @property
    def alerts(self) -> List[SecurityAlert]:
        return list(self._alerts)


# ── Manifest ─────────────────────────────────────────────────────────

SECURITY_PATROL_MANIFEST = AppManifest(
    app_id="com.axisos.security_patrol",
    name="Security Patrol",
    version="1.0.0",
    author="AXIS-OS Team",
    author_email="dev@axis-os.example",
    description=(
        "Ronde de sécurité nocturne autonome. Génère un chemin aléatoire "
        "à travers des checkpoints, observe, vérifie les badges des "
        "personnes rencontrées et transmet une alerte au serveur de "
        "sécurité en cas d'accès non autorisé."
    ),
    category=AppCategory.SECURITY,

    permissions=[
        Permission.ACTUATORS_READ,
        Permission.ACTUATORS_WRITE_LEGS,   # se déplacer entre checkpoints
        Permission.SENSORS_IMU,
        Permission.SENSORS_CAMERA,          # détection de personne
        Permission.SENSORS_MICROPHONE,       # interaction vocale (badge)
        Permission.WATCHDOG_READ,
        Permission.NETWORK_CLOUD,             # transmission d'alerte
    ],

    hardware=HardwareRequirements(
        required=[
            RobotCapability.LEGS,
            RobotCapability.IMU,
            RobotCapability.CAMERA,
        ],
        min_dof=12,
        compatible_models=["G1"],
    ),

    # Une ronde peut durer plusieurs dizaines de minutes — quota par
    # défaut (30s) totalement inadapté, contrairement à PackageSorting
    # dont chaque mission est courte. Premier vrai cas où ResourceQuota
    # doit être personnalisé plutôt que laissé par défaut.
    quota=ResourceQuota(
        max_cpu_percent=40.0,
        max_memory_mb=256,
        max_runtime_s=3600.0,   # jusqu'à 1h pour une ronde complète
    ),

    pricing_model=PricingModel.SUBSCRIPTION,
    base_price_eur=49.0,   # abonnement mensuel

    max_velocity_ratio=0.5,          # vitesse réduite en ronde (prudence)
    requires_human_supervision=False, # explicite : conçue pour tourner
                                        # seule la nuit, sans supervision —
                                        # contrairement au défaut du manifest
)
