"""
AXIS-OS — SecurityPatrolApp
==============================
Ronde de sécurité nocturne autonome. C'est la deuxième app de référence
du Play Store (la boutique d'apps d'AXIS-OS), écrite pour éprouver le SDK
(le kit de développement des apps) sur un cas d'usage très différent de
PackageSortingApp, l'app de tri de colis. Au lieu d'un tri d'objets
déterministe, elle enchaîne un parcours autonome, des pauses
d'observation, une interaction avec une personne détectée et, au besoin,
l'escalade d'une alerte.

Scénario demandé pour cette app :
  - La ronde se déclenche à une heure planifiée (23 h par défaut,
    configurable).
  - Elle suit un chemin ALÉATOIRE à travers des checkpoints (points de
    passage) connus.
  - Elle s'arrête à intervalles pour « observer » (pause puis scan).
  - Si une personne est détectée, le robot lui demande son badge :
      → badge valide (présent dans la base connue) : il la laisse
        passer et continue la ronde ;
      → pas de badge, ou badge invalide : il déclenche une ALERTE.
  - L'alerte part vers un serveur de sécurité, grâce à la permission
    NETWORK_CLOUD (l'accès réseau vers des services en ligne). Le robot
    n'appelle jamais la police lui-même : c'est une décision produit
    volontaire, expliquée dans la docstring de _send_alert().

Portée volontairement simulée à ce stade, et dite comme telle :
  - La détection de personne et la lecture de badge sont des stubs
    (des bouchons : simulations simples qui remplacent le vrai
    composant), comme OpenVLAConnector, le connecteur vers un modèle
    d'IA VLA (vision-langage-action), lui aussi simulé par défaut. Quand
    cette app a été écrite, le HAL (la couche d'abstraction matérielle,
    seul chemin vers les moteurs et les capteurs) n'offrait ni caméra
    réelle ni capteur biométrique. Une caméra réelle existe désormais
    (pilote RealSense du G1, lu par look_for() et describe_scene()), mais
    cette app ne s'en sert pas encore ; aucun capteur biométrique n'existe.
  - Le « chemin aléatoire » n'est qu'un ordre tiré au hasard parmi des
    checkpoints prédéfinis, pas une vraie planification de trajectoire :
    ni Nav2 (la pile de navigation de ROS 2, le cadre logiciel standard
    de la robotique) ni un SLAM (cartographie et localisation
    simultanées) ne sont branchés à ce stade.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE). Proposée en
exemple public, comme PackageSortingApp.
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
    ResourceQuota, HardwareRequirements, Capability,
)
from axis_os.hal.core.types import ActuatorCommand, ControlMode
from axis_os.hal.core.capabilities import RobotCapability


# ── Base des badges connus ─────────────────────────────────────────────
# Stub : en production, une vraie base de badges la remplacera.

KNOWN_BADGES = {
    "BADGE-4471": "Agent d'entretien de nuit",
    "BADGE-1092": "Responsable sécurité",
    "BADGE-7734": "Technicien maintenance",
}


class AlertSeverity:
    """Gravité d'une alerte de sécurité du site.

    Volontairement pas un Enum partagé : cette gravité est propre à CETTE
    app. Elle est distincte de AlertSeverity du Watchdog (le superviseur
    qui surveille l'état des moteurs, axis_os/hal/watchdog/events.py),
    qui concerne le robot lui-même et non la sécurité du site."""
    INFO    = "INFO"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"


@dataclass
class PatrolCheckpoint:
    """Point de passage de la ronde, avec ses coordonnées x, y sur le
    plan du site."""
    checkpoint_id: str
    label:         str
    x: float
    y: float


@dataclass
class SecurityAlert:
    """Alerte émise quand une personne détectée ne présente pas de badge
    valide. badge_seen vaut None si elle n'a présenté aucun badge."""
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

    Son manifeste (la fiche qui déclare les permissions, les capacités,
    les quotas et le prix de l'app) demande plus de permissions que
    PackageSortingApp : caméra (SENSORS_CAMERA), micro
    (SENSORS_MICROPHONE) et jambes (ACTUATORS_WRITE_LEGS). C'est le
    premier vrai test du système de permissions fines sur un cas d'usage
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
        self._rng = random.Random()  # générateur propre, jamais random global

    def on_start(self, ctx: RobotContext) -> bool:
        ctx.log("SecurityPatrolApp démarrée — vérification hardware")
        # Les permissions ont déjà été vérifiées par le Play Store à
        # l'installation : on_start() se contente de confirmer que le
        # contexte est opérationnel.
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
        Tire un ordre de passage aléatoire par les checkpoints.

        Pourquoi l'aléatoire : avec un parcours fixe, un rôdeur pourrait
        apprendre l'horaire de passage du robot à chaque endroit. Seul le
        départ ne change pas : la ronde commence toujours par 'entrance',
        point de départ logique du robot, et le reste est mélangé. Une
        graine (seed) rend l'ordre reproductible, par exemple pour
        rejouer un test.
        """
        rng = random.Random(seed) if seed is not None else self._rng
        start = [c for c in checkpoints if c.checkpoint_id == "entrance"]
        rest  = [c for c in checkpoints if c.checkpoint_id != "entrance"]
        rng.shuffle(rest)
        return start + rest

    # ── Détection et interaction (stubs simulés) ────────────────────

    def _scan_for_person(self, ctx: RobotContext, checkpoint: PatrolCheckpoint) -> bool:
        """
        Stub : simule la détection d'une présence humaine.

        En production, cette méthode utiliserait la caméra
        (SENSORS_CAMERA, vision) et/ou le micro (SENSORS_MICROPHONE).
        Ici, une règle simple, sans dépendance externe, pour rester
        testable : un tirage pseudo-aléatoire sur self._rng. Attention :
        la graine (seed) de la mission ne fixe que l'ordre du chemin, pas
        ce tirage.
        """
        # Détection simulée : en moyenne, 1 checkpoint sur 4 a « quelqu'un ».
        return self._rng.random() < 0.25

    def _request_badge(self, ctx: RobotContext, checkpoint: PatrolCheckpoint) -> Optional[str]:
        """
        Stub : simule la demande de badge à la personne détectée.
        Rend le badge présenté, ou None si la personne ne répond pas.
        """
        # Trois scénarios simulés : badge valide (60 %), pas de réponse
        # (20 %), badge invalide (20 %).
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
        seed        = params.get("seed", None)  # rejouer le même chemin

        path = self._generate_random_path(checkpoints, seed=seed)
        ctx.log(f"Ronde {mission_id} — chemin généré : "
                f"{' → '.join(c.checkpoint_id for c in path)}")

        checkpoints_visited = 0
        persons_encountered  = 0
        alerts_triggered      = 0

        try:
            for checkpoint in path:
                ctx.log(f"→ Arrivée à {checkpoint.label} ({checkpoint.checkpoint_id})")
                # Déplacement réel vers le checkpoint. Il exerce vraiment la
                # permission ACTUATORS_WRITE_LEGS, au lieu de la déclarer
                # dans le manifeste sans jamais s'en servir. La navigation
                # complète (calcul du chemin, évitement d'obstacles) reste
                # hors de portée tant que Nav2 n'est pas branché : ce n'est
                # qu'une commande de posture symbolique à chaque
                # checkpoint, pour passer par le vrai chemin de commande du
                # HAL.
                ctx.move_legs({
                    "left_hip_pitch_joint": ActuatorCommand(ControlMode.POSITION, position=0.05),
                    "right_hip_pitch_joint": ActuatorCommand(ControlMode.POSITION, position=0.05),
                })

                checkpoints_visited += 1

                # Pause d'observation, demandée explicitement par le
                # scénario. Pour l'instant, elle n'est que journalisée.
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
        Transmet l'alerte au serveur de sécurité (permission NETWORK_CLOUD).

        Décision produit volontaire : cette app n'appelle JAMAIS
        d'elle-même les secours ni la police. Elle transmet l'alerte à un
        serveur de sécurité, et c'est lui qui décide de l'escalade
        (prévenir une personne, appeler les autorités, etc.). Un robot
        autonome qui déclenche directement une intervention d'urgence,
        sans validation humaine, représente un risque de sécurité et de
        responsabilité juridique trop grand pour cette version.

        Limite actuelle : l'envoi n'est que journalisé (voir le
        commentaire ci-dessous).
        """
        ctx.log(f"  → Alerte transmise au serveur de sécurité : {alert.alert_id}")
        # En production : ctx.send_cloud_event(alert). Cette méthode
        # n'existe pas encore dans RobotContext ; d'ici là, l'alerte est
        # seulement inscrite dans le journal de mission et dans
        # self._alerts.

    @property
    def alerts(self) -> List[SecurityAlert]:
        return list(self._alerts)


# ── Manifeste ────────────────────────────────────────────────────────

SECURITY_PATROL_MANIFEST = AppManifest(
    app_id="com.axisos.security_patrol",
    name="Security Patrol",
    version="1.0.0",
    sdk_version_min="0.2.0",
    author="AXIS-OS Team",
    author_email="dev@axis-os.example",
    description=(
        "Ronde de sécurité nocturne autonome. Génère un chemin aléatoire "
        "à travers des checkpoints, observe, vérifie les badges des "
        "personnes rencontrées et transmet une alerte au serveur de "
        "sécurité en cas d'accès non autorisé."
    ),
    category=AppCategory.SECURITY,

    # NETWORK_CLOUD n'est pas encore vérifiée à l'exécution : la sandbox
    # (le processus isolé où tourne l'app) ne filtre pas le réseau. C'est
    # une faiblesse connue de l'isolation des apps, décrite avec le plan
    # prévu pour la fermer dans docs/isolation_apps.md.
    permissions=[
        Permission.ACTUATORS_READ,
        Permission.ACTUATORS_WRITE_LEGS,   # se déplacer entre checkpoints
        Permission.SENSORS_IMU,
        Permission.SENSORS_CAMERA,          # détection de personne
        Permission.SENSORS_MICROPHONE,       # interaction vocale (badge)
        Permission.WATCHDOG_READ,
        Permission.NETWORK_CLOUD,             # transmission d'alerte
    ],

    # Capacités fonctionnelles réellement exercées (ce que l'app doit
    # savoir FAIRE, à distinguer du matériel exigé dans hardware
    # ci-dessous) : WALK, pour se déplacer entre les checkpoints
    # (ACTUATORS_WRITE_LEGS ci-dessus), et CAMERA_RGB (l'image couleur),
    # pour la détection de personne. Aucune image de profondeur n'est
    # utilisée dans ce fichier (voir _scan_for_person() plus haut).
    #
    # SPEECH_IN (l'écoute de la parole) n'est volontairement PAS
    # déclarée, malgré SENSORS_MICROPHONE ci-dessus : aucune ligne de ce
    # fichier n'utilise réellement le micro, la demande de badge étant
    # simulée. Quand ce choix a été fait, le G1 (le robot humanoïde
    # Unitree G1) ne déclarait pas non plus RobotCapability.MICROPHONE,
    # faute de micro modélisé côté HAL, et manifest.py marquait
    # SENSORS_MICROPHONE « NON APPLIQUÉE ». Depuis, le G1 déclare son
    # micro et la permission est vérifiée par listen() et
    # declare_intent(), mais la raison principale demeure : déclarer
    # SPEECH_IN ferait refuser l'installation sur tout robot sans micro,
    # pour une capacité que l'app n'utilise jamais.
    capabilities_required=[
        Capability.WALK, Capability.CAMERA_RGB,
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

    # Quota de ressources (processeur, mémoire, durée maximale) imposé à
    # l'app par la sandbox. Une ronde peut durer plusieurs dizaines de
    # minutes : la durée maximale par défaut (30 s) serait totalement
    # inadaptée, alors qu'elle convient à PackageSortingApp, dont chaque
    # mission est courte. C'est le premier vrai cas où ResourceQuota doit
    # être personnalisé plutôt que laissé par défaut.
    quota=ResourceQuota(
        max_cpu_percent=40.0,
        max_memory_mb=256,
        max_runtime_s=3600.0,   # jusqu'à 1h pour une ronde complète
    ),

    pricing_model=PricingModel.SUBSCRIPTION,
    base_price_eur=49.0,   # abonnement mensuel (facturé une fois par 30 jours)

    max_velocity_ratio=0.5,          # vitesse plafonnée à 50 % en ronde (prudence)
    # Explicitement False, contrairement à la valeur par défaut du
    # manifeste : l'app est conçue pour tourner seule la nuit, sans
    # surveillance humaine. Pour l'instant, ce champ n'est qu'une
    # déclaration : le Play Store ne s'en sert pour rien bloquer (voir
    # manifest.py).
    requires_human_supervision=False,
)
