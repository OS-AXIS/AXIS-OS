"""
AXIS-OS — Resource Manager
=============================
Contrôle d'accès exclusif aux ACTIONNEURS (bras, jambes) — les
CAPTEURS (caméra, microphone, IMU, force/torque) restent librement
partagés entre toutes les apps.

Origine : revue externe du 2026-09-06, priorité 2. Point soulevé :
rien n'empêche aujourd'hui deux apps différentes de contrôler le même
bras en même temps avec des commandes contradictoires — AXIS-OS
n'avait aucune notion de "qui a le droit de commander quoi".

Principe de partage retenu (proposé et validé avec l'utilisateur en
session) :

  PARTAGEABLE  (plusieurs apps simultanément) :
    caméra, microphone, IMU, force/torque, tout capteur en LECTURE —
    plusieurs apps peuvent "regarder" ou "écouter" en même temps sans
    conflit physique possible, comme plusieurs personnes qui
    regardent le même écran.

  EXCLUSIF  (une seule app à la fois) :
    bras, jambes, tout actionneur en ÉCRITURE — un bras ne peut pas
    obéir à deux ordres contradictoires en même temps ; c'est ici que
    des commandes concurrentes peuvent physiquement endommager le
    robot ou produire un mouvement erratique et dangereux.

Ce composant ne remplace RIEN de l'existant — JointSafetyGuard
continue de valider chaque commande individuelle (limites physiques),
WatchdogSystem continue de surveiller l'état global, PlayStoreRuntime
continue de vérifier les permissions déclarées. Le ResourceManager
ajoute une couche supplémentaire, orthogonale : même une app qui a la
permission ACTUATORS_WRITE_ARMS et une commande dans les limites de
sécurité doit d'abord RÉSERVER la ressource "bras" avant de pouvoir
l'utiliser, et voir cette réservation refusée si une autre app la
détient déjà.

Politique retenue pour cette version : premier arrivé, premier servi,
avec libération explicite obligatoire (pas de vol de ressource, pas
de priorité par catégorie d'app). Volontairement simple — une
politique de priorité ou de préemption plus sophistiquée est un
travail futur distinct, non fait ici pour éviter d'ajouter de la
complexité non validée par l'usage réel.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Optional, Set


class ResourceKind(Enum):
    """
    Granularité volontairement large (par MEMBRE, pas par joint
    individuel) — cohérent avec RobotContext.move_arms()/move_legs()
    qui contrôlent déjà un groupe de joints ensemble, pas un joint à
    la fois. Une app qui veut bouger UN SEUL joint du bras réserve
    quand même ARMS entier ; affiner à la granularité du joint
    individuel est un raffinement futur possible, non nécessaire pour
    fermer le trou de sécurité identifié.
    """
    ARMS = auto()
    LEGS = auto()
    ALL_ACTUATORS = auto()  # réservation de move_all() — bloque implicitement ARMS et LEGS


# Capteurs volontairement ABSENTS de ResourceKind — cohérent avec le
# principe ci-dessus : ils ne sont jamais réservés, jamais exclusifs,
# donc ne nécessitent aucune entrée dans ce système. Une app qui
# accède à ctx.get_imu() ou une future caméra continue de le faire
# sans jamais passer par le ResourceManager.


class ResourceConflictError(Exception):
    """
    Levée quand une app tente d'utiliser un actionneur déjà réservé
    par une autre app. Distincte de PermissionError (l'app AVAIT le
    droit déclaré d'utiliser cet actionneur) et de EstopActiveError
    (aucune condition de sécurité n'est en cause) — c'est un conflit
    de RESSOURCE entre deux apps légitimes, une troisième catégorie
    de refus que PlayStoreRuntime doit pouvoir distinguer des deux
    autres pour un diagnostic clair.
    """
    pass


@dataclass
class ResourceReservation:
    resource:      ResourceKind
    held_by_app_id: str
    acquired_at_monotonic: float


class ResourceManager:
    """
    Usage typique, du point de vue de PlayStoreRuntime (intégration
    non faite dans cette version, voir note en fin de fichier) :

        rm = ResourceManager()
        rm.acquire(ResourceKind.ARMS, app_id="com.acme.app_a")
        try:
            ... exécuter la mission, qui appelle ctx.move_arms() ...
        finally:
            rm.release(ResourceKind.ARMS, app_id="com.acme.app_a")

    Thread-safe — plusieurs missions peuvent tourner en parallèle
    (dans des process sandboxés séparés, voir SandboxRunner) et
    tenter d'acquérir des ressources au même instant.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._reservations: Dict[ResourceKind, ResourceReservation] = {}

    def acquire(self, resource: ResourceKind, app_id: str) -> None:
        """
        Réserve la ressource pour app_id. Lève ResourceConflictError
        si elle est déjà détenue par une AUTRE app — jamais d'attente
        bloquante ici, cohérent avec le reste d'AXIS-OS où une
        commande est acceptée ou refusée immédiatement, jamais mise en
        file d'attente silencieuse (une app qui attendrait
        indéfiniment une ressource sans le savoir serait un risque en
        soi, pas une amélioration).

        ALL_ACTUATORS réserve implicitement ARMS et LEGS en même
        temps — une app qui a acquis ALL_ACTUATORS bloque toute
        tentative d'acquisition de ARMS ou LEGS par une autre app, et
        réciproquement une app qui détient ARMS ou LEGS bloque une
        tentative d'ALL_ACTUATORS.
        """
        with self._lock:
            conflicting = self._find_conflicts(resource)
            for other in conflicting:
                existing = self._reservations.get(other)
                if existing is not None and existing.held_by_app_id != app_id:
                    raise ResourceConflictError(
                        f"Ressource '{resource.name}' indisponible — "
                        f"'{other.name}' déjà détenue par '{existing.held_by_app_id}'"
                    )

            # Idempotent : une app qui ré-acquiert une ressource
            # qu'elle détient déjà ne lève pas d'erreur et ne perd pas
            # sa réservation existante — évite un cas d'usage courant
            # (mission qui appelle acquire() à chaque étape de sa
            # boucle) de casser sur son propre appel précédent.
            self._reservations[resource] = ResourceReservation(
                resource=resource, held_by_app_id=app_id,
                acquired_at_monotonic=time.monotonic(),
            )

    def release(self, resource: ResourceKind, app_id: str) -> None:
        """
        Libère la ressource. Ne fait RIEN (silencieusement) si la
        ressource n'est pas détenue, ou détenue par une AUTRE app —
        une app ne peut jamais libérer par erreur ou malveillance une
        ressource qui ne lui appartient pas.
        """
        with self._lock:
            existing = self._reservations.get(resource)
            if existing is not None and existing.held_by_app_id == app_id:
                del self._reservations[resource]

    def release_all_for_app(self, app_id: str) -> None:
        """
        Libère TOUTES les ressources détenues par une app — à appeler
        impérativement quand une mission se termine (succès, échec,
        crash, timeout sandbox) pour ne jamais laisser une ressource
        bloquée indéfiniment par une app qui a planté sans passer par
        son propre release(). C'est le filet de sécurité qui rend ce
        système utilisable en pratique : sans ça, une app buguée qui
        oublie de libérer bloquerait ARMS ou LEGS pour toujours.
        """
        with self._lock:
            held = [r for r, res in self._reservations.items()
                   if res.held_by_app_id == app_id]
            for r in held:
                del self._reservations[r]

    def is_held(self, resource: ResourceKind) -> bool:
        with self._lock:
            return resource in self._reservations or \
                   any(o in self._reservations for o in self._find_conflicts(resource))

    def get_holder(self, resource: ResourceKind) -> Optional[str]:
        with self._lock:
            existing = self._reservations.get(resource)
            if existing is not None:
                return existing.held_by_app_id
            for other in self._find_conflicts(resource):
                existing_other = self._reservations.get(other)
                if existing_other is not None:
                    return existing_other.held_by_app_id
            return None

    def get_all_reservations(self) -> Dict[str, str]:
        """Vue de diagnostic — {nom_ressource: app_id détentrice}."""
        with self._lock:
            return {r.name: res.held_by_app_id for r, res in self._reservations.items()}

    def _find_conflicts(self, resource: ResourceKind) -> Set[ResourceKind]:
        """
        Retourne l'ensemble des ResourceKind qui entrent en conflit
        avec la ressource demandée — la relation n'est PAS juste
        "égalité de clé", à cause de ALL_ACTUATORS qui chevauche ARMS
        et LEGS.
        """
        if resource == ResourceKind.ALL_ACTUATORS:
            return {ResourceKind.ARMS, ResourceKind.LEGS, ResourceKind.ALL_ACTUATORS}
        if resource in (ResourceKind.ARMS, ResourceKind.LEGS):
            return {resource, ResourceKind.ALL_ACTUATORS}
        return {resource}


# NOTE D'INTÉGRATION — ce fichier définit le ResourceManager de façon
# autonome et testée, mais ne modifie PAS RobotContext ni
# PlayStoreRuntime pour l'appeler automatiquement à chaque
# move_arms()/move_legs()/move_all(). Cette intégration est le
# prochain travail à faire, volontairement séparé de la création du
# composant lui-même : brancher ceci dans app_base.py touche un
# fichier central déjà modifié plusieurs fois cette session
# (EstopActiveError, _require_no_estop) — une intégration non testée
# sur le vrai G1 avant la fin de cette session serait le même risque
# que celui déjà rencontré et annulé ce soir avec conventions.py.
