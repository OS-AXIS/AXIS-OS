"""
AXIS-OS — Resource Manager (gestionnaire de ressources)
=======================================================
Pourquoi ce module : rien n'empêchait deux apps différentes de commander
le même bras en même temps, avec des ordres contradictoires. AXIS-OS
n'avait aucune notion de « qui a le droit de commander quoi ». Une revue
extérieure du code (6 septembre 2026) a classé ce manque en deuxième
priorité.

Ce que fait le module : il réserve les actionneurs (les moteurs qui font
bouger le robot : bras, jambes) à une seule app à la fois. Les capteurs
restent librement partagés entre toutes les apps : caméra, microphone,
IMU (centrale inertielle, qui mesure orientation et accélérations),
capteurs de force et de couple.

Principe de partage retenu (proposé puis validé avec le responsable du
projet) :

  PARTAGEABLE (plusieurs apps en même temps) :
    caméra, microphone, IMU, force/couple, et plus généralement tout
    capteur en lecture. Plusieurs apps peuvent « regarder » ou
    « écouter » en même temps sans aucun conflit physique possible,
    comme plusieurs personnes devant le même écran.

  EXCLUSIF (une seule app à la fois) :
    bras, jambes, et tout actionneur en écriture. Un bras ne peut pas
    obéir à deux ordres contradictoires en même temps : c'est là que des
    commandes concurrentes peuvent endommager physiquement le robot ou
    produire un mouvement erratique et dangereux.

Ce composant ne remplace rien de l'existant :
  - JointSafetyGuard continue de valider chaque commande individuelle
    (limites physiques des articulations) ;
  - WatchdogSystem (le « chien de garde ») continue de surveiller l'état
    global du robot ;
  - PlayStoreRuntime (le moteur qui installe et exécute les apps)
    continue de vérifier les permissions déclarées.
Le ResourceManager ajoute une couche supplémentaire, indépendante des
trois autres : même une app qui a la permission ACTUATORS_WRITE_ARMS
(droit de commander les bras) et dont la commande respecte les limites
de sécurité doit d'abord réserver la ressource « bras » avant de
l'utiliser. Cette réservation lui est refusée si une autre app la
détient déjà.

Politique retenue pour cette version : premier arrivé, premier servi,
avec libération explicite obligatoire. Pas de vol de ressource, pas de
priorité selon la catégorie d'app. C'est volontairement simple : une
politique de priorité ou de préemption (retirer la ressource à une app
au profit d'une autre) plus élaborée est un travail futur distinct, non
fait ici pour ne pas ajouter une complexité que l'usage réel n'a pas
encore validée.

Limite actuelle : le module n'est pas encore branché sur RobotContext
(l'interface par laquelle une app commande le robot, dans app_base.py)
ni sur PlayStoreRuntime ; voir la note d'intégration en fin de fichier.

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
    Ressources réservables, c'est-à-dire les groupes d'actionneurs.

    La granularité est volontairement large : on réserve un membre
    entier, pas une articulation (« joint ») isolée. C'est cohérent avec
    RobotContext.move_arms() / move_legs(), qui commandent déjà un groupe
    d'articulations ensemble et non une à la fois. Une app qui veut
    bouger une seule articulation du bras réserve donc quand même ARMS en
    entier. Descendre à l'articulation individuelle reste un raffinement
    futur possible ; il n'est pas nécessaire pour fermer le trou de
    sécurité identifié (deux apps qui commandent le même membre).
    """
    ARMS = auto()
    LEGS = auto()
    ALL_ACTUATORS = auto()  # pour move_all() : couvre aussi ARMS et LEGS


# Les capteurs sont volontairement absents de ResourceKind, conformément
# au principe exposé en tête de fichier : ils ne sont jamais réservés ni
# exclusifs, et n'ont donc besoin d'aucune entrée ici. Une app qui lit
# ctx.get_imu() (ou, demain, une caméra) continue de le faire sans jamais
# passer par le ResourceManager.


class ResourceConflictError(Exception):
    """
    Levée quand une app tente d'utiliser un actionneur déjà réservé par
    une autre app.

    Pourquoi une exception à part : PlayStoreRuntime doit pouvoir
    distinguer trois motifs de refus pour un diagnostic clair. Ce n'est
    ni une PermissionError (l'app avait bien le droit déclaré d'utiliser
    cet actionneur), ni une EstopActiveError (refus dû à un arrêt
    d'urgence, ou « e-stop », actif ; ici, aucune condition de sécurité
    n'est en cause). C'est un conflit de ressource entre deux apps
    légitimes, une troisième catégorie de refus.
    """
    pass


@dataclass
class ResourceReservation:
    """
    Une réservation en cours : quelle ressource, quelle app la détient,
    et depuis quand (time.monotonic(), une horloge qui ne recule jamais).
    """
    resource:      ResourceKind
    held_by_app_id: str
    acquired_at_monotonic: float


class ResourceManager:
    """
    Registre des réservations d'actionneurs : une seule app par ressource.

    Usage typique, vu depuis PlayStoreRuntime (intégration pas encore
    faite dans cette version, voir la note en fin de fichier) :

        rm = ResourceManager()
        rm.acquire(ResourceKind.ARMS, app_id="com.acme.app_a")
        try:
            ... exécuter la mission, qui appelle ctx.move_arms() ...
        finally:
            rm.release(ResourceKind.ARMS, app_id="com.acme.app_a")

    Sûr en accès concurrent (« thread-safe », grâce à un verrou) :
    plusieurs missions peuvent tourner en parallèle, chacune dans son
    propre processus isolé (sandbox, ou « bac à sable » : voir
    SandboxRunner dans sandbox.py), et tenter d'acquérir des ressources
    au même instant.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._reservations: Dict[ResourceKind, ResourceReservation] = {}

    def acquire(self, resource: ResourceKind, app_id: str) -> None:
        """
        Réserve la ressource pour app_id.

        Lève ResourceConflictError si elle est déjà détenue par une autre
        app. Jamais d'attente bloquante ici, comme partout dans AXIS-OS :
        une commande est acceptée ou refusée immédiatement, jamais mise
        en file d'attente en silence. Une app qui attendrait indéfiniment
        une ressource sans le savoir serait un risque en soi, pas une
        amélioration.

        ALL_ACTUATORS réserve implicitement ARMS et LEGS à la fois : une
        app qui détient ALL_ACTUATORS bloque toute acquisition de ARMS ou
        de LEGS par une autre app et, réciproquement, une app qui détient
        ARMS ou LEGS bloque l'acquisition d'ALL_ACTUATORS par une autre.
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

            # Idempotent (appeler deux fois revient à appeler une fois) :
            # une app qui ré-acquiert une ressource qu'elle détient déjà
            # ne lève pas d'erreur et garde sa réservation (seul
            # l'horodatage est renouvelé). Sans cela, un usage courant
            # casserait : une mission qui appelle acquire() à chaque tour
            # de sa boucle buterait sur son propre appel précédent.
            self._reservations[resource] = ResourceReservation(
                resource=resource, held_by_app_id=app_id,
                acquired_at_monotonic=time.monotonic(),
            )

    def release(self, resource: ResourceKind, app_id: str) -> None:
        """
        Libère la ressource.

        Ne fait rien, sans signaler d'erreur, si la ressource n'est pas
        détenue ou si elle est détenue par une autre app : une app ne
        peut jamais libérer, par erreur ou par malveillance, une
        ressource qui ne lui appartient pas.
        """
        with self._lock:
            existing = self._reservations.get(resource)
            if existing is not None and existing.held_by_app_id == app_id:
                del self._reservations[resource]

    def release_all_for_app(self, app_id: str) -> None:
        """
        Libère toutes les ressources détenues par une app.

        Pourquoi : une app qui plante sans passer par son propre release()
        laisserait sinon une ressource bloquée indéfiniment, et une app
        boguée qui oublie de libérer bloquerait ARMS ou LEGS pour
        toujours. C'est le filet de sécurité qui rend ce système
        utilisable en pratique. À appeler impérativement à la fin de
        chaque mission, quelle qu'en soit l'issue : succès, échec, crash
        ou dépassement du temps alloué par la sandbox (timeout).
        """
        with self._lock:
            held = [r for r, res in self._reservations.items()
                   if res.held_by_app_id == app_id]
            for r in held:
                del self._reservations[r]

    def is_held(self, resource: ResourceKind) -> bool:
        """
        Indique si la ressource est indisponible : réservée elle-même, ou
        couverte par une réservation qui la chevauche (ARMS ou LEGS quand
        ALL_ACTUATORS est pris, et inversement).
        """
        with self._lock:
            return resource in self._reservations or \
                   any(o in self._reservations for o in self._find_conflicts(resource))

    def get_holder(self, resource: ResourceKind) -> Optional[str]:
        """
        Retourne l'app_id de l'app qui bloque cette ressource, ou None si
        elle est libre. La réservation directe est regardée d'abord, puis
        les réservations qui la chevauchent ; si plusieurs apps la
        bloquent (ARMS et LEGS détenus par deux apps différentes, pour
        ALL_ACTUATORS), une seule d'entre elles est renvoyée.
        """
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
        """Vue de diagnostic : {nom_ressource: app_id détentrice}."""
        with self._lock:
            return {r.name: res.held_by_app_id for r, res in self._reservations.items()}

    def _find_conflicts(self, resource: ResourceKind) -> Set[ResourceKind]:
        """
        Retourne l'ensemble des ResourceKind en conflit avec la ressource
        demandée, elle comprise. La relation n'est pas une simple égalité
        de clé, parce qu'ALL_ACTUATORS chevauche ARMS et LEGS.
        """
        if resource == ResourceKind.ALL_ACTUATORS:
            return {ResourceKind.ARMS, ResourceKind.LEGS, ResourceKind.ALL_ACTUATORS}
        if resource in (ResourceKind.ARMS, ResourceKind.LEGS):
            return {resource, ResourceKind.ALL_ACTUATORS}
        return {resource}


# NOTE D'INTÉGRATION. Ce fichier définit le ResourceManager de façon
# autonome, et il est testé (test/test_resource_manager.py). En revanche,
# il ne modifie ni RobotContext ni PlayStoreRuntime pour qu'ils
# l'appellent automatiquement à chaque move_arms() / move_legs() /
# move_all() : tant que ce branchement n'est pas fait, la réservation
# n'est pas imposée aux apps. C'est le prochain travail à faire,
# volontairement séparé de la création du composant lui-même.
# Pourquoi : le brancher touche app_base.py, un fichier central déjà
# modifié plusieurs fois pendant la même séance de travail (ajout
# d'EstopActiveError et de _require_no_estop, la vérification d'arrêt
# d'urgence faite avant chaque mouvement). Une intégration non testée
# sur le vrai G1 (le robot humanoïde Unitree G1) avant la fin de cette
# séance ferait courir le même risque que celui déjà rencontré avec
# conventions.py, où une modification a dû être annulée le soir même.
