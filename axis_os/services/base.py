"""
AXIS-OS — Services système, contrat de base
================================================
Les services système forment une couche intermédiaire entre le HAL
(couche d'abstraction matérielle, axis_os/hal/ : le code qui parle aux
moteurs et aux capteurs) et le Play Store (axis_os/store/ : le magasin
d'apps d'AXIS-OS). La conception complète est décrite dans
docs/architecture_services.md.

Un service système est un singleton (une seule instance pour tout le
robot) démarré au boot et partagé par toutes les apps, ainsi que par un
éventuel futur orchestrateur (le composant qui enchaînera des appels de
services pour accomplir une mission). Contrairement à une app, il n'a
ni manifeste (la fiche où une app déclare son identité, ses permissions
et les capacités matérielles qu'elle exige), ni facturation, ni sandbox
(le bac à sable, processus isolé dans lequel tourne chaque app). Quatre
raisons à cela, détaillées dans docs/architecture_services.md §6.1 :
les besoins d'un service en latence, en durée de vie et en accès
concurrent sont incompatibles avec le bac à sable, et faire de la
perception une app créerait une dépendance circulaire avec
l'orchestrateur.

Ce contrat a été écrit en même temps que sa première implémentation
réelle, MemoryService (axis_os/services/memory/service.py), et pas
avant : un contrat abstrait écrit sans une seule implémentation réelle
est le meilleur moyen d'écrire le mauvais contrat. MemoryService l'a
validé par l'usage, pas par supposition ; les autres services système
(parole, perception, navigation…) ont été écrits ensuite sur ce même
contrat.

Licence : Apache 2.0
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Optional

from axis_os.hal.core.types import HALStatus
from axis_os.store.sdk.manifest import Capability


@dataclass(frozen=True)
class ServiceResult:
    """
    Type de retour de toute méthode de service qui agit sur le monde.

    Pourquoi ni exception nue, ni simple booléen : une exception nue
    mélange les échecs attendus (objet non trouvé, chemin bloqué) avec
    les vraies erreurs de programmation ; un booléen ne peut pas porter
    la raison de l'échec, ni distinguer un échec net d'un état
    intermédiaire.

    `partial=True` est essentiel : « j'ai marché 3 m sur 8 puis j'ai été
    bloqué » n'est pas la même chose qu'un échec net. Un éventuel futur
    orchestrateur doit savoir dans quel état physique le robot se
    retrouve pour replanifier, pas seulement si l'appel a « réussi » au
    sens booléen.
    """
    ok:      bool
    value:   Any = None
    reason:  Optional[str] = None   # "objet non trouvé", "chemin bloqué"
    partial: bool = False           # action entamée puis interrompue


@dataclass(frozen=True)
class ServiceHealth:
    """
    État de santé d'un service.

    Il est distinct de HALStatus, qui décrit l'état d'un actionneur ou
    d'un capteur précis, et distinct de RobotHealthState, qui décrit
    l'état global du robot (voir axis_os/hal/watchdog/events.py). Un
    service peut être dégradé (il fonctionne, mais avec des limitations)
    sans que le robot lui-même soit en mauvaise santé, et inversement.
    """
    running:  bool
    healthy:  bool
    detail:   Optional[str] = None


class IService(ABC):
    """
    Service système : singleton démarré par le noyau au boot, partagé
    par toutes les apps.

    Invariant de sécurité non négociable (voir
    docs/hardware_estop_notes.md et docs/architecture_services.md §8) :
    un service ne commande JAMAIS un actionneur directement. Toute
    action physique passe par NormalizedRobotHAL (l'interface commune
    du robot, indépendante du modèle) et donc par JointSafetyGuard (le
    garde qui vérifie chaque consigne articulaire contre les limites de
    sécurité). Cette règle n'a aucune exception : un service n'est pas
    plus privilégié qu'une app. Un service qui doit faire bouger le
    robot (la navigation, par exemple) reçoit pour cela, à sa
    construction, une référence vers NormalizedRobotHAL ou vers un
    RobotContext (l'interface sécurisée entre une app et le robot, voir
    axis_os/store/sdk/app_base.py), exactement comme une app.
    """

    @property
    @abstractmethod
    def service_id(self) -> str:
        """Identifiant stable du service, par exemple 'memory' ou
        'navigation'. Il servira de clé dans un registre de services
        (pas encore implémenté) et de `provider` dans une étape de plan
        de l'orchestrateur (PlanStep, prévu par
        docs/architecture_services.md §4.2, pas encore implémenté)."""
        ...

    @property
    @abstractmethod
    def capability(self) -> Capability:
        """
        Capacité fonctionnelle déclarée par le service (par exemple
        mémoire persistante ou entrée vocale).

        Si le matériel ne la fournit pas, le service n'est pas démarré,
        et les apps qui l'exigent dans AppManifest.capabilities_required
        ne s'installent pas. La correspondance entre les capacités du
        matériel (RobotCapability) et ces capacités fonctionnelles est
        calculée par PlayStoreRuntime._available_capabilities().
        """
        ...

    @abstractmethod
    def start(self) -> HALStatus:
        """Démarre le service. Doit être idempotent : appeler start()
        sur un service déjà démarré ne doit ni le redémarrer ni lever
        d'exception."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Arrête le service proprement. Doit être idempotent, comme
        start() : un second appel ne fait rien de plus."""
        ...

    @abstractmethod
    def health(self) -> ServiceHealth:
        """État de santé courant du service (voir ServiceHealth)."""
        ...
