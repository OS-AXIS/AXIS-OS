"""
AXIS-OS — Services système, contrat de base
================================================
Nouvelle couche entre le HAL (axis_os/hal/) et le Play Store
(axis_os/store/) — voir docs/architecture_services*.md pour la
conception complète. Un service système est un singleton démarré au
boot, partagé par toutes les apps et par un éventuel futur
orchestrateur, sans manifest, sans facturation, sans sandbox
(latence, durée de vie et concurrence incompatibles avec le modèle
sandbox — voir la conception complète pour le détail des quatre
raisons).

Ce fichier écrit le contrat AVANT toute implémentation autre que
MemoryService (axis_os/services/memory/service.py) — délibérément :
un contrat abstrait sans une seule implémentation réelle est le
meilleur moyen d'écrire le mauvais contrat. MemoryService valide ce
contrat par l'usage, pas par supposition.

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
    Type de retour de toute méthode de service qui agit sur le monde
    — jamais une exception nue, jamais un simple booléen. Une
    exception nue mélange les échecs ATTENDUS (objet non trouvé,
    chemin bloqué) avec les vraies erreurs de programmation ; un
    booléen ne peut pas porter la RAISON de l'échec, ni distinguer un
    échec net d'un état intermédiaire.

    `partial=True` est essentiel : « j'ai marché 3 m sur 8 puis j'ai
    été bloqué » n'est pas la même chose qu'un échec net — un
    éventuel futur orchestrateur doit savoir dans quel état physique
    le robot se retrouve pour replanifier, pas seulement si l'appel a
    « réussi » au sens booléen.
    """
    ok:      bool
    value:   Any = None
    reason:  Optional[str] = None   # "objet non trouvé", "chemin bloqué"
    partial: bool = False           # action entamée puis interrompue


@dataclass(frozen=True)
class ServiceHealth:
    """
    État de santé d'un service — distinct de HALStatus (qui décrit
    l'état d'UN actionneur/capteur précis), et distinct de
    RobotHealthState (qui décrit l'état global du ROBOT, voir
    axis_os/hal/watchdog/events.py). Un service peut être dégradé
    (fonctionne mais avec des limitations) sans que le robot
    lui-même soit en mauvaise santé, et inversement.
    """
    running:  bool
    healthy:  bool
    detail:   Optional[str] = None


class IService(ABC):
    """
    Service système. Singleton, démarré par le noyau au boot, partagé
    par toutes les apps.

    Invariant de sécurité non négociable (voir docs/hardware_estop_notes.md
    et docs/architecture_services*.md §8) : un service ne commande
    JAMAIS un actionneur directement. Toute action physique passe par
    NormalizedRobotHAL, donc par JointSafetyGuard — cette règle n'a
    aucune exception, un service n'est pas plus privilégié qu'une
    app. Un service qui a besoin de bouger le robot (ex: un futur
    NavigationService) reçoit une référence vers NormalizedRobotHAL
    ou un RobotContext à sa construction, exactement comme une app.
    """

    @property
    @abstractmethod
    def service_id(self) -> str:
        """Identifiant stable, ex: 'memory', 'navigation' — utilisé
        comme clé par tout registre de services (non implémenté ce
        soir) et comme `provider` dans un futur PlanStep."""
        ...

    @property
    @abstractmethod
    def capability(self) -> Capability:
        """
        Capacité fonctionnelle déclarée. Si le matériel ne la
        supporte pas (voir PlayStoreRuntime._available_capabilities()
        pour la dérivation depuis RobotCapability), le service n'est
        pas démarré et les apps qui l'exigent via
        AppManifest.capabilities_required ne s'installent pas.
        """
        ...

    @abstractmethod
    def start(self) -> HALStatus:
        """Démarre le service. Idempotent : appeler start() sur un
        service déjà démarré ne doit ni le redémarrer ni lever."""
        ...

    @abstractmethod
    def stop(self) -> None:
        """Arrête le service proprement. Idempotent, même remarque
        que start()."""
        ...

    @abstractmethod
    def health(self) -> ServiceHealth:
        ...
