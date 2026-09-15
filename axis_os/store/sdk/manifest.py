"""
AXIS-OS Robotics Play Store — App Manifest
============================================
Contrat déclaratif que toute app doit fournir avant installation.

Licence : Apache 2.0 (open-source — SDK public)
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

from axis_os.hal.core.capabilities import RobotCapability


class Permission(Enum):
    """
    ⚠️ Trouvé par un audit de sécurité (M1) : 7 des permissions
    ci-dessous sont déclarées mais AUCUNE méthode de RobotContext ne
    les vérifie via _require() — les marquer explicitement plutôt que
    de laisser croire qu'elles protègent quelque chose aujourd'hui.
    Retirer ces entrées casserait la rétrocompatibilité de tout
    manifest existant qui les déclare déjà ; les laisser sans
    documentation laisserait croire à une protection qui n'existe
    pas. Choix retenu : documenter honnêtement, implémenter au fil
    de l'eau quand la fonctionnalité correspondante existera vraiment
    côté RobotContext (ex: SENSORS_FORCE_TORQUE dès qu'une méthode
    get_force_torque() y sera exposée — le HAL l'a déjà, RobotContext
    ne l'expose pas encore).

    Les deux permissions NETWORK_* sont les plus significatives à ce
    titre : rien n'empêche aujourd'hui une app sandboxée d'ouvrir une
    socket réseau elle-même — le sandbox multiprocessing ne filtre
    pas le réseau (voir docs/sandboxing_hardening_notes.md, qui
    documente déjà ce trou plus large comme piste future).
    """
    ACTUATORS_READ       = auto()
    ACTUATORS_WRITE_ARMS = auto()
    ACTUATORS_WRITE_LEGS = auto()
    ACTUATORS_WRITE_ALL  = auto()
    SENSORS_IMU          = auto()
    SENSORS_FORCE_TORQUE = auto()  # NON APPLIQUÉE — voir docstring de classe
    SENSORS_CAMERA       = auto()  # NON APPLIQUÉE — aucune API caméra dans RobotContext
    SENSORS_MICROPHONE   = auto()  # NON APPLIQUÉE — aucune API micro dans RobotContext
    WATCHDOG_READ        = auto()
    EMERGENCY_STOP       = auto()
    FLEET_BROADCAST      = auto()  # NON APPLIQUÉE — Fleet Manager non implémenté (voir docs/)
    NETWORK_CLOUD        = auto()  # NON APPLIQUÉE — sandbox ne filtre pas le réseau
    NETWORK_LOCAL        = auto()  # NON APPLIQUÉE — sandbox ne filtre pas le réseau


class AppCategory(Enum):
    LOGISTICS     = "Logistique"
    SECURITY      = "Sécurité"
    MAINTENANCE   = "Maintenance"
    HEALTHCARE    = "Santé"
    MANUFACTURING = "Fabrication"
    RESEARCH      = "Recherche"
    UTILITIES     = "Utilitaires"


class PricingModel(Enum):
    FREE           = "free"
    ONE_TIME       = "one_time"
    SUBSCRIPTION   = "sub_month"
    PAY_PER_MISSION = "per_mission"


@dataclass
class ResourceQuota:
    """
    Quotas d'isolation pour le sandbox multiprocessing.
    Une app qui dépasse ces limites est tuée par le SandboxRunner.
    """
    max_cpu_percent: float = 50.0   # % d'un cœur CPU
    max_memory_mb:   int   = 256    # Mo de RAM
    max_runtime_s:   float = 30.0   # Timeout dur d'une mission


@dataclass
class HardwareRequirements:
    """
    Déclaration des capacités matérielles dont l'app a besoin.

    `required`  : indispensables — un robot qui ne les possède pas
                  (au sens CapabilitySet.has(), donc physiquement
                  présentes ET actuellement AVAILABLE) rend l'app
                  INCOMPATIBLE : installation refusée par
                  PlayStoreRuntime.install().
    `optional`  : améliorent l'app sans lui être indispensables — une
                  capacité manquante ici ne bloque pas l'installation,
                  seulement un avertissement est affiché. C'est à
                  l'app elle-même de vérifier ctx.robot.get_capabilities()
                  en cours de mission avant d'utiliser une capacité
                  optionnelle.

    Le manifest est un contrat (voir revue externe du 2026-09-06) :
    une capacité dont l'app ne peut réellement pas se passer doit être
    déclarée dans `required`, jamais dans `optional` par prudence mal
    placée — sinon l'app plantera en mission au lieu d'être refusée
    proprement à l'installation.

    min_dof et compatible_models restent des vérifications distinctes
    (degré de liberté total, modèle exact) — indépendantes du système
    de capacités, qui raisonne en catégories fonctionnelles, pas en
    nombre de joints ou en identité de fabricant.
    """
    required:             list[RobotCapability] = field(default_factory=list)
    optional:             list[RobotCapability] = field(default_factory=list)
    min_dof:              int        = 0
    compatible_models:    list[str]  = field(default_factory=list)


@dataclass
class AppManifest:
    app_id:        str
    name:          str
    version:       str
    author:        str
    author_email:  str
    description:   str
    category:      AppCategory

    permissions:   list[Permission]        = field(default_factory=list)
    hardware:      HardwareRequirements    = field(default_factory=HardwareRequirements)
    quota:         ResourceQuota           = field(default_factory=ResourceQuota)

    pricing_model:  PricingModel = PricingModel.FREE
    base_price_eur: float        = 0.0

    max_velocity_ratio:        float = 0.8
    requires_human_supervision: bool = True

    icon_url:      Optional[str] = None
    website_url:   Optional[str] = None
    support_email: Optional[str] = None

    # Version minimale d'AXIS-OS requise par cette app (SemVer, ex:
    # "0.2.0"). Optionnelle et laissée à None par défaut pour ne
    # jamais casser une app déjà écrite avant l'introduction de ce
    # champ (voir axis_os/__init__.py pour la politique de version
    # complète). Quand elle est renseignée, PlayStoreRuntime.install()
    # la vérifie contre axis_os.__version__ avant d'installer — voir
    # SemVer.is_compatible_with() pour la règle exacte appliquée
    # (stricte tant qu'on reste en version 0.x).
    sdk_version_min: Optional[str] = None

    # Signature Ed25519 du code source de l'app (voir
    # axis_os/store/sdk/signing.py) — optionnelle et laissée à None
    # par défaut, pour ne jamais casser une app déjà écrite avant
    # l'introduction de ce mécanisme. Quand code_signature ET
    # developer_public_key sont TOUS LES DEUX renseignés,
    # PlayStoreRuntime.install() vérifie que le code source fourni à
    # l'installation correspond bien à cette signature — refuse
    # sinon. Si un seul des deux est renseigné (signature sans clé,
    # ou clé sans signature), c'est traité comme une erreur de
    # manifest (voir validate()) : un développeur qui commence à
    # signer ses apps doit le faire correctement, pas à moitié.
    code_signature:        Optional[str] = None
    developer_public_key:  Optional[str] = None

    COMMISSION_RATE = 0.30

    @property
    def store_price_eur(self) -> float:
        if self.pricing_model == PricingModel.FREE: return 0.0
        return round(self.base_price_eur / (1 - self.COMMISSION_RATE), 2)

    @property
    def developer_revenue_eur(self) -> float:
        return self.base_price_eur

    @property
    def platform_commission_eur(self) -> float:
        return round(self.store_price_eur - self.base_price_eur, 2)

    def has_permission(self, perm: Permission) -> bool:
        return perm in self.permissions

    def validate(self) -> list[str]:
        errors = []
        if not self.app_id or "." not in self.app_id:
            errors.append("app_id doit être en format 'com.author.name'")
        if self.base_price_eur < 0:
            errors.append("base_price_eur ne peut pas être négatif")
        if self.pricing_model != PricingModel.FREE and self.base_price_eur == 0:
            errors.append("Un modèle payant doit avoir un base_price_eur > 0")
        # Trou identifié lors d'un audit : avec base_price_eur trop bas
        # (ex: 0.01€), l'arrondi à 2 décimales de store_price_eur peut
        # ramener platform_commission_eur à 0.00€ — la plateforme ne
        # touche alors LITTÉRALEMENT rien sur la transaction, alors que
        # le manifest prétend un modèle payant à 30% de commission. Se
        # calcule dynamiquement à partir de COMMISSION_RATE plutôt qu'un
        # seuil codé en dur, pour rester correct si le taux change un
        # jour. Ne s'applique qu'aux modèles payants (FREE a toujours
        # commission=0.0€ par conception, ce n'est pas une anomalie).
        if self.pricing_model != PricingModel.FREE and self.base_price_eur > 0:
            if self.platform_commission_eur <= 0:
                errors.append(
                    f"base_price_eur={self.base_price_eur}€ est trop bas : "
                    f"la commission de plateforme calculée "
                    f"({self.platform_commission_eur}€) est nulle après "
                    f"arrondi — la plateforme ne serait jamais rémunérée "
                    f"sur cette app malgré le modèle payant déclaré."
                )
        if not (0.0 < self.max_velocity_ratio <= 1.0):
            errors.append("max_velocity_ratio doit être dans ]0.0, 1.0]")
        if self.sdk_version_min is not None:
            from axis_os import SemVer
            try:
                SemVer.parse(self.sdk_version_min)
            except ValueError as e:
                errors.append(f"sdk_version_min invalide : {e}")
        if (self.code_signature is None) != (self.developer_public_key is None):
            errors.append(
                "code_signature et developer_public_key doivent être renseignés "
                "ENSEMBLE ou pas du tout — un seul des deux ne permet aucune "
                "vérification cohérente."
            )
        return errors
