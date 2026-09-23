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
    # Commandes articulaires brutes (ControlMode.TORQUE inclus), sans
    # passer par les routages move_arms/move_legs/move_all — réservée
    # au niveau 1 du SDK à trois niveaux (voir docs/architecture, non
    # implémenté ce soir). Volontairement ProtectionLevel.SIGNATURE
    # (voir PERMISSION_LEVELS ci-dessous) : c'est le niveau avec
    # lequel on casse du matériel, jamais accordé à une app tierce.
    ACTUATORS_RAW        = auto()
    SENSORS_IMU          = auto()
    SENSORS_FORCE_TORQUE = auto()  # NON APPLIQUÉE — voir docstring de classe
    SENSORS_CAMERA       = auto()  # appliquée : look_for / describe_scene
    SENSORS_MICROPHONE   = auto()  # appliquée : listen / declare_intent
    WATCHDOG_READ        = auto()
    EMERGENCY_STOP       = auto()
    FLEET_BROADCAST      = auto()  # NON APPLIQUÉE — Fleet Manager non implémenté (voir docs/)
    NETWORK_CLOUD        = auto()  # NON APPLIQUÉE — sandbox ne filtre pas le réseau
    NETWORK_LOCAL        = auto()  # NON APPLIQUÉE — sandbox ne filtre pas le réseau
    # Mouvements dont les consignes proviennent d'une POLITIQUE
    # APPRISE (GR00T, VLA...) plutôt que d'un script. Ne remplace pas
    # ACTUATORS_WRITE_* : une app pilotée par IA déclare les DEUX, ce
    # qui rend le pilotage IA visible à l'installation plutôt
    # qu'implicite. Le profil de risque justifie la distinction : les
    # mouvements d'une app scriptée sont auditables à l'avance, ceux
    # d'une politique apprise ne le sont pas. Tranche la question
    # explicitement reportée dans docs/ ("distinction IA/script dans
    # les permissions").
    ACTUATORS_AI_DRIVEN  = auto()
    # Faire SE DÉPLACER le robot dans l'espace (marche, roues...).
    # Distincte d'ACTUATORS_WRITE_LEGS, qui autorise à commander les
    # articulations des jambes : une app qui déplace le robot change son
    # environnement, peut heurter des objets, quitter une zone, tomber
    # dans un escalier. Une app qui marche déclare les DEUX, pour que
    # l'utilisateur voie à l'installation que le robot bougera de place.
    LOCOMOTION_WALK      = auto()
    # Mémoire persistante PROPRE à l'app : ce qu'elle écrit survit à un
    # redémarrage du robot. Le stockage est cloisonné par app_id (voir
    # MemoryService.app_store), donc une app ne peut ni lire ni écraser
    # les données d'une autre. La permission reste nécessaire : un robot
    # qui retient des lieux et des objets accumule de l'information sur
    # son environnement, et l'utilisateur doit savoir quelles apps en
    # gardent une trace.
    STORAGE_PERSISTENT   = auto()


class ProtectionLevel(Enum):
    """
    Niveau de protection d'une permission — calque du mécanisme
    Android (une permission `signature` n'est accordée qu'à une app
    signée avec la même clé que le système). S'appuie sur la
    signature Ed25519 déjà existante (axis_os/store/sdk/signing.py),
    pas un nouveau mécanisme : une app dont `code_signature` vérifie
    contre la clé plateforme est une app système, sans liste blanche
    codée en dur à maintenir séparément.
    """
    NORMAL    = auto()   # accordée automatiquement — ex. ACTUATORS_READ
    DANGEROUS = auto()   # demande confirmation humaine — ex. ACTUATORS_WRITE_ALL
    SIGNATURE = auto()   # réservée aux apps signées par la clé plateforme


# Niveau de protection par permission — permissions absentes de ce
# dict traitées comme NORMAL par défaut (voir get_protection_level()
# plus bas). Volontairement une table statique, pas une méthode sur
# Permission lui-même : garde la définition des permissions et leur
# politique de protection séparées, cohérent avec la remarque de
# conception "pas de découverte dynamique, une enum en dur suffit".
PERMISSION_LEVELS: dict[Permission, ProtectionLevel] = {
    Permission.ACTUATORS_READ:       ProtectionLevel.NORMAL,
    Permission.ACTUATORS_WRITE_ARMS: ProtectionLevel.DANGEROUS,
    Permission.ACTUATORS_WRITE_LEGS: ProtectionLevel.DANGEROUS,
    Permission.ACTUATORS_WRITE_ALL:  ProtectionLevel.DANGEROUS,
    Permission.ACTUATORS_RAW:        ProtectionLevel.SIGNATURE,
    # EMERGENCY_STOP délibérément DANGEROUS, pas SIGNATURE — décision
    # prise en corrigeant une régression réelle : security_patrol et
    # warehouse_ops (apps de référence déjà existantes, voir M11 dans
    # CHANGELOG.md) déclarent légitimement cette permission pour
    # pouvoir déclencher un arrêt d'urgence si elles détectent un
    # danger (personne dans la trajectoire, anomalie de sécurité).
    # SIGNATURE la réserverait aux seules apps plateforme, contredisant
    # directement l'intention de M11 ("aucune app n'avait de moyen de
    # déclencher elle-même un e-stop" — corrigé précisément pour
    # PERMETTRE ça aux apps tierces, avec confirmation humaine).
    Permission.EMERGENCY_STOP:       ProtectionLevel.DANGEROUS,
    Permission.FLEET_BROADCAST:      ProtectionLevel.SIGNATURE,
    # DANGEROUS et non SIGNATURE, pour la même raison qu'EMERGENCY_STOP
    # ci-dessus : réserver le pilotage par IA aux seules apps
    # plateforme fermerait le Play Store aux développeurs tiers
    # travaillant sur des politiques apprises, ce qui est justement
    # l'usage visé. La confirmation humaine à l'installation est le
    # bon niveau — l'utilisateur doit savoir qu'une IA commandera le
    # robot, pas en être protégé par interdiction.
    Permission.ACTUATORS_AI_DRIVEN:  ProtectionLevel.DANGEROUS,
    # DANGEROUS : un robot qui se déplace seul dans un local est un
    # risque que l'utilisateur doit accepter explicitement.
    Permission.LOCOMOTION_WALK:      ProtectionLevel.DANGEROUS,
    # NORMAL : écrire dans son propre espace cloisonné ne met rien en
    # danger physiquement. C'est une question de traçabilité, pas de
    # sécurité.
    Permission.STORAGE_PERSISTENT:   ProtectionLevel.NORMAL,
}


def get_protection_level(permission: Permission) -> ProtectionLevel:
    """Niveau de protection d'une permission — NORMAL si absente de
    PERMISSION_LEVELS (permissions sans risque particulier, jamais
    besoin d'être listées explicitement)."""
    return PERMISSION_LEVELS.get(permission, ProtectionLevel.NORMAL)


class Capability(Enum):
    """
    Capacités FONCTIONNELLES qu'une app peut exiger — distinct de
    RobotCapability (axis_os/hal/core/capabilities.py), qui décrit le
    MATÉRIEL physique d'un robot (ARMS, LEGS, CAMERA...) et alimente
    déjà HardwareRequirements. Capability décrit ce qu'une app a
    besoin de FAIRE, à une granularité parfois plus fine (CAMERA_RGB
    vs CAMERA_DEPTH, que RobotCapability.CAMERA ne distingue pas) et
    incluant des notions purement logicielles (MEMORY_PERSISTENT)
    qu'aucun capteur ne représente.

    Volontairement une enum plate, pas de découverte dynamique de
    capacités à l'exécution — cohérent avec l'intention : le jour où
    un service système existe vraiment pour l'une de ces valeurs, son
    IService.capability (voir axis_os/services/base.py) la référence,
    mais la liste elle-même reste statique.
    """
    WALK = auto(); GRASP_SINGLE = auto(); GRASP_DUAL = auto()
    CAMERA_RGB = auto(); CAMERA_DEPTH = auto()
    SPEECH_IN = auto();  SPEECH_OUT = auto()
    NAVIGATION_MAP = auto();  MEMORY_PERSISTENT = auto()
    # Le robot peut faire tourner une politique apprise (GR00T, VLA) —
    # exige un GPU embarqué capable de l'inférence, ou un accès réseau
    # à un serveur de politique. Sépare volontairement la CAPACITÉ
    # (le robot en est-il matériellement capable) de la PERMISSION
    # ACTUATORS_AI_DRIVEN (l'app a-t-elle le droit de s'en servir) :
    # une app refusée à l'installation faute de GPU est un message
    # clair, une app installée qui échoue à l'exécution ne l'est pas.
    AI_POLICY = auto()


@dataclass(frozen=True)
class IntentDecl:
    """
    Déclaration d'intention qu'une app sait servir — la brique de
    base d'un futur registre d'intentions (orchestrateur, non
    implémenté ce soir). Volontairement une simple dataclass de champ
    de manifest, sans aucune logique de correspondance associée : le
    manifest déclare, l'éventuel orchestrateur futur consomme.
    """
    intent_id:   str                 # "logistics.sort_packages"
    description: str                 # "trier des colis par destination"
    examples:    list[str] = field(default_factory=list)


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

    # Capacités fonctionnelles et intentions déclarées — champs ajoutés
    # maintenant, même vides pour la quasi-totalité des apps existantes
    # (voir docs/architecture_services*.md) : le jour où des apps
    # tierces existent en nombre, changer le format du manifest casse
    # tout le catalogue. Ajouter ces deux champs aujourd'hui coûte
    # quelques lignes ; le faire après coûterait une migration.
    capabilities_required: list[Capability] = field(default_factory=list)
    intents:                list[IntentDecl] = field(default_factory=list)

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
