"""
AXIS-OS Robotics Play Store — manifeste d'app
==============================================
Le manifeste est le contrat déclaratif que toute app doit fournir avant
d'être installée sur un robot par le Play Store d'AXIS-OS (le magasin
d'apps du robot). Il dit qui a écrit l'app, ce qu'elle a le droit de faire
(ses permissions), le matériel dont elle a besoin, les ressources qu'elle
peut consommer et son prix.

PlayStoreRuntime (axis_os/store/runtime/play_store.py) le vérifie à
l'installation. RobotContext, l'objet par lequel une app commande le robot
(axis_os/store/sdk/app_base.py), le consulte ensuite avant chaque action
protégée.

Contenu du fichier :
  - Permission, ProtectionLevel, PERMISSION_LEVELS : les droits qu'une app
    peut demander et le niveau d'accord exigé pour chacun ;
  - Capability, IntentDecl : ce que l'app a besoin de faire, et les
    demandes qu'elle sait servir ;
  - ResourceQuota, HardwareRequirements : ressources et matériel requis ;
  - AppManifest : le manifeste lui-même, avec sa méthode validate().

Licence : Apache 2.0 (open-source — SDK public)
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

from axis_os.hal.core.capabilities import RobotCapability


class Permission(Enum):
    """
    Droits qu'une app peut demander dans son manifeste. RobotContext vérifie
    le droit au moment de l'appel, avec _require() : une méthode protégée
    lève PermissionError si le manifeste ne déclare pas la permission.
    Chaque permission a aussi un niveau de protection (voir ProtectionLevel
    et PERMISSION_LEVELS plus bas) : NORMAL, accordée d'office ; DANGEROUS,
    confirmée par une personne à l'installation ; SIGNATURE, réservée aux
    apps de la plateforme.

    Attention : certaines permissions ne protègent encore rien. Un audit
    externe du code (14 septembre 2026, voir
    docs/audit_externe_2026-09-14.md et CHANGELOG.md) a relevé que sept
    d'entre elles étaient déclarées sans qu'AUCUNE méthode de RobotContext ne
    les vérifie via _require(). Depuis, SENSORS_CAMERA et SENSORS_MICROPHONE
    sont appliquées ; celles qui ne le sont toujours pas portent un
    commentaire « NON APPLIQUÉE ».

    Pourquoi les garder : les retirer casserait la rétrocompatibilité de
    tout manifeste existant qui les déclare déjà, et les laisser sans
    explication ferait croire à une protection qui n'existe pas. Choix
    retenu : le dire clairement ici, et appliquer chaque permission au fur
    et à mesure que la fonction correspondante existera vraiment dans
    RobotContext. Exemple : SENSORS_FORCE_TORQUE le sera dès qu'une méthode
    get_force_torque() y sera exposée. Le HAL (la couche d'abstraction
    matérielle, seul chemin vers les moteurs et les capteurs) a déjà cette
    fonction ; RobotContext ne l'expose pas encore.

    Les deux permissions NETWORK_* sont les plus importantes à ce titre :
    rien n'empêche aujourd'hui une app d'ouvrir elle-même une connexion
    réseau, car la sandbox (le processus séparé dans lequel tourne une app,
    créé avec le module multiprocessing de Python) ne filtre pas le réseau.
    C'est une faiblesse connue, plus large que ces deux permissions, de
    l'isolation des apps. docs/isolation_apps.md la décrit avec le plan
    prévu pour la fermer ; docs/sandboxing_hardening_notes.md la recense
    parmi les pistes de durcissement.
    """
    ACTUATORS_READ       = auto()  # lire l'état des articulations
    ACTUATORS_WRITE_ARMS = auto()  # commander les bras (move_arms)
    ACTUATORS_WRITE_LEGS = auto()  # commander les jambes (move_legs)
    ACTUATORS_WRITE_ALL  = auto()  # commander tout le corps (move_all)
    # Commandes articulaires brutes, y compris la commande en couple
    # (ControlMode.TORQUE), sans passer par move_arms, move_legs ou move_all.
    # Réservée au niveau 1, le plus bas, du SDK (le kit de développement
    # fourni aux apps) à trois niveaux, qui n'est pas encore implémenté
    # (voir docs/architecture_services.md, section 5).
    # Volontairement ProtectionLevel.SIGNATURE (voir PERMISSION_LEVELS plus
    # bas) : c'est le niveau avec lequel on casse du matériel, et il n'est
    # jamais accordé à une app tierce.
    ACTUATORS_RAW        = auto()
    # IMU : la centrale inertielle, qui mesure l'orientation et les
    # accélérations du robot (lue par get_imu).
    SENSORS_IMU          = auto()
    # Capteurs de force et de couple.
    SENSORS_FORCE_TORQUE = auto()  # NON APPLIQUÉE : voir la docstring
    SENSORS_CAMERA       = auto()  # appliquée : look_for / describe_scene
    SENSORS_MICROPHONE   = auto()  # appliquée : listen / declare_intent
    # Recevoir les alertes du Watchdog, la surveillance temps réel du robot
    # (état des moteurs notamment), qui peut déclencher l'arrêt d'urgence
    # (subscribe_alerts).
    WATCHDOG_READ        = auto()
    # Déclencher soi-même un e-stop, c'est-à-dire un arrêt d'urgence
    # (emergency_stop).
    EMERGENCY_STOP       = auto()
    # Envoyer un message à toute une flotte de robots. Le Fleet Manager (le
    # futur gestionnaire de flotte) n'existe pas encore : sa conception est
    # dans docs/fleet_manager_design.md.
    FLEET_BROADCAST      = auto()  # NON APPLIQUÉE : pas encore de Fleet Manager
    # Accès réseau vers des services en ligne (CLOUD) ou vers le réseau
    # local (LOCAL). La sandbox ne filtre pas encore le réseau : voir la
    # docstring de la classe.
    NETWORK_CLOUD        = auto()  # NON APPLIQUÉE : réseau non filtré
    NETWORK_LOCAL        = auto()  # NON APPLIQUÉE : réseau non filtré
    # Mouvements dont les consignes viennent d'une politique apprise (un
    # modèle d'IA entraîné, comme GR00T, le modèle de NVIDIA pour robots
    # humanoïdes, ou un VLA, modèle vision-langage-action) plutôt que d'un
    # script écrit à la main. Ne remplace pas ACTUATORS_WRITE_* : une app
    # pilotée par IA déclare les deux, ce qui rend le pilotage par IA
    # visible à l'installation au lieu de le laisser implicite.
    #
    # Pourquoi distinguer IA et script : les mouvements d'une app scriptée
    # peuvent être relus et vérifiés à l'avance, ceux d'une politique
    # apprise non. Cette permission tranche la question « distinction
    # IA/script dans les permissions », que la documentation avait
    # volontairement reportée (voir docs/groot_architecture.md, section 6).
    #
    # DANGEROUS, donc confirmée à l'installation. À l'exécution, toute
    # consigne produite par une action d'IA du SDK (VLAAction,
    # GR00TAction, LocomotionAction : ActuatorCommand.origine == "ia")
    # l'exige aussi (RobotContext._exiger_pilotage_ia). Limite : une app
    # qui fabrique elle-même ses consignes depuis la sortie d'un modèle
    # n'est pas détectée (voir SECURITY.md). La règle « Capability.AI_POLICY
    # exige ACTUATORS_AI_DRIVEN » (voir validate()) vaudra pour le futur
    # service VLA ; le runtime ne fournit pas encore AI_POLICY.
    ACTUATORS_AI_DRIVEN  = auto()
    # Faire se déplacer le robot dans l'espace (marche, roues...). Distincte
    # d'ACTUATORS_WRITE_LEGS, qui autorise à commander les articulations des
    # jambes : une app qui déplace le robot change son environnement, peut
    # heurter des objets, quitter une zone ou tomber dans un escalier. Une
    # app qui marche déclare donc les deux (RobotContext.walk() exige les
    # deux), pour que l'utilisateur voie à l'installation que le robot
    # changera de place.
    LOCOMOTION_WALK      = auto()
    # Mémoire persistante propre à l'app : ce qu'elle écrit survit à un
    # redémarrage du robot. Le stockage est cloisonné par app_id
    # (l'identifiant unique de l'app, voir MemoryService.app_store) : par
    # cette interface, une app ne peut ni lire ni écraser les données d'une
    # autre. Ce cloisonnement est logique, assuré par le service de mémoire,
    # et pas encore imposé par le système (faiblesse connue, voir
    # docs/isolation_apps.md). La permission reste nécessaire : un robot qui
    # retient des lieux et des objets accumule de l'information sur son
    # environnement, et l'utilisateur doit savoir quelles apps en gardent
    # une trace.
    STORAGE_PERSISTENT   = auto()


class ProtectionLevel(Enum):
    """
    Niveau de protection d'une permission, sur le modèle d'Android (où une
    permission « signature » n'est accordée qu'à une app signée avec la
    même clé que le système).

    Le niveau SIGNATURE s'appuie sur la signature Ed25519 (un algorithme de
    signature numérique) déjà en place dans axis_os/store/sdk/signing.py,
    et non sur un nouveau mécanisme : une app dont la `code_signature` se
    vérifie avec la clé de la plateforme est une app système. Il n'y a donc
    pas de liste blanche codée en dur à maintenir à part.
    """
    NORMAL    = auto()   # accordée automatiquement, ex. ACTUATORS_READ
    # Confirmation humaine explicite exigée à l'installation. Appliquée
    # depuis un audit de sécurité (10 octobre 2026) qui avait constaté que
    # l'installation ne demandait en fait rien : le `confirmer_permissions`
    # passé à PlayStoreRuntime doit répondre exactement True, sinon
    # l'installation est refusée. Essais :
    # test/test_permissions_dangereuses.py.
    DANGEROUS = auto()   # ex. ACTUATORS_WRITE_ALL, ACTUATORS_AI_DRIVEN
    # Réservée aux apps signées par la clé de la plateforme. Sans clé
    # plateforme configurée (platform_public_key de PlayStoreRuntime),
    # aucune permission SIGNATURE n'est accordée.
    SIGNATURE = auto()


# Niveau de protection de chaque permission. Une permission absente de ce
# dictionnaire est traitée comme NORMAL (voir get_protection_level() plus
# bas). C'est volontairement une table statique et non une méthode de
# Permission : la définition des permissions reste séparée de leur
# politique de protection, dans l'esprit de la règle de conception « pas
# de découverte dynamique, une enum en dur suffit ».
PERMISSION_LEVELS: dict[Permission, ProtectionLevel] = {
    Permission.ACTUATORS_READ:       ProtectionLevel.NORMAL,
    Permission.ACTUATORS_WRITE_ARMS: ProtectionLevel.DANGEROUS,
    Permission.ACTUATORS_WRITE_LEGS: ProtectionLevel.DANGEROUS,
    Permission.ACTUATORS_WRITE_ALL:  ProtectionLevel.DANGEROUS,
    Permission.ACTUATORS_RAW:        ProtectionLevel.SIGNATURE,
    # EMERGENCY_STOP est délibérément DANGEROUS et non SIGNATURE. Décision
    # prise en corrigeant une régression réelle : la classer SIGNATURE
    # cassait security_patrol et warehouse_ops, deux apps de référence qui
    # déclarent légitimement cette permission pour pouvoir déclencher un
    # arrêt d'urgence si elles détectent un danger (personne dans la
    # trajectoire, anomalie de sécurité). SIGNATURE l'aurait réservée aux
    # seules apps de la plateforme, à l'opposé d'une correction antérieure
    # décrite dans CHANGELOG.md : un audit externe avait relevé
    # qu'« aucune app n'avait de moyen de déclencher elle-même un e-stop »,
    # et RobotContext.emergency_stop() a été ajouté précisément pour le
    # permettre aux apps tierces, avec confirmation humaine.
    Permission.EMERGENCY_STOP:       ProtectionLevel.DANGEROUS,
    Permission.FLEET_BROADCAST:      ProtectionLevel.SIGNATURE,
    # DANGEROUS et non SIGNATURE, pour la même raison qu'EMERGENCY_STOP :
    # réserver le pilotage par IA aux seules apps de la plateforme fermerait
    # le Play Store aux développeurs tiers qui travaillent sur des
    # politiques apprises, alors que c'est justement l'usage visé. La
    # confirmation humaine à l'installation est le bon niveau : l'utilisateur
    # doit savoir qu'une IA commandera le robot, pas en être protégé par une
    # interdiction.
    Permission.ACTUATORS_AI_DRIVEN:  ProtectionLevel.DANGEROUS,
    # DANGEROUS : un robot qui se déplace seul dans un local est un risque
    # que l'utilisateur doit accepter explicitement.
    Permission.LOCOMOTION_WALK:      ProtectionLevel.DANGEROUS,
    # NORMAL : écrire dans son propre espace cloisonné ne met rien en danger
    # physiquement. C'est une question de traçabilité, pas de sécurité.
    Permission.STORAGE_PERSISTENT:   ProtectionLevel.NORMAL,
}


def get_protection_level(permission: Permission) -> ProtectionLevel:
    """Niveau de protection d'une permission. Renvoie NORMAL si elle est
    absente de PERMISSION_LEVELS : les permissions sans risque particulier
    n'ont jamais besoin d'y être listées."""
    return PERMISSION_LEVELS.get(permission, ProtectionLevel.NORMAL)


class Capability(Enum):
    """
    Capacités fonctionnelles qu'une app peut exiger : ce qu'elle a besoin
    de FAIRE (marcher, saisir un objet, voir, parler, garder une mémoire...).

    À ne pas confondre avec RobotCapability (axis_os/hal/core/
    capabilities.py), qui décrit le matériel physique d'un robot (ARMS,
    LEGS, CAMERA...) et alimente HardwareRequirements. Capability est
    parfois plus fine : elle distingue CAMERA_RGB (l'image couleur) et
    CAMERA_DEPTH (l'image de profondeur, qui donne la distance de chaque
    point), que RobotCapability.CAMERA confond. Elle inclut aussi des
    notions purement logicielles (MEMORY_PERSISTENT) qu'aucun capteur ne
    représente.

    C'est volontairement une enum plate, sans découverte dynamique de
    capacités à l'exécution. Le jour où un service système existe vraiment
    pour l'une de ces valeurs, son IService.capability (voir
    axis_os/services/base.py) la référence ; la liste, elle, reste
    statique.
    """
    WALK = auto(); GRASP_SINGLE = auto(); GRASP_DUAL = auto()
    CAMERA_RGB = auto(); CAMERA_DEPTH = auto()
    SPEECH_IN = auto();  SPEECH_OUT = auto()
    NAVIGATION_MAP = auto();  MEMORY_PERSISTENT = auto()
    # Le robot peut faire tourner une politique apprise (GR00T, VLA). Cela
    # exige un GPU (processeur graphique) embarqué capable de l'inférence
    # (le calcul des actions par le modèle), ou un accès réseau à un serveur
    # de politique. On sépare volontairement la CAPACITÉ (le robot en
    # est-il matériellement capable ?) de la PERMISSION ACTUATORS_AI_DRIVEN
    # (l'app a-t-elle le droit de s'en servir ?) : une app refusée à
    # l'installation faute de GPU reçoit un message clair, alors qu'un échec
    # en pleine exécution, après installation, n'en donne pas.
    AI_POLICY = auto()


@dataclass(frozen=True)
class IntentDecl:
    """
    Une intention qu'une app sait servir, c'est-à-dire un type de demande
    exprimé en termes métier (ex. « trier des colis par destination »).

    C'est la brique de base d'un futur registre d'intentions, que
    consultera l'orchestrateur (la future couche qui reliera les demandes
    aux apps et services capables de les servir). Ni l'un ni l'autre n'est
    encore implémenté (voir docs/architecture_services.md, section 4).
    Volontairement une simple dataclass, champ du manifeste, sans aucune
    logique de correspondance : le manifeste déclare, le futur
    orchestrateur consommera.
    """
    intent_id:   str                 # ex. "logistics.sort_packages"
    description: str                 # ex. "trier des colis par destination"
    examples:    list[str] = field(default_factory=list)


class AppCategory(Enum):
    """Catégorie de l'app dans le catalogue. La valeur est le libellé
    affiché, par exemple à l'installation."""
    LOGISTICS     = "Logistique"
    SECURITY      = "Sécurité"
    MAINTENANCE   = "Maintenance"
    HEALTHCARE    = "Santé"
    MANUFACTURING = "Fabrication"
    RESEARCH      = "Recherche"
    UTILITIES     = "Utilitaires"


class PricingModel(Enum):
    """Modèle de tarification : gratuit, achat unique, abonnement mensuel
    ou paiement à la mission."""
    FREE           = "free"
    ONE_TIME       = "one_time"
    SUBSCRIPTION   = "sub_month"
    PAY_PER_MISSION = "per_mission"


@dataclass
class ResourceQuota:
    """
    Quotas de ressources imposés à l'app dans sa sandbox, pour qu'elle ne
    puisse pas accaparer le processeur (CPU) ou la mémoire du robot, ni faire
    durer une mission indéfiniment.

    Le SandboxRunner (axis_os/store/runtime/sandbox.py) tue une app qui
    dépasse ces limites. Les règles exactes y sont décrites : par exemple,
    le CPU n'est sanctionné qu'au-delà du double du quota, sur trois
    mesures consécutives, et la surveillance du CPU et de la mémoire
    suppose que psutil (une bibliothèque de mesure des processus) soit
    installé.
    """
    max_cpu_percent: float = 50.0   # en % d'un cœur de processeur
    max_memory_mb:   int   = 256    # mémoire vive, en Mo
    max_runtime_s:   float = 30.0   # durée max. d'une mission (s), arrêt forcé


@dataclass
class HardwareRequirements:
    """
    Matériel dont l'app a besoin, exprimé en capacités (RobotCapability).

    `required`  : indispensables. Un robot qui ne les possède pas (au sens
                  de CapabilitySet.has(), donc physiquement présentes ET
                  actuellement disponibles, AVAILABLE) rend l'app
                  incompatible : PlayStoreRuntime.install() refuse
                  l'installation.
    `optional`  : utiles sans être indispensables. Une capacité manquante
                  ici ne bloque pas l'installation ; un avertissement est
                  seulement affiché. C'est à l'app de vérifier
                  ctx.robot.get_capabilities() en cours de mission avant
                  d'utiliser une capacité optionnelle.

    Le manifeste est un contrat, comme l'a rappelé une revue extérieure du
    code (6 septembre 2026) : une capacité dont l'app ne peut réellement pas
    se passer doit être déclarée dans `required`, jamais dans `optional`
    par prudence mal placée. Sinon l'app plantera en mission au lieu
    d'être refusée proprement à l'installation.

    min_dof (le nombre minimal de degrés de liberté, c'est-à-dire
    d'articulations que le robot peut commander au total) et
    compatible_models (les modèles de robot acceptés) restent des
    vérifications à part, indépendantes du système de capacités : celui-ci
    raisonne en catégories fonctionnelles, pas en nombre d'articulations
    ni en identité de fabricant.
    """
    required:             list[RobotCapability] = field(default_factory=list)
    optional:             list[RobotCapability] = field(default_factory=list)
    min_dof:              int        = 0
    compatible_models:    list[str]  = field(default_factory=list)


@dataclass
class AppManifest:
    """
    Manifeste d'une app : identité, permissions, besoins matériels, quotas,
    prix et, si l'auteur signe son code, signature.

    validate() renvoie la liste des erreurs du manifeste ;
    PlayStoreRuntime.install() l'appelle d'abord et refuse l'app si cette
    liste n'est pas vide.
    """
    # Identité de l'app. app_id est un identifiant unique au format
    # « com.auteur.nom » (vérifié par validate()). app_id, name et author
    # sont affichés à l'utilisateur quand il confirme les permissions.
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

    # Capacités fonctionnelles exigées et intentions servies. Ces champs
    # sont ajoutés dès maintenant, même s'ils restent vides pour presque
    # toutes les apps existantes (voir docs/architecture_services.md) : le
    # jour où des apps tierces existeront en nombre, changer le format du
    # manifeste casserait tout le catalogue. Les ajouter aujourd'hui coûte
    # quelques lignes ; le faire après coûterait une migration.
    capabilities_required: list[Capability] = field(default_factory=list)
    intents:                list[IntentDecl] = field(default_factory=list)

    # Tarification : voir PricingModel, et COMMISSION_RATE plus bas pour le
    # calcul du prix payé par le client.
    pricing_model:  PricingModel = PricingModel.FREE
    base_price_eur: float        = 0.0

    # Plafond de vitesse : RobotContext multiplie par ce ratio la vitesse de
    # chaque commande articulaire envoyée par move_arms(), move_legs() ou
    # move_all() (voir _apply_velocity_cap dans
    # axis_os/store/sdk/app_base.py). La marche demandée par walk() n'est
    # pas concernée : sa vitesse est bornée par le service de locomotion.
    # validate() exige une valeur dans ]0.0, 1.0].
    # requires_human_supervision : l'auteur indique si l'app doit tourner
    # sous la surveillance d'une personne. Pour l'instant c'est une simple
    # déclaration : le runtime ne s'en sert pas pour bloquer quoi que ce
    # soit.
    max_velocity_ratio:        float = 0.8
    requires_human_supervision: bool = True

    # Liens et contact de l'auteur, facultatifs.
    icon_url:      Optional[str] = None
    website_url:   Optional[str] = None
    support_email: Optional[str] = None

    # Version minimale d'AXIS-OS requise par l'app, au format SemVer
    # (versionnage sémantique MAJEUR.MINEUR.CORRECTIF, ex. "0.2.0").
    # Facultative et à None par défaut, pour ne jamais casser une app écrite
    # avant l'introduction de ce champ (politique de version complète dans
    # axis_os/__init__.py). Quand elle est renseignée,
    # PlayStoreRuntime.install() la compare à axis_os.__version__ avant
    # d'installer. La règle exacte est dans SemVer.is_compatible_with() ;
    # elle est plus stricte tant qu'on reste en version 0.x (même version
    # MINEUR exigée).
    sdk_version_min: Optional[str] = None

    # Signature Ed25519 du code source de l'app (voir
    # axis_os/store/sdk/signing.py). Facultative et à None par défaut, pour
    # ne jamais casser une app écrite avant l'introduction de ce mécanisme.
    # Quand code_signature et developer_public_key sont renseignés tous les
    # deux, PlayStoreRuntime.install() vérifie que le code source fourni à
    # l'installation correspond bien à cette signature, et refuse sinon. Un
    # seul des deux renseigné (signature sans clé, ou clé sans signature)
    # est une erreur de manifeste (voir validate()) : un développeur qui
    # commence à signer ses apps doit le faire correctement, pas à moitié.
    code_signature:        Optional[str] = None
    developer_public_key:  Optional[str] = None

    # Commission de la plateforme : 30 % du prix payé par le client. Le prix
    # public est calculé pour que le développeur touche exactement
    # base_price_eur : prix public = base_price_eur / (1 - COMMISSION_RATE).
    # Exemple : 7 € demandés par le développeur donnent 10 € payés par le
    # client, dont 3 € pour la plateforme.
    COMMISSION_RATE = 0.30

    @property
    def store_price_eur(self) -> float:
        """Prix payé par le client, commission comprise, arrondi au
        centime (0 pour une app gratuite)."""
        if self.pricing_model == PricingModel.FREE: return 0.0
        return round(self.base_price_eur / (1 - self.COMMISSION_RATE), 2)

    @property
    def developer_revenue_eur(self) -> float:
        """Ce que touche le développeur : son prix de base."""
        return self.base_price_eur

    @property
    def platform_commission_eur(self) -> float:
        """Part de la plateforme : prix public moins prix de base, arrondie
        au centime."""
        return round(self.store_price_eur - self.base_price_eur, 2)

    def has_permission(self, perm: Permission) -> bool:
        """Vrai si le manifeste déclare cette permission. C'est le test
        qu'utilise RobotContext._require() avant chaque action protégée."""
        # Comparaison par identité (« is »), pas avec « in », qui appelle
        # __eq__ (la méthode d'égalité de l'objet). Relevé à la relecture
        # d'un audit de sécurité (octobre 2026) : un objet dont __eq__ répond
        # toujours vrai, glissé dans permissions, passait pour TOUTES les
        # permissions à la fois.
        return any(p is perm for p in self.permissions)

    def validate(self) -> list[str]:
        """Vérifie la cohérence du manifeste. Renvoie la liste des erreurs,
        en texte lisible ; une liste vide signifie que le manifeste est
        valide."""
        errors = []
        # Types exacts, vérifiés avec type() et non isinstance(). Relevé à la
        # relecture d'un audit de sécurité (octobre 2026) : une entrée qui
        # n'était pas une Permission était tenue pour NORMAL
        # (get_protection_level ne la connaît pas) et échappait ainsi à la
        # confirmation exigée pour DANGEROUS comme au contrôle SIGNATURE.
        # Même risque avec une liste dont « in » répondrait oui à tout. La
        # même règle s'applique aux capacités.
        if type(self.permissions) is not list \
                or any(type(p) is not Permission for p in self.permissions):
            errors.append("permissions doit être une liste de Permission (rien d'autre)")
        if type(self.capabilities_required) is not list \
                or any(type(c) is not Capability for c in self.capabilities_required):
            errors.append("capabilities_required doit être une liste de Capability")
        # Ces textes sont affichés à l'utilisateur quand il confirme les
        # permissions : ils ne doivent contenir aucun caractère de contrôle
        # (séquences d'échappement qui réécriraient l'écran, sauts de ligne
        # qui imiteraient une autre question).
        for champ in ("app_id", "name", "author"):
            v = getattr(self, champ)
            if not isinstance(v, str) or any(not c.isprintable() for c in v):
                errors.append(f"{champ} doit être un texte sans caractère de contrôle")
        if not self.app_id or "." not in self.app_id:
            errors.append("app_id doit être en format 'com.author.name'")
        if self.base_price_eur < 0:
            errors.append("base_price_eur ne peut pas être négatif")
        if self.pricing_model != PricingModel.FREE and self.base_price_eur == 0:
            errors.append("Un modèle payant doit avoir un base_price_eur > 0")
        # Faille relevée lors d'un audit : avec un base_price_eur trop bas
        # (ex. 0,01 €), l'arrondi au centime de store_price_eur peut ramener
        # platform_commission_eur à 0,00 €. La plateforme ne toucherait alors
        # rien du tout sur la transaction, alors que le manifeste annonce un
        # modèle payant à 30 % de commission. Le contrôle se calcule à partir
        # de COMMISSION_RATE plutôt que d'un seuil codé en dur, pour rester
        # juste si le taux change un jour. Il ne concerne que les modèles
        # payants : FREE a par conception une commission de 0,00 €, ce n'est
        # pas une anomalie.
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
        # Ici, seul le format de la version est vérifié ; la compatibilité
        # avec la version installée l'est par PlayStoreRuntime.install().
        if self.sdk_version_min is not None:
            from axis_os import SemVer
            try:
                SemVer.parse(self.sdk_version_min)
            except ValueError as e:
                errors.append(f"sdk_version_min invalide : {e}")
        # Une app qui exige une politique apprise (Capability.AI_POLICY) doit
        # déclarer ACTUATORS_AI_DRIVEN (règle ajoutée après un audit de
        # sécurité, octobre 2026). Sinon le pilotage par IA resterait
        # invisible à l'installation, alors que cette permission, DANGEROUS,
        # est justement ce que l'humain confirme.
        if Capability.AI_POLICY in self.capabilities_required \
                and Permission.ACTUATORS_AI_DRIVEN not in self.permissions:
            errors.append(
                "Capability.AI_POLICY exige la permission ACTUATORS_AI_DRIVEN : une "
                "app pilotée par une politique apprise doit le déclarer, pour que "
                "l'utilisateur le confirme à l'installation.")
        # Signature et clé vont ensemble : voir le commentaire de
        # code_signature plus haut.
        if (self.code_signature is None) != (self.developer_public_key is None):
            errors.append(
                "code_signature et developer_public_key doivent être renseignés "
                "ENSEMBLE ou pas du tout — un seul des deux ne permet aucune "
                "vérification cohérente."
            )
        return errors
