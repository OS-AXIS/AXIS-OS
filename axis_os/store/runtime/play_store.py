"""
AXIS-OS — Robotics Play Store Runtime
=======================================
Moteur d'exécution du Play Store côté robot. Le Play Store est la
boutique d'apps du robot : des développeurs tiers y publient des apps
(tri de colis, inventaire d'entrepôt…) que le robot installe puis
exécute.

Ce module gère, sur le robot :
  - l'installation, précédée de toutes les vérifications : manifeste
    (la fiche que chaque app déclare : identité, version, permissions,
    matériel requis, prix, quotas), compatibilité avec le SDK (le kit de
    développement des apps), signature du code, niveau de protection des
    permissions, capacités du robot ;
  - le cycle de vie : démarrage, missions, arrêt, mise à jour ;
  - l'exécution de chaque appel à l'app dans une sandbox (un « bac à
    sable » : un processus séparé, avec des quotas de CPU, de mémoire
    et de durée ; voir sandbox.py) ;
  - la facturation : la plateforme prélève une commission de 30 % sur
    le prix payé par le client, le développeur reçoit les 70 % restants.

Limite connue : la sandbox protège contre une app buggée, pas encore
contre une app délibérément hostile. Le constat et le plan de correction
sont dans docs/isolation_apps.md et SECURITY.md (« Limites connues ») ;
en attendant, ne faire tourner sur un vrai robot que des apps dont on
connaît et relit le code.

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations
import copy
from dataclasses import dataclass, field
from typing import Callable, Optional, Dict, Any, List
import time
import uuid
import traceback

from axis_os import SemVer
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.core.capabilities import RobotCapability
from axis_os.store.sdk.manifest import (
    AppManifest, Permission, PricingModel, Capability, ProtectionLevel,
    get_protection_level,
)
from axis_os.store.sdk.app_base import (
    RobotApp, RobotContext, MissionResult, PermissionError, EstopActiveError,
    _RuntimeSessionToken,
)
from axis_os.store.runtime.sandbox import SandboxRunner, SandboxState, SandboxErrorKind
from axis_os.store.runtime.hal_bridge import (
    HALBridgeServer, HALBridgeClient, HALCommand, HALResponse
)
from axis_os.services.memory.service import MemoryService
import multiprocessing as mp
import threading


# ── Confirmation des permissions DANGEROUS ────────────────────────
#
# Chaque permission a un niveau de protection (ProtectionLevel, voir
# axis_os/store/sdk/manifest.py). Le niveau DANGEROUS regroupe les
# permissions qui donnent à l'app un pouvoir physique sur le robot (bouger
# ses bras ou ses jambes, le faire marcher…) ; il promet une confirmation
# humaine avant l'installation.
#
# Pourquoi ce code : un audit de sécurité (2026-10-10) a montré que cette
# promesse n'était pas tenue. install() ne demandait rien, et une app
# déclarant ACTUATORS_WRITE_ALL (commander tout le corps) ou
# ACTUATORS_AI_DRIVEN (mouvements dont les consignes viennent d'un modèle
# d'IA) s'installait sans que personne n'ait accepté qu'elle bouge le
# robot.
#
# Règle désormais : refus par défaut. Une permission DANGEROUS n'est
# accordée que si la fonction `confirmer_permissions` fournie au runtime
# répond exactement True ; il n'existe aucun accord implicite. Essais :
# test/test_permissions_dangereuses.py.

ConfirmationPermissions = Callable[[AppManifest, List[Permission]], bool]


def accepter_tout_SIMULATION(manifest: AppManifest,
                             permissions: List[Permission]) -> bool:
    """Accorde toutes les permissions DANGEROUS sans rien demander.

    Réservée à la simulation et aux tests. Son nom est choisi pour qu'on ne
    la passe pas par erreur sur un vrai robot ; en plus, PlayStoreRuntime
    refuse de la recevoir quand il pilote un robot réel connu : l'humanoïde
    Unitree G1 en backend DDS (Data Distribution Service, le bus de
    communication par lequel le vrai G1 reçoit ses commandes ; voir
    _robot_reel). Sur un vrai robot, utiliser confirmer_au_terminal ou un
    écran de confirmation."""
    print(f"  [SIMULATION] permissions DANGEROUS accordées SANS confirmation "
          f"humaine : {[p.name for p in permissions]}")
    return True


def _affichable(texte: Any, n: int = 80) -> str:
    """Rend inoffensif pour un terminal un texte venu de l'app.

    Pourquoi : un nom ou un auteur d'app peut contenir des caractères de
    contrôle (séquences d'échappement du terminal, sauts de ligne) qui
    fausseraient l'affichage de la question posée à l'opérateur. Ils sont
    remplacés par « ? », et la longueur est bornée à `n` caractères.
    validate() refuse déjà ces caractères ; ceci est une seconde
    barrière."""
    t = "".join(c if c.isprintable() else "?" for c in str(texte))
    return t if len(t) <= n else t[:n - 1] + "…"


def confirmer_au_terminal(manifest: AppManifest,
                          permissions: List[Permission]) -> bool:
    """Demande à l'opérateur, au terminal, d'accepter les permissions
    DANGEROUS.

    Seul « o » ou « oui » accorde ; tout le reste (Entrée seule, fin de
    saisie, erreur de lecture) refuse. Le nom, l'auteur et l'app_id viennent
    de l'app : ils sont affichés neutralisés par _affichable(), car la
    relecture de l'audit de sécurité du 2026-10-10 a montré qu'une
    séquence d'échappement placée dans ces champs pouvait masquer la vraie
    liste des permissions et afficher une fausse question. Pour la même
    raison, la question elle-même répète l'app_id et les permissions
    demandées."""
    app_id = _affichable(manifest.app_id)
    noms = ", ".join(p.name for p in permissions)
    print(f"\n  L'app « {_affichable(manifest.name)} » ({app_id}, "
          f"{_affichable(manifest.author)}) demande des permissions DANGEREUSES :")
    for p in permissions:
        print(f"    - {p.name}")
    try:
        rep = input(f"  Accorder {noms} à {app_id} ? [o/N] ")
    except (EOFError, OSError):
        return False
    return rep.strip().lower() in ("o", "oui")


def _robot_reel(robot: Any) -> bool:
    """Vrai si le HAL pilote un robot réel connu.

    Le HAL (couche d'abstraction matérielle : l'interface commune par
    laquelle l'OS commande les moteurs et lit les capteurs) peut être
    enveloppé, par exemple par NormalizedRobotHAL, qui garde le HAL
    d'origine dans `_hal` ; on remonte au plus cinq niveaux. Aujourd'hui,
    seul le G1 en backend « dds » est reconnu comme robot réel.

    C'est un filet de sécurité pour accepter_tout_SIMULATION, pas une
    détection exhaustive : c'est d'abord le nom de cette fonction qui doit
    empêcher l'erreur."""
    vus = 0
    while robot is not None and vus < 5:
        if getattr(robot, "_backend", None) == "dds":
            return True
        robot, vus = getattr(robot, "_hal", None), vus + 1
    return False


def _run_app_in_sandbox(app: RobotApp, params: Dict[str, Any],
                        command_queue: mp.Queue, response_queue: mp.Queue) -> MissionResult:
    """
    Exécutée dans le processus enfant de la sandbox (lancé par
    SandboxRunner) pour un appel à run_mission().

    Pourquoi un pont : le vrai HAL et le vrai WatchdogSystem (le chien de
    garde qui déclenche l'e-stop, l'arrêt d'urgence, quand une condition
    dangereuse est détectée) contiennent des threads et ne passent pas
    par pickle (le format de Python qui transforme un objet en octets
    pour le faire passer d'un processus à l'autre). La fonction ne reçoit
    donc jamais ces objets : elle reçoit l'app elle-même (SandboxRunner
    exige qu'elle passe par pickle) et construit un HALBridgeClient sur
    les deux files mp.Queue déjà ouvertes. C'est le pont IPC
    (communication entre processus) de hal_bridge.py : les commandes de
    l'app partent par une file, le processus parent les exécute sur le
    vrai HAL et répond par l'autre.

    L'app appelle ctx.move_arms/move_legs/log/get_imu/get_mission_log
    exactement comme avec un RobotContext normal (l'objet que le runtime
    remet à une app pour commander le robot) : HALBridgeClient offre
    la même interface, donc aucune app existante n'a besoin d'être
    modifiée pour bénéficier de l'isolation.

    PermissionError et EstopActiveError sont converties ici en
    MissionResult d'échec plutôt que de remonter jusqu'à SandboxRunner,
    qui les classerait en APPLICATION_ERROR générique : on garde ainsi la
    distinction entre un refus de sécurité et un bug de l'app.

    Limite connue : cette isolation protège d'une app buggée, pas encore
    d'une app hostile (voir docs/isolation_apps.md).
    """
    bridge_ctx = HALBridgeClient(command_queue, response_queue)
    try:
        result = app.run_mission(bridge_ctx, params)
        if result is not None:
            result.log = bridge_ctx.get_mission_log()
        return result
    except PermissionError as exc:
        return MissionResult(
            success=False, mission_id="", app_id="",
            duration_s=0.0, error_message=f"Permission refusée : {exc}",
            log=bridge_ctx.get_mission_log(),
        )
    except EstopActiveError as exc:
        return MissionResult(
            success=False, mission_id="", app_id="",
            duration_s=0.0, error_message=f"Mission interrompue par e-stop : {exc}",
            log=bridge_ctx.get_mission_log(),
        )


def _run_start_hook_in_sandbox(app: RobotApp, command_queue: mp.Queue,
                               response_queue: mp.Queue):
    """
    Équivalent de _run_app_in_sandbox() pour le hook on_start() (la
    méthode que l'app exécute à son démarrage).

    Pourquoi : un audit de sécurité externe (2026-09-14, voir
    docs/audit_externe_2026-09-14.md et CHANGELOG.md) a montré que
    start_app() et stop_app() appelaient jusque-là entry.app.on_start()
    et on_stop() directement dans le processus parent, avec un vrai
    RobotContext : accès complet au HAL, sans quota, sans délai maximal,
    sans isolation de la mémoire. C'était exactement le trou que
    SandboxRunner et HALBridgeServer comblent pour run_mission(), mais ils
    n'avaient jamais été branchés ici. Le risque
    n'était pas théorique : axis_os/apps/package_sorting/app.py appelle
    ctx.move_arms(...) dans son on_start(), donc le robot bougeait
    physiquement pendant ce hook non isolé.

    Renvoie un couple (bool, list[str]) plutôt qu'un simple bool : le
    journal de mission tenu par le HALBridgeClient serait perdu s'il
    n'était pas renvoyé explicitement d'un processus à l'autre.
    """
    bridge_ctx = HALBridgeClient(command_queue, response_queue)
    try:
        ready = app.on_start(bridge_ctx)
        return ready, bridge_ctx.get_mission_log()
    except (PermissionError, EstopActiveError) as exc:
        return False, bridge_ctx.get_mission_log() + [f"on_start refusé : {exc}"]


def _run_stop_hook_in_sandbox(app: RobotApp, command_queue: mp.Queue,
                              response_queue: mp.Queue):
    """Équivalent de _run_start_hook_in_sandbox() pour le hook on_stop()
    (exécuté à l'arrêt de l'app) : voir sa docstring pour le contexte."""
    bridge_ctx = HALBridgeClient(command_queue, response_queue)
    try:
        app.on_stop(bridge_ctx)
        return True, bridge_ctx.get_mission_log()
    except (PermissionError, EstopActiveError) as exc:
        return False, bridge_ctx.get_mission_log() + [f"on_stop refusé : {exc}"]


@dataclass
class InstalledApp:
    """Une app installée sur ce robot, avec son manifeste validé, ses
    compteurs (missions, revenus) et son état de facturation."""
    app:           RobotApp
    manifest:      AppManifest
    install_time:  float = field(default_factory=time.monotonic)
    install_id:    str   = field(default_factory=lambda: str(uuid.uuid4())[:8])
    mission_count: int   = 0
    total_revenue: float = 0.0
    is_started:    bool  = False
    # État de facturation de l'app sur ce robot.
    #
    # Pourquoi ces deux champs : un audit de sécurité externe a montré que
    # _bill() facturait le prix complet à chaque mission, quel que soit le
    # modèle de prix (pricing_model). Une app ONE_TIME (achat unique)
    # était facturée à chaque mission au lieu d'une seule fois, et une app
    # SUBSCRIPTION (abonnement) facturait l'abonnement mensuel entier à
    # chaque mission au lieu d'une fois par période. Ces champs mémorisent
    # ce qui a déjà été facturé, pour que _bill() applique la bonne règle
    # à chaque modèle (détail dans sa docstring).
    #
    # Attention : sans memory_service, cet état n'existe qu'en mémoire et
    # se perd à chaque redémarrage du processus. Un second audit externe
    # l'a relevé : un redémarrage refacture alors une app ONE_TIME comme
    # neuve et remet à zéro la fenêtre SUBSCRIPTION de 30 jours. La
    # persistance sur disque a été ajoutée depuis, en option, par le
    # paramètre memory_service de PlayStoreRuntime (voir __init__). Sans
    # lui, la limite demeure : à traiter avant tout déploiement où un
    # redémarrage de PlayStoreRuntime est un événement réaliste en
    # production.
    #
    # last_subscription_bill_time_s utilise time.time(), l'horloge murale
    # (la date et l'heure réelles), et non time.monotonic(). Correction
    # faite après le second audit externe, qui a relevé que monotonic() ne
    # représente pas une date : sa valeur n'a de sens que comme différence
    # entre deux appels dans le même processus, jamais pour dire « il y a
    # 30 jours », et son point de départ est arbitraire et change à chaque
    # redémarrage. Ce défaut aggravait concrètement, et pas seulement en
    # théorie, le problème de persistance décrit ci-dessus : une valeur
    # monotonic() enregistrée puis relue après un redémarrage ne veut plus
    # rien dire.
    one_time_billed:              bool            = False
    last_subscription_bill_time_s: Optional[float] = None

    @property
    def commission_due(self) -> float:
        return round(self.total_revenue * AppManifest.COMMISSION_RATE, 4)


@dataclass
class BillingReport:
    """Reçu d'une facturation : montant payé par le client, commission de
    la plateforme (30 %) et part du développeur (70 %)."""
    mission_id:       str
    app_id:           str
    pricing_model:    PricingModel
    gross_amount_eur: float
    commission_eur:   float
    developer_eur:    float
    timestamp:        float = field(default_factory=time.time)

    def __str__(self) -> str:
        return (f"[BILLING] {self.app_id} | mission={self.mission_id} | "
                f"total={self.gross_amount_eur:.2f}€ | "
                f"commission={self.commission_eur:.2f}€ (30%) | "
                f"dev={self.developer_eur:.2f}€ (70%)")


class PlayStoreRuntime:
    """
    Moteur d'exécution du Play Store sur un robot : installe les apps, les
    démarre, exécute leurs missions dans la sandbox, les arrête, les met à
    jour et tient la facturation.

    Usage :
        runtime = PlayStoreRuntime(norm_robot, watchdog,
                                   confirmer_permissions=confirmer_au_terminal)
        runtime.install(PackageSortingApp())
        runtime.start_app("com.acme.package_sorting")
        result = runtime.run_mission("com.acme.package_sorting", params={})
    """

    def __init__(
        self,
        robot: NormalizedRobotHAL,
        watchdog: WatchdogSystem,
        require_signed_apps: bool = False,
        platform_public_key: Optional[str] = None,
        memory_service: Optional[MemoryService] = None,
        perception_service=None,
        locomotion_service=None,
        speech_service=None,
        clock=None,
        confirmer_permissions: Optional[ConfirmationPermissions] = None,
    ):
        """
        `robot` : le HAL du robot, vu à travers NormalizedRobotHAL, la
        façade qui présente aux apps les positions et vitesses dans une
        convention commune plutôt que celle du driver brut.
        `watchdog` : le chien de garde du robot, consulté avant chaque
        mission pour savoir si un e-stop a été déclenché, et transmis au
        RobotContext de chaque app.

        `confirmer_permissions` : fonction (manifeste, permissions
        DANGEROUS) -> bool. install() l'appelle (donc aussi update_app(),
        qui passe par install()) pour toute app qui déclare au moins une
        permission de niveau ProtectionLevel.DANGEROUS ; seule la réponse
        True accorde. None par défaut : aucune permission DANGEROUS n'est
        accordée et l'app est refusée. En cas de doute, le système refuse
        (comportement dit fail-safe) ; voir accepter_tout_SIMULATION et
        confirmer_au_terminal plus haut. Passer accepter_tout_SIMULATION
        avec un robot réel lève ValueError.

        `require_signed_apps` : si True, install() refuse toute app dont
        code_signature et developer_public_key ne sont pas renseignés.
        False par défaut, pour rester compatible avec les apps existantes
        non signées. Pourquoi ce paramètre : un audit de sécurité a montré
        que la signature était jusque-là facultative, au choix de l'app
        (les deux champs valent None par défaut) ; une app malveillante
        n'avait qu'à omettre le champ pour s'installer sans aucune
        vérification. Le CHANGELOG présentait la signature comme réglant
        le fait que « rien n'empêchait un fichier .py quelconque de se
        faire passer pour une app légitime » : ce n'est vrai qu'avec ce
        paramètre à True. À activer explicitement avant d'ouvrir un vrai
        Play Store à des développeurs externes non vérifiés.

        `platform_public_key` : clé publique Ed25519 (un algorithme de
        signature numérique ; même format que
        AppManifest.developer_public_key) qui distingue une app système
        d'une app tierce, voir ProtectionLevel.SIGNATURE dans
        axis_os/store/sdk/manifest.py. Une app dont developer_public_key
        est exactement cette clé, et dont la signature est valide, est
        traitée comme signée par la plateforme : elle peut déclarer des
        permissions de niveau SIGNATURE (par exemple ACTUATORS_RAW, les
        commandes articulaires brutes). None par défaut (compatible avec
        l'existant) : sans clé plateforme configurée, aucune app, même
        correctement signée par un vrai développeur, n'est traitée comme
        système, et toute permission SIGNATURE est refusée à
        l'installation. Là encore, le défaut est le refus (fail-safe),
        jamais l'accord (fail-open).

        `memory_service` : service de mémoire persistante (voir
        axis_os/services/memory/service.py), qui doit être déjà démarré
        par MemoryService.start(). S'il est fourni, l'état de facturation
        (one_time_billed, last_subscription_bill_time_s) y est lu à
        l'installation et réécrit à chaque changement. Cela comble le trou
        relevé par deux audits externes successifs : cet état n'existait
        qu'en mémoire et se perdait à chaque redémarrage (une app ONE_TIME
        refacturée comme neuve, une fenêtre SUBSCRIPTION de 30 jours
        remise à zéro). None par défaut (compatible avec l'existant) :
        comportement strictement inchangé, état en mémoire seulement,
        comme avant ce paramètre. Essais : test/test_billing_persistence.py.

        `perception_service`, `locomotion_service`, `speech_service` :
        services système (vision, marche, parole) transmis à chaque app
        par son RobotContext ; voir _make_ctx(). `clock` : horloge du
        robot, voir le commentaire dans le corps de cette méthode.
        """
        if confirmer_permissions is accepter_tout_SIMULATION and _robot_reel(robot):
            raise ValueError(
                "accepter_tout_SIMULATION refusé : ce runtime pilote un robot RÉEL. "
                "Les permissions DANGEROUS doivent y être confirmées par un humain "
                "(confirmer_au_terminal, ou un écran de confirmation).")
        self._confirmer_permissions = confirmer_permissions
        self._robot   = robot
        self._watchdog = watchdog
        self._apps:    Dict[str, InstalledApp] = {}
        self._billing: List[BillingReport]     = []
        self._id = str(uuid.uuid4())[:8]
        self._require_signed_apps = require_signed_apps
        if not require_signed_apps:
            # Avertissement affiché à chaque démarrage, et pas seulement
            # documenté : un choix silencieux s'oublie facilement dans une
            # configuration de production. Sans require_signed_apps=True,
            # n'importe quel fichier .py dont une classe hérite de
            # RobotApp s'installe sans vérification d'identité : install()
            # ne contrôle alors que les permissions et les capacités
            # déclarées, jamais qui a écrit le code. Contexte complet dans
            # la docstring de __init__.
            print(
                "  [SECURITY] PlayStoreRuntime démarré avec "
                "require_signed_apps=False — toute app non signée "
                "s'installe sans vérification d'identité. Adapté au "
                "développement, à activer explicitement avant "
                "d'ouvrir ce Play Store à des développeurs externes."
            )
        self._platform_public_key = platform_public_key
        self._memory_service = memory_service
        self._perception_service = perception_service
        self._locomotion_service = locomotion_service
        self._speech_service = speech_service
        # Horloge du robot : simulée en simulation, réelle sur le robot.
        # Elle est transmise à la sandbox (pour mesurer les quotas) et aux
        # apps (ctx.now / ctx.sleep). C'est l'équivalent de use_sim_time
        # dans ROS (Robot Operating System, l'intergiciel robotique de
        # référence), qui fait suivre aux programmes l'horloge du
        # simulateur.
        self._clock = clock or time.monotonic
        # Jeton de session. Pourquoi : sans lui, une app (ou tout code
        # tiers) pourrait contourner install() en construisant son propre
        # RobotContext avec un manifeste arbitraire. Avec lui, seule cette
        # instance de PlayStoreRuntime peut construire un RobotContext
        # utilisable. Voir _RuntimeSessionToken et UnauthorizedContextError
        # dans axis_os/store/sdk/app_base.py, et
        # test/test_context_lockdown.py.
        self._session_token = _RuntimeSessionToken()
        # Verrou global de mission. Pourquoi : un audit a montré que rien
        # n'empêchait deux appels concurrents à run_mission() (deux apps
        # différentes, ou la même app deux fois) d'envoyer en même temps
        # des commandes contradictoires aux mêmes actionneurs physiques.
        # Règle : un seul robot, une seule mission active à la fois,
        # quelle que soit l'app. Ce n'est pas une file d'attente : un appel
        # concurrent est refusé immédiatement, et non mis en attente en
        # silence, pour rester prévisible et éviter qu'un appelant croie
        # sa mission en cours alors qu'elle attend indéfiniment derrière
        # une autre.
        self._mission_lock = threading.Lock()
        print(f"\n[PlayStore] Runtime démarré | id={self._id} | robot={robot.robot_id}")

    # ── Installation ──────────────────────────────────────────────

    def install(self, app: RobotApp) -> bool:
        """
        Installe une app sur ce robot. Renvoie True si elle est installée,
        False (raison affichée) dès qu'un contrôle échoue.

        Ordre des contrôles : validité du manifeste, compatibilité du SDK,
        signature du code, permissions réservées au système (SIGNATURE),
        capacités fonctionnelles, matériel, puis, en dernier, confirmation
        humaine des permissions DANGEROUS.
        """
        # On travaille sur une copie du manifeste, validée puis conservée :
        # RobotContext consultera celle-ci, jamais l'objet fourni par
        # l'app, que l'app pourrait modifier après la confirmation des
        # permissions (relevé à la relecture de l'audit de sécurité du
        # 2026-10-10).
        manifest = copy.deepcopy(app.manifest())
        app_id   = manifest.app_id

        print(f"\n[PlayStore] Installation '{manifest.name}' v{manifest.version}")
        print(f"  Auteur    : {manifest.author}")
        print(f"  Catégorie : {manifest.category.value}")

        errors = manifest.validate()
        if errors:
            for e in errors: print(f"  [ERREUR] {e}")
            return False

        # Compatibilité du SDK, dont la version est celle d'AXIS-OS (voir
        # axis_os/__init__.py ; versions au format SemVer, le
        # versionnement sémantique majeur.mineur.correctif). Vérifiée
        # avant le reste : une app écrite pour une version incompatible
        # peut attendre un autre format pour HardwareRequirements ou
        # Capabilities, contrôlés plus loin ; mieux vaut échouer ici avec
        # un message clair qu'avec une erreur confuse ensuite.
        if manifest.sdk_version_min is not None:
            from axis_os import __version__ as axis_os_version, SemVer
            required = SemVer.parse(manifest.sdk_version_min)
            current  = SemVer.parse(axis_os_version)
            if not required.is_compatible_with(current):
                print(
                    f"  [ERREUR] Incompatible avec AXIS-OS {current} — "
                    f"cette app requiert {required} (SDK mineur ou majeur "
                    f"différent). Mettez à jour l'app ou installez-la sur "
                    f"une version d'AXIS-OS compatible."
                )
                return False

        # Signature du code (voir axis_os/store/sdk/signing.py). Contrôlée
        # après le SDK (inutile d'aller plus loin si le SDK est déjà
        # incompatible) et avant le matériel (une signature invalide est
        # une raison de refus plus fondamentale qu'une capacité
        # manquante).
        #
        # Un audit de sécurité a montré que cette vérification était
        # facultative : elle n'avait lieu que si l'app avait choisi de
        # renseigner les deux champs. self._require_signed_apps permet de
        # la rendre obligatoire (contexte complet dans __init__).
        if self._require_signed_apps and manifest.code_signature is None:
            print(
                "  [ERREUR] Ce Play Store exige des apps signées "
                "(require_signed_apps=True) — code_signature et "
                "developer_public_key doivent être renseignés."
            )
            return False

        if manifest.code_signature is not None:
            from axis_os.store.sdk.signing import verify_app_signature, SignatureError
            import inspect
            try:
                # La signature porte sur le fichier source entier (le
                # module), pas seulement sur le texte de la classe. Un
                # audit de sécurité a jugé insuffisant
                # inspect.getsource(type(app)), qui ne couvre ni le reste
                # du module, ni ses imports, ni ses classes de base : du
                # code placé au niveau du module (une fonction, une
                # variable globale exécutée à l'import) pouvait être
                # modifié après signature sans l'invalider, puisqu'il ne
                # faisait pas partie du texte signé et vérifié.
                source_file = inspect.getsourcefile(type(app))
                if source_file is None:
                    raise OSError("aucun fichier source associé à cette classe")
                with open(source_file, "r", encoding="utf-8") as f:
                    source = f.read()
            except (OSError, TypeError) as exc:
                print(
                    f"  [ERREUR] Impossible de récupérer le code source de "
                    f"l'app pour vérifier sa signature : {exc}"
                )
                return False
            try:
                valid = verify_app_signature(
                    source, manifest.code_signature, manifest.developer_public_key
                )
            except SignatureError as exc:
                print(f"  [ERREUR] Signature ou clé malformée : {exc}")
                return False
            if not valid:
                print(
                    "  [ERREUR] Signature invalide — le code de l'app ne "
                    "correspond pas à sa signature déclarée (code modifié "
                    "après signature, ou mauvaise clé)."
                )
                return False

        # Permissions de niveau ProtectionLevel.SIGNATURE, réservées aux
        # apps système (par exemple ACTUATORS_RAW, les commandes
        # articulaires brutes, ou FLEET_BROADCAST, l'envoi d'un message à
        # toute une flotte de robots ; EMERGENCY_STOP, elle, est
        # délibérément DANGEROUS, voir manifest.py). Pourquoi (voir
        # docs/architecture_services.md, §6.3) : elles ne doivent être
        # accordées qu'à une app dont la signature vérifiée correspond
        # exactement à self._platform_public_key, jamais à n'importe
        # quelle app correctement signée par un développeur tiers. Si
        # aucune clé plateforme n'est configurée
        # (self._platform_public_key is None), toute permission SIGNATURE
        # est refusée : le défaut est le refus, jamais l'accord.
        is_platform_app = (
            self._platform_public_key is not None
            and manifest.code_signature is not None
            and manifest.developer_public_key == self._platform_public_key
        )
        for perm in manifest.permissions:
            if get_protection_level(perm) == ProtectionLevel.SIGNATURE and not is_platform_app:
                print(
                    f"  [ERREUR] Permission '{perm.name}' réservée aux apps "
                    f"système (ProtectionLevel.SIGNATURE) — cette app n'est "
                    f"pas signée par la clé plateforme."
                )
                return False

        # Capacités fonctionnelles (Capability : ce que le robot sait faire,
        # comme marcher ou parler), à ne pas confondre avec RobotCapability
        # (le matériel présent) : voir la docstring de Capability dans
        # manifest.py. Ce contrôle est bloquant, contrairement à
        # hw.optional plus bas : une app qui déclare avoir besoin d'une
        # Capability en a besoin, il n'existe pas ici de notion
        # « optionnelle » comme dans HardwareRequirements. Une app qui veut
        # fonctionner en mode dégradé sans une capacité ne la déclare
        # simplement pas dans capabilities_required.
        if manifest.capabilities_required:
            available = self._available_capabilities()
            missing_caps = [c for c in manifest.capabilities_required if c not in available]
            if missing_caps:
                names = sorted(c.name for c in missing_caps)
                print(f"  [ERREUR] Capacité(s) fonctionnelle(s) requise(s) "
                      f"absente(s) sur ce robot : {names}")
                return False

        # Matériel : nombre minimal de degrés de liberté (DoF, en pratique
        # le nombre d'articulations motorisées) et modèles de robot
        # compatibles.
        hw = manifest.hardware
        robot_info = self._robot.get_robot_info()
        robot_dof  = int(robot_info.get("dof", 0))
        if hw.min_dof > 0 and robot_dof < hw.min_dof:
            print(f"  [ERREUR] Requiert {hw.min_dof} DoF, robot en a {robot_dof}")
            return False
        if hw.compatible_models:
            model = robot_info.get("model", "")
            if model not in hw.compatible_models:
                print(f"  [ERREUR] Modèle '{model}' non supporté. Requis: {hw.compatible_models}")
                return False

        # Capacités matérielles : une capacité de hw.required qui manque
        # bloque l'installation ; une capacité de hw.optional qui manque
        # donne seulement un avertissement. Voir
        # HardwareRequirements.required et .optional dans manifest.py.
        caps = self._robot.get_capabilities()
        if hw.required:
            missing = caps.missing_from(hw.required)
            if missing:
                names = sorted(c.name for c in missing)
                print(f"  [ERREUR] Capacité(s) requise(s) manquante(s) : {names}")
                return False
        if hw.optional:
            missing_optional = caps.missing_from(hw.optional)
            if missing_optional:
                names = sorted(c.name for c in missing_optional)
                print(f"  [ATTENTION] Capacité(s) optionnelle(s) absente(s) — "
                      f"l'app fonctionnera en mode dégradé : {names}")

        # Permissions DANGEROUS : confirmation humaine explicite, demandée en
        # dernier pour ne pas faire accepter à un humain une app qui serait
        # refusée de toute façon (capacités, matériel). Voir l'en-tête de
        # section au-dessus de accepter_tout_SIMULATION.
        dangereuses = list(dict.fromkeys(
            p for p in manifest.permissions
            if get_protection_level(p) == ProtectionLevel.DANGEROUS))
        if dangereuses:
            noms = [p.name for p in dangereuses]
            if self._confirmer_permissions is None:
                print(f"  [ERREUR] Permission(s) DANGEROUS {noms} : aucune confirmation "
                      f"configurée sur ce runtime (confirmer_permissions) — refus par "
                      f"défaut.")
                return False
            try:
                accord = self._confirmer_permissions(manifest, list(dangereuses))
            except Exception as exc:          # noqa: BLE001 — refus, jamais accord
                print(f"  [ERREUR] Confirmation des permissions {noms} impossible "
                      f"({exc!r:.120}) — refus.")
                return False
            if accord is not True:
                # « is True » plutôt qu'un simple test de vérité : une
                # réponse floue (chaîne, 1, None) n'est pas un accord.
                print(f"  [ERREUR] Permission(s) DANGEROUS {noms} non accordée(s) — "
                      f"installation refusée.")
                return False
            print(f"  Permissions DANGEROUS accordées explicitement : {noms}")

        # Récapitulatif des permissions déclarées.
        print(f"  Permissions : {[p.name for p in manifest.permissions]}")

        # Prix payé par le client et partage : part du développeur,
        # commission de 30 % pour la plateforme.
        if manifest.pricing_model != PricingModel.FREE:
            print(f"  Prix client : {manifest.store_price_eur:.2f}€ "
                  f"| Dev : {manifest.developer_revenue_eur:.2f}€ "
                  f"| Commission : {manifest.platform_commission_eur:.2f}€ (30%)")

        # État de facturation repris depuis MemoryService s'il est fourni
        # (contexte complet : paramètre memory_service de __init__). Sans
        # lui, comportement strictement inchangé par rapport à l'époque
        # où ce paramètre n'existait pas : one_time_billed=False et
        # last_subscription_bill_time_s=None. Une app ONE_TIME ou
        # SUBSCRIPTION redémarrée sans memory_service est donc facturée
        # comme neuve : c'est le comportement documenté et assumé quand
        # la persistance n'est pas branchée.
        one_time_billed = False
        last_subscription_bill_time_s = None
        if self._memory_service is not None:
            one_time_billed = self._read_billing_flag(app_id, "one_time_billed", False)
            last_subscription_bill_time_s = self._read_billing_flag(
                app_id, "last_subscription_bill_time_s", None)

        self._apps[app_id] = InstalledApp(
            app=app, manifest=manifest,
            one_time_billed=one_time_billed,
            last_subscription_bill_time_s=last_subscription_bill_time_s,
        )
        print(f"  ✓ '{manifest.name}' installée.")
        return True

    # ── Cycle de vie ──────────────────────────────────────────────

    def start_app(self, app_id: str) -> bool:
        """Démarre une app installée en exécutant son hook on_start() dans
        la sandbox. Renvoie True si l'app s'est déclarée prête."""
        entry = self._get(app_id)
        if entry is None: return False

        # Exécuté dans la sandbox : la raison (correctif demandé par un
        # audit de sécurité externe) est dans la docstring de
        # _run_start_hook_in_sandbox(). Même pont
        # HALBridgeServer/HALBridgeClient que run_mission(), même
        # isolation par processus séparé, avec les quotas déclarés dans le
        # manifeste de l'app.
        command_queue: mp.Queue  = mp.Queue()
        response_queue: mp.Queue = mp.Queue()
        # Ce RobotContext reste dans le processus parent : il sert au
        # journal et au HALBridgeServer, qui exécute pour l'app les
        # commandes reçues par le pont. L'app ne le reçoit jamais.
        ctx = self._make_ctx(entry.manifest)
        ctx.log(f"Démarrage '{entry.manifest.name}'")
        bridge_server = HALBridgeServer(ctx, command_queue, response_queue)
        bridge_server.start()

        try:
            runner = SandboxRunner(app_id, entry.manifest.quota, clock=self._clock)
            sandbox_result = runner.run(
                _run_start_hook_in_sandbox,
                args=(entry.app, command_queue, response_queue),
            )
            if sandbox_result.success and sandbox_result.result is not None:
                ready, _log = sandbox_result.result
            else:
                kind = sandbox_result.error_kind
                reason = kind.name if kind else "INCONNU"
                print(f"  [SANDBOX] on_start arrêté par l'isolation : "
                      f"{sandbox_result.state.name} ({reason})")
                ready = False
        except Exception as exc:
            print(f"  [ERREUR] on_start: {exc}")
            ready = False
        finally:
            bridge_server.stop()

        entry.is_started = ready
        ctx.log("Prête." if ready else "Échec initialisation.")
        return ready

    def run_mission(
        self, app_id: str, params: Optional[Dict[str, Any]] = None
    ) -> Optional[MissionResult]:
        """Exécute une mission de l'app dans la sandbox et la facture si
        elle a réellement abouti. Renvoie None si l'app n'est pas installée
        ou pas démarrée, et un MissionResult d'échec si la mission est
        refusée (e-stop actif, autre mission déjà en cours)."""
        entry = self._get(app_id)
        if entry is None: return None
        if not entry.is_started:
            print(f"  [ERREUR] App '{app_id}' non démarrée.")
            return None

        # Barrière critique, ajoutée après un test de charge et
        # d'intégration : aucune mission ne doit pouvoir s'exécuter, même
        # « avec succès », une fois qu'un e-stop global a été déclenché.
        # Sans ce contrôle, une app qui ne consulte jamais elle-même le
        # Watchdog pouvait continuer indéfiniment après une condition de
        # sécurité FATAL (le niveau d'alerte le plus grave du Watchdog).
        #
        # On consulte aussi self._robot.estop_active. Un audit de sécurité
        # externe a montré qu'un e-stop déclenché par le Supervisor (le
        # thread qui arrête le robot quand le Watchdog lui-même cesse de
        # fonctionner, donc précisément quand le Watchdog est mort ou
        # bloqué) ne positionnait pas watchdog.estop_fired, et restait
        # donc invisible ici.
        if self._watchdog.estop_fired or self._robot.estop_active:
            mission_id = str(uuid.uuid4())[:8].upper()
            print(f"  [SECURITY] Mission {mission_id} refusée : "
                  f"e-stop actif sur le robot — {app_id}")
            return MissionResult(
                success=False, mission_id=mission_id, app_id=app_id,
                duration_s=0.0,
                error_message="Robot en état E-STOP — aucune mission autorisée.",
            )

        # Verrou global de mission (raison expliquée dans __init__).
        # acquire(blocking=False) : un appel concurrent est refusé tout de
        # suite avec un résultat explicite, jamais mis en attente en
        # silence derrière une mission déjà en cours.
        if not self._mission_lock.acquire(blocking=False):
            mission_id = str(uuid.uuid4())[:8].upper()
            print(f"  [SECURITY] Mission {mission_id} refusée : "
                  f"une autre mission est déjà en cours sur ce robot — {app_id}")
            return MissionResult(
                success=False, mission_id=mission_id, app_id=app_id,
                duration_s=0.0,
                error_message=(
                    "Une autre mission est déjà en cours d'exécution sur "
                    "ce robot — un seul robot ne peut exécuter qu'une "
                    "mission à la fois, quelle que soit l'app."
                ),
            )

        try:
            return self._run_mission_locked(entry, app_id, params)
        finally:
            self._mission_lock.release()

    def _run_mission_locked(
        self, entry: "InstalledApp", app_id: str, params: Optional[Dict[str, Any]]
    ) -> Optional[MissionResult]:
        """Corps de run_mission(), exécuté sous _mission_lock (contexte dans
        run_mission()). Il est extrait dans sa propre méthode pour que le
        verrou couvre tout le corps par un simple try/finally, sans
        dupliquer sa logique."""

        mission_id = str(uuid.uuid4())[:8].upper()
        params     = params or {}
        ctx        = self._make_ctx(entry.manifest)
        print(f"\n[PlayStore] ▶ Mission {mission_id} | app={app_id}")

        t_start = time.monotonic()

        # Isolation par processus séparé. Pourquoi : jusqu'à ce correctif,
        # entry.app.run_mission(ctx, params) était appelé directement dans
        # ce processus, sans aucune des protections déjà construites
        # (SandboxRunner, quotas de CPU, de mémoire et de durée) que le
        # reste du projet suppose actives pour toute app. Le pont
        # HALBridgeServer/HALBridgeClient, déjà écrit mais jamais branché,
        # permet à l'app isolée d'appeler move_arms/move_legs/log/get_imu
        # exactement comme avant, sans qu'aucune app existante ait besoin
        # d'être modifiée.
        command_queue: mp.Queue  = mp.Queue()
        response_queue: mp.Queue = mp.Queue()
        bridge_server = HALBridgeServer(ctx, command_queue, response_queue)
        bridge_server.start()

        try:
            runner = SandboxRunner(app_id, entry.manifest.quota, clock=self._clock)
            sandbox_result = runner.run(
                _run_app_in_sandbox,
                args=(entry.app, params, command_queue, response_queue),
            )

            if sandbox_result.success and sandbox_result.result is not None:
                result = sandbox_result.result
            elif sandbox_result.success and sandbox_result.result is None:
                # L'app a renvoyé None au lieu d'un MissionResult. Les
                # PermissionError et EstopActiveError n'arrivent pas
                # ici : _run_app_in_sandbox les convertit déjà en
                # MissionResult d'échec, plutôt que de les laisser
                # remonter jusqu'à SandboxRunner, qui les aurait classées
                # en APPLICATION_ERROR générique en perdant la distinction
                # entre refus de sécurité et bug de l'app.
                result = MissionResult(
                    success=False, mission_id=mission_id, app_id=app_id,
                    duration_s=time.monotonic() - t_start,
                    error_message="Mission sandboxée n'a retourné aucun résultat.",
                )
            else:
                # La sandbox a signalé un échec : délai dépassé, quota
                # dépassé, plantage du processus, fonction qui ne passe
                # pas par pickle, ou exception levée par l'app
                # (APPLICATION_ERROR). Le SandboxErrorKind indique
                # précisément la cause, pour le diagnostic après coup.
                kind = sandbox_result.error_kind
                reason = kind.name if kind else "INCONNU"
                print(f"  [SANDBOX] Mission arrêtée par l'isolation : "
                      f"{sandbox_result.state.name} ({reason})")
                result = MissionResult(
                    success=False, mission_id=mission_id, app_id=app_id,
                    duration_s=time.monotonic() - t_start,
                    error_message=(f"Isolation sandbox : {sandbox_result.state.name} "
                                   f"— {sandbox_result.error}"),
                )
        finally:
            bridge_server.stop()

        if result:
            result.mission_id = mission_id
            result.app_id     = app_id
            result.duration_s = time.monotonic() - t_start

        # Facturation. Pourquoi cette double condition : un audit de
        # sécurité externe a montré que la facturation reposait
        # uniquement sur result.success, une valeur que l'app déclare
        # elle-même depuis le processus enfant. Il faut désormais en plus
        # qu'au moins une commande motrice ait réellement abouti
        # (bridge_server.had_any_motor_success, tenu dans le processus
        # parent, hors de portée de l'app). Cela élimine le cas le plus
        # grave : une app qui ne bouge jamais le robot mais renvoie
        # toujours success=True pour déclencher la facturation du client.
        # Limite connue : le contrôle ne vérifie pas que le mouvement
        # correspond à la mission déclarée, donc il ne couvre pas tous les
        # mensonges possibles ; il supprime la fraude la plus simple et la
        # plus rentable à automatiser.
        if result and result.success and bridge_server.had_any_motor_success:
            entry.mission_count += 1
            billing = self._bill(entry, mission_id)
            if billing:
                entry.total_revenue += billing.developer_eur
                self._billing.append(billing)
                print(f"  {billing}")

        print(f"  {result}")
        return result

    def stop_app(self, app_id: str) -> None:
        """Arrête une app démarrée en exécutant son hook on_stop() dans la
        sandbox. Sans effet si l'app n'est pas installée ou pas démarrée.
        L'app est marquée arrêtée même si on_stop() échoue."""
        entry = self._get(app_id)
        if entry is None or not entry.is_started: return

        # Exécuté dans la sandbox, pour la même raison que start_app()
        # ci-dessus : voir _run_stop_hook_in_sandbox().
        command_queue: mp.Queue  = mp.Queue()
        response_queue: mp.Queue = mp.Queue()
        ctx = self._make_ctx(entry.manifest)
        bridge_server = HALBridgeServer(ctx, command_queue, response_queue)
        bridge_server.start()

        try:
            runner = SandboxRunner(app_id, entry.manifest.quota, clock=self._clock)
            sandbox_result = runner.run(
                _run_stop_hook_in_sandbox,
                args=(entry.app, command_queue, response_queue),
            )
            if not (sandbox_result.success and sandbox_result.result is not None):
                kind = sandbox_result.error_kind
                reason = kind.name if kind else "INCONNU"
                print(f"  [SANDBOX] on_stop arrêté par l'isolation : "
                      f"{sandbox_result.state.name} ({reason})")
        except Exception as exc:
            print(f"  [ERREUR] on_stop: {exc}")
        finally:
            bridge_server.stop()

        entry.is_started = False

    def update_app(
        self, new_app: RobotApp, *, allow_downgrade: bool = False,
    ) -> bool:
        """
        Met à jour une app déjà installée : même app_id, nouvelle instance
        ou nouvelle version.

        Pourquoi cette méthode : avant elle, rien n'empêchait install()
        d'écraser en silence une app déjà installée sous le même app_id.
        Une version en cours d'exécution n'était pas arrêtée proprement,
        le sens de la mise à jour n'était pas vérifié (une version plus
        ancienne pouvait remplacer une plus récente sans avertissement),
        et l'état de facturation (one_time_billed,
        last_subscription_bill_time_s) repartait de zéro comme pour une
        installation neuve : une app ONE_TIME déjà payée aurait été
        refacturée à sa prochaine mission après une simple mise à jour.

        Le comportement s'inspire du cycle réel d'une mise à jour sous
        Android (son gestionnaire de paquets, PackageManager) :
          1. Tout retour à une version antérieure (downgrade : nouvelle
             version < version installée) est refusé par défaut, comme
             le fait Android avec l'erreur INSTALL_FAILED_VERSION_DOWNGRADE.
             C'est une protection de sécurité connue : sans elle, un
             attaquant pourrait faire « mettre à jour » une app vers une
             ancienne version contenant une faille corrigée depuis.
             `allow_downgrade=True` lève ce refus explicitement (comme
             l'option `-d` de `adb install`, l'outil Android
             d'installation en ligne de commande) ; ce n'est jamais le
             comportement par défaut.
          2. La mise à jour est refusée si une mission est en cours sur
             ce robot (self._mission_lock déjà pris), quelle que soit
             l'app concernée. C'est le principe déjà appliqué à
             run_mission() : un seul robot, une seule activité à la fois.
             Remplacer le code d'une app pendant qu'une mission, même
             d'une autre app, contrôle physiquement le robot serait
             dangereux.
          3. L'ancienne version est arrêtée proprement si elle est
             démarrée, par stop_app() tel quel : mêmes garanties
             d'isolation que l'arrêt normal, jamais un raccourci qui
             sauterait on_stop().
          4. La nouvelle version est installée par install(), qui
             applique toutes les vérifications existantes (signature,
             permissions, dont la confirmation des permissions DANGEROUS,
             capacités, compatibilité du SDK, ProtectionLevel.SIGNATURE)
             exactement comme pour une première installation. Une mise à
             jour n'est jamais un chemin plus permissif qu'une
             installation neuve.
          5. L'état de facturation de l'ancienne entrée est recopié sur
             la nouvelle (one_time_billed, last_subscription_bill_time_s) :
             une mise à jour n'est pas une réinstallation au sens
             commercial, l'app reste « la même » pour un utilisateur qui
             a déjà payé.

        L'app n'est pas redémarrée automatiquement après la mise à jour,
        même si l'ancienne version l'était. C'est délibéré et conforme au
        comportement réel d'Android (ACTION_MY_PACKAGE_REPLACED prévient
        l'app remplacée, mais c'est à elle, ou ici à l'appelant, de
        décider explicitement de redémarrer) : appeler start_app(app_id)
        après update_app() reste la responsabilité de l'appelant.

        Si la nouvelle version échoue aux vérifications d'install(),
        l'ancienne reste installée, mais arrêtée si elle tournait.
        Essais : test/test_app_update.py.
        """
        new_manifest = new_app.manifest()
        app_id = new_manifest.app_id

        existing_entry = self._get(app_id)
        if existing_entry is None:
            print(f"  [ERREUR] update_app() : aucune app installée avec "
                  f"l'app_id '{app_id}' — utilisez install() pour une "
                  f"première installation.")
            return False

        try:
            new_version = SemVer.parse(new_manifest.version)
            old_version = SemVer.parse(existing_entry.manifest.version)
        except ValueError as e:
            print(f"  [ERREUR] update_app() : version invalide — {e}")
            return False

        if new_version < old_version and not allow_downgrade:
            print(
                f"  [ERREUR] update_app() : la nouvelle version "
                f"({new_version}) est ANTÉRIEURE à la version installée "
                f"({old_version}) — downgrade refusé par défaut (risque "
                f"de sécurité connu : revenir à une version contenant une "
                f"faille déjà corrigée). Passez allow_downgrade=True pour "
                f"forcer ce remplacement explicitement."
            )
            return False

        if not self._mission_lock.acquire(blocking=False):
            print(
                f"  [ERREUR] update_app() : une mission est actuellement "
                f"en cours sur ce robot — impossible de mettre à jour "
                f"'{app_id}' tant qu'elle n'est pas terminée."
            )
            return False
        self._mission_lock.release()

        was_started = existing_entry.is_started
        if was_started:
            print(f"  [PlayStore] Arrêt de l'ancienne version de "
                  f"'{app_id}' (v{old_version}) avant mise à jour...")
            self.stop_app(app_id)

        # On garde l'état de facturation avant qu'install() ne crée une
        # nouvelle InstalledApp aux valeurs par défaut (raison : étape 5
        # de la docstring de update_app()).
        preserved_one_time_billed = existing_entry.one_time_billed
        preserved_last_subscription_bill_time_s = existing_entry.last_subscription_bill_time_s

        print(f"  [PlayStore] Mise à jour de '{app_id}' : "
              f"v{old_version} → v{new_version}")
        installed = self.install(new_app)
        if not installed:
            print(f"  [ERREUR] update_app() : la nouvelle version de "
                  f"'{app_id}' a échoué aux vérifications d'install() — "
                  f"l'ANCIENNE version reste installée mais ARRÊTÉE "
                  f"(voir ci-dessus). Réinstallez-la explicitement ou "
                  f"corrigez la nouvelle version avant de réessayer.")
            return False

        new_entry = self._get(app_id)
        if new_entry is not None:
            new_entry.one_time_billed = preserved_one_time_billed
            new_entry.last_subscription_bill_time_s = preserved_last_subscription_bill_time_s

        return True

    # ── Rapport financier ─────────────────────────────────────────

    def billing_summary(self) -> Dict[str, Any]:
        """Résumé financier de ce runtime : totaux (montant brut,
        commission, part développeur) sur toutes les facturations émises,
        puis détail par app installée. « total_missions » compte les
        facturations émises, pas toutes les missions exécutées."""
        total_gross = sum(b.gross_amount_eur for b in self._billing)
        total_comm  = sum(b.commission_eur   for b in self._billing)
        total_dev   = sum(b.developer_eur    for b in self._billing)
        return {
            "runtime_id":      self._id,
            "total_missions":  len(self._billing),
            "total_gross_eur": round(total_gross, 2),
            "commission_eur":  round(total_comm, 2),
            "developer_eur":   round(total_dev, 2),
            "commission_rate": "30%",
            "apps_installed":  len(self._apps),
            "per_app": {
                aid: {"missions": e.mission_count,
                      "revenue":  round(e.total_revenue, 2),
                      "commission": round(e.commission_due, 2)}
                for aid, e in self._apps.items()
            }
        }

    # ── Helpers ───────────────────────────────────────────────────

    def _available_capabilities(self) -> set:
        """
        Déduit les capacités fonctionnelles (Capability) disponibles sur ce
        robot à partir des capacités matérielles (RobotCapability) déjà
        exposées par le HAL (get_capabilities()). Ce n'est pas une
        nouvelle source de vérité indépendante : deux déclarations du même
        robot ne peuvent ainsi jamais diverger.

        Correspondance volontairement prudente pour la perception :
        RobotCapability.CAMERA (générique, ne distingue pas une caméra
        couleur RGB d'une caméra de profondeur) ne donne que
        Capability.CAMERA_RGB, jamais CAMERA_DEPTH. Annoncer la profondeur
        sans information plus précise serait mentir à l'installation.

        WALK et GRASP_SINGLE sont déduites de LEGS et ARMS. Correction
        faite après un essai réel sur les trois apps de référence, qui
        bougent déjà bras et jambes directement par move_arms() et
        move_legs(), donc par un mouvement physique réel via le HAL, pas
        hypothétique. Une version précédente de cette correspondance les
        excluait au motif qu'« aucun service système ne les fournit
        encore » : elle confondait la capacité de mouvement de base, déjà
        réelle et exercée par les apps existantes, avec un futur service
        intelligent de haut niveau (planification de la marche, préhension
        guidée par la vision), qui, lui, n'existe effectivement pas encore.
        GRASP_DUAL demande les deux bras coordonnés : elle n'est pas
        déduite d'un simple ARMS générique, et aucune app de référence
        actuelle ne l'utilise ni n'en a besoin.
        """
        robot_caps = self._robot.get_capabilities()
        available = set()
        if robot_caps.has(RobotCapability.CAMERA):
            available.add(Capability.CAMERA_RGB)
        if robot_caps.has(RobotCapability.MICROPHONE):
            available.add(Capability.SPEECH_IN)
        if robot_caps.has(RobotCapability.SPEAKER):
            available.add(Capability.SPEECH_OUT)
        if robot_caps.has(RobotCapability.LEGS):
            available.add(Capability.WALK)
        if robot_caps.has(RobotCapability.ARMS):
            available.add(Capability.GRASP_SINGLE)

        # Capacités logicielles : elles ne viennent pas du matériel mais
        # des services effectivement présents sur ce robot. Une mémoire
        # persistante n'est pas un capteur : elle existe si le noyau
        # fournit le service, et pas autrement.
        #
        # Oubli constaté : la table ne traduisait que des capacités
        # matérielles, donc MEMORY_PERSISTENT n'était jamais disponible et
        # toute app qui l'exigeait était refusée à l'installation, sans
        # que rien n'indique pourquoi la mission suivante échouait.
        if self._memory_service is not None:
            available.add(Capability.MEMORY_PERSISTENT)
        return available

    def _make_ctx(self, manifest: AppManifest) -> RobotContext:
        """Construit le RobotContext d'une app (l'objet par lequel elle
        commande le robot), avec le jeton de session de ce runtime,
        l'horloge du robot et les services système."""
        ctx = RobotContext(
            self._robot, self._watchdog, manifest,
            _session_token=self._session_token,
            _runtime_token_holder=self._session_token,
        )
        # Les services sont transmis à toutes les apps : c'est la
        # permission déclarée dans le manifeste qui décide du droit de
        # s'en servir, pas la présence ou l'absence du service. Un service
        # absent (robot sans caméra, sans jambes) donne une réponse vide
        # et explicite, jamais une erreur obscure.
        ctx._set_clock(self._clock)
        ctx._attach_services(perception=self._perception_service,
                             locomotion=self._locomotion_service,
                             speech=self._speech_service,
                             memory=self._memory_service)
        return ctx

    def _get(self, app_id: str) -> Optional[InstalledApp]:
        entry = self._apps.get(app_id)
        if entry is None:
            print(f"  [ERREUR] App '{app_id}' non installée.")
        return entry

    SUBSCRIPTION_PERIOD_S = 30 * 24 * 3600  # 30 jours en secondes ; voir _bill()

    def _read_billing_flag(self, app_id: str, key: str, default):
        """
        Lit un indicateur de facturation dans MemoryService. N'est appelée
        que si self._memory_service n'est pas None (voir install()).

        Renvoie `default` quand la lecture échoue (clé absente, service
        non démarré ou qui signale une erreur) plutôt que de lever une
        exception : un problème de lecture de l'état persisté ne doit
        jamais empêcher une app de s'installer. Au pire, l'app est traitée
        comme neuve pour cet app_id précis, ce qui n'est jamais pire que
        le comportement sans memory_service.
        """
        result = self._memory_service.app_load(f"billing::{app_id}", key)
        return result.value if result.ok else default

    def _write_billing_flag(self, app_id: str, key: str, value) -> None:
        """Écrit un indicateur de facturation dans MemoryService ; ne fait
        rien si self._memory_service est None. Un échec d'écriture n'est
        jamais propagé : la facturation en mémoire (InstalledApp, déjà mise
        à jour par l'appelant avant cet appel) reste correcte pour la
        session en cours même si la persistance échoue. Seule la survie au
        prochain redémarrage serait perdue, pas la session actuelle."""
        if self._memory_service is None:
            return
        self._memory_service.app_store(f"billing::{app_id}", key, value)

    def _bill(self, entry: InstalledApp, mission_id: str) -> Optional[BillingReport]:
        """
        Facture une mission selon le modèle de prix (pricing_model) de
        l'app. Renvoie None quand il n'y a rien à facturer.

        Pourquoi : un audit de sécurité externe a montré qu'avant ce
        correctif tous les modèles étaient facturés comme PAY_PER_MISSION
        (le prix complet à chaque mission), une erreur financière directe
        au détriment du client. Règles appliquées désormais :
          - FREE (gratuit) : jamais facturé (inchangé).
          - PAY_PER_MISSION (paiement à la mission) : facturé à chaque
            mission (comportement historique, correct pour ce modèle).
          - ONE_TIME (achat unique) : facturé une seule fois, à la
            première mission réussie après l'installation, et plus jamais
            ensuite, quel que soit le nombre de missions suivantes.
          - SUBSCRIPTION (abonnement) : facturé une fois par période de
            30 jours glissants depuis la dernière facturation, jamais à
            chaque mission. Une mission qui tombe dans la même période
            qu'une facturation déjà émise ne produit aucun BillingReport
            supplémentaire.

        one_time_billed et last_subscription_bill_time_s sont désormais
        enregistrés dans MemoryService quand il est fourni (voir
        _write_billing_flag). Cela comble le trou relevé par deux audits
        externes successifs : sans ce branchement, cet état ne survivait
        jamais à un redémarrage du processus (historique complet dans le
        commentaire des champs de facturation d'InstalledApp).
        """
        m = entry.manifest
        if m.pricing_model == PricingModel.FREE:
            return None

        if m.pricing_model == PricingModel.ONE_TIME:
            if entry.one_time_billed:
                return None
            entry.one_time_billed = True
            self._write_billing_flag(m.app_id, "one_time_billed", True)

        elif m.pricing_model == PricingModel.SUBSCRIPTION:
            # time.time() (horloge murale), pas time.monotonic() : voir le
            # commentaire des champs de facturation d'InstalledApp
            # (correction demandée par le second audit externe).
            now = time.time()
            last = entry.last_subscription_bill_time_s
            if last is not None and (now - last) < self.SUBSCRIPTION_PERIOD_S:
                return None
            entry.last_subscription_bill_time_s = now
            self._write_billing_flag(m.app_id, "last_subscription_bill_time_s", now)

        return BillingReport(
            mission_id=mission_id, app_id=m.app_id,
            pricing_model=m.pricing_model,
            gross_amount_eur=m.store_price_eur,
            commission_eur=m.platform_commission_eur,
            developer_eur=m.developer_revenue_eur,
        )
