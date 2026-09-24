"""
AXIS-OS — Robotics Play Store Runtime
=======================================
Moteur d'exécution du Play Store côté robot.
Gère : installation, cycle de vie, sandbox, facturation 30%.

Licence : Propriétaire — AXIS-OS
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List
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


def _run_app_in_sandbox(app: RobotApp, params: Dict[str, Any],
                        command_queue: mp.Queue, response_queue: mp.Queue) -> MissionResult:
    """
    Exécutée DANS le process enfant sandboxé (via SandboxRunner). Reçoit
    l'app elle-même (picklée par multiprocessing) et un HALBridgeClient
    construit sur les deux Queues déjà ouvertes — jamais le vrai HAL,
    jamais le vrai WatchdogSystem, qui contiennent tous deux des threads
    non picklables.

    L'app appelle ctx.move_arms/move_legs/log/get_imu/get_mission_log
    exactement comme avec un RobotContext normal — HALBridgeClient
    a la même surface d'API, donc aucune app existante n'a besoin
    d'être modifiée pour bénéficier de l'isolation.
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
    Équivalent de _run_app_in_sandbox() pour on_start() — trouvé
    nécessaire par un audit de sécurité (C4) : start_app()/stop_app()
    appelaient jusqu'ici entry.app.on_start()/on_stop() DIRECTEMENT
    dans le process parent, avec un vrai RobotContext donnant un accès
    HAL complet, sans quota, sans timeout, sans isolation mémoire —
    exactement le trou que SandboxRunner/HALBridgeServer existent pour
    combler côté run_mission(), mais jamais câblé ici. Concret et pas
    théorique : axis_os/apps/package_sorting/app.py appelle
    ctx.move_arms(...) directement dans son on_start() — le robot
    bougeait physiquement pendant ce hook non sandboxé.

    Retourne (bool, list[str]) plutôt qu'un simple bool : le journal
    de mission du HALBridgeClient est perdu si on ne le renvoie pas
    explicitement à travers la frontière de process.
    """
    bridge_ctx = HALBridgeClient(command_queue, response_queue)
    try:
        ready = app.on_start(bridge_ctx)
        return ready, bridge_ctx.get_mission_log()
    except (PermissionError, EstopActiveError) as exc:
        return False, bridge_ctx.get_mission_log() + [f"on_start refusé : {exc}"]


def _run_stop_hook_in_sandbox(app: RobotApp, command_queue: mp.Queue,
                              response_queue: mp.Queue):
    """Équivalent de _run_start_hook_in_sandbox() pour on_stop() —
    voir sa docstring pour le contexte complet (C4)."""
    bridge_ctx = HALBridgeClient(command_queue, response_queue)
    try:
        app.on_stop(bridge_ctx)
        return True, bridge_ctx.get_mission_log()
    except (PermissionError, EstopActiveError) as exc:
        return False, bridge_ctx.get_mission_log() + [f"on_stop refusé : {exc}"]


@dataclass
class InstalledApp:
    app:           RobotApp
    manifest:      AppManifest
    install_time:  float = field(default_factory=time.monotonic)
    install_id:    str   = field(default_factory=lambda: str(uuid.uuid4())[:8])
    mission_count: int   = 0
    total_revenue: float = 0.0
    is_started:    bool  = False
    # Trouvé par un audit de sécurité (M2) : _bill() facturait le prix
    # complet à CHAQUE mission, quel que soit pricing_model — une app
    # ONE_TIME facturait son prix à chaque mission (pas une seule fois),
    # une app SUBSCRIPTION facturait l'abonnement mensuel entier à
    # chaque mission (pas une fois par période). Ces deux champs
    # trackent l'état de facturation par app installée, nécessaire pour
    # que _bill() applique enfin la bonne règle par modèle — voir sa
    # docstring pour le détail par pricing_model.
    #
    # ⚠️ État tenu EN MÉMOIRE uniquement, perdu à tout redémarrage du
    # process — trouvé par un second audit externe : un redémarrage
    # refacture une app ONE_TIME comme neuve, et relance la fenêtre
    # SUBSCRIPTION de 30 jours à zéro. Une vraie persistance disque
    # (ou base de données) reste un chantier distinct, hors de portée
    # de ce correctif — documenté honnêtement ici plutôt que laissé
    # implicite. À traiter avant tout déploiement où un redémarrage
    # de PlayStoreRuntime est un événement réaliste en production.
    #
    # last_subscription_bill_time_s utilise time.time() (horloge
    # murale, wall-clock) — corrigé après un audit externe qui a
    # relevé que time.monotonic() ne représente PAS une date
    # calendaire : monotonic() n'a de sens que comme différence entre
    # deux appels dans le MÊME process, jamais comme "il y a 30 jours"
    # au sens propre, et son epoch de référence est arbitraire et
    # change à chaque redémarrage — la aggravant, pas seulement
    # symbolique, dans le cas de la persistance déjà absente ci-dessus.
    one_time_billed:              bool            = False
    last_subscription_bill_time_s: Optional[float] = None

    @property
    def commission_due(self) -> float:
        return round(self.total_revenue * AppManifest.COMMISSION_RATE, 4)


@dataclass
class BillingReport:
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
    Moteur d'exécution Play Store sur un robot.

    Usage :
        runtime = PlayStoreRuntime(norm_robot, watchdog)
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
    ):
        """
        `require_signed_apps` : si True, install() refuse toute app
        dont code_signature/developer_public_key ne sont pas renseignés
        — False par défaut, rétrocompatible avec toutes les apps
        existantes non signées. Trou identifié par un audit de
        sécurité : la signature était jusqu'ici purement opt-in (les
        deux champs valent None par défaut, une app malveillante omet
        simplement le champ et s'installe sans aucune vérification).
        Le CHANGELOG présentait la signature comme comblant "rien
        n'empêchait un fichier .py quelconque de se faire passer pour
        une app légitime" — ce n'était vrai qu'une fois ce flag mis à
        True. À activer explicitement avant toute ouverture d'un vrai
        Play Store à des développeurs externes non vérifiés.

        `platform_public_key` : clé publique Ed25519 (même format que
        AppManifest.developer_public_key) qui distingue une app
        SYSTÈME d'une app tierce — voir ProtectionLevel.SIGNATURE
        (axis_os/store/sdk/manifest.py). Une app dont
        developer_public_key correspond exactement à cette clé, ET
        dont la signature vérifie, est traitée comme signée par la
        plateforme : autorisée à déclarer des permissions
        ProtectionLevel.SIGNATURE (ex: ACTUATORS_RAW). None par
        défaut (rétrocompatible) : sans clé plateforme configurée,
        AUCUNE app — même correctement signée par un vrai
        développeur — n'est traitée comme système, et toute
        permission SIGNATURE est refusée à l'installation (fail-safe,
        pas fail-open).

        `memory_service` : si fourni (démarré — voir MemoryService.
        start(), axis_os/services/memory/service.py), l'état de
        facturation (one_time_billed, last_subscription_bill_time_s)
        est lu depuis lui à l'installation et réécrit dedans à chaque
        mise à jour — comble le trou documenté depuis deux audits
        externes : cet état était tenu EN MÉMOIRE uniquement, perdu à
        tout redémarrage (une app ONE_TIME refacturée comme neuve, une
        fenêtre SUBSCRIPTION de 30 jours relancée à zéro). None par
        défaut (rétrocompatible) : sans service fourni, comportement
        strictement inchangé — état en mémoire seule, comme avant ce
        paramètre.
        """
        self._robot   = robot
        self._watchdog = watchdog
        self._apps:    Dict[str, InstalledApp] = {}
        self._billing: List[BillingReport]     = []
        self._id = str(uuid.uuid4())[:8]
        self._require_signed_apps = require_signed_apps
        if not require_signed_apps:
            # Avertissement d'exécution, pas seulement documenté —
            # trouvé nécessaire pour rendre visible à chaque démarrage
            # un choix qui, silencieux, se retrouve facilement oublié
            # en configuration de production : sans require_signed_apps
            # =True, n'importe quel fichier .py qui hérite de RobotApp
            # s'installe sans aucune vérification d'identité. Un
            # simple appel install() ne fait alors QUE vérifier les
            # permissions/capacités déclarées, jamais qui a écrit le
            # code — voir la docstring de ce paramètre pour le
            # contexte complet.
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
        # Horloge du robot : simulée en simulation, réelle sur le
        # robot. Transmise au sandbox (quotas) et aux apps
        # (ctx.now / ctx.sleep) — l'équivalent de use_sim_time
        # dans ROS.
        self._clock = clock or time.monotonic
        # Jeton de session — voir _RuntimeSessionToken (axis_os/store/
        # sdk/app_base.py) et UnauthorizedContextError pour le contexte
        # complet : seule cette instance de PlayStoreRuntime peut
        # construire un RobotContext utilisable, empêchant qu'une app
        # (ou tout code tiers) contourne install() en construisant son
        # propre RobotContext avec un AppManifest arbitraire.
        self._session_token = _RuntimeSessionToken()
        # Verrou global de mission — trou identifié lors d'un audit :
        # rien n'empêchait deux appels concurrents à run_mission()
        # (deux apps différentes, ou la même app deux fois) d'envoyer
        # des commandes contradictoires aux MÊMES actionneurs physiques
        # en même temps. Un seul robot = une seule mission active à la
        # fois, quelle que soit l'app — pas une file d'attente : un
        # appel concurrent est REFUSÉ immédiatement (pas mis en attente
        # silencieusement), pour rester prévisible et éviter qu'un
        # appelant croie sa mission en cours d'exécution alors qu'elle
        # patiente indéfiniment derrière une autre.
        self._mission_lock = threading.Lock()
        print(f"\n[PlayStore] Runtime démarré | id={self._id} | robot={robot.robot_id}")

    # ── Installation ──────────────────────────────────────────────

    def install(self, app: RobotApp) -> bool:
        manifest = app.manifest()
        app_id   = manifest.app_id

        print(f"\n[PlayStore] Installation '{manifest.name}' v{manifest.version}")
        print(f"  Auteur    : {manifest.author}")
        print(f"  Catégorie : {manifest.category.value}")

        errors = manifest.validate()
        if errors:
            for e in errors: print(f"  [ERREUR] {e}")
            return False

        # Vérification compatibilité SDK (voir axis_os/__init__.py) —
        # avant toute autre vérification : une app codée pour une
        # version incompatible d'AXIS-OS peut avoir des attentes sur
        # le FORMAT même de HardwareRequirements/Capabilities qui
        # suivent, donc autant échouer ici avec un message clair
        # plutôt qu'une erreur confuse plus loin.
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

        # Vérification signature du code (voir axis_os/store/sdk/signing.py)
        # — après le check de version SDK (une app dont le SDK est déjà
        # incompatible n'a pas besoin d'aller plus loin), avant le check
        # hardware (une signature invalide est une raison de refus plus
        # fondamentale que des capacités manquantes).
        #
        # Trouvé par un audit de sécurité : cette vérification était
        # purement opt-in (uniquement si l'app a choisi de renseigner
        # les deux champs) — self._require_signed_apps permet de la
        # rendre obligatoire (voir __init__ pour le contexte complet).
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
                # Signe le FICHIER SOURCE ENTIER (module), pas
                # seulement le texte de la classe — trouvé insuffisant
                # par un audit de sécurité : inspect.getsource(type(app))
                # ne couvre ni le module, ni ses imports, ni ses classes
                # de base. Du code utile placé au niveau module (une
                # fonction, une variable globale exécutée à l'import)
                # pouvait être modifié après signature sans invalider
                # cette dernière, puisqu'il ne faisait jamais partie du
                # texte réellement signé/vérifié.
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

        # Vérification ProtectionLevel.SIGNATURE — trouvé nécessaire
        # (§6.3 de la conception services) : une permission SIGNATURE
        # (ex: ACTUATORS_RAW, EMERGENCY_STOP, FLEET_BROADCAST) ne doit
        # être accordée qu'à une app dont la signature vérifiée
        # correspond EXACTEMENT à self._platform_public_key — jamais à
        # n'importe quelle app correctement signée par un développeur
        # tiers quelconque. Fail-safe : si aucune clé plateforme n'est
        # configurée (self._platform_public_key is None), TOUTE
        # permission SIGNATURE est refusée, jamais accordée par défaut.
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

        # Vérification des capacités fonctionnelles (Capability, pas
        # RobotCapability — voir docstring de Capability dans
        # manifest.py pour la distinction). Bloquant, contrairement à
        # hw.optional ci-dessous : une app qui déclare avoir besoin
        # d'une Capability en a besoin, il n'existe pas ici de notion
        # "optionnelle" comme pour HardwareRequirements — si une app
        # veut un usage dégradé sans une capacité, elle ne la déclare
        # simplement pas dans capabilities_required.
        if manifest.capabilities_required:
            available = self._available_capabilities()
            missing_caps = [c for c in manifest.capabilities_required if c not in available]
            if missing_caps:
                names = sorted(c.name for c in missing_caps)
                print(f"  [ERREUR] Capacité(s) fonctionnelle(s) requise(s) "
                      f"absente(s) sur ce robot : {names}")
                return False

        # Vérification hardware
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

        # Vérification capacités (required = bloquant, optional = avertissement)
        # Voir HardwareRequirements.required/.optional dans manifest.py.
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

        # Permissions
        print(f"  Permissions : {[p.name for p in manifest.permissions]}")

        # Prix
        if manifest.pricing_model != PricingModel.FREE:
            print(f"  Prix client : {manifest.store_price_eur:.2f}€ "
                  f"| Dev : {manifest.developer_revenue_eur:.2f}€ "
                  f"| Commission : {manifest.platform_commission_eur:.2f}€ (30%)")

        # État de facturation repris depuis MemoryService s'il est
        # fourni — voir le paramètre memory_service dans __init__()
        # pour le contexte complet. Sans lui (None), comportement
        # strictement inchangé : one_time_billed=False,
        # last_subscription_bill_time_s=None, comme avant ce
        # paramètre — une app ONE_TIME/SUBSCRIPTION redémarrée sans
        # memory_service configuré reste facturée comme neuve,
        # exactement le comportement documenté et assumé quand la
        # persistance n'est pas branchée.
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
        entry = self._get(app_id)
        if entry is None: return False

        # Sandboxé — voir _run_start_hook_in_sandbox() pour le contexte
        # complet (correctif C4, audit de sécurité). Même pont
        # HALBridgeServer/Client que run_mission(), même isolation par
        # process séparé avec les quotas du manifest de l'app.
        command_queue: mp.Queue  = mp.Queue()
        response_queue: mp.Queue = mp.Queue()
        ctx = self._make_ctx(entry.manifest)  # utilisé uniquement pour le log parent, pas passé à l'app
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
        entry = self._get(app_id)
        if entry is None: return None
        if not entry.is_started:
            print(f"  [ERREUR] App '{app_id}' non démarrée.")
            return None

        # Barrière critique trouvée par un test de charge/intégration
        # (aucune mission ne doit pouvoir s'exécuter, même "avec succès",
        # une fois qu'un e-stop global a été déclenché — sans ce check,
        # une app qui ne consulte jamais elle-même le Watchdog pouvait
        # continuer indéfiniment après une condition de sécurité FATAL).
        #
        # Consulte aussi self._robot.estop_active — correctif E10, audit
        # de sécurité : un e-stop déclenché par le Supervisor (le cas
        # précis où le Watchdog est mort ou bloqué) ne positionnait pas
        # watchdog.estop_fired, donc restait invisible ici.
        if self._watchdog.estop_fired or self._robot.estop_active:
            mission_id = str(uuid.uuid4())[:8].upper()
            print(f"  [SECURITY] Mission {mission_id} refusée : "
                  f"e-stop actif sur le robot — {app_id}")
            return MissionResult(
                success=False, mission_id=mission_id, app_id=app_id,
                duration_s=0.0,
                error_message="Robot en état E-STOP — aucune mission autorisée.",
            )

        # Verrou global de mission — voir __init__ pour la justification.
        # acquire(blocking=False) : un appel concurrent est refusé tout
        # de suite avec un résultat explicite, jamais mis en attente
        # silencieuse derrière une mission déjà en cours.
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
        """Corps réel de run_mission(), exécuté sous _mission_lock — voir
        run_mission() pour le contexte. Extrait dans sa propre méthode
        pour que le verrou couvre tout le corps existant via un simple
        try/finally, sans dupliquer sa logique."""

        mission_id = str(uuid.uuid4())[:8].upper()
        params     = params or {}
        ctx        = self._make_ctx(entry.manifest)
        print(f"\n[PlayStore] ▶ Mission {mission_id} | app={app_id}")

        t_start = time.monotonic()

        # Isolation réelle par processus séparé — trou identifié
        # explicitement : jusqu'ici, entry.app.run_mission(ctx, params)
        # était appelé directement dans CE process, sans aucune des
        # protections déjà construites (SandboxRunner, quotas CPU/RAM/
        # temps) que le reste du projet suppose actives pour toute app.
        # Le pont HALBridgeServer/Client (déjà écrit, jamais branché)
        # permet à l'app sandboxée d'appeler move_arms/move_legs/log/
        # get_imu exactement comme avant, sans qu'aucune app existante
        # n'ait besoin d'être modifiée.
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
                # L'app a levé PermissionError/EstopActiveError dans
                # _run_app_in_sandbox — celle-ci retourne alors un
                # MissionResult d'échec directement plutôt que de
                # laisser l'exception remonter jusqu'au SandboxRunner
                # (qui l'aurait classée en APPLICATION_ERROR générique,
                # perdant la distinction sécurité vs bug applicatif).
                result = MissionResult(
                    success=False, mission_id=mission_id, app_id=app_id,
                    duration_s=time.monotonic() - t_start,
                    error_message="Mission sandboxée n'a retourné aucun résultat.",
                )
            else:
                # Le sandbox lui-même a détecté un problème : timeout,
                # dépassement de quota, crash du process, fonction non
                # picklable. Le SandboxErrorKind distingue clairement
                # la CAUSE pour le diagnostic après coup.
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

        # Facturation — correctif M3, audit de sécurité : exige
        # désormais qu'au moins une commande motrice ait RÉELLEMENT
        # abouti (bridge_server.had_any_motor_success, tenu côté
        # process PARENT, hors de portée de l'app) EN PLUS de
        # result.success (auto-déclaré par l'app dans le process
        # enfant). Élimine le cas le plus grave : une app qui ne bouge
        # jamais le robot mais retourne toujours success=True pour
        # forcer la facturation client. Ne couvre pas tous les cas de
        # mensonge possibles (une app pourrait bouger un joint sans
        # rapport avec sa mission déclarée) mais élimine la fraude la
        # plus simple et la plus rentable à automatiser.
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
        entry = self._get(app_id)
        if entry is None or not entry.is_started: return

        # Sandboxé — voir _run_stop_hook_in_sandbox() (correctif C4,
        # même contexte que start_app() ci-dessus).
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
        Met à jour une app déjà installée — même app_id, nouvelle
        instance/version. Trouvé nécessaire : avant cette méthode,
        rien n'empêchait install() d'écraser SILENCIEUSEMENT une app
        déjà installée (même app_id) — sans arrêter proprement une
        version en cours d'exécution, sans vérifier le sens de la
        mise à jour (une version PLUS ANCIENNE pouvait remplacer une
        plus récente sans avertissement), et l'état de facturation
        (one_time_billed, last_subscription_bill_time_s) se serait
        remis à zéro comme pour une toute nouvelle installation —
        une app ONE_TIME déjà payée aurait été refacturée à sa
        prochaine mission après une simple mise à jour.

        Comportement, inspiré du cycle de vie réel d'une mise à jour
        Android (PackageManager) :
          1. Refuse tout downgrade par défaut (nouvelle version <
             version installée) — exactement INSTALL_FAILED_
             VERSION_DOWNGRADE d'Android, une protection de sécurité
             connue : sans elle, un attaquant pourrait faire
             "mettre à jour" une app vers une ancienne version
             contenant une faille corrigée depuis. `allow_downgrade=
             True` lève ce refus explicitement (équivalent du flag
             `-d` d'`adb install`), jamais le comportement par défaut.
          2. Refuse la mise à jour si une mission est actuellement en
             cours sur CE ROBOT (self._mission_lock déjà pris) — quelle
             que soit l'app concernée, cohérent avec le principe déjà
             appliqué à run_mission() : un seul robot, une seule
             activité à la fois. Remplacer le code d'une app PENDANT
             qu'une mission (même d'une AUTRE app) contrôle
             physiquement le robot serait dangereux.
          3. Arrête proprement l'ancienne version si elle est démarrée
             (réutilise stop_app() tel quel — mêmes garanties de
             sandboxing que l'arrêt normal, jamais un raccourci qui
             sauterait on_stop()).
          4. Réinstalle via install() — réutilise TOUTES les
             vérifications déjà en place (signature, permissions,
             capacités, compatibilité SDK, ProtectionLevel.SIGNATURE)
             pour la NOUVELLE version, exactement comme une première
             installation. Une mise à jour n'est jamais un chemin
             plus permissif qu'une installation neuve.
          5. Restaure l'état de facturation de l'ANCIENNE entrée sur
             la NOUVELLE (one_time_billed, last_subscription_bill_
             time_s) — une mise à jour n'est PAS une réinstallation
             au sens commercial, l'app reste "la même" pour un
             utilisateur qui a déjà payé.

        Ne redémarre PAS automatiquement l'app après la mise à jour,
        même si l'ancienne version était démarrée — décision
        délibérée, cohérente avec le comportement Android réel
        (ACTION_MY_PACKAGE_REPLACED notifie l'app remplacée, mais
        c'est à elle, ou à l'appelant ici, de décider explicitement
        de redémarrer) : appeler start_app(app_id) reste la
        responsabilité de l'appelant après update_app().
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

        # Préserve l'état de facturation AVANT que install() ne
        # construise une nouvelle InstalledApp par défaut (voir sa
        # docstring de section 5 ci-dessus pour la justification).
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
        Dérive les Capability fonctionnelles disponibles à partir des
        RobotCapability matérielles déjà exposées par le HAL
        (get_capabilities()) — pas une nouvelle source de vérité
        indépendante, pour ne jamais désynchroniser deux déclarations
        du même robot.

        Mapping délibérément CONSERVATEUR pour la perception :
        RobotCapability.CAMERA (générique, ne distingue pas RGB de
        profondeur) ne donne QUE Capability.CAMERA_RGB, jamais
        CAMERA_DEPTH — affirmer la profondeur disponible sans
        information plus précise serait un mensonge à l'installation.

        WALK/GRASP_SINGLE dérivées de LEGS/ARMS — corrigé après un
        test réel sur les 3 apps de référence (elles bougent déjà
        bras/jambes directement via move_arms()/move_legs(), un
        mouvement PHYSIQUE réel via le HAL, pas hypothétique) : une
        version précédente de ce mapping les excluait au nom d'un
        « aucun service système ne les fournit encore », confondant
        la capacité de MOUVEMENT DE BASE (déjà réelle, exercée par
        les apps existantes) et un futur SERVICE intelligent de haut
        niveau (planification de marche, préhension guidée par
        vision — celui-là effectivement inexistant). GRASP_DUAL
        nécessite les deux bras coordonnés — non dérivée d'un simple
        ARMS générique, aucune app de référence actuelle ne
        l'exerce ni n'en a besoin.
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

        # CAPACITÉS LOGICIELLES : elles ne viennent pas du matériel mais
        # des SERVICES effectivement présents sur ce robot. Une mémoire
        # persistante n'est pas un capteur — elle existe si le noyau
        # fournit le service, et pas autrement.
        #
        # Oubli constaté : la table ne traduisait que des capacités
        # matérielles, donc MEMORY_PERSISTENT n'était JAMAIS disponible
        # et toute app l'exigeant était refusée à l'installation — sans
        # que rien n'indique pourquoi la mission suivante échouait.
        if self._memory_service is not None:
            available.add(Capability.MEMORY_PERSISTENT)
        return available

    def _make_ctx(self, manifest: AppManifest) -> RobotContext:
        ctx = RobotContext(
            self._robot, self._watchdog, manifest,
            _session_token=self._session_token,
            _runtime_token_holder=self._session_token,
        )
        # Les services sont injectés à TOUTE app : c'est la PERMISSION
        # déclarée dans son manifeste qui décide de son droit d'usage,
        # pas la présence ou l'absence du service. Un service absent
        # (robot sans caméra, sans jambes) donne une réponse vide et
        # explicite, jamais une erreur obscure.
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

    SUBSCRIPTION_PERIOD_S = 30 * 24 * 3600  # 30 jours — voir _bill()

    def _read_billing_flag(self, app_id: str, key: str, default):
        """
        Lit un flag de facturation depuis MemoryService — jamais
        appelée si self._memory_service is None (voir install()).
        Retourne `default` sur tout échec de lecture (clé absente,
        service en erreur) plutôt que de lever — un problème de
        lecture de l'état persisté ne doit jamais empêcher une app de
        s'installer, au pire elle est traitée comme neuve pour cette
        app_id précise, jamais pire que le comportement sans
        memory_service configuré du tout.
        """
        result = self._memory_service.app_load(f"billing::{app_id}", key)
        return result.value if result.ok else default

    def _write_billing_flag(self, app_id: str, key: str, value) -> None:
        """Écrit un flag de facturation dans MemoryService — no-op si
        self._memory_service is None. Erreur d'écriture jamais
        propagée : la facturation en mémoire (InstalledApp, déjà mise
        à jour par l'appelant AVANT ce helper) reste correcte pour la
        session en cours même si la persistance échoue — seule la
        survie au PROCHAIN redémarrage serait perdue, pas la session
        actuelle."""
        if self._memory_service is None:
            return
        self._memory_service.app_store(f"billing::{app_id}", key, value)

    def _bill(self, entry: InstalledApp, mission_id: str) -> Optional[BillingReport]:
        """
        Facture selon pricing_model — correctif M2, audit de sécurité :
        avant ce correctif, TOUS les modèles étaient facturés comme
        PAY_PER_MISSION (le prix complet à chaque mission), un bug
        financier direct côté client :
          - FREE : jamais facturé (inchangé).
          - PAY_PER_MISSION : facturé à chaque mission (comportement
            historique, correct pour ce modèle précis).
          - ONE_TIME : facturé une SEULE fois, à la première mission
            réussie après installation — jamais à nouveau ensuite,
            quel que soit le nombre de missions suivantes.
          - SUBSCRIPTION : facturé une fois par période de 30 jours
            glissants depuis la dernière facturation, jamais à chaque
            mission individuelle. Une mission qui tombe dans la même
            période qu'une facturation déjà émise ne génère aucun
            BillingReport supplémentaire.

        Persiste désormais one_time_billed/last_subscription_bill_time_s
        via MemoryService quand il est fourni (voir _write_billing_flag)
        — comble le trou documenté depuis deux audits externes : sans
        ce branchement, l'état ne survivait jamais à un redémarrage du
        process (voir InstalledApp pour l'historique complet du bug).
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
            # time.time() (horloge murale), pas time.monotonic() — voir
            # la docstring de InstalledApp.last_subscription_bill_time_s
            # pour le contexte complet (correctif suite à un second
            # audit externe).
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
