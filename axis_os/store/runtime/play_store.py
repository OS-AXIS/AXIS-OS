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

from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.store.sdk.manifest import AppManifest, Permission, PricingModel
from axis_os.store.sdk.app_base import (
    RobotApp, RobotContext, MissionResult, PermissionError, EstopActiveError
)
from axis_os.store.runtime.sandbox import SandboxRunner, SandboxState, SandboxErrorKind
from axis_os.store.runtime.hal_bridge import (
    HALBridgeServer, HALBridgeClient, HALCommand, HALResponse
)
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
        """
        self._robot   = robot
        self._watchdog = watchdog
        self._apps:    Dict[str, InstalledApp] = {}
        self._billing: List[BillingReport]     = []
        self._id = str(uuid.uuid4())[:8]
        self._require_signed_apps = require_signed_apps
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

        self._apps[app_id] = InstalledApp(app=app, manifest=manifest)
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
            runner = SandboxRunner(app_id, entry.manifest.quota)
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
        if self._watchdog.estop_fired:
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
            runner = SandboxRunner(app_id, entry.manifest.quota)
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

        # Facturation
        if result and result.success:
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
            runner = SandboxRunner(app_id, entry.manifest.quota)
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

    def _make_ctx(self, manifest: AppManifest) -> RobotContext:
        return RobotContext(self._robot, self._watchdog, manifest)

    def _get(self, app_id: str) -> Optional[InstalledApp]:
        entry = self._apps.get(app_id)
        if entry is None:
            print(f"  [ERREUR] App '{app_id}' non installée.")
        return entry

    def _bill(self, entry: InstalledApp, mission_id: str) -> Optional[BillingReport]:
        m = entry.manifest
        if m.pricing_model == PricingModel.FREE: return None
        return BillingReport(
            mission_id=mission_id, app_id=m.app_id,
            pricing_model=m.pricing_model,
            gross_amount_eur=m.store_price_eur,
            commission_eur=m.platform_commission_eur,
            developer_eur=m.developer_revenue_eur,
        )
