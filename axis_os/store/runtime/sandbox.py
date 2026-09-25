"""
AXIS-OS — Sandbox Runner v2
=============================
Isolation par processus (multiprocessing) pour chaque app Play Store.

Historique : v1 avait un bug de synchronisation entre le thread moniteur
et le thread principal (accès concurrent à une liste sans verrou), pas
de cycle de vie explicite, pas de taxonomie d'erreur, et un crash
silencieux si la fonction cible n'était pas picklable. Corrigé en v2
suite à une revue externe.

Pourquoi multiprocessing et pas threading :
  threading      → mémoire partagée → un crash app peut tuer l'OS
  multiprocessing → processus séparé → un crash app n'affecte QUE l'app

Portée actuelle (documentée honnêtement) :
  - Isole des fonctions dont les arguments/retour sont picklables
    (calculs, logique métier, appels à un modèle IA tiers).
  - N'isole PAS encore l'accès direct au HAL (RobotContext contient
    des threads, non picklable). Voir hal_bridge.py pour le pattern
    Command/Response Queue déjà prêt — l'intégration automatique dans
    SandboxRunner viendra avec le Fleet Manager.

Licence : Propriétaire — AXIS-OS
"""

from __future__ import annotations
import multiprocessing as mp
# Sous-module IMPORTÉ EXPLICITEMENT : run() teste isinstance(a,
# mp.queues.Queue), et `multiprocessing.queues` n'est pas chargé par
# `import multiprocessing`. En production cela fonctionnait PAR HASARD —
# le pont du sandbox crée des files avant et charge le sous-module au
# passage. Dans tout autre contexte, CHAQUE exécution sandboxée
# échouait en « Fonction ou arguments non picklables : module
# 'multiprocessing' has no attribute 'queues' ». Trouvé par le test de
# l'horloge du robot, qui appelait le sandbox directement.
import multiprocessing.queues  # noqa: F401
import pickle
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Any, Callable

from axis_os.store.sdk.manifest import ResourceQuota


class SandboxState(Enum):
    CREATED  = auto()
    STARTING = auto()
    RUNNING  = auto()
    STOPPING = auto()
    STOPPED  = auto()
    CRASHED  = auto()
    KILLED   = auto()
    TIMEOUT  = auto()
    VIOLATED = auto()


class SandboxErrorKind(Enum):
    APPLICATION_ERROR = auto()
    CPU_QUOTA          = auto()
    MEMORY_QUOTA        = auto()
    TIMEOUT             = auto()
    SANDBOX_ERROR        = auto()
    PROCESS_CRASH        = auto()


@dataclass
class SandboxViolation:
    app_id: str
    rule:   str
    value:  float
    limit:  float

    def __str__(self) -> str:
        return (f"[SANDBOX VIOLATION] app={self.app_id} | "
                f"rule={self.rule} | value={self.value:.1f} | limit={self.limit:.1f}")


class SandboxResult:
    def __init__(self, success: bool, state: SandboxState,
                 result: Optional[Any] = None,
                 violation: Optional[SandboxViolation] = None,
                 error: Optional[str] = None,
                 error_kind: Optional[SandboxErrorKind] = None,
                 duration_s: float = 0.0):
        self.success    = success
        self.state       = state
        self.result      = result
        self.violation   = violation
        self.error       = error
        self.error_kind  = error_kind
        self.duration_s  = duration_s

    def __str__(self) -> str:
        status = "✓ SUCCESS" if self.success else f"✗ {self.state.name}"
        err = f" | {self.error_kind.name}: {self.error}" if self.error else ""
        return f"[Sandbox] {status} | durée={self.duration_s:.2f}s{err}"


def _try_import_psutil():
    try:
        import psutil
        return psutil
    except ImportError:
        return None


class SandboxRunner:
    """
    Exécute une fonction dans un processus isolé avec quotas et cycle de vie.

    Usage :
        runner = SandboxRunner(app_id="com.acme.app", quota=manifest.quota)
        result = runner.run(target_fn, args=(ctx, params))
        print(result.state, result.success)
    """

    def __init__(self, app_id: str, quota: Optional[ResourceQuota] = None,
                strict: bool = False,
                clock: Optional[Callable[[], float]] = None):
        """
        `strict` : si True, refuse de démarrer (lève RuntimeError) si
        `psutil` n'est pas disponible — False par défaut, rétrocompatible
        avec tout appelant existant. Trouvé nécessaire par un audit de
        sécurité (E9) : sans psutil, `_monitor_loop()` retourne
        immédiatement et silencieusement (voir son code) — les quotas
        CPU/RAM ne sont alors plus du tout appliqués, mais `run()`
        continue de rapporter des exécutions "réussies" sans qu'aucune
        de ces protections ne soit en vigueur. Le timeout dur (via
        `proc.join(timeout=...)`) reste couvert même sans psutil — seuls
        les quotas CPU/RAM en cours d'exécution dépendent de lui.
        À activer explicitement dès qu'une app non vérifiée (développeur
        externe) est exécutée.
        """
        self._app_id = app_id
        # HORLOGE DU ROBOT, pas du mur. En simulation, l'horloge simulée ;
        # sur le vrai robot, l'horloge réelle — donc aucun changement de
        # comportement. Constaté : une simulation ralentie par le rendu vidéo
        # faisait tuer une mission par son quota de 300 s MURALES alors
        # qu'elle n'avait vécu qu'une trentaine de secondes simulées.
        self._clock = clock or time.monotonic
        self._quota  = quota or ResourceQuota()
        self._psutil = _try_import_psutil()
        if self._psutil is None:
            print(
                f"  [SECURITY] psutil indisponible — les quotas CPU/RAM "
                f"de '{app_id}' ne seront PAS appliqués (seul le timeout "
                f"dur reste en vigueur). Installez psutil pour une "
                f"isolation complète."
            )
            if strict:
                raise RuntimeError(
                    f"SandboxRunner(strict=True) refuse de démarrer sans "
                    f"psutil — app '{app_id}'."
                )
        self._strict = strict
        self._state  = SandboxState.CREATED
        self._lock       = threading.Lock()
        self._violations: list[SandboxViolation] = []

    @property
    def state(self) -> SandboxState:
        with self._lock:
            return self._state

    def _set_state(self, state: SandboxState) -> None:
        with self._lock:
            self._state = state

    def _add_violation(self, v: SandboxViolation) -> None:
        with self._lock:
            self._violations.append(v)

    def _get_violation(self) -> Optional[SandboxViolation]:
        with self._lock:
            return self._violations[0] if self._violations else None

    def run(self, target_fn: Callable, args: tuple = (),
            kwargs: Optional[dict] = None) -> SandboxResult:
        kwargs = kwargs or {}
        t_start = self._clock()
        self._set_state(SandboxState.STARTING)

        try:
            pickle.dumps(target_fn)
            # mp.Queue est délibérément EXCLUE de ce check : elle n'est
            # picklable que dans le contexte précis du passage à
            # Process(args=...) au moment du vrai fork, jamais via un
            # pickle.dumps() isolé comme celui-ci (RuntimeError
            # "Queue objects should only be shared through inheritance").
            # Bug réel trouvé en branchant HALBridgeServer/Client dans
            # PlayStoreRuntime — les Queues du bridge faisaient échouer
            # ce check préventif alors qu'elles fonctionnent normalement
            # une fois transmises au vrai Process ci-dessous.
            picklable_args = tuple(a for a in args if not isinstance(a, mp.queues.Queue))
            pickle.dumps(picklable_args)
            pickle.dumps(kwargs)
        except (pickle.PicklingError, TypeError, AttributeError) as exc:
            self._set_state(SandboxState.CRASHED)
            return SandboxResult(
                success=False, state=SandboxState.CRASHED,
                error=f"Fonction ou arguments non picklables : {exc}",
                error_kind=SandboxErrorKind.SANDBOX_ERROR,
                duration_s=self._clock() - t_start,
            )

        # Contexte "fork" forcé explicitement — indépendant de la méthode
        # de démarrage par défaut du système. Trouvé en testant sur un
        # vrai serveur Ubuntu/Python 3.14 : le défaut y est "forkserver",
        # qui exige que la fonction cible soit importable depuis un vrai
        # module. Une fonction définie dans un script exécuté directement
        # (ou en `python3 -c`) échoue alors silencieusement avec
        # "AttributeError: module '__main__' has no attribute ...".
        # "fork" duplique le process tel quel, la fonction est déjà en
        # mémoire — fonctionne dans tous les cas sur Linux (pas Windows,
        # non concerné par AXIS-OS).
        ctx = mp.get_context("fork")
        result_queue: mp.Queue = ctx.Queue()
        proc = ctx.Process(
            target=_sandbox_entrypoint,
            args=(target_fn, args, kwargs, result_queue),
            daemon=True,
        )

        try:
            proc.start()
        except Exception as exc:
            self._set_state(SandboxState.CRASHED)
            return SandboxResult(
                success=False, state=SandboxState.CRASHED,
                error=f"Échec démarrage process : {exc}",
                error_kind=SandboxErrorKind.SANDBOX_ERROR,
                duration_s=self._clock() - t_start,
            )

        self._set_state(SandboxState.RUNNING)

        monitor = threading.Thread(
            target=self._monitor_loop, args=(proc, t_start), daemon=True
        )
        monitor.start()

        # Attente bornée en temps du ROBOT : join() ne connaît que le temps
        # mural, on attend donc par petits pas en consultant l'horloge.
        while (proc.is_alive()
               and self._clock() - t_start < self._quota.max_runtime_s + 1.0):
            proc.join(timeout=0.05)
        self._set_state(SandboxState.STOPPING)

        try:
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=2.0)
                if proc.is_alive():
                    proc.kill()
                    proc.join(timeout=1.0)

                violation = self._get_violation()
                if violation is None:
                    violation = SandboxViolation(
                        app_id=self._app_id, rule="TIMEOUT",
                        value=self._clock() - t_start,
                        limit=self._quota.max_runtime_s
                    )
                self._set_state(SandboxState.TIMEOUT)
                return SandboxResult(
                    success=False, state=SandboxState.TIMEOUT,
                    violation=violation, error="Timeout — processus tué.",
                    error_kind=SandboxErrorKind.TIMEOUT,
                    duration_s=self._clock() - t_start,
                )

            violation = self._get_violation()
            if violation is not None:
                kind = (SandboxErrorKind.CPU_QUOTA if violation.rule == "CPU_QUOTA"
                       else SandboxErrorKind.MEMORY_QUOTA if violation.rule == "MEMORY_QUOTA"
                       else SandboxErrorKind.TIMEOUT)
                self._set_state(SandboxState.VIOLATED)
                return SandboxResult(
                    success=False, state=SandboxState.VIOLATED,
                    violation=violation, error=f"Quota dépassé : {violation.rule}",
                    error_kind=kind, duration_s=self._clock() - t_start,
                )

            if result_queue.empty():
                self._set_state(SandboxState.CRASHED)
                return SandboxResult(
                    success=False, state=SandboxState.CRASHED,
                    error="Processus terminé sans résultat (crash probable).",
                    error_kind=SandboxErrorKind.PROCESS_CRASH,
                    duration_s=self._clock() - t_start,
                )

            payload = result_queue.get()
            if payload.get("error"):
                self._set_state(SandboxState.CRASHED)
                return SandboxResult(
                    success=False, state=SandboxState.CRASHED,
                    error=payload["error"],
                    error_kind=SandboxErrorKind.APPLICATION_ERROR,
                    duration_s=self._clock() - t_start,
                )

            self._set_state(SandboxState.STOPPED)
            return SandboxResult(
                success=True, state=SandboxState.STOPPED,
                result=payload.get("result"),
                duration_s=self._clock() - t_start,
            )

        finally:
            self._cleanup(proc, result_queue)

    def _cleanup(self, proc: mp.Process, result_queue: mp.Queue) -> None:
        try:
            if proc.is_alive():
                proc.kill()
            proc.join(timeout=1.0)
        except Exception:
            pass
        try:
            result_queue.close()
            result_queue.cancel_join_thread()
        except Exception:
            pass

    def _monitor_loop(self, proc: mp.Process, t_start: float) -> None:
        """
        Surveille CPU/RAM du process enfant. Tue si dépassement persistant.

        Correction v2 : exige 3 échantillons CPU consécutifs au-dessus du
        seuil avant de déclencher — évite les faux positifs d'un seul pic.
        """
        if self._psutil is None:
            return

        ps_proc = None
        consecutive_cpu_over = 0
        CONFIRM_THRESHOLD = 3

        while proc.is_alive():
            elapsed = self._clock() - t_start
            if elapsed > self._quota.max_runtime_s:
                self._add_violation(SandboxViolation(
                    app_id=self._app_id, rule="TIMEOUT",
                    value=elapsed, limit=self._quota.max_runtime_s
                ))
                proc.terminate()
                return
            try:
                if ps_proc is None:
                    ps_proc = self._psutil.Process(proc.pid)
                cpu    = ps_proc.cpu_percent(interval=0.1)
                mem_mb = ps_proc.memory_info().rss / (1024 * 1024)

                if mem_mb > self._quota.max_memory_mb:
                    self._add_violation(SandboxViolation(
                        app_id=self._app_id, rule="MEMORY_QUOTA",
                        value=mem_mb, limit=self._quota.max_memory_mb
                    ))
                    proc.terminate()
                    return

                if cpu > self._quota.max_cpu_percent * 2:
                    consecutive_cpu_over += 1
                    if consecutive_cpu_over >= CONFIRM_THRESHOLD:
                        self._add_violation(SandboxViolation(
                            app_id=self._app_id, rule="CPU_QUOTA",
                            value=cpu, limit=self._quota.max_cpu_percent
                        ))
                        proc.terminate()
                        return
                else:
                    consecutive_cpu_over = 0
            except Exception:
                pass
            time.sleep(0.05)


def _reensemencer_alea() -> None:
    """Donne au processus fils son PROPRE aléa.

    Le fils est créé par DUPLICATION du parent : il hérite de l'état de
    tous les générateurs aléatoires tel qu'il était à ce moment-là, et ce
    qu'il consomme n'est jamais renvoyé au parent. Sans réensemencement,
    CHAQUE mission tire exactement les mêmes nombres.

    Constaté : une app qui simulait 30 % d'échecs de saisie échouait
    soit 20 missions sur 20, soit 0 sur 20 — selon un seul tirage fait à
    sa création. Le test qui la couvrait passait ou échouait au hasard.

    Ne couvre que les générateurs GLOBAUX (random, numpy). Un générateur
    PROPRE à l'app (random.Random() créé dans __init__) reste figé : c'est
    à l'app de le recréer par mission — voir WarehouseOpsApp.
    """
    import os
    import random
    random.seed(int.from_bytes(os.urandom(8), "little"))
    try:
        import sys
        if "numpy" in sys.modules:
            import numpy as np
            np.random.seed(int.from_bytes(os.urandom(4), "little"))
    except Exception:                   # noqa: BLE001
        pass


def _sandbox_entrypoint(target_fn, args, kwargs, result_queue: mp.Queue) -> None:
    try:
        _reensemencer_alea()
        result = target_fn(*args, **kwargs)
        result_queue.put({"result": result, "error": None})
    except Exception as exc:
        result_queue.put({"result": None, "error": f"{type(exc).__name__}: {exc}"})
