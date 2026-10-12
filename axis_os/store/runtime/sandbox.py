"""
AXIS-OS — Exécuteur de sandbox (SandboxRunner), version 2
==========================================================
Chaque app du Play Store (la boutique d'apps du robot) tourne dans un
sandbox (un « bac à sable ») : un processus séparé, surveillé, avec des
quotas de CPU, de mémoire et de durée. Ce module crée ce processus, le
surveille et rapporte comment il s'est terminé.

Pourquoi un processus séparé (multiprocessing) plutôt qu'un thread :
  - un thread partage la mémoire de l'OS : un plantage de l'app peut
    emporter l'OS avec elle ;
  - un processus séparé a sa propre mémoire : un plantage de l'app
    n'affecte que l'app.

Historique : la version 1 avait quatre défauts, corrigés dans la
version 2 à la suite d'une revue extérieure du code :
  - le thread de surveillance et le thread principal accédaient à la
    même liste sans verrou ;
  - pas de cycle de vie explicite (voir SandboxState) ;
  - pas de classement des erreurs (voir SandboxErrorKind) ;
  - un plantage silencieux quand la fonction à exécuter ne pouvait pas
    passer par pickle (le format de Python qui transforme un objet en
    octets et inversement). Le sandbox exige que la fonction et ses
    arguments passent par pickle, et le résultat revient au processus
    de l'OS sous cette forme.

Portée actuelle :
  - Isole des fonctions dont les arguments et le résultat passent par
    pickle : calculs, logique métier, appels à un modèle d'IA tiers.
  - N'isole pas encore, à lui seul, l'accès direct à la HAL (la couche
    d'abstraction matérielle, qui donne accès aux moteurs et aux
    capteurs) : RobotContext, l'objet par lequel une app commande le
    robot, contient des threads et ne passe pas par pickle. Le pont IPC
    (la communication entre deux processus) de hal_bridge.py comble ce
    manque par un schéma commande/réponse : l'app envoie ses commandes
    par une file, le processus de l'OS les exécute et répond par une
    autre. PlayStoreRuntime (play_store.py) branche déjà ce pont autour
    de chaque appel à une app ; son intégration automatique dans
    SandboxRunner lui-même est prévue avec le Fleet Manager (le futur
    gestionnaire de flotte, voir docs/fleet_manager_design.md).

Limite connue : cette isolation protège contre une app buggée
(plantage, boucle infinie, dépassement de quota), pas contre une app
délibérément hostile. Le processus de l'app démarre comme une copie de
celui de l'OS (fork), et le canal entre les deux transporte des objets
pickle, pas de simples données. Le constat et le plan de correction sont
dans docs/isolation_apps.md et dans SECURITY.md (« Limites connues »).

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations
import multiprocessing as mp
# Ce sous-module est importé explicitement parce que run() teste
# isinstance(a, mp.queues.Queue) et que `import multiprocessing` ne le
# charge pas. En production, cela fonctionnait par hasard : les files du
# pont HAL sont créées avant l'appel au sandbox, ce qui charge le
# sous-module au passage. Dans tout autre contexte, chaque exécution
# sandboxée échouait avec « Fonction ou arguments non picklables :
# module 'multiprocessing' has no attribute 'queues' ». Le défaut a été
# trouvé par le test de l'horloge du robot (test/test_robot_clock.py),
# qui appelait le sandbox directement.
import multiprocessing.queues  # noqa: F401
import pickle
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Any, Callable

from axis_os.store.sdk.manifest import ResourceQuota


class SandboxState(Enum):
    """Étape du cycle de vie d'une exécution sandboxée.

    CREATED à la construction, puis STARTING, RUNNING et STOPPING
    pendant run(), puis un état final : STOPPED (fin normale), CRASHED
    (erreur de l'app, plantage ou échec du lancement), TIMEOUT
    (processus encore vivant après l'attente bornée de run(), donc tué)
    ou VIOLATED (quota de CPU, de mémoire ou de durée constaté par
    _monitor_loop()). Un dépassement de durée peut donc finir en TIMEOUT
    ou en VIOLATED, selon qui le voit en premier ; error_kind vaut
    TIMEOUT dans les deux cas. KILLED est défini mais n'est attribué
    nulle part aujourd'hui.
    """
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
    """Cause d'un échec, pour le diagnostic après coup.

    APPLICATION_ERROR : l'app a levé une exception. CPU_QUOTA,
    MEMORY_QUOTA, TIMEOUT : un quota a été dépassé. SANDBOX_ERROR : le
    sandbox n'a pas pu lancer l'app (fonction ou arguments qui ne passent
    pas par pickle, échec de création du processus). PROCESS_CRASH : le
    processus s'est terminé sans renvoyer de résultat.
    """
    APPLICATION_ERROR = auto()
    CPU_QUOTA          = auto()
    MEMORY_QUOTA        = auto()
    TIMEOUT             = auto()
    SANDBOX_ERROR        = auto()
    PROCESS_CRASH        = auto()


@dataclass
class SandboxViolation:
    """Quota dépassé : la règle enfreinte (« TIMEOUT », « CPU_QUOTA »,
    « MEMORY_QUOTA »), la valeur mesurée et la limite."""
    app_id: str
    rule:   str
    value:  float
    limit:  float

    def __str__(self) -> str:
        return (f"[SANDBOX VIOLATION] app={self.app_id} | "
                f"rule={self.rule} | value={self.value:.1f} | limit={self.limit:.1f}")


class SandboxResult:
    """Bilan d'une exécution : succès, état final, résultat renvoyé par
    la fonction, quota enfreint, message et cause de l'erreur, durée."""
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
    """Renvoie le module psutil, ou None s'il n'est pas installé (voir
    `strict` dans SandboxRunner.__init__ pour les conséquences)."""
    try:
        import psutil
        return psutil
    except ImportError:
        return None


class SandboxRunner:
    """
    Exécute une fonction dans un processus séparé, avec des quotas et un
    cycle de vie suivi (voir SandboxState).

    Usage :
        runner = SandboxRunner(app_id="com.acme.app", quota=manifest.quota)
        result = runner.run(target_fn, args=(ctx, params))
        print(result.state, result.success)
    """

    def __init__(self, app_id: str, quota: Optional[ResourceQuota] = None,
                strict: bool = False,
                clock: Optional[Callable[[], float]] = None):
        """
        `quota` : limites de CPU, de mémoire et de durée (ResourceQuota),
        déclarées dans le manifeste de l'app (la fiche que toute app
        fournit pour dire ce qu'elle a le droit de faire et ce qu'elle
        consomme) ; les valeurs par défaut de ResourceQuota si None.

        `strict` : si True, refuse de démarrer (lève RuntimeError) quand
        `psutil` (la bibliothèque qui mesure le CPU et la mémoire d'un
        processus) n'est pas installé. Un audit de sécurité externe
        (2026-09-14, voir CHANGELOG.md et docs/audit_externe_2026-09-14.md)
        a montré pourquoi : sans psutil, `_monitor_loop()` s'arrête
        aussitôt, sans rien signaler (voir son code). Les quotas de CPU
        et de mémoire ne sont alors plus appliqués du tout, mais `run()`
        continue de rapporter des exécutions « réussies » sans qu'aucune
        de ces protections soit en vigueur. Le délai maximal (timeout
        dur, appliqué par l'attente bornée de `run()`) reste en vigueur
        même sans psutil ; seuls les quotas de CPU et de mémoire en cours
        d'exécution dépendent de lui. False par défaut, pour rester
        compatible avec les appelants existants. À activer explicitement
        dès qu'on exécute une app non vérifiée (écrite par un développeur
        externe).

        `clock` : l'horloge du robot (voir le commentaire ci-dessous) ;
        time.monotonic si None.
        """
        self._app_id = app_id
        # Horloge du robot, pas l'horloge murale (le temps réellement
        # écoulé, celui d'une montre). Constaté : une simulation ralentie
        # par le rendu vidéo faisait tuer une mission par son quota de
        # 300 s de temps mural, alors qu'elle n'avait vécu qu'une
        # trentaine de secondes simulées. On prend donc l'horloge simulée
        # en simulation et l'horloge réelle sur le vrai robot, où le
        # comportement ne change pas.
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
        """Exécute target_fn(*args, **kwargs) dans un processus séparé et
        attend sa fin, dans la limite de la durée maximale du quota.

        Les échecs prévus (lancement impossible, exception de l'app, quota
        dépassé, plantage) sont rapportés dans le SandboxResult renvoyé,
        pas levés en exception."""
        kwargs = kwargs or {}
        t_start = self._clock()
        self._set_state(SandboxState.STARTING)

        # Contrôle préventif : vérifier dès maintenant que la fonction et
        # ses arguments passent par pickle, pour renvoyer une erreur
        # claire (SANDBOX_ERROR) au lieu du plantage silencieux de la
        # version 1.
        try:
            pickle.dumps(target_fn)
            # Les files mp.Queue passées en arguments positionnels sont
            # volontairement exclues de ce contrôle (celles passées dans
            # kwargs ne le sont pas) : une file ne peut être transmise
            # qu'au moment précis où elle est passée à Process(args=...),
            # lors de la création du processus qui en hérite ; un
            # pickle.dumps() isolé comme celui-ci la refuse (RuntimeError
            # « Queue objects should only be shared through
            # inheritance »). Bug réel, trouvé en branchant le pont HAL
            # (HALBridgeServer/Client) dans PlayStoreRuntime : les files
            # du pont faisaient échouer ce contrôle préventif, alors
            # qu'elles fonctionnent normalement une fois transmises au
            # vrai Process ci-dessous.
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

        # Méthode de démarrage « fork » (le nouveau processus est une copie
        # de celui-ci) imposée, quel que soit le défaut du système.
        # Constaté sur un vrai serveur Ubuntu avec Python 3.14 : le défaut
        # y est « forkserver » (un processus serveur, lancé à part, crée
        # les nouveaux processus à la demande), qui exige que la fonction
        # cible soit importable depuis un vrai module. Une fonction
        # définie dans un script exécuté directement (ou par `python3 -c`)
        # échoue alors en silence avec « AttributeError: module
        # '__main__' has no attribute ... ». « fork » duplique le
        # processus tel quel, la fonction est déjà en mémoire : cela
        # marche dans tous les cas sous Linux (pas sous Windows,
        # qu'AXIS-OS ne vise pas).
        #
        # Contrepartie connue : le processus de l'app hérite ainsi d'une
        # copie de toute la mémoire de l'OS. C'est l'une des faiblesses
        # d'isolation décrites dans docs/isolation_apps.md, dont le plan
        # prévoit de passer à « spawn » (un interpréteur Python neuf).
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

        # Attente bornée en temps du robot : join() ne connaît que le temps
        # mural, on attend donc par petits pas (50 ms) en consultant
        # l'horloge du robot.
        while (proc.is_alive()
               and self._clock() - t_start < self._quota.max_runtime_s + 1.0):
            proc.join(timeout=0.05)
        self._set_state(SandboxState.STOPPING)

        try:
            # Encore vivant après l'attente : la durée maximale (plus une
            # marge d'une seconde) est dépassée. On demande l'arrêt
            # (terminate), puis on force (kill) si le processus résiste.
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

            # Processus terminé, de lui-même ou tué par _monitor_loop() :
            # un quota enfreint l'emporte sur tout résultat renvoyé.
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
        """Nettoyage final, toujours appelé à la fin de run() : tue le
        processus s'il vit encore, puis ferme la file de résultat. Les
        erreurs y sont ignorées : le résultat de run() est déjà établi à
        ce stade."""
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
        Surveille le CPU et la mémoire du processus de l'app, dans un
        thread à part, et le tue en cas de dépassement persistant ou de
        durée dépassée.

        Ne fait rien sans psutil (voir `strict` dans __init__).

        Correction de la version 2 : il faut 3 mesures CPU consécutives
        au-dessus du seuil (deux fois quota.max_cpu_percent) pour
        déclencher, afin d'éviter une fausse alerte sur un pic isolé. Pour
        la mémoire, en revanche, le processus est arrêté dès la première
        mesure au-dessus de quota.max_memory_mb.
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
    """Donne au processus fils (celui de l'app) son propre aléa.

    Le fils est créé par duplication du parent : il hérite de l'état de
    tous les générateurs aléatoires tel qu'il était à cet instant, et ce
    qu'il en consomme n'est jamais renvoyé au parent. Sans
    réensemencement, chaque mission tirerait exactement les mêmes
    nombres.

    Constaté : une app qui simulait 30 % d'échecs de saisie échouait
    soit 20 missions sur 20, soit 0 sur 20, selon un seul tirage fait à
    sa création. Le test qui la couvrait passait ou échouait au hasard.

    Ne couvre que les générateurs globaux (module random, et numpy s'il
    est déjà chargé). Un générateur propre à l'app (un random.Random()
    créé dans son __init__) reste figé : c'est à l'app de le recréer à
    chaque mission, comme le fait WarehouseOpsApp
    (axis_os/apps/warehouse_ops/app.py).
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
    """Point d'entrée du processus de l'app : réensemence l'aléa, appelle
    la fonction, puis renvoie au parent par result_queue soit son
    résultat, soit le type et le message de l'exception levée."""
    try:
        _reensemencer_alea()
        result = target_fn(*args, **kwargs)
        result_queue.put({"result": result, "error": None})
    except Exception as exc:
        result_queue.put({"result": None, "error": f"{type(exc).__name__}: {exc}"})
