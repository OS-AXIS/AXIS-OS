"""
AXIS-OS — Exécuteur de sandbox (SandboxRunner), version 3
==========================================================
Chaque app du Play Store (la boutique d'apps du robot) tourne dans un
sandbox (un « bac à sable ») : un processus séparé, surveillé, avec des
quotas de CPU, de mémoire et de durée. Ce module crée ce processus, le
surveille et rapporte comment il s'est terminé.

Pourquoi un processus séparé plutôt qu'un thread :
  - un thread partage la mémoire de l'OS : un plantage de l'app peut
    emporter l'OS avec elle ;
  - un processus séparé a sa propre mémoire : un plantage de l'app
    n'affecte que l'app.

Historique : la version 1 avait quatre défauts, corrigés dans la
version 2 à la suite d'une revue extérieure du code (accès concurrent
sans verrou, pas de cycle de vie explicite, pas de classement des
erreurs, plantage silencieux sur un objet non sérialisable). La version 3
change la façon de créer le processus et d'échanger avec lui :

  - le processus est un interpréteur Python NEUF (hote_app.py, lancé par
    processus_app.py), et non plus une copie du processus de l'OS (fork)
    qui en héritait la mémoire, les objets HAL et les connexions ouvertes ;
  - tout ce qui passe entre les deux est du JSON validé (canal_ipc.py), et
    non plus des objets pickle, dont la lecture pouvait exécuter du code
    dans l'OS.

Deux façons de s'en servir :
  - run_app() : un hook d'app (on_start, run_mission, on_stop),
    avec le pont vers le robot (hal_bridge.py). C'est ce qu'utilise
    PlayStoreRuntime pour chaque appel à une app ;
  - run() : une fonction de confiance de l'OS désignée par son nom, avec
    des arguments et un résultat JSON, sans accès au robot.

Limite connue : le processus de l'app ne détient plus rien de l'OS, mais
il tourne encore sous le même utilisateur, avec le même accès au réseau et
aux fichiers (étapes 2 à 4 de docs/isolation_apps.md, non faites ; voir
aussi SECURITY.md, « Limites connues »).

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations
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
    TIMEOUT dans les deux cas. KILLED : processus arrêté par un arrêt
    d'urgence (voir `arret` dans run_app), error_kind ARRET_URGENCE.
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
    sandbox n'a pas pu lancer l'app (fonction, arguments ou app impossibles
    à transmettre au processus neuf, échec de création du processus). PROCESS_CRASH : le
    processus s'est terminé sans renvoyer de résultat.
    """
    APPLICATION_ERROR = auto()
    CPU_QUOTA          = auto()
    MEMORY_QUOTA        = auto()
    TIMEOUT             = auto()
    SANDBOX_ERROR        = auto()
    PROCESS_CRASH        = auto()
    # Le processus de l'app a envoyé par le pont un message que l'OS
    # refuse (trame trop longue, JSON invalide, résultat hors schéma) : il
    # est arrêté. Voir run_app().
    PROTOCOL_VIOLATION   = auto()
    # Un arrêt d'urgence est survenu pendant le hook : le processus de l'app
    # a été arrêté (voir `arret` dans run_app()).
    ARRET_URGENCE        = auto()


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
        """Exécute target_fn(*args, **kwargs) dans un processus neuf et
        attend sa fin, dans la limite de la durée maximale du quota.

        Réservé à du code de confiance de l'OS (et aux essais) : une app
        passe par run_app(). target_fn doit pouvoir être retrouvée par son
        module et son nom (fonction de module, méthode intégrée comme
        random.random ou time.sleep) ; args, kwargs et le résultat doivent
        être des données JSON. Autrefois, la fonction et ses arguments
        passaient par pickle vers un processus copié par fork ; ni l'un ni
        l'autre n'existe plus (voir docs/isolation_apps.md).

        Les échecs prévus (lancement impossible, exception, quota dépassé,
        plantage) sont rapportés dans le SandboxResult renvoyé, pas levés
        en exception."""
        from axis_os.store.runtime.processus_app import (
            NonTransmissible, encoder_donnee, reference_fonction,
        )
        t_start = self._clock()
        try:
            spec = reference_fonction(target_fn)
            args_json = [encoder_donnee(a) for a in args]
            kwargs_json = {str(k): encoder_donnee(v) for k, v in (kwargs or {}).items()}
        except (NonTransmissible, OSError, UnicodeDecodeError) as exc:
            # Même cause et même catégorie qu'au temps de pickle : la
            # fonction ou ses arguments ne peuvent pas être transmis.
            self._set_state(SandboxState.CRASHED)
            return SandboxResult(
                success=False, state=SandboxState.CRASHED,
                error=(f"Fonction ou arguments non transmissibles (ni désignables par "
                       f"leur nom, ni données JSON ; autrefois : non picklables) : {exc}"),
                error_kind=SandboxErrorKind.SANDBOX_ERROR,
                duration_s=self._clock() - t_start,
            )
        res = self.run_app(spec, "fonction", args=args_json, kwargs=kwargs_json)
        if res.success:
            res.result = res.result.get("valeur")
        return res

    # Au-delà de ce nombre de messages refusés, le processus de l'app est
    # arrêté : une app honnête n'en envoie jamais un seul.
    MAX_MESSAGES_REFUSES = 32
    # Le contrôle CPU commence quand l'app est chargée, ou au plus tard
    # après ce délai (temps du robot).
    GRACE_CPU_S = 5.0
    # Après un arrêt d'urgence, l'app a ce délai (temps RÉEL : une horloge
    # simulée arrêtée ne doit pas retarder l'arrêt) pour rendre son
    # résultat ; puis son processus est tué.
    DELAI_ARRET_URGENCE_S = 0.5
    # Seules actions encore servies après un arrêt d'urgence : sans effet
    # sur le robot, ou qui l'arrêtent. Toute autre reçoit ESTOP_ACTIVE, que
    # l'app voit comme EstopActiveError : c'est ainsi qu'elle reçoit l'arrêt.
    ACTIONS_APRES_ARRET = frozenset({"log", "get_recent_alerts", "stop_walking",
                                     "emergency_stop", "robot_info", "has_capability",
                                     "safety_status", "should_stop", "report_progress",
                                     "poll_events", "stream_metrics", "stream_close",
                                     "stop_animation", "follow_status", "stop_follow",
                                     "notify"})
    # Appels que l'OS sert par une capture ou une inférence de modèle, sans
    # moyen de les interrompre : sous `arret`, ils sont servis dans un thread
    # que run_app abandonne si le processus de l'app est tué entre-temps.
    ACTIONS_LENTES = frozenset({"listen", "look_for", "describe_scene",
                                "go_to", "pick", "place", "rest_pose",
                                "ask_confirmation", "play_animation", "speak",
                                "people", "person_in_front"})
    # Appels lents abandonnés après un arrêt d'urgence et encore en cours,
    # dans tout le processus de l'OS : tant qu'il en reste un, micro et
    # caméra sont occupés, et un nouvel appel lent est refusé.
    _abandonnes: list = []
    _verrou_abandonnes = threading.Lock()

    def run_app(self, spec: dict, hook: str, *, serveur=None, params=None,
                etat: Optional[dict] = None, tolerance_lenteur: float = 60.0,
                args: Optional[list] = None,
                kwargs: Optional[dict] = None,
                arret: Optional[Callable[[], bool]] = None) -> SandboxResult:
        """Exécute un hook d'app (on_start, run_mission, on_stop)
        dans un processus NEUF (voir hote_app.py et processus_app.py), et
        sert ses appels au robot par `serveur` (un HALBridgeServer).

        Aucun fork, aucun pickle : le processus de l'app ne reçoit que le
        pont, recharge l'app depuis `spec`, et tout ce qui en revient est du
        JSON vérifié par schéma. Quotas, horloge et cycle de vie sont ceux
        de la classe ; `result` est le dict vérifié rendu par le hook
        (processus_app.SCHEMA_RESULTAT).

        `arret` : fonction sans argument qui rend True dès qu'un arrêt
        d'urgence est actif (PlayStoreRuntime la passe pour run_mission,
        on_start et on_stop).
        Elle est consultée toutes les 20 ms par un thread à part, que le
        hook soit en plein calcul ou en attente d'un appel au robot. Dès
        qu'elle répond True : les appels de l'app reçoivent ESTOP_ACTIVE
        (voir ACTIONS_APRES_ARRET), l'app a DELAI_ARRET_URGENCE_S pour rendre
        son résultat, puis son groupe de processus est tué et run_app rend
        la main, même si une écoute ou une inférence de l'OS est en cours
        (ACTIONS_LENTES) ou si un descendant sorti du groupe tient encore le
        pont. Si l'app n'a rien rendu, l'état final est KILLED
        (ARRET_URGENCE).
        """
        from axis_os.store.runtime.canal_ipc import (
            CanalCorrompu, CanalFerme, MessageRefuse,
        )
        from axis_os.store.runtime.processus_app import (
            ProcessusHote, SCHEMA_FIN, SCHEMA_RESULTAT, _affichable, message_depart,
        )

        t_start = self._clock()
        self._set_state(SandboxState.STARTING)
        arret_vu, boucle_finie, tue = threading.Event(), threading.Event(), threading.Event()
        veilleur = None
        try:
            proc = ProcessusHote(self._app_id)
        except Exception as exc:                 # noqa: BLE001
            self._set_state(SandboxState.CRASHED)
            return SandboxResult(False, SandboxState.CRASHED,
                                 error=f"Échec démarrage process : {exc}",
                                 error_kind=SandboxErrorKind.SANDBOX_ERROR,
                                 duration_s=self._clock() - t_start)
        try:
            try:
                proc.canal.envoyer(message_depart(hook, spec, params, etat,
                                                  tolerance_lenteur, args, kwargs))
            except (TypeError, ValueError) as exc:
                self._set_state(SandboxState.CRASHED)
                return SandboxResult(False, SandboxState.CRASHED,
                                     error=f"Paramètres non transmissibles en JSON : {exc}",
                                     error_kind=SandboxErrorKind.SANDBOX_ERROR,
                                     duration_s=self._clock() - t_start)
            except CanalFerme as exc:
                self._set_state(SandboxState.CRASHED)
                return SandboxResult(False, SandboxState.CRASHED,
                                     error=f"Processus de l'app parti avant le départ : {exc}",
                                     error_kind=SandboxErrorKind.PROCESS_CRASH,
                                     duration_s=self._clock() - t_start)

            self._set_state(SandboxState.RUNNING)
            cpu_actif = threading.Event()
            monitor = threading.Thread(target=self._monitor_loop,
                                       args=(proc, t_start, cpu_actif), daemon=True)
            monitor.start()
            if arret is not None:
                fil = threading.Thread(
                    target=self._veiller_arret, args=(proc, arret, arret_vu, boucle_finie, tue),
                    daemon=True, name=f"arret-urgence-{self._app_id}")
                fil.start()
                veilleur = fil          # seulement une fois démarré : voir le finally

            fin, violation_protocole, refus = None, None, 0
            limite = self._quota.max_runtime_s + 1.0
            # Après un arrêt d'urgence, la boucle s'arrête dès que le processus
            # est tué, sans attendre la fin du pont : un descendant qui a quitté
            # le groupe (setsid) peut encore le tenir ouvert.
            while self._clock() - t_start < limite and not tue.is_set():
                if not cpu_actif.is_set() and self._clock() - t_start > self.GRACE_CPU_S:
                    cpu_actif.set()
                try:
                    msg = proc.canal.recevoir(timeout=0.05)
                except TimeoutError:
                    continue
                except CanalFerme:
                    break
                except CanalCorrompu as exc:
                    violation_protocole = str(exc)
                    break
                except MessageRefuse as exc:
                    refus += 1
                    print(f"  [SECURITY] app '{self._app_id}' : message refusé par le "
                          f"pont ({str(exc)[:200]})")
                    if refus > self.MAX_MESSAGES_REFUSES:
                        violation_protocole = f"plus de {self.MAX_MESSAGES_REFUSES} messages refusés"
                        break
                    continue
                genre = msg.get("type")
                if genre == "appel":
                    cpu_actif.set()
                    rid = msg.get("request_id")
                    # Pas encore vérifiée par le schéma (serveur.traiter) : une
                    # action qui n'est pas un texte ne doit pas faire lever le
                    # test d'appartenance ; le schéma la refusera.
                    action = msg.get("action") if isinstance(msg.get("action"), str) else None
                    if arret_vu.is_set() and action not in self.ACTIONS_APRES_ARRET:
                        if isinstance(rid, bool) or not isinstance(rid, int):
                            continue
                        reponse = {"request_id": rid, "success": False, "data": None,
                                   "error": "ESTOP_ACTIVE:arrêt d'urgence, mission interrompue"}
                    elif action in self.ACTIONS_LENTES and self._peripherique_occupe():
                        if isinstance(rid, bool) or not isinstance(rid, int):
                            continue
                        reponse = {"request_id": rid, "success": False, "data": None,
                                   "error": "périphérique encore occupé par un appel interrompu "
                                            "par un arrêt d'urgence"}
                    elif serveur is not None:
                        # Le serveur reçoit le temps qui reste au hook : une
                        # action qui bloque par nature (listen) ne doit pas
                        # retenir le thread de l'OS au-delà du quota.
                        reste = max(0.0, limite - 1.0 - (self._clock() - t_start))
                        if action in self.ACTIONS_LENTES:
                            reponse = self._servir_abandonnable(serveur, msg, reste, tue, proc)
                            if tue.is_set():
                                break
                        else:
                            reponse = serveur.traiter(msg, reste_s=reste)
                    else:
                        reponse = {"request_id": rid, "success": False, "data": None,
                                   "error": "aucun accès au robot pendant ce hook"}
                    if reponse is None:
                        continue
                    reponse["type"] = "reponse"
                    try:
                        proc.canal.envoyer(reponse)
                    except (TypeError, ValueError) as exc:
                        proc.canal.envoyer({"type": "reponse",
                                            "request_id": reponse.get("request_id"),
                                            "success": False, "data": None,
                                            "error": f"réponse non transmissible : {exc}"[:500]})
                    except CanalFerme:
                        break
                elif genre == "fin":
                    try:
                        fin = SCHEMA_FIN.verifier(msg, "fin")
                    except MessageRefuse as exc:
                        violation_protocole = f"message de fin refusé : {exc}"
                    break
                else:
                    refus += 1
                    print(f"  [SECURITY] app '{self._app_id}' : message de type "
                          f"inattendu refusé ({str(genre)[:40]!r})")
                    if refus > self.MAX_MESSAGES_REFUSES:
                        violation_protocole = f"plus de {self.MAX_MESSAGES_REFUSES} messages refusés"
                        break
            self._set_state(SandboxState.STOPPING)
            boucle_finie.set()
            if veilleur is not None:
                veilleur.join()
            if arret_vu.is_set():
                # Arrêt d'urgence : pas d'attente polie, le groupe est tué.
                proc.kill()
                proc.join(timeout=1.0)

            # Le processus a fini son hook : il doit sortir de lui-même.
            if fin is not None or violation_protocole is not None:
                proc.join(timeout=0.5 if violation_protocole else 2.0)
            encore_vivant = proc.is_alive()
            if encore_vivant:
                proc.terminate()
                proc.join(timeout=2.0)
                if proc.is_alive():
                    proc.kill()
                    proc.join(timeout=1.0)

            if violation_protocole is not None:
                print(f"  [SECURITY] app '{self._app_id}' arrêtée : violation du "
                      f"protocole du pont ({violation_protocole[:200]})")
                self._set_state(SandboxState.CRASHED)
                return SandboxResult(False, SandboxState.CRASHED,
                                     error=f"Violation du protocole du pont : {violation_protocole}",
                                     error_kind=SandboxErrorKind.PROTOCOL_VIOLATION,
                                     duration_s=self._clock() - t_start)

            if fin is None and arret_vu.is_set():
                self._set_state(SandboxState.KILLED)
                return SandboxResult(False, SandboxState.KILLED,
                                     error="Arrêt d'urgence (e-stop) : processus de l'app arrêté.",
                                     error_kind=SandboxErrorKind.ARRET_URGENCE,
                                     duration_s=self._clock() - t_start)

            if fin is None and encore_vivant:
                violation = self._get_violation() or SandboxViolation(
                    app_id=self._app_id, rule="TIMEOUT",
                    value=self._clock() - t_start, limit=self._quota.max_runtime_s)
                self._set_state(SandboxState.TIMEOUT)
                return SandboxResult(False, SandboxState.TIMEOUT, violation=violation,
                                     error="Timeout — processus tué.",
                                     error_kind=SandboxErrorKind.TIMEOUT,
                                     duration_s=self._clock() - t_start)

            violation = self._get_violation()
            if violation is not None:
                kind = (SandboxErrorKind.CPU_QUOTA if violation.rule == "CPU_QUOTA"
                        else SandboxErrorKind.MEMORY_QUOTA if violation.rule == "MEMORY_QUOTA"
                        else SandboxErrorKind.TIMEOUT)
                self._set_state(SandboxState.VIOLATED)
                return SandboxResult(False, SandboxState.VIOLATED, violation=violation,
                                     error=f"Quota dépassé : {violation.rule}",
                                     error_kind=kind, duration_s=self._clock() - t_start)

            if fin is None:
                self._set_state(SandboxState.CRASHED)
                return SandboxResult(False, SandboxState.CRASHED,
                                     error="Processus terminé sans résultat (crash probable).",
                                     error_kind=SandboxErrorKind.PROCESS_CRASH,
                                     duration_s=self._clock() - t_start)

            if fin.get("erreur"):
                self._set_state(SandboxState.CRASHED)
                # Texte choisi par l'app, affiché ensuite par l'OS : neutralisé.
                return SandboxResult(False, SandboxState.CRASHED,
                                     error=_affichable(fin["erreur"], 2000),
                                     error_kind=SandboxErrorKind.APPLICATION_ERROR,
                                     duration_s=self._clock() - t_start)
            try:
                resultat = SCHEMA_RESULTAT[hook].verifier(fin.get("resultat"), "resultat")
            except MessageRefuse as exc:
                print(f"  [SECURITY] app '{self._app_id}' : résultat refusé ({str(exc)[:200]})")
                self._set_state(SandboxState.CRASHED)
                return SandboxResult(False, SandboxState.CRASHED,
                                     error=f"Résultat refusé : {exc}",
                                     error_kind=SandboxErrorKind.PROTOCOL_VIOLATION,
                                     duration_s=self._clock() - t_start)
            self._set_state(SandboxState.STOPPED)
            return SandboxResult(True, SandboxState.STOPPED, result=resultat,
                                 duration_s=self._clock() - t_start)
        finally:
            # Le veilleur est joint AVANT fermer(), qui récolte le processus :
            # il ne peut jamais viser un numéro de processus déjà réutilisé.
            try:
                boucle_finie.set()
                if veilleur is not None:
                    veilleur.join()
            finally:
                proc.fermer()

    @classmethod
    def _servir_abandonnable(cls, serveur, msg, reste: float, tue: threading.Event,
                             proc=None):
        """Sert un appel lent (ACTIONS_LENTES) dans un thread et attend sa
        réponse, sauf si le processus de l'app est tué entre-temps : la
        mission rend alors la main tout de suite, et la réponse, arrivée
        trop tard, est jetée ; le thread reste noté comme abandonné tant
        qu'il tourne (voir _peripherique_occupe).

        Si le processus de l'app meurt pendant l'appel (plantage, quota de
        CPU ou de mémoire), son contexte est fermé aussitôt : l'action qu'il
        avait demandée (go_to, geste, animation, question à l'humain)
        s'arrête au pas suivant. Pourquoi : une revue adversariale a mesuré
        un go_to qui roulait encore 13 s, jusqu'à 3.9 m, après la mort de
        l'app qui l'avait lancé."""
        boite = {}
        fil = threading.Thread(target=lambda: boite.update(r=serveur.traiter(msg, reste_s=reste)),
                               daemon=True, name="appel-lent")
        fil.start()
        ferme = False
        while fil.is_alive() and not tue.is_set():
            fil.join(0.05)
            if not ferme and proc is not None and not proc.is_alive():
                ctx = getattr(serveur, "_ctx", None)
                if ctx is not None:
                    ctx._fermer()
                ferme = True
        if fil.is_alive():
            with cls._verrou_abandonnes:
                cls._abandonnes.append(fil)
        return boite.get("r")

    @classmethod
    def _peripherique_occupe(cls) -> bool:
        with cls._verrou_abandonnes:
            cls._abandonnes[:] = [f for f in cls._abandonnes if f.is_alive()]
            return bool(cls._abandonnes)

    def _veiller_arret(self, proc, arret: Callable[[], bool],
                       arret_vu: threading.Event, boucle_finie: threading.Event,
                       tue: threading.Event) -> None:
        """Thread de run_app : voir `arret`. Un arret() qui lève est tenu
        pour un arrêt d'urgence (dans le doute, on arrête)."""
        while not boucle_finie.wait(0.02):
            try:
                actif = arret()
            except Exception:              # noqa: BLE001 — dans le doute, arrêter
                actif = True
            if actif:
                arret_vu.set()
                print(f"  [SECURITY] app '{self._app_id}' : arrêt d'urgence, son "
                      f"processus est arrêté")
                if not boucle_finie.wait(self.DELAI_ARRET_URGENCE_S):
                    proc.kill()
                    tue.set()
                return

    def _monitor_loop(self, proc, t_start: float,
                      cpu_actif: Optional[threading.Event] = None) -> None:
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

                if cpu_actif is not None and not cpu_actif.is_set():
                    # Démarrage de l'interpréteur et chargement de l'app
                    # (voir run_app) : la durée et la mémoire sont
                    # contrôlées, le CPU pas encore.
                    consecutive_cpu_over = 0
                elif cpu > self._quota.max_cpu_percent * 2:
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
