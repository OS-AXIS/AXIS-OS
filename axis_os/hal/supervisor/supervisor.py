"""
AXIS-OS — Supervisor
=======================
Surveille le WatchdogSystem lui-même, depuis l'extérieur.

Origine : revue externe du 2026-09-06 sur limits.py/system.py/
normalized_robot.py/play_store.py. Point le plus critique soulevé :
un Watchdog ne peut pas fiablement se surveiller lui-même s'il meurt
complètement — si son thread se bloque en deadlock ou crashe d'une
façon que son propre try/except ne peut pas attraper, plus rien ne
surveille le robot, et personne ne le sait.

Architecture recommandée par la revue, implémentée ici :

    Supervisor → Watchdog → Safety Layer → HAL → Robot

Le Supervisor est un thread INDÉPENDANT du Watchdog — il ne partage
aucun état interne avec lui, seulement WatchdogSystem.last_cycle_monotonic
(une propriété en lecture seule) et un accès direct au HAL pour pouvoir
déclencher un arrêt d'urgence MÊME SI le Watchdog est totalement mort
et ne peut donc plus appeler lui-même hal.emergency_stop_all().

Ce que ce Supervisor fait :
  - Vérifie périodiquement que le Watchdog a bien complété un cycle
    de surveillance récemment (last_cycle_monotonic).
  - Si le Watchdog semble mort (pas de cycle récent) OU que son thread
    Python a littéralement cessé d'exister (is_alive() == False), le
    Supervisor déclenche lui-même l'arrêt d'urgence, directement sur
    le HAL, sans dépendre du Watchdog pour le faire.
  - Journalise cet événement de façon persistante, distincte des logs
    du Watchdog lui-même (un Watchdog mort ne peut plus écrire ses
    propres logs de façon fiable).

Ce que ce Supervisor ne fait PAS (portée volontairement limitée) :
  - Il ne remplace aucune des vérifications métier du Watchdog
    (température, couple, comm par joint, etc.) — il vérifie
    uniquement que le Watchdog LUI-MÊME est vivant et progresse.
  - Il ne redémarre pas automatiquement le Watchdog après un
    déclenchement — cohérent avec le principe déjà appliqué ailleurs
    dans AXIS-OS (interdiction d'un redémarrage automatique du
    mouvement après un E-STOP sans validation explicite, point
    également soulevé par la revue externe).
  - Une seule couche de supervision est implémentée ici (qui surveille
    le Supervisor lui-même ? — question légitime, mais un empilement
    infini de surveillants n'apporte plus de valeur pratique passé un
    certain point ; ce choix est documenté, pas caché).

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

from axis_os.hal.core.interfaces import IRobotHAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.core.structured_log import (
    StructuredLogger, LogCategory, get_default_logger
)


@dataclass
class SupervisorAlert:
    """Événement émis par le Supervisor — distinct de WatchdogAlert
    pour ne jamais confondre une alerte métier (température, couple)
    avec un événement de supervision du système de sécurité lui-même."""
    timestamp_ns: int
    kind:         str      # "WATCHDOG_UNRESPONSIVE" | "WATCHDOG_THREAD_DEAD" | "SUPERVISOR_STARTED" | "SUPERVISOR_STOPPED"
    message:      str
    seconds_since_last_cycle: Optional[float] = None

    def __str__(self) -> str:
        detail = (f" ({self.seconds_since_last_cycle:.1f}s depuis le dernier cycle)"
                 if self.seconds_since_last_cycle is not None else "")
        return f"[SUPERVISOR] {self.kind}: {self.message}{detail}"


class Supervisor:
    """
    Usage :
        wd  = WatchdogSystem(robot_hal)
        sup = Supervisor(watchdog=wd, robot_hal=robot_hal)
        wd.start()
        sup.start()
        # ...
        sup.stop()
        wd.stop()

    L'ordre de démarrage importe : le Watchdog doit démarrer avant le
    Supervisor, sinon le Supervisor verrait immédiatement
    last_cycle_monotonic=0.0 et pourrait déclencher un faux-positif
    avant même que le Watchdog ait eu le temps de faire son premier
    cycle. Le Supervisor gère ce cas via une période de grâce au
    démarrage (grace_period_s) plutôt que d'exiger un ordre strict —
    plus robuste face à l'erreur d'un intégrateur qui inverserait
    l'ordre par mégarde.
    """

    def __init__(self, watchdog: WatchdogSystem, robot_hal: IRobotHAL,
                check_interval_s: float = 0.5,
                unresponsive_threshold_s: float = 2.0,
                grace_period_s: float = 3.0,
                logger: Optional[StructuredLogger] = None):
        """
        check_interval_s          : fréquence à laquelle le Supervisor
            vérifie l'état du Watchdog.
        unresponsive_threshold_s  : si aucun cycle Watchdog n'a été
            complété depuis ce délai, le Watchdog est considéré mort.
            Doit être significativement plus grand que le
            poll_interval_s configuré du Watchdog surveillé, sinon des
            faux-positifs sont probables sous charge normale.
        grace_period_s            : délai après start() pendant lequel
            l'absence de cycle Watchdog n'est PAS traitée comme une
            anomalie — laisse le temps au Watchdog de démarrer et de
            compléter son premier cycle, quel que soit l'ordre de
            démarrage réel des deux composants.
        """
        self._watchdog = watchdog
        self._hal      = robot_hal
        self._check_interval_s = check_interval_s
        self._unresponsive_threshold_s = unresponsive_threshold_s
        self._grace_period_s = grace_period_s
        self._struct_logger = logger or get_default_logger()

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._start_time = 0.0
        self._triggered = False
        self._trigger_lock = threading.Lock()
        self._subscribers: List[Callable[[SupervisorAlert], None]] = []

    def subscribe(self, callback: Callable[[SupervisorAlert], None]) -> None:
        self._subscribers.append(callback)

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._start_time = time.monotonic()
        self._thread = threading.Thread(
            target=self._monitor_loop, name="supervisor_monitor", daemon=True)
        self._thread.start()
        self._emit(SupervisorAlert(
            timestamp_ns=time.time_ns(), kind="SUPERVISOR_STARTED",
            message=f"Supervisor démarré | seuil={self._unresponsive_threshold_s}s",
        ))

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self._emit(SupervisorAlert(
            timestamp_ns=time.time_ns(), kind="SUPERVISOR_STOPPED",
            message="Supervisor arrêté",
        ))

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def has_triggered(self) -> bool:
        with self._trigger_lock:
            return self._triggered

    def is_robot_stopped(self) -> bool:
        """
        Vérité globale sur l'arrêt du robot — à utiliser à la place de
        wd.estop_fired seul dans tout code qui a besoin de savoir "le
        robot est-il vraiment arrêté", pas seulement "le Watchdog
        pense-t-il l'avoir arrêté".

        Découvert en testant ce Supervisor : quand LUI déclenche
        l'arrêt (Watchdog mort/bloqué), hal.emergency_stop_all() est
        bien appelé et le robot est réellement arrêté, mais
        WatchdogSystem.estop_fired reste False — ce flag est interne
        au Watchdog et n'a aucune raison d'être mis à jour par un
        composant tiers qui l'a court-circuité précisément parce qu'il
        ne répondait plus. Un code qui ne vérifierait que
        wd.estop_fired se tromperait silencieusement dans ce cas
        précis, en pensant le robot actif alors qu'il est arrêté.
        """
        return self._watchdog.estop_fired or self.has_triggered

    def _monitor_loop(self) -> None:
        while self._running:
            self._check_once()
            time.sleep(self._check_interval_s)

    def _check_once(self) -> None:
        now = time.monotonic()

        if now - self._start_time < self._grace_period_s:
            return

        # Cas 1 — le thread Python du Watchdog n'existe littéralement
        # plus (crash total, pas juste un blocage). Distinct du cas 2
        # ci-dessous car ça peut arriver même si is_running renvoie
        # encore True côté Watchdog (le flag n'a pas pu être mis à jour
        # par un thread qui a cessé d'exister).
        watchdog_thread = getattr(self._watchdog, "_thread", None)
        if self._watchdog.is_running and watchdog_thread is not None \
                and not watchdog_thread.is_alive():
            self._trigger_safety_response(
                kind="WATCHDOG_THREAD_DEAD",
                message="Le thread de surveillance du Watchdog a cessé d'exister "
                        "alors que le système le croyait actif — le robot n'est "
                        "plus surveillé.",
            )
            return

        # Cas 2 — le Watchdog tourne peut-être encore (thread vivant),
        # mais ne progresse plus (deadlock, boucle bloquée avant d'avoir
        # pu compléter _run_cycle). C'est le cas que le try/except
        # interne du Watchdog ne peut PAS attraper — une exception
        # suppose que le code continue de s'exécuter jusqu'au point du
        # raise, un deadlock ne lève jamais rien.
        if not self._watchdog.is_running:
            return  # arrêt normal (stop() appelé), pas une anomalie

        elapsed = now - self._watchdog.last_cycle_monotonic
        if elapsed > self._unresponsive_threshold_s:
            self._trigger_safety_response(
                kind="WATCHDOG_UNRESPONSIVE",
                message=(f"Aucun cycle de surveillance complété depuis "
                         f"{elapsed:.1f}s (seuil={self._unresponsive_threshold_s}s) "
                         f"— le Watchdog est vivant mais ne progresse plus."),
                seconds_since_last_cycle=elapsed,
            )

    def _trigger_safety_response(self, kind: str, message: str,
                                 seconds_since_last_cycle: Optional[float] = None) -> None:
        """
        Idempotent — même principe que WatchdogSystem._trigger_estop :
        un seul déclenchement réel même si _check_once() détecte la
        même anomalie sur plusieurs cycles consécutifs avant que le
        prochain redémarrage manuel du système ne remette tout à zéro.
        """
        with self._trigger_lock:
            if self._triggered:
                return
            self._triggered = True

        # Déclenché HORS du lock, même raisonnement que pour le Watchdog :
        # ne jamais bloquer sur l'appel HAL pendant qu'on tient un verrou
        # de détection.
        self._hal.emergency_stop_all()

        alert = SupervisorAlert(
            timestamp_ns=time.time_ns(), kind=kind, message=message,
            seconds_since_last_cycle=seconds_since_last_cycle,
        )
        print(f"  {alert}")

        # Persistance disque — catégorie distincte du Watchdog lui-même :
        # un Watchdog mort ne peut plus fiablement écrire son propre
        # log d'e-stop, donc ce log DOIT provenir d'un composant qui
        # fonctionne encore, à savoir le Supervisor.
        self._struct_logger.log(
            LogCategory.WATCHDOG_ALERT, alert, robot_id=self._hal.robot_id
        )
        self._emit(alert)

    def _emit(self, alert: SupervisorAlert) -> None:
        for cb in self._subscribers:
            try:
                cb(alert)
            except Exception:
                # Un abonné qui plante ne doit jamais faire tomber le
                # Supervisor — c'est précisément le genre de composant
                # qui ne peut pas se permettre d'être fragile.
                pass
