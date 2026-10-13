"""
AXIS-OS — Supervisor
=======================
Surveille le WatchdogSystem lui-même, depuis l'extérieur. Le Watchdog
(le « chien de garde » logiciel qui vérifie en continu chaque
articulation du robot : température, couple, communication, entre
autres) peut
lui-même tomber en panne ; ce module détecte cette panne et arrête le
robot.

Pourquoi ce module existe : une revue extérieure du code, menée le
2026-09-06 sur limits.py, system.py, normalized_robot.py et
play_store.py, a relevé comme point le plus critique qu'un Watchdog ne
peut pas se surveiller lui-même de façon fiable s'il meurt
complètement. Si son thread (le fil d'exécution qui tourne en parallèle
du reste du programme) reste bloqué en deadlock (interblocage : il
attend indéfiniment quelque chose qui n'arrivera jamais), ou plante
d'une façon que son propre try/except (le bloc qui intercepte les
erreurs Python) ne peut pas attraper, plus rien ne surveille le robot,
et personne ne le sait.

Architecture recommandée par cette revue, et implémentée ici :

    Supervisor → Watchdog → Safety Layer → HAL → Robot

La Safety Layer est la couche de sécurité qui vérifie chaque commande
moteur contre les limites des articulations (JointSafetyGuard, par
exemple) ; le HAL est la couche d'abstraction matérielle, seul chemin
vers les moteurs.

Le Supervisor tourne dans un thread indépendant de celui du Watchdog.
Il ne partage avec lui aucun état modifiable : il se contente de lire
WatchdogSystem.last_cycle_monotonic (une propriété en lecture seule :
l'instant, lu sur l'horloge time.monotonic() qui ne recule jamais, où
le dernier cycle de surveillance s'est terminé sans erreur), ainsi que
is_running, estop_fired et l'objet thread du Watchdog. Il garde en
outre un accès direct au HAL, pour pouvoir déclencher un arrêt
d'urgence même si le Watchdog est totalement mort et ne peut donc plus
appeler lui-même hal.emergency_stop_all().

Ce que fait ce Supervisor :
  - Il vérifie périodiquement que le Watchdog a bien terminé un cycle
    de surveillance récemment (last_cycle_monotonic).
  - Si le Watchdog semble mort (pas de cycle récent), ou si son thread
    Python a littéralement cessé d'exister (is_alive() == False), il
    déclenche lui-même l'arrêt d'urgence, directement sur le HAL, sans
    dépendre du Watchdog pour le faire.
  - Il consigne lui-même cet événement sur disque, sans passer par le
    Watchdog : un Watchdog mort ne peut plus écrire ses propres
    journaux de façon fiable (la catégorie de journal utilisée est
    détaillée dans _trigger_safety_response()).

Ce que ce Supervisor ne fait pas (portée volontairement limitée) :
  - Il ne remplace aucune des vérifications propres au Watchdog
    (température, couple, communication de chaque articulation, etc.) :
    il vérifie uniquement que le Watchdog lui-même est vivant et
    progresse.
  - Il ne redémarre pas automatiquement le Watchdog après un
    déclenchement. C'est cohérent avec la règle déjà appliquée ailleurs
    dans AXIS-OS, un point également soulevé par la revue extérieure :
    pas de reprise automatique du mouvement après un E-STOP (arrêt
    d'urgence) sans validation explicite.
  - Une seule couche de supervision est implémentée ici. La question
    « qui surveille le Supervisor ? » est légitime, mais empiler les
    surveillants à l'infini n'apporte plus de valeur pratique passé un
    certain point. Ce choix est documenté, pas caché.

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
    """Événement émis par le Supervisor.

    Classe distincte de WatchdogAlert, pour ne jamais confondre une
    alerte sur l'état du robot (température, couple) avec un événement
    qui concerne la surveillance du système de sécurité lui-même.

    timestamp_ns             : instant de l'événement (time.time_ns(), en
        nanosecondes).
    kind                     : type d'événement, parmi
        "WATCHDOG_UNRESPONSIVE" (Watchdog vivant mais qui ne progresse
        plus), "WATCHDOG_THREAD_DEAD" (thread du Watchdog disparu),
        "SUPERVISOR_STARTED" et "SUPERVISOR_STOPPED".
    message                  : description lisible de l'événement.
    seconds_since_last_cycle : temps écoulé depuis le dernier cycle
        terminé du Watchdog, renseigné seulement quand il a été mesuré
        (cas WATCHDOG_UNRESPONSIVE)."""
    timestamp_ns: int
    kind:         str      # l'une des quatre valeurs listées ci-dessus
    message:      str
    seconds_since_last_cycle: Optional[float] = None

    def __str__(self) -> str:
        detail = (f" ({self.seconds_since_last_cycle:.1f}s depuis le dernier cycle)"
                 if self.seconds_since_last_cycle is not None else "")
        return f"[SUPERVISOR] {self.kind}: {self.message}{detail}"


class Supervisor:
    """
    Thread de surveillance qui déclenche l'arrêt d'urgence du robot si
    le Watchdog cesse de fonctionner (voir l'en-tête du module).

    Usage :
        wd  = WatchdogSystem(robot_hal)
        sup = Supervisor(watchdog=wd, robot_hal=robot_hal)
        wd.start()
        sup.start()
        # ...
        sup.stop()
        wd.stop()

    L'ordre de démarrage compte : le Watchdog doit démarrer avant le
    Supervisor. Sinon, le Supervisor verrait immédiatement
    last_cycle_monotonic=0.0 et pourrait déclencher une fausse alerte
    (un faux positif) avant même que le Watchdog ait eu le temps de
    faire son premier cycle. Plutôt que d'exiger un ordre strict, le
    Supervisor gère ce cas par une période de grâce au démarrage
    (grace_period_s) : c'est plus robuste face à l'erreur d'un
    intégrateur qui inverserait l'ordre par mégarde.
    """

    def __init__(self, watchdog: WatchdogSystem, robot_hal: IRobotHAL,
                check_interval_s: float = 0.5,
                unresponsive_threshold_s: float = 2.0,
                grace_period_s: float = 3.0,
                logger: Optional[StructuredLogger] = None):
        """
        check_interval_s          : intervalle, en secondes, entre deux
            vérifications de l'état du Watchdog.
        unresponsive_threshold_s  : si le Watchdog n'a terminé aucun
            cycle depuis ce délai (en secondes), il est considéré comme
            mort. Ce seuil doit être nettement plus grand que le
            poll_interval_s (l'intervalle entre deux cycles) configuré
            pour le Watchdog surveillé ; sinon, de fausses alertes sont
            probables sous une charge normale.
        grace_period_s            : délai après start() pendant lequel
            l'absence de cycle du Watchdog n'est pas traitée comme une
            anomalie. Il laisse au Watchdog le temps de démarrer et de
            terminer son premier cycle, quel que soit l'ordre réel de
            démarrage des deux composants.
        logger                    : journal structuré, écrit sur disque,
            où consigner les déclenchements ; par défaut, celui que
            renvoie get_default_logger().
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
        """Enregistre une fonction appelée à chaque SupervisorAlert émise
        (démarrage, arrêt, déclenchement). Une exception levée par cette
        fonction est ignorée : voir _emit()."""
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
        """True dès que le Supervisor a lui-même déclenché l'arrêt
        d'urgence. Aucune méthode ne le remet à False (voir
        _trigger_safety_response())."""
        with self._trigger_lock:
            return self._triggered

    def is_robot_stopped(self) -> bool:
        """
        Indique si le robot a été arrêté en urgence, que ce soit par le
        Watchdog ou par ce Supervisor. À utiliser à la place de
        wd.estop_fired seul dans tout code qui a besoin de savoir
        « le robot est-il vraiment arrêté ? », et pas seulement
        « le Watchdog pense-t-il l'avoir arrêté ? ».

        Pourquoi : constaté en testant ce Supervisor. Quand c'est lui qui
        déclenche l'arrêt (Watchdog mort ou bloqué),
        hal.emergency_stop_all() est bien appelé et le robot est
        réellement arrêté, mais WatchdogSystem.estop_fired reste False.
        Ce drapeau est interne au Watchdog : un composant tiers, qui l'a
        court-circuité précisément parce qu'il ne répondait plus, n'a
        aucune raison de le mettre à jour. Un code qui ne vérifierait
        que wd.estop_fired se tromperait sans le signaler dans ce cas
        précis, en croyant le robot actif alors qu'il est arrêté. Ce cas
        est vérifié par test/test_supervisor.py (Watchdog gelé).

        Limite : un arrêt d'urgence demandé directement au HAL par un
        autre composant n'est pas vu par cette méthode, qui ne consulte
        pas IRobotHAL.estop_active (axis_os/hal/core/interfaces.py).
        Cet état, porté par le HAL et positionné par
        emergency_stop_all() quel que soit l'appelant, couvre tous les
        cas ; le contexte est décrit dans CHANGELOG.md (entrée sur
        l'e-stop du Supervisor qui ne positionnait pas
        watchdog.estop_fired).
        """
        return self._watchdog.estop_fired or self.has_triggered

    def _monitor_loop(self) -> None:
        """Boucle du thread de surveillance : une vérification toutes les
        check_interval_s secondes, jusqu'à l'appel de stop()."""
        while self._running:
            self._check_once()
            time.sleep(self._check_interval_s)

    def _check_once(self) -> None:
        now = time.monotonic()

        # Période de grâce : juste après start(), le Watchdog n'a
        # peut-être pas encore terminé son premier cycle (voir
        # grace_period_s dans __init__).
        if now - self._start_time < self._grace_period_s:
            return

        # Cas 1 : le thread Python du Watchdog n'existe littéralement
        # plus (plantage complet, pas un simple blocage). Ce cas est distinct
        # du cas 2 ci-dessous, car il peut se produire alors que
        # is_running renvoie encore True côté Watchdog : un thread qui a
        # cessé d'exister n'a pas pu remettre ce drapeau à jour. Le thread
        # est lu dans l'attribut interne _thread du Watchdog (None tant
        # que celui-ci n'a pas été démarré).
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

        # Cas 2 : le thread du Watchdog tourne peut-être encore, mais le
        # Watchdog ne progresse plus (deadlock, boucle bloquée avant
        # d'avoir pu terminer _run_cycle). C'est le cas que le try/except
        # interne du Watchdog ne peut pas attraper : une exception
        # suppose que le code continue de s'exécuter jusqu'à l'instruction
        # raise qui la lève, alors qu'un deadlock ne lève jamais rien.
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
        Déclenche l'arrêt d'urgence, puis le consigne et le signale.

        Idempotent (l'appeler plusieurs fois a le même effet que l'appeler
        une fois), sur le même principe que WatchdogSystem._trigger_estop :
        un seul déclenchement réel a lieu, même si _check_once() détecte
        la même anomalie sur plusieurs cycles consécutifs. Seul un
        redémarrage manuel du système remet ensuite tout à zéro.
        """
        with self._trigger_lock:
            if self._triggered:
                return
            self._triggered = True

        # L'arrêt d'urgence est appelé hors du verrou, pour la même raison
        # que dans le Watchdog : ne jamais rester bloqué sur l'appel au HAL
        # pendant qu'on tient le verrou de détection.
        self._hal.emergency_stop_all()

        alert = SupervisorAlert(
            timestamp_ns=time.time_ns(), kind=kind, message=message,
            seconds_since_last_cycle=seconds_since_last_cycle,
        )
        print(f"  {alert}")

        # Écriture sur disque, faite par le Supervisor et non par le
        # Watchdog : un Watchdog mort ne peut plus écrire de façon fiable
        # son propre journal d'e-stop (catégorie ESTOP, fichier
        # estop.jsonl). Cette trace doit donc venir d'un composant qui
        # fonctionne encore, le Supervisor. Elle est rangée sous la
        # catégorie WATCHDOG_ALERT (fichier watchdog_alert.jsonl, où le
        # Watchdog écrit aussi ses alertes les plus graves) et non sous
        # ESTOP : un e-stop déclenché par le Supervisor n'apparaît donc
        # pas dans estop.jsonl.
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
                # Supervisor : c'est précisément le genre de composant
                # qui ne peut pas se permettre d'être fragile.
                pass
