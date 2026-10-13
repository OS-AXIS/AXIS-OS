"""
AXIS-OS — WatchdogSystem v2
=============================
Thread temps réel qui surveille en permanence toutes les articulations
(joints) du robot : communication, état des moteurs, température,
couple, vitesse et position. Il déclenche automatiquement l'e-stop
(arrêt d'urgence) quand un seuil FATAL est franchi (les alertes vont de
INFO, simple information, à FATAL, condition jugée dangereuse ; voir
events.py).

Il passe par le HAL (la couche d'abstraction matérielle, seul chemin
vers les moteurs) : il y lit le feedback de chaque joint (la mesure que
le moteur renvoie à chaque cycle) et lui demande l'arrêt d'urgence.

Version 2, corrigée à la suite d'une revue extérieure du code :
  - l'e-stop est protégé par un verrou (Lock) : il reste idempotent
    (déclenché une seule fois) même si deux conditions FATAL arrivent
    en même temps depuis des joints différents ;
  - un état de santé global (RobotHealthState) est dérivé des alertes
    actives ;
  - une alerte CRITICAL exige plusieurs mesures consécutives hors
    limite, pour éviter les faux positifs ;
  - auto-surveillance : le watchdog détecte son propre ralentissement ;
  - les alertes FATAL ne sont jamais perdues (file d'attente dédiée,
    sans limite de taille) ;
  - chaque e-stop est consigné dans un journal structuré (raison,
    joint, valeur mesurée, seuil).

Ce que le watchdog ne peut pas voir lui-même, son propre blocage
complet, est surveillé de l'extérieur : par le chien de garde de systemd,
le gestionnaire de services de Linux (voir _sd_notify_watchdog()), et
par le Supervisor, le module qui vérifie que le watchdog avance encore
et arrête le robot sinon (axis_os/hal/supervisor/supervisor.py, via
last_cycle_monotonic).

Licence : Apache 2.0, comme tout le dépôt (voir LICENSE).
"""

from __future__ import annotations
import threading
import time
import queue
import math
import os
import socket
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional
from concurrent.futures import ThreadPoolExecutor

from axis_os.hal.core.interfaces import IRobotHAL
from axis_os.hal.core.types import ActuatorFeedback, ActuatorState, JointLimits
from .events import WatchdogAlert, AlertSeverity, AlertKind, RobotHealthState
from .config import WatchdogConfig
from axis_os.hal.core.structured_log import (
    StructuredLogger, LogCategory, get_default_logger
)


class JointMonitorState:
    """
    Ce que le watchdog retient d'un joint entre deux cycles : ses
    limites, le dernier feedback reçu et son heure d'arrivée, les
    drapeaux des alertes déjà émises (pour ne pas répéter la même alerte
    à chaque cycle) et les compteurs de dépassements consécutifs.
    """
    __slots__ = (
        "joint_id", "limits",
        "last_feedback_time", "last_feedback",
        "temp_warning_active", "temp_critical_active",
        "comm_timeout_active", "comm_lost_active",
        "torque_over_count", "velocity_over_count",
        "drift_over_count", "position_breach_count",
    )

    def __init__(self, joint_id: str, limits: JointLimits):
        self.joint_id              = joint_id
        self.limits                = limits
        self.last_feedback_time    = time.monotonic()
        self.last_feedback         = None
        self.temp_warning_active   = False
        self.temp_critical_active  = False
        self.comm_timeout_active   = False
        self.comm_lost_active      = False
        # Compteurs de mesures consécutives hors limite (voir
        # confirm_samples dans config.py).
        self.torque_over_count     = 0
        self.velocity_over_count   = 0
        self.position_breach_count = 0   # position hors [pos_min, pos_max]
        self.drift_over_count      = 0


@dataclass
class EstopLogEntry:
    """
    Entrée du journal des e-stops (arrêts d'urgence) : raison, joint
    concerné, valeur mesurée, seuil franchi, et l'état de chaque joint au
    moment de l'arrêt (robot_state_snapshot). Ajouté à la suite d'une
    revue extérieure du watchdog. Chaque entrée est aussi écrite sur
    disque (voir _trigger_estop).
    """
    timestamp_ns: int
    reason:       str
    joint_id:     Optional[str]
    measured:     Optional[float]
    threshold:    Optional[float]
    robot_state_snapshot: Dict[str, str] = field(default_factory=dict)

    def __str__(self) -> str:
        joint = f" joint={self.joint_id}" if self.joint_id else ""
        val   = f" mesuré={self.measured:.2f}/seuil={self.threshold:.2f}" \
                if self.measured is not None else ""
        return f"[ESTOP LOG] {self.reason}{joint}{val}"


def _sd_notify_watchdog() -> None:
    """
    Envoie « WATCHDOG=1 » au chien de garde de systemd (le gestionnaire
    de services de Linux) par le protocole sd_notify : un datagramme
    (message isolé) sur le socket Unix (canal de communication local
    entre processus) dont l'adresse est donnée par la variable
    d'environnement NOTIFY_SOCKET. Ce protocole est documenté
    officiellement par systemd ; il est implémenté ici avec la seule
    bibliothèque standard, sans dépendre de python-systemd.

    Pourquoi : si _monitor_loop() se fige complètement (deadlock,
    c'est-à-dire des threads bloqués à s'attendre l'un l'autre, ou
    boucle infinie dans une dépendance tierce : tout ce qui échappe au
    try/except autour de _run_cycle()), rien dans le processus Python
    lui-même ne peut s'en apercevoir. Avec WatchdogSec=2s dans le
    fichier d'unité systemd (deploy/systemd/axis-os.service) et cette
    notification envoyée après chaque cycle réussi, c'est l'init système
    (systemd, extérieur à ce processus) qui constate l'absence de signal
    de vie et tue puis redémarre le service. Ajouté sur une
    recommandation extérieure, comme la mesure au meilleur rapport
    gain/effort de la liste proposée.

    Sans effet si NOTIFY_SOCKET n'est pas défini (développement, tests,
    lancement hors systemd, plateforme non compatible). Ne lève jamais
    d'exception : cet appel ne doit en aucun cas pouvoir perturber la
    boucle de surveillance elle-même.
    """
    addr = os.environ.get("NOTIFY_SOCKET")
    if not addr:
        return
    try:
        if addr.startswith("@"):
            addr = "\0" + addr[1:]  # socket abstrait Linux (« @ » = octet nul)
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.connect(addr)
            sock.sendall(b"WATCHDOG=1")
    except OSError:
        # Environnement inattendu (permissions, socket fermé...). Jamais
        # fatal : au pire, l'absence de notification fera redémarrer le
        # service par systemd ; cette fonction ne doit jamais faire
        # planter AXIS-OS lui-même.
        pass


class WatchdogSystem:
    """
    Surveillance temps réel d'un robot complet, à travers son IRobotHAL
    (l'interface que chaque robot fournit au HAL, la couche
    d'abstraction matérielle).

    Deux threads : wdg_monitor exécute un cycle de surveillance toutes
    les poll_interval_s secondes ; wdg_dispatch remet les alertes aux
    abonnés, chaque callback (fonction de rappel) étant exécuté dans un
    pool de 4 threads (wdg_cb).

    Usage :
        wd = WatchdogSystem(robot_hal, config=WatchdogConfig.strict())
        wd.subscribe(my_handler)
        wd.start()
        # ...
        print(wd.health_state)   # RobotHealthState.HEALTHY / WARNING / ...
        wd.stop()
    """

    def __init__(self, robot_hal: IRobotHAL, config: Optional[WatchdogConfig] = None,
                logger: Optional[StructuredLogger] = None):
        self._hal    = robot_hal
        self._config = config or WatchdogConfig.standard()
        # Journal structuré (StructuredLogger, qui conserve les
        # événements sur disque), optionnel. Même choix de conception
        # que pour MotionStateTracker : injecté plutôt qu'imposé, avec
        # une instance partagée par défaut si rien n'est fourni. Aucune
        # erreur d'écriture ne remonte jusqu'ici (voir StructuredLogger) :
        # le watchdog continue de fonctionner même si le disque est plein.
        self._struct_logger = logger or get_default_logger()

        self._joint_states: Dict[str, JointMonitorState] = {}
        self._running      = False
        self._start_time   = 0.0
        self._last_hb      = 0.0
        self._last_cycle_monotonic = 0.0

        # Verrou de l'e-stop, ajouté à la suite d'une revue extérieure du
        # watchdog : il garantit l'idempotence même si deux conditions
        # FATAL sont détectées dans la même microseconde sur deux joints
        # différents. Sans lui, une « race condition » (deux threads qui
        # modifient la même donnée en même temps) pouvait en théorie
        # laisser passer un double déclenchement incohérent.
        self._estop_lock   = threading.Lock()
        # _estop_fired n'est jamais remis à False automatiquement : la
        # justification de ce choix de sécurité est dans la propriété
        # estop_fired ci-dessous.
        self._estop_fired  = False
        self._estop_log: List[EstopLogEntry] = []

        self._thread:          Optional[threading.Thread] = None
        self._dispatch_thread: Optional[threading.Thread] = None
        self._executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="wdg_cb")
        self._subscribers: List[Callable[[WatchdogAlert], None]] = []
        # Tampon des 256 dernières alertes, lu par polling (interrogation
        # périodique) avec get_recent_alerts(). Pourquoi : une app tourne
        # dans une sandbox (un processus séparé, isolé du reste
        # d'AXIS-OS) et y appelle subscribe_alerts() à travers
        # HALBridgeClient, son accès au HAL depuis la sandbox. Une
        # fonction Python ne peut pas traverser la multiprocessing.Queue
        # qui relie les deux processus : elle n'est pas sérialisable (on
        # ne peut pas la convertir en octets pour l'envoyer). Le
        # processus enfant ne peut donc pas recevoir de callback en temps
        # réel. Ce tampon lui offre un
        # polling explicite, équivalent pour ce cas précis. Les abonnés
        # habituels (self._subscribers ci-dessus) ne s'en servent jamais :
        # les callbacks restent le mécanisme principal, sans délai de
        # polling.
        self._alert_buffer: List[WatchdogAlert] = []
        self._alert_buffer_lock = threading.Lock()
        self._alert_buffer_maxlen = 256

        # Deux files d'attente vers le thread de distribution. La file
        # normale (INFO, WARNING, CRITICAL) est bornée : si elle sature,
        # les nouvelles alertes sont abandonnées, perte jugée acceptable.
        # Les FATAL ont leur propre file, sans limite de taille, à la
        # suite d'une revue extérieure : une alerte FATAL ne doit JAMAIS
        # être perdue silencieusement.
        self._queue: queue.Queue[WatchdogAlert] = queue.Queue(maxsize=256)
        self._fatal_queue: queue.Queue[WatchdogAlert] = queue.Queue()  # illimitée

        # Sévérité la plus haute relevée au dernier cycle ; lue par
        # health_state.
        self._active_max_severity = AlertSeverity.INFO

        # Compteurs exposés par get_stats(). watchdog_degraded : vrai
        # tant que les cycles dépassent max_cycle_time_ms (voir
        # _monitor_loop).
        self._stats = {"cycles": 0, "alerts_total": 0,
                       "alerts_fatal": 0, "estop_count": 0,
                       "max_cycle_time_ms": 0.0,
                       "watchdog_degraded": False}

    # ── API publique ──────────────────────────────────────────────

    def subscribe(self, callback: Callable[[WatchdogAlert], None]) -> None:
        """
        Abonne callback à toutes les alertes. Il est appelé dans le pool
        de threads wdg_cb, jamais dans le thread de surveillance.
        """
        self._subscribers.append(callback)

    def get_recent_alerts(self, since_index: int = 0) -> List[WatchdogAlert]:
        """
        Retourne les alertes reçues depuis `since_index`. C'est un
        indice dans le tampon interne, pas un horodatage : deux alertes
        peuvent avoir le même timestamp_ns, jamais le même indice. Pour
        un polling continu, l'appelant garde l'indice suivant
        (`since_index + len(result)`) et le repasse à l'appel d'après,
        afin de ne jamais retraiter une alerte déjà vue.

        Le principal utilisateur est HALBridgeClient.subscribe_alerts()
        (axis_os/store/runtime/hal_bridge.py) : ce polling y remplace un
        vrai callback, impossible à travers la frontière entre deux
        processus.

        Limite connue : le tampon ne garde que les 256 dernières alertes
        (_alert_buffer_maxlen). Une fois plein, chaque nouvelle alerte
        retire la plus ancienne et les indices se décalent : un appelant
        en retard en manque, et un appelant arrivé à l'indice 256 ne
        reçoit plus rien.
        """
        with self._alert_buffer_lock:
            return list(self._alert_buffer[since_index:])

    @property
    def health_state(self) -> RobotHealthState:
        """
        État de santé global du robot (voir RobotHealthState), ajouté à
        la suite d'une revue extérieure du watchdog.

        Ordre de priorité : un e-stop déclenché donne ESTOP ; sinon, la
        sévérité la plus haute du dernier cycle décide (CRITICAL donne
        DEGRADED, WARNING donne WARNING, sinon HEALTHY).

        Limite connue : une sévérité FATAL sans e-stop (possible quand
        l'e-stop automatique est désactivé, comme dans le profil
        simulation) ne correspond à aucune de ces branches et donne
        HEALTHY.
        """
        if self.estop_fired:
            return RobotHealthState.ESTOP
        if self._active_max_severity == AlertSeverity.CRITICAL:
            return RobotHealthState.DEGRADED
        if self._active_max_severity == AlertSeverity.WARNING:
            return RobotHealthState.WARNING
        return RobotHealthState.HEALTHY

    @property
    def estop_log(self) -> List[EstopLogEntry]:
        """
        Copie du journal des e-stops déclenchés par ce watchdog depuis le
        démarrage du processus. Ce journal vit en mémoire ; la trace
        durable est écrite sur disque (voir _trigger_estop).
        """
        return list(self._estop_log)

    def start(self) -> None:
        """
        Démarre la surveillance : prépare l'état de chaque joint déclaré
        par le HAL, lance le thread de distribution puis celui de
        surveillance, et émet WATCHDOG_STARTED. Sans effet si la
        surveillance tourne déjà.
        """
        if self._running:
            return
        self._build_joint_states()
        self._running    = True
        self._start_time = time.monotonic()
        self._last_hb    = self._start_time

        self._dispatch_thread = threading.Thread(
            target=self._dispatch_loop, name="wdg_dispatch", daemon=True)
        self._dispatch_thread.start()

        self._thread = threading.Thread(
            target=self._monitor_loop, name="wdg_monitor", daemon=True)
        self._thread.start()

        self._emit(WatchdogAlert(
            kind=AlertKind.WATCHDOG_STARTED, severity=AlertSeverity.INFO,
            joint_id=None,
            message=(f"WatchdogSystem démarré | robot={self._hal.robot_id} | "
                     f"profil={self._config.profile_name} | "
                     f"joints={len(self._joint_states)}"),
            value=None, threshold=None
        ))

    def stop(self) -> None:
        """
        Arrête la surveillance : attend la fin du thread de surveillance
        (2 s au plus), dépose WATCHDOG_STOPPED pour les abonnés, puis
        arrête le thread de distribution (le None placé dans la file est
        son signal de fin). WATCHDOG_STOPPED va directement dans la file
        normale, sans passer par _emit() : il est perdu si cette file est
        pleine et n'apparaît pas dans get_recent_alerts().
        """
        if not self._running:
            return
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

        stopped = WatchdogAlert(
            kind=AlertKind.WATCHDOG_STOPPED, severity=AlertSeverity.INFO,
            joint_id=None,
            message=f"WatchdogSystem arrêté | cycles={self._stats['cycles']}",
            value=None, threshold=None
        )
        self._stats["alerts_total"] += 1
        try: self._queue.put_nowait(stopped)
        except queue.Full: pass
        time.sleep(0.05)

        self._queue.put(None)
        if self._dispatch_thread:
            self._dispatch_thread.join(timeout=2.0)

    def get_stats(self) -> Dict:
        """
        Compteurs de fonctionnement (cycles, alertes, e-stops, durée de
        cycle maximale, état dégradé), complétés par la durée de
        fonctionnement, le robot, le profil, le nombre de joints
        surveillés et l'état de santé.
        """
        uptime = time.monotonic() - self._start_time if self._start_time else 0
        return {**self._stats, "uptime_s": round(uptime, 2),
                "robot_id": self._hal.robot_id,
                "profile": self._config.profile_name,
                "joints_monitored": len(self._joint_states),
                "health_state": self.health_state.name}

    @property
    def is_running(self) -> bool:
        """True entre start() et stop()."""
        return self._running

    @property
    def estop_fired(self) -> bool:
        """
        True dès qu'un e-stop (arrêt d'urgence) a été déclenché par ce
        watchdog. Ce drapeau n'est JAMAIS remis à False automatiquement,
        par aucun mécanisme du watchdog lui-même.

        C'est un choix de sécurité explicite, documenté à la suite d'un
        audit externe (voir CHANGELOG.md, « audit externe, vague 3 ») :
        un e-stop signale qu'une
        condition jugée dangereuse a été détectée (surcharge,
        communication perdue, joint en défaut...). Un réarmement
        automatique, même une fois la condition disparue, risquerait de
        relancer le mouvement d'un robot dont l'état physique réel n'a
        jamais été vérifié par un humain : la cause de l'e-stop peut
        avoir disparu des mesures sans que le problème physique
        sous-jacent (jeu mécanique, capteur endommagé) soit réellement
        résolu. Le seul recours actuel est de redémarrer le processus,
        ce qui impose une réinitialisation complète plutôt qu'un simple
        effacement de drapeau. Un réarmement piloté et audité (par
        exemple, une confirmation humaine explicite avant la reprise)
        reste un chantier futur légitime, mais ne doit jamais devenir un
        comportement implicite ou automatique.

        Un e-stop déclenché ailleurs (par exemple par le Supervisor) ne
        passe pas par ce drapeau : il est visible dans
        IRobotHAL.estop_active.
        """
        with self._estop_lock:
            return self._estop_fired

    @property
    def last_cycle_monotonic(self) -> float:
        """
        Horodatage time.monotonic() (une horloge qui ne recule jamais,
        même si l'heure système change) du dernier cycle de surveillance
        terminé avec succès. Il permet à un Supervisor EXTERNE de
        vérifier que ce watchdog est toujours en vie, sans dépendre d'un
        mécanisme interne au watchdog lui-même.

        Principe : un watchdog ne peut pas se surveiller lui-même de façon
        fiable s'il cesse complètement de fonctionner (thread bloqué par
        un deadlock, c'est-à-dire des threads qui s'attendent l'un
        l'autre, ou plantage silencieux d'un composant sous-jacent). Le
        try/except de _monitor_loop protège contre les exceptions levées
        pendant un cycle, mais pas contre un thread qui cesse tout
        simplement d'avancer. Voir
        Supervisor dans axis_os/hal/supervisor/supervisor.py.

        Vaut 0.0 avant le premier cycle (avant start(), ou juste après,
        tant que _run_cycle() n'a pas encore tourné une fois). Un
        Supervisor qui compare cette valeur à time.monotonic() voit alors
        un écart énorme et le traite correctement comme « pas encore de
        signal de vie », sans cas particulier à gérer.
        """
        return self._last_cycle_monotonic

    # ── Boucle principale ────────────────────────────────────────

    def _monitor_loop(self) -> None:
        """
        Boucle du thread wdg_monitor : un cycle de surveillance toutes
        les poll_interval_s secondes tant que la surveillance est active,
        avec mesure de la durée de chaque cycle.
        """
        while self._running:
            t0 = time.monotonic()
            try:
                self._run_cycle()
            except Exception as exc:
                # Ici, c'est le watchdog lui-même qui a planté (bug
                # interne), pas le robot. L'alerte réutilise le type
                # JOINT_FAULT, mais en CRITICAL, sans joint, et avec un
                # message préfixé [BUG INTERNE WATCHDOG] : on ne doit plus
                # la confondre avec une anomalie du robot lui-même.
                self._emit(WatchdogAlert(
                    kind=AlertKind.JOINT_FAULT, severity=AlertSeverity.CRITICAL,
                    joint_id=None,
                    message=f"[BUG INTERNE WATCHDOG] {type(exc).__name__}: {exc}",
                    value=None, threshold=None
                ))
            else:
                # Signal de vie pour le chien de garde de systemd (voir
                # _sd_notify_watchdog()), ajouté sur une recommandation
                # extérieure. Si _monitor_loop() se fige (deadlock, boucle
                # infinie dans une dépendance tierce, tout bug qui échappe
                # au try/except ci-dessus), rien dans le processus Python
                # lui-même ne peut le détecter : c'est par nature le rôle
                # de l'init système (systemd), pas de l'application. Le
                # signal n'est envoyé qu'après un cycle TERMINÉ (dans le
                # else du try, donc jamais si _run_cycle() a levé une
                # exception) : un signal envoyé même en cas de blocage
                # réel viderait cette protection de son sens. Le réglage
                # côté systemd (WatchdogSec=2s) est dans
                # deploy/systemd/axis-os.service ; sans lui, cet appel ne
                # fait rien.
                _sd_notify_watchdog()
            elapsed = time.monotonic() - t0
            ms = elapsed * 1000
            if ms > self._stats["max_cycle_time_ms"]:
                self._stats["max_cycle_time_ms"] = ms

            # Le watchdog détecte son propre ralentissement (demandé par
            # une revue extérieure) : un watchdog trop lent pour
            # surveiller correctement le robot est lui-même un problème de
            # sécurité. L'alerte n'est émise qu'une fois par épisode de
            # lenteur ; watchdog_degraded repasse à False dès qu'un cycle
            # redevient assez rapide.
            if ms > self._config.max_cycle_time_ms and not self._stats["watchdog_degraded"]:
                self._stats["watchdog_degraded"] = True
                self._emit(WatchdogAlert(
                    kind=AlertKind.JOINT_FAULT, severity=AlertSeverity.CRITICAL,
                    joint_id=None,
                    message=(f"Watchdog dégradé : cycle={ms:.1f}ms "
                             f"> seuil={self._config.max_cycle_time_ms:.0f}ms — "
                             f"la surveillance elle-même n'est plus fiable"),
                    value=ms, threshold=self._config.max_cycle_time_ms
                ))
            elif ms <= self._config.max_cycle_time_ms:
                self._stats["watchdog_degraded"] = False

            # Attend le reste de la période ; pas d'attente si le cycle a
            # déjà duré plus que poll_interval_s.
            sleep = self._config.poll_interval_s - elapsed
            if sleep > 0:
                time.sleep(sleep)

    def _run_cycle(self) -> None:
        """
        Un cycle de surveillance : lit le feedback de tous les joints
        auprès du HAL, applique chaque contrôle à chaque joint, retient la
        sévérité la plus haute pour health_state et émet un HEARTBEAT
        (signal de vie) toutes les heartbeat_interval_s secondes. Pour un
        joint qui n'a encore jamais envoyé de feedback, seul le contrôle
        de communication s'applique.
        """
        now = time.monotonic()
        self._stats["cycles"] += 1
        feedbacks = self._hal.get_full_state()

        for jid, fb in feedbacks.items():
            if jid in self._joint_states:
                self._joint_states[jid].last_feedback_time = now
                self._joint_states[jid].last_feedback      = fb

        max_severity_this_cycle = AlertSeverity.INFO
        for jid, jstate in self._joint_states.items():
            sev = self._check_comm(jstate, now)
            max_severity_this_cycle = max(max_severity_this_cycle, sev)
            fb = jstate.last_feedback
            if fb is None: continue
            sev = self._check_joint_state(fb)
            max_severity_this_cycle = max(max_severity_this_cycle, sev)
            sev = self._check_temperature(fb, jstate)
            max_severity_this_cycle = max(max_severity_this_cycle, sev)
            sev = self._check_mechanical(fb, jstate)
            max_severity_this_cycle = max(max_severity_this_cycle, sev)
            sev = self._check_feedback_drift(jid, jstate)
            max_severity_this_cycle = max(max_severity_this_cycle, sev)

        self._active_max_severity = max_severity_this_cycle

        if now - self._last_hb >= self._config.heartbeat_interval_s:
            self._last_hb = now
            self._emit(WatchdogAlert(
                kind=AlertKind.HEARTBEAT, severity=AlertSeverity.INFO,
                joint_id=None,
                message=(f"Heartbeat | cycles={self._stats['cycles']} | "
                         f"alertes={self._stats['alerts_total']} | "
                         f"health={self.health_state.name}"),
                value=None, threshold=None
            ))

        # Mis à jour seulement si tout le cycle ci-dessus s'est terminé
        # sans exception : un Supervisor externe qui lit cette valeur ne
        # doit voir un signal de vie récent QUE si la surveillance a
        # réellement eu lieu. Si _run_cycle() est interrompue en cours de
        # route par une exception, le try/except de _monitor_loop laisse
        # volontairement cette ligne non atteinte.
        self._last_cycle_monotonic = now

    def _check_comm(self, jstate: JointMonitorState, now: float) -> AlertSeverity:
        """
        Contrôle de communication : temps écoulé depuis le dernier
        feedback de ce joint. Au-delà de comm_timeout_s, alerte WARNING ;
        au-delà de comm_lost_s, alerte FATAL et e-stop (si
        auto_estop_on_comm_lost). Chaque alerte n'est émise qu'une fois
        par épisode ; les drapeaux se réarment quand le feedback revient.
        """
        elapsed = now - jstate.last_feedback_time
        if elapsed >= self._config.comm_lost_s:
            if not jstate.comm_lost_active:
                jstate.comm_lost_active = True
                self._emit(WatchdogAlert(
                    kind=AlertKind.COMM_LOST, severity=AlertSeverity.FATAL,
                    joint_id=jstate.joint_id,
                    message=f"Comm perdue depuis {elapsed*1000:.0f}ms",
                    value=elapsed*1000, threshold=self._config.comm_lost_s*1000
                ))
                if self._config.auto_estop_on_comm_lost:
                    self._trigger_estop(f"COMM_LOST:{jstate.joint_id}",
                                        joint_id=jstate.joint_id,
                                        measured=elapsed*1000,
                                        threshold=self._config.comm_lost_s*1000)
            return AlertSeverity.FATAL
        if elapsed >= self._config.comm_timeout_s:
            if not jstate.comm_timeout_active:
                jstate.comm_timeout_active = True
                self._emit(WatchdogAlert(
                    kind=AlertKind.COMM_TIMEOUT, severity=AlertSeverity.WARNING,
                    joint_id=jstate.joint_id,
                    message=f"Timeout {elapsed*1000:.0f}ms",
                    value=elapsed*1000, threshold=self._config.comm_timeout_s*1000
                ))
            return AlertSeverity.WARNING
        jstate.comm_timeout_active = False
        jstate.comm_lost_active    = False
        return AlertSeverity.INFO

    def _check_joint_state(self, fb: ActuatorFeedback) -> AlertSeverity:
        """
        Contrôle de l'état signalé par le moteur lui-même : FAULT (défaut)
        donne une alerte FATAL et l'e-stop (si auto_estop_on_joint_fault).
        Contrairement aux autres contrôles, l'alerte est réémise à chaque
        cycle tant que le défaut dure.
        """
        if fb.state == ActuatorState.FAULT:
            self._emit(WatchdogAlert(
                kind=AlertKind.JOINT_FAULT, severity=AlertSeverity.FATAL,
                joint_id=fb.joint_id,
                message=f"Joint FAULT (code={fb.error_code})",
                value=float(fb.error_code), threshold=0.0
            ))
            if self._config.auto_estop_on_joint_fault:
                self._trigger_estop(f"JOINT_FAULT:{fb.joint_id}",
                                    joint_id=fb.joint_id,
                                    measured=float(fb.error_code), threshold=0.0)
            return AlertSeverity.FATAL
        return AlertSeverity.INFO

    def _check_temperature(self, fb: ActuatorFeedback,
                           jstate: JointMonitorState) -> AlertSeverity:
        """
        Contrôle de température moteur : WARNING à partir de
        temp_warning_c, FATAL et e-stop (si auto_estop_on_temp_critical)
        à partir de temp_critical_c. Chaque alerte n'est émise qu'une fois
        par épisode ; les drapeaux se réarment quand la température
        repasse sous temp_warning_c.
        """
        temp = fb.temperature
        if temp >= self._config.temp_critical_c:
            if not jstate.temp_critical_active:
                jstate.temp_critical_active = True
                self._emit(WatchdogAlert(
                    kind=AlertKind.TEMPERATURE_CRITICAL, severity=AlertSeverity.FATAL,
                    joint_id=fb.joint_id,
                    message=f"Température critique : {temp:.1f}°C",
                    value=temp, threshold=self._config.temp_critical_c
                ))
                if self._config.auto_estop_on_temp_critical:
                    self._trigger_estop(f"TEMP_CRITICAL:{fb.joint_id}",
                                        joint_id=fb.joint_id,
                                        measured=temp,
                                        threshold=self._config.temp_critical_c)
            return AlertSeverity.FATAL
        if temp >= self._config.temp_warning_c:
            if not jstate.temp_warning_active:
                jstate.temp_warning_active = True
                self._emit(WatchdogAlert(
                    kind=AlertKind.TEMPERATURE_WARNING, severity=AlertSeverity.WARNING,
                    joint_id=fb.joint_id,
                    message=f"Température élevée : {temp:.1f}°C",
                    value=temp, threshold=self._config.temp_warning_c
                ))
            return AlertSeverity.WARNING
        jstate.temp_warning_active  = False
        jstate.temp_critical_active = False
        return AlertSeverity.INFO

    def _check_mechanical(self, fb: ActuatorFeedback,
                          jstate: JointMonitorState) -> AlertSeverity:
        """
        Contrôles mécaniques : couple, vitesse et position mesurés,
        comparés aux limites du joint (JointLimits).

        Une alerte CRITICAL n'est émise qu'après confirm_samples
        dépassements consécutifs, pour éviter les faux positifs : depuis
        une revue extérieure du watchdog, un seul pic de couple ou de
        vitesse ne déclenche plus d'alerte. Une fois la confirmation
        atteinte, l'alerte est réémise à chaque cycle tant que le
        dépassement dure ; une mesure revenue dans les limites remet le
        compteur à zéro. Ces alertes ne déclenchent pas d'e-stop par
        elles-mêmes. Une limite torque_max ou vel_max nulle ou négative
        désactive le contrôle correspondant.
        """
        lim = jstate.limits
        severity = AlertSeverity.INFO

        if lim.torque_max > 0 and abs(fb.torque) > lim.torque_max * self._config.torque_overload_ratio:
            jstate.torque_over_count += 1
            if jstate.torque_over_count >= self._config.confirm_samples:
                self._emit(WatchdogAlert(
                    kind=AlertKind.TORQUE_OVERLOAD, severity=AlertSeverity.CRITICAL,
                    joint_id=fb.joint_id,
                    message=(f"Surcharge couple confirmée ({jstate.torque_over_count} "
                             f"échantillons) : {abs(fb.torque):.1f} Nm"),
                    value=abs(fb.torque), threshold=lim.torque_max * self._config.torque_overload_ratio
                ))
                severity = AlertSeverity.CRITICAL
        else:
            jstate.torque_over_count = 0

        if lim.vel_max > 0 and abs(fb.velocity) > lim.vel_max * self._config.velocity_overrun_ratio:
            jstate.velocity_over_count += 1
            if jstate.velocity_over_count >= self._config.confirm_samples:
                self._emit(WatchdogAlert(
                    kind=AlertKind.VELOCITY_OVERRUN, severity=AlertSeverity.CRITICAL,
                    joint_id=fb.joint_id,
                    message=(f"Dépassement vitesse confirmé ({jstate.velocity_over_count} "
                             f"échantillons) : {abs(fb.velocity):.2f} rad/s"),
                    value=abs(fb.velocity), threshold=lim.vel_max * self._config.velocity_overrun_ratio
                ))
                severity = AlertSeverity.CRITICAL
        else:
            jstate.velocity_over_count = 0

        # Position mesurée hors des limites physiques du joint. Contrôle
        # ajouté à la suite d'un audit de sécurité externe (voir
        # CHANGELOG.md, « audit externe, vague 2 ») : rien ne surveillait
        # ce cas auparavant. À ne pas
        # confondre avec POSITION_DRIFT, qui compare la commande au
        # feedback (« le moteur suit-il sa consigne ? ») : ici, on compare
        # la position RÉELLEMENT mesurée à pos_min/pos_max, sans aucun
        # rapport avec une commande envoyée. Causes possibles : butée
        # mécanique franchie, encodeur (le capteur de position du moteur)
        # qui dérive. Une position non finie (NaN, « pas un nombre », ou
        # infinie) ne compte pas comme un dépassement et remet le
        # compteur à zéro.
        if math.isfinite(fb.position) and not (lim.pos_min <= fb.position <= lim.pos_max):
            jstate.position_breach_count += 1
            if jstate.position_breach_count >= self._config.confirm_samples:
                self._emit(WatchdogAlert(
                    kind=AlertKind.POSITION_LIMIT_BREACH, severity=AlertSeverity.CRITICAL,
                    joint_id=fb.joint_id,
                    message=(f"Position mesurée hors limites confirmée "
                             f"({jstate.position_breach_count} échantillons) : "
                             f"{fb.position:.3f} rad (limites [{lim.pos_min:.3f}, "
                             f"{lim.pos_max:.3f}]) — butée mécanique ou dérive capteur ?"),
                    value=fb.position, threshold=(lim.pos_min if fb.position < lim.pos_min else lim.pos_max),
                ))
                severity = AlertSeverity.CRITICAL
        else:
            jstate.position_breach_count = 0

        return severity

    def _check_feedback_drift(self, joint_id: str,
                              jstate: JointMonitorState) -> AlertSeverity:
        """
        Détecte un écart entre la dernière commande de position envoyée
        et la position réellement mesurée : signe qu'un moteur ne suit
        plus sa consigne (moteur bloqué, calibration qui a dérivé). Une
        revue extérieure du watchdog avait relevé ce manque ; il est
        resté ouvert faute d'historique des commandes, puis a été comblé
        quand MotionStateTracker (le composant qui garde, pour chaque
        joint, la dernière commande et le dernier feedback) a été branché
        dans UnitreeG1HAL (le HAL du robot Unitree G1).

        Alerte CRITICAL après confirm_samples dépassements consécutifs de
        position_drift_max_rad, sans e-stop automatique. Aucun contrôle
        quand l'écart n'est pas calculable (valeur absente ou non finie).

        Sans effet si le HAL n'expose pas de tracker (get_motion_tracker()
        retourne None par défaut dans IRobotHAL) : rien ne change pour un
        backend (le pilote d'un robot ou d'un simulateur) qui n'a pas
        encore ce composant.
        """
        tracker = self._hal.get_motion_tracker()
        if tracker is None:
            return AlertSeverity.INFO

        state = tracker.get(joint_id)
        if state is None:
            return AlertSeverity.INFO

        error = state.compute_feedback_error()
        if error is None or not math.isfinite(error):
            return AlertSeverity.INFO

        threshold = self._config.position_drift_max_rad
        if abs(error) > threshold:
            jstate.drift_over_count += 1
            if jstate.drift_over_count >= self._config.confirm_samples:
                self._emit(WatchdogAlert(
                    kind=AlertKind.POSITION_DRIFT, severity=AlertSeverity.CRITICAL,
                    joint_id=joint_id,
                    message=(f"Écart commande/feedback confirmé "
                             f"({jstate.drift_over_count} échantillons) : "
                             f"{abs(error):.3f} rad — moteur ne suit plus sa consigne ?"),
                    value=abs(error), threshold=threshold,
                ))
                return AlertSeverity.CRITICAL
        else:
            jstate.drift_over_count = 0

        return AlertSeverity.INFO

    # ── E-STOP ────────────────────────────────────────────────────

    def _trigger_estop(self, reason: str, joint_id: Optional[str] = None,
                       measured: Optional[float] = None,
                       threshold: Optional[float] = None) -> None:
        """
        Déclenche l'e-stop global : arrêt d'urgence de tous les moteurs
        par le HAL, entrée dans le journal (en mémoire et sur disque),
        puis alerte ESTOP_TRIGGERED.

        Idempotent (seul le premier appel agit) et sûr entre threads, à
        la suite d'une revue extérieure du watchdog : le verrou garantit
        qu'un seul appelant déclenche réellement l'e-stop, même si
        plusieurs conditions FATAL sont détectées en même temps sur des
        joints différents pendant le même cycle.
        """
        with self._estop_lock:
            if self._estop_fired:
                return
            self._estop_fired = True

        # emergency_stop_all() est appelé hors du verrou, pour ne jamais
        # bloquer la détection d'autres conditions pendant que le HAL
        # exécute l'arrêt.
        self._stats["estop_count"] += 1
        self._hal.emergency_stop_all()

        entry = EstopLogEntry(
            timestamp_ns=time.time_ns(), reason=reason, joint_id=joint_id,
            measured=measured, threshold=threshold,
            robot_state_snapshot={
                jid: js.last_feedback.state.name if js.last_feedback else "UNKNOWN"
                for jid, js in self._joint_states.items()
            }
        )
        self._estop_log.append(entry)
        print(f"  {entry}")

        # Écriture sur disque : c'est l'événement qu'on peut le moins se
        # permettre de perdre. La liste self._estop_log, en mémoire, ne
        # survit pas à un redémarrage du processus ; sans cette écriture,
        # un e-stop survenu il y a une semaine sur un robot resté allumé
        # ensuite ne laisserait plus aucune trace après le prochain
        # redémarrage.
        self._struct_logger.log(LogCategory.ESTOP, entry, robot_id=self._hal.robot_id)

        self._emit(WatchdogAlert(
            kind=AlertKind.ESTOP_TRIGGERED, severity=AlertSeverity.FATAL,
            joint_id=joint_id, message=f"E-STOP GLOBAL déclenché | raison={reason}",
            value=measured, threshold=threshold
        ))

    # ── Système d'événements ──────────────────────────────────────

    def _emit(self, alert: WatchdogAlert) -> None:
        """
        Publie une alerte : l'ajoute au tampon de polling, met à jour les
        compteurs, puis la dépose dans la file FATAL ou dans la file
        normale. Les alertes FATAL sont en plus écrites sur disque.
        """
        # Tampon de polling (voir get_recent_alerts() et l'explication
        # dans __init__). Rempli ici, avant tout passage par l'une des
        # deux files, pour qu'AUCUNE alerte n'échappe au polling : ni une
        # FATAL, qui suit un chemin séparé plus bas, ni une alerte
        # abandonnée par la file normale saturée (voir plus bas).
        with self._alert_buffer_lock:
            self._alert_buffer.append(alert)
            if len(self._alert_buffer) > self._alert_buffer_maxlen:
                self._alert_buffer.pop(0)

        self._stats["alerts_total"] += 1
        if alert.severity == AlertSeverity.FATAL:
            self._stats["alerts_fatal"] += 1
            # Écriture sur disque des alertes FATAL, pour la même raison
            # que l'e-stop : elles doivent rester consultables après coup
            # sans dépendre de la seule mémoire du processus.
            self._struct_logger.log(
                LogCategory.WATCHDOG_ALERT, alert, robot_id=self._hal.robot_id
            )
            # Les FATAL passent par leur file sans limite de taille
            # (demandé par une revue extérieure) : elles ne sont jamais
            # abandonnées, même quand la file normale sature.
            self._fatal_queue.put(alert)
            return
        try:
            self._queue.put_nowait(alert)
        except queue.Full:
            pass  # perte acceptée, seulement pour INFO/WARNING/CRITICAL

    def _dispatch_loop(self) -> None:
        """
        Boucle du thread wdg_dispatch : remet chaque alerte à tous les
        abonnés, chaque appel étant confié au pool de threads wdg_cb.
        S'arrête sur le None déposé par stop(), ou quand la surveillance
        est arrêtée et qu'il ne reste plus de FATAL à distribuer. Une
        erreur à la soumission d'un callback est ignorée et la
        distribution continue.
        """
        while True:
            # Les FATAL en attente sont toujours distribuées en premier.
            while not self._fatal_queue.empty():
                alert = self._fatal_queue.get()
                for cb in self._subscribers:
                    try: self._executor.submit(cb, alert)
                    except Exception: pass

            try:
                alert = self._queue.get(timeout=0.1)
                if alert is None:
                    break
                for cb in self._subscribers:
                    try: self._executor.submit(cb, alert)
                    except Exception: pass
            except queue.Empty:
                if not self._running and self._fatal_queue.empty():
                    break

    def _build_joint_states(self) -> None:
        """
        Crée l'état de surveillance de chaque joint déclaré par le HAL,
        avec ses limites (JointLimits).
        """
        for jdef in self._hal.get_joint_definitions():
            self._joint_states[jdef.joint_id] = JointMonitorState(
                joint_id=jdef.joint_id, limits=jdef.limits
            )
