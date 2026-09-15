"""
AXIS-OS — WatchdogSystem v2
=============================
Thread temps-réel qui surveille en permanence tous les joints.
Déclenche l'e-stop automatique si seuil FATAL franchi.

Corrections v2 suite à revue externe :
  - E-STOP protégé par Lock (idempotent garanti même si 2 conditions
    FATAL arrivent en concurrence depuis des joints différents)
  - RobotHealthState global dérivé des alertes actives
  - Confirmation multi-échantillons avant CRITICAL (anti faux-positif)
  - Auto-surveillance : le watchdog détecte son propre ralentissement
  - Alertes FATAL jamais perdues (queue dédiée illimitée)
  - Journal structuré de chaque E-STOP (raison, joint, valeurs, seuil)

Licence : Propriétaire — AXIS-OS
"""

from __future__ import annotations
import threading
import time
import queue
import math
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
        self.torque_over_count     = 0   # échantillons consécutifs (#12)
        self.velocity_over_count   = 0
        self.position_breach_count = 0   # M10 — audit de sécurité
        self.drift_over_count      = 0


@dataclass
class EstopLogEntry:
    """Journal structuré d'un E-STOP — point #19 revue externe."""
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


class WatchdogSystem:
    """
    Surveillance temps-réel pour un IRobotHAL complet.

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
        # Logger structuré, optionnel — décision de conception cohérente
        # avec MotionStateTracker : injecté plutôt que forcé, avec une
        # instance partagée par défaut si rien n'est fourni. Aucune
        # panne d'écriture ne remonte jusqu'ici (voir StructuredLogger),
        # le Watchdog continue de fonctionner même si le disque est plein.
        self._struct_logger = logger or get_default_logger()

        self._joint_states: Dict[str, JointMonitorState] = {}
        self._running      = False
        self._start_time   = 0.0
        self._last_hb      = 0.0
        self._last_cycle_monotonic = 0.0

        # E-STOP protégé par Lock — corrige le point #8 (idempotence garantie
        # même si deux conditions FATAL sont détectées la même microseconde
        # sur deux joints différents ; sans Lock, une race condition pouvait
        # théoriquement laisser passer un double-trigger incohérent).
        self._estop_lock   = threading.Lock()
        # _estop_fired n'est JAMAIS remis à False automatiquement — voir
        # sa propriété estop_fired ci-dessous (M9, audit de sécurité)
        # pour la justification complète de ce choix de conception.
        self._estop_fired  = False
        self._estop_log: List[EstopLogEntry] = []

        self._thread:          Optional[threading.Thread] = None
        self._dispatch_thread: Optional[threading.Thread] = None
        self._executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="wdg_cb")
        self._subscribers: List[Callable[[WatchdogAlert], None]] = []

        # Queue normale bornée (INFO/WARNING/CRITICAL) — perte tolérée en cas
        # de saturation. Queue FATAL séparée, illimitée : point #18, une
        # alerte FATAL ne doit JAMAIS être perdue silencieusement.
        self._queue: queue.Queue[WatchdogAlert] = queue.Queue(maxsize=256)
        self._fatal_queue: queue.Queue[WatchdogAlert] = queue.Queue()  # illimitée

        self._active_max_severity = AlertSeverity.INFO  # pour health_state

        self._stats = {"cycles": 0, "alerts_total": 0,
                       "alerts_fatal": 0, "estop_count": 0,
                       "max_cycle_time_ms": 0.0,
                       "watchdog_degraded": False}  # #14

    # ── API publique ──────────────────────────────────────────────

    def subscribe(self, callback: Callable[[WatchdogAlert], None]) -> None:
        self._subscribers.append(callback)

    @property
    def health_state(self) -> RobotHealthState:
        """
        État global du robot — point #10.
        Dérivé de : e-stop actif > sévérité max des alertes en cours.
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
        return list(self._estop_log)

    def start(self) -> None:
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
        uptime = time.monotonic() - self._start_time if self._start_time else 0
        return {**self._stats, "uptime_s": round(uptime, 2),
                "robot_id": self._hal.robot_id,
                "profile": self._config.profile_name,
                "joints_monitored": len(self._joint_states),
                "health_state": self.health_state.name}

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def estop_fired(self) -> bool:
        """
        True dès qu'un e-stop a été déclenché — JAMAIS remis à False
        automatiquement, par aucun mécanisme du Watchdog lui-même.
        Choix de sécurité explicite (documenté suite à un audit
        externe, M9) : un e-stop signale qu'une condition jugée
        dangereuse a été détectée (surcharge, communication perdue,
        joint FAULT...). Permettre un réarmement automatique, même
        après que la condition déclenchante ait disparu, risquerait
        de relancer le mouvement sur un robot dont l'état physique
        réel n'a jamais été vérifié par un humain — la cause de
        l'e-stop peut avoir disparu du feedback sans que le problème
        physique sous-jacent (jeu mécanique, capteur endommagé) soit
        réellement résolu. Le seul recours actuel est le redémarrage
        du process, ce qui force explicitement une réinitialisation
        complète plutôt qu'un simple effacement de drapeau. Un
        réarmement piloté et audité (ex: confirmation humaine explicite
        avant reprise) reste un chantier futur légitime, mais ne doit
        jamais devenir un comportement implicite ou automatique.
        """
        with self._estop_lock:
            return self._estop_fired

    @property
    def last_cycle_monotonic(self) -> float:
        """
        Timestamp monotonic (time.monotonic()) du dernier cycle de
        surveillance complété avec succès — ajouté pour permettre à un
        Supervisor EXTERNE de vérifier que ce Watchdog est toujours en
        vie, sans dépendre d'un mécanisme interne au Watchdog lui-même.

        Principe : un Watchdog ne peut pas fiablement se surveiller
        lui-même s'il meurt complètement (thread bloqué en deadlock,
        crash silencieux d'un composant sous-jacent) — le try/except
        de _monitor_loop protège contre les exceptions métier, mais pas
        contre un thread qui cesse purement et simplement de progresser.
        Voir Supervisor dans axis_os/hal/supervisor/supervisor.py.

        Retourne 0.0 avant le premier cycle (avant start(), ou start()
        appelé mais _run_cycle() pas encore exécuté une fois) — un
        Supervisor qui compare ça à time.monotonic() verra un écart
        énorme et le traitera correctement comme "pas encore de
        heartbeat", sans cas particulier à gérer.
        """
        return self._last_cycle_monotonic

    # ── Boucle principale ────────────────────────────────────────

    def _monitor_loop(self) -> None:
        while self._running:
            t0 = time.monotonic()
            try:
                self._run_cycle()
            except Exception as exc:
                # Erreur RÉELLE du watchdog (bug interne), distincte d'un
                # JOINT_FAULT métier — on la nomme clairement pour ne plus
                # la confondre avec une anomalie du robot lui-même.
                self._emit(WatchdogAlert(
                    kind=AlertKind.JOINT_FAULT, severity=AlertSeverity.CRITICAL,
                    joint_id=None,
                    message=f"[BUG INTERNE WATCHDOG] {type(exc).__name__}: {exc}",
                    value=None, threshold=None
                ))
            elapsed = time.monotonic() - t0
            ms = elapsed * 1000
            if ms > self._stats["max_cycle_time_ms"]:
                self._stats["max_cycle_time_ms"] = ms

            # Point #14 : le watchdog détecte son propre ralentissement.
            # Un watchdog trop lent pour surveiller correctement le robot
            # est lui-même une condition de sécurité.
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

            sleep = self._config.poll_interval_s - elapsed
            if sleep > 0:
                time.sleep(sleep)

    def _run_cycle(self) -> None:
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

        # Mis à jour uniquement si tout le cycle ci-dessus s'est terminé
        # sans exception — un Supervisor externe lisant cette valeur ne
        # doit voir un heartbeat frais QUE si la surveillance a réellement
        # eu lieu, jamais si _run_cycle() a été interrompue en cours de
        # route par une exception (voir le try/except dans _monitor_loop,
        # qui laisse volontairement cette ligne non atteinte dans ce cas).
        self._last_cycle_monotonic = now

    def _check_comm(self, jstate: JointMonitorState, now: float) -> AlertSeverity:
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
        Point #12 : confirmation multi-échantillons avant CRITICAL.
        Un seul pic de couple/vitesse ne déclenche plus d'alerte —
        il faut confirm_samples dépassements consécutifs.
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

        # Position hors des vraies limites physiques — correctif M10,
        # audit de sécurité. Distinct de POSITION_DRIFT (qui compare
        # commande vs feedback, "le moteur suit-il sa consigne") : ici
        # on compare le feedback RÉEL à pos_min/pos_max — butée
        # mécanique franchie, encodeur qui dérive, aucun rapport avec
        # une commande envoyée. Rien ne surveillait ce cas avant ce
        # correctif.
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
        et le feedback réel — signe qu'un moteur ne suit plus sa consigne
        (bloqué, dérive de calibration). Point #4 identifié par la revue
        externe du Watchdog, laissé ouvert jusqu'ici faute d'historique
        de commande disponible ; comblé maintenant que MotionStateTracker
        est branché dans UnitreeG1HAL.

        No-op si le HAL n'expose pas de tracker (get_motion_tracker()
        retourne None par défaut dans IRobotHAL) — comportement inchangé
        pour tout backend qui n'a pas encore ce composant.
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
        Idempotent et thread-safe — point #8. Le Lock garantit qu'un seul
        appelant peut réellement déclencher l'e-stop même si plusieurs
        conditions FATAL sont détectées en concurrence sur des joints
        différents pendant le même cycle.
        """
        with self._estop_lock:
            if self._estop_fired:
                return
            self._estop_fired = True

        # hal.emergency_stop_all() est appelé HORS du lock pour ne jamais
        # bloquer la détection d'autres conditions pendant l'exécution HAL.
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

        # Persistance disque — c'est LE moment le plus critique à ne
        # jamais perdre : sans ça, un e-stop survenu il y a une semaine
        # sur un robot resté allumé ensuite n'aurait laissé aucune trace
        # une fois self._estop_log réinitialisé au prochain redémarrage
        # du process (la liste en mémoire ne survit pas à un restart).
        self._struct_logger.log(LogCategory.ESTOP, entry, robot_id=self._hal.robot_id)

        self._emit(WatchdogAlert(
            kind=AlertKind.ESTOP_TRIGGERED, severity=AlertSeverity.FATAL,
            joint_id=joint_id, message=f"E-STOP GLOBAL déclenché | raison={reason}",
            value=measured, threshold=threshold
        ))

    # ── Système d'événements ──────────────────────────────────────

    def _emit(self, alert: WatchdogAlert) -> None:
        self._stats["alerts_total"] += 1
        if alert.severity == AlertSeverity.FATAL:
            self._stats["alerts_fatal"] += 1
            # Persistance des alertes FATAL — même raisonnement que
            # pour l'e-stop : ces événements ne doivent pas dépendre
            # uniquement de la mémoire du process pour être consultables
            # après coup.
            self._struct_logger.log(
                LogCategory.WATCHDOG_ALERT, alert, robot_id=self._hal.robot_id
            )
            # Point #18 : les FATAL passent par une queue illimitée,
            # jamais droppées même si la queue normale sature.
            self._fatal_queue.put(alert)
            return
        try:
            self._queue.put_nowait(alert)
        except queue.Full:
            pass  # perte tolérée uniquement pour INFO/WARNING/CRITICAL

    def _dispatch_loop(self) -> None:
        while True:
            # Les FATAL sont toujours traitées en priorité.
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
        for jdef in self._hal.get_joint_definitions():
            self._joint_states[jdef.joint_id] = JointMonitorState(
                joint_id=jdef.joint_id, limits=jdef.limits
            )
