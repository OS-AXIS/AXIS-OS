"""
AXIS-OS — WatchdogSystem
==========================
Thread temps-réel qui surveille en permanence tous les joints.
Déclenche l'e-stop automatique si seuil FATAL franchi.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
import threading
import time
import queue
from typing import Callable, Dict, List, Optional
from concurrent.futures import ThreadPoolExecutor

from axis_os.hal.core.interfaces import IRobotHAL
from axis_os.hal.core.types import ActuatorFeedback, ActuatorState, JointLimits
from .events import WatchdogAlert, AlertSeverity, AlertKind
from .config import WatchdogConfig


class JointMonitorState:
    __slots__ = (
        "joint_id", "limits",
        "last_feedback_time", "last_feedback",
        "temp_warning_active", "temp_critical_active",
        "comm_timeout_active", "comm_lost_active",
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


class WatchdogSystem:
    """
    Surveillance temps-réel pour un IRobotHAL complet.

    Usage :
        wd = WatchdogSystem(robot_hal, config=WatchdogConfig.strict())
        wd.subscribe(my_handler)
        wd.start()
        # ...
        wd.stop()
    """

    def __init__(self, robot_hal: IRobotHAL, config: Optional[WatchdogConfig] = None):
        self._hal    = robot_hal
        self._config = config or WatchdogConfig.standard()

        self._joint_states: Dict[str, JointMonitorState] = {}
        self._running      = False
        self._estop_fired  = False
        self._start_time   = 0.0
        self._last_hb      = 0.0

        self._thread:          Optional[threading.Thread] = None
        self._dispatch_thread: Optional[threading.Thread] = None
        self._executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="wdg_cb")
        self._subscribers: List[Callable[[WatchdogAlert], None]] = []
        self._queue: queue.Queue[WatchdogAlert] = queue.Queue(maxsize=256)

        self._stats = {"cycles": 0, "alerts_total": 0,
                       "alerts_fatal": 0, "estop_count": 0,
                       "max_cycle_time_ms": 0.0}

    def subscribe(self, callback: Callable[[WatchdogAlert], None]) -> None:
        self._subscribers.append(callback)

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
                "joints_monitored": len(self._joint_states)}

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def estop_fired(self) -> bool:
        return self._estop_fired

    def _monitor_loop(self) -> None:
        while self._running:
            t0 = time.monotonic()
            try:
                self._run_cycle()
            except Exception as exc:
                self._emit(WatchdogAlert(
                    kind=AlertKind.JOINT_FAULT, severity=AlertSeverity.CRITICAL,
                    joint_id=None, message=f"Erreur interne watchdog: {exc}",
                    value=None, threshold=None
                ))
            elapsed = time.monotonic() - t0
            ms = elapsed * 1000
            if ms > self._stats["max_cycle_time_ms"]:
                self._stats["max_cycle_time_ms"] = ms
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

        for jid, jstate in self._joint_states.items():
            self._check_comm(jstate, now)
            fb = jstate.last_feedback
            if fb is None: continue
            self._check_joint_state(fb)
            self._check_temperature(fb, jstate)
            self._check_mechanical(fb, jstate)

        if now - self._last_hb >= self._config.heartbeat_interval_s:
            self._last_hb = now
            self._emit(WatchdogAlert(
                kind=AlertKind.HEARTBEAT, severity=AlertSeverity.INFO,
                joint_id=None,
                message=(f"Heartbeat | cycles={self._stats['cycles']} | "
                         f"alertes={self._stats['alerts_total']}"),
                value=None, threshold=None
            ))

    def _check_comm(self, jstate: JointMonitorState, now: float) -> None:
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
                    self._trigger_estop(f"COMM_LOST:{jstate.joint_id}")
            return
        if elapsed >= self._config.comm_timeout_s:
            if not jstate.comm_timeout_active:
                jstate.comm_timeout_active = True
                self._emit(WatchdogAlert(
                    kind=AlertKind.COMM_TIMEOUT, severity=AlertSeverity.WARNING,
                    joint_id=jstate.joint_id,
                    message=f"Timeout {elapsed*1000:.0f}ms",
                    value=elapsed*1000, threshold=self._config.comm_timeout_s*1000
                ))
        else:
            jstate.comm_timeout_active = False
            jstate.comm_lost_active    = False

    def _check_joint_state(self, fb: ActuatorFeedback) -> None:
        if fb.state == ActuatorState.FAULT:
            self._emit(WatchdogAlert(
                kind=AlertKind.JOINT_FAULT, severity=AlertSeverity.FATAL,
                joint_id=fb.joint_id,
                message=f"Joint FAULT (code={fb.error_code})",
                value=float(fb.error_code), threshold=0.0
            ))
            if self._config.auto_estop_on_joint_fault:
                self._trigger_estop(f"JOINT_FAULT:{fb.joint_id}")

    def _check_temperature(self, fb: ActuatorFeedback, jstate: JointMonitorState) -> None:
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
                    self._trigger_estop(f"TEMP_CRITICAL:{fb.joint_id}")
            return
        if temp >= self._config.temp_warning_c:
            if not jstate.temp_warning_active:
                jstate.temp_warning_active = True
                self._emit(WatchdogAlert(
                    kind=AlertKind.TEMPERATURE_WARNING, severity=AlertSeverity.WARNING,
                    joint_id=fb.joint_id,
                    message=f"Température élevée : {temp:.1f}°C",
                    value=temp, threshold=self._config.temp_warning_c
                ))
        else:
            jstate.temp_warning_active  = False
            jstate.temp_critical_active = False

    def _check_mechanical(self, fb: ActuatorFeedback, jstate: JointMonitorState) -> None:
        lim = jstate.limits
        if lim.torque_max > 0 and abs(fb.torque) > lim.torque_max * self._config.torque_overload_ratio:
            self._emit(WatchdogAlert(
                kind=AlertKind.TORQUE_OVERLOAD, severity=AlertSeverity.CRITICAL,
                joint_id=fb.joint_id,
                message=f"Surcharge couple : {abs(fb.torque):.1f} Nm",
                value=abs(fb.torque), threshold=lim.torque_max * self._config.torque_overload_ratio
            ))
        if lim.vel_max > 0 and abs(fb.velocity) > lim.vel_max * self._config.velocity_overrun_ratio:
            self._emit(WatchdogAlert(
                kind=AlertKind.VELOCITY_OVERRUN, severity=AlertSeverity.CRITICAL,
                joint_id=fb.joint_id,
                message=f"Dépassement vitesse : {abs(fb.velocity):.2f} rad/s",
                value=abs(fb.velocity), threshold=lim.vel_max * self._config.velocity_overrun_ratio
            ))

    def _trigger_estop(self, reason: str) -> None:
        if self._estop_fired: return
        self._estop_fired = True
        self._stats["estop_count"] += 1
        self._hal.emergency_stop_all()
        self._emit(WatchdogAlert(
            kind=AlertKind.ESTOP_TRIGGERED, severity=AlertSeverity.FATAL,
            joint_id=None, message=f"E-STOP | raison={reason}",
            value=None, threshold=None
        ))

    def _emit(self, alert: WatchdogAlert) -> None:
        self._stats["alerts_total"] += 1
        if alert.severity == AlertSeverity.FATAL:
            self._stats["alerts_fatal"] += 1
        try: self._queue.put_nowait(alert)
        except queue.Full: pass

    def _dispatch_loop(self) -> None:
        while True:
            try:
                alert = self._queue.get(timeout=1.0)
                if alert is None: break
                for cb in self._subscribers:
                    try: self._executor.submit(cb, alert)
                    except Exception: pass
            except queue.Empty:
                if not self._running: break

    def _build_joint_states(self) -> None:
        for jdef in self._hal.get_joint_definitions():
            self._joint_states[jdef.joint_id] = JointMonitorState(
                joint_id=jdef.joint_id, limits=jdef.limits
            )
