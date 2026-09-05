"""
AXIS-OS — Watchdog Configuration
===================================
Profils de surveillance par déploiement.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from dataclasses import dataclass


@dataclass
class WatchdogConfig:
    poll_interval_s:           float = 0.01
    comm_timeout_s:            float = 0.05
    comm_lost_s:               float = 0.20
    heartbeat_interval_s:      float = 5.0
    temp_warning_c:            float = 65.0
    temp_critical_c:           float = 80.0
    torque_overload_ratio:     float = 1.10
    velocity_overrun_ratio:    float = 1.10
    current_overload_ratio:    float = 1.10
    confirm_samples:           int   = 3   # échantillons consécutifs requis
                                             # avant CRITICAL (anti faux-positif, #12)
    max_cycle_time_ms:         float = 100.0  # au-delà : le watchdog lui-même
                                                # est considéré dégradé (#14)
    position_drift_max_rad:    float = 0.15
    """
    Écart maximal toléré entre la dernière commande de position
    envoyée et le feedback réel avant de considérer que le joint ne
    suit plus sa consigne (moteur bloqué, calibration dérivée, etc.).
    Nécessite un MotionStateTracker exposé par le HAL — sans lui,
    ce check est un no-op (voir WatchdogSystem._check_feedback_error).
    """
    auto_estop_on_comm_lost:   bool  = True
    auto_estop_on_temp_critical: bool = True
    auto_estop_on_joint_fault: bool  = True
    profile_name:              str   = "standard"

    @classmethod
    def standard(cls) -> "WatchdogConfig":
        return cls(profile_name="standard")

    @classmethod
    def strict(cls) -> "WatchdogConfig":
        return cls(
            poll_interval_s=0.005, comm_timeout_s=0.02, comm_lost_s=0.1,
            temp_warning_c=55.0, temp_critical_c=70.0,
            torque_overload_ratio=1.05, velocity_overrun_ratio=1.05,
            current_overload_ratio=1.05, profile_name="strict"
        )

    @classmethod
    def simulation(cls) -> "WatchdogConfig":
        return cls(
            poll_interval_s=0.02, comm_timeout_s=1.0, comm_lost_s=5.0,
            temp_warning_c=999.0, temp_critical_c=999.0,
            auto_estop_on_comm_lost=False,
            auto_estop_on_temp_critical=False,
            auto_estop_on_joint_fault=False,
            profile_name="simulation"
        )
