"""
AXIS-OS — Watchdog Events
===========================
Types d'événements produits par le WatchdogSystem.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, IntEnum, auto
from typing import Optional
import time


class AlertSeverity(IntEnum):
    """
    IntEnum (pas Enum) délibérément : permet max(sev1, sev2) directement,
    utilisé par WatchdogSystem pour calculer la sévérité du cycle.
    Bug réel trouvé et corrigé : avec Enum simple, max() lève TypeError,
    capturé silencieusement par le try/except de _monitor_loop — les
    conditions FATAL n'étaient alors jamais reflétées dans health_state.
    """
    INFO     = 0
    WARNING  = 1
    CRITICAL = 2
    FATAL    = 3


class RobotHealthState(Enum):
    """
    État global du robot, dérivé de l'ensemble des alertes actives.
    Point #10 revue externe : permet au reste d'AXIS-OS de savoir
    immédiatement dans quel état de sécurité se trouve le robot,
    sans avoir à parcourir chaque joint individuellement.
    """
    HEALTHY  = auto()   # aucune alerte active
    WARNING  = auto()   # au moins une alerte WARNING active
    DEGRADED = auto()   # au moins une alerte CRITICAL active
    ESTOP    = auto()   # e-stop déclenché — robot immobilisé


class AlertKind(Enum):
    TEMPERATURE_WARNING  = auto()
    TEMPERATURE_CRITICAL = auto()
    COMM_TIMEOUT         = auto()
    COMM_LOST            = auto()
    TORQUE_OVERLOAD      = auto()
    VELOCITY_OVERRUN     = auto()
    CURRENT_OVERLOAD     = auto()
    POSITION_DRIFT       = auto()
    JOINT_FAULT          = auto()
    ESTOP_TRIGGERED      = auto()
    WATCHDOG_STARTED     = auto()
    WATCHDOG_STOPPED     = auto()
    HEARTBEAT            = auto()
    POSITION_LIMIT_BREACH = auto()  # M10 — feedback réel hors [pos_min, pos_max]


@dataclass
class WatchdogAlert:
    kind:         AlertKind
    severity:     AlertSeverity
    joint_id:     Optional[str]
    message:      str
    value:        Optional[float]
    threshold:    Optional[float]
    timestamp_ns: int = field(default_factory=time.time_ns)

    def __str__(self) -> str:
        joint  = f"[{self.joint_id}]" if self.joint_id else "[SYSTEM]"
        val    = ""
        if self.value is not None and self.threshold is not None:
            val = f" val={self.value:.3f}/limit={self.threshold:.3f}"
        return f"[WDG-{self.severity.name}] {joint} {self.kind.name}{val} — {self.message}"
