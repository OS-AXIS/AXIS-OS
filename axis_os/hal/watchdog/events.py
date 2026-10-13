"""
AXIS-OS — Watchdog Events
===========================
Types d'événements produits par le WatchdogSystem, le thread qui
surveille en continu chaque articulation (joint) du robot (voir
system.py) : niveaux de sévérité, état de santé global du robot,
catégories d'alertes et l'alerte elle-même.

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, IntEnum, auto
from typing import Optional
import time


class AlertSeverity(IntEnum):
    """
    Gravité d'une alerte, de la moins grave (INFO) à la plus grave
    (FATAL). Une alerte FATAL peut déclencher l'e-stop (arrêt
    d'urgence) automatique.

    C'est volontairement un IntEnum (énumération dont les valeurs sont
    des entiers) et non un Enum simple : WatchdogSystem calcule la
    sévérité d'un cycle de surveillance avec max(sev1, sev2), ce qui
    exige des valeurs comparables. Un vrai bug a été trouvé puis
    corrigé ici, en écrivant test/test_watchdog_v2.py : avec un Enum
    simple, max() lève TypeError ; l'exception était avalée sans bruit
    par le try/except de _monitor_loop, et les conditions FATAL
    n'apparaissaient alors jamais dans health_state.
    """
    INFO     = 0
    WARNING  = 1
    CRITICAL = 2
    FATAL    = 3


class RobotHealthState(Enum):
    """
    État de santé global du robot, dérivé des alertes actives (la
    sévérité la plus haute relevée au dernier cycle de surveillance).
    Ajouté à la suite d'une revue extérieure du watchdog : le reste
    d'AXIS-OS doit savoir immédiatement dans quel état de sécurité se
    trouve le robot, sans parcourir chaque joint un par un. Calculé par
    WatchdogSystem.health_state.
    """
    HEALTHY  = auto()   # aucune alerte active
    WARNING  = auto()   # au moins une alerte WARNING active
    DEGRADED = auto()   # au moins une alerte CRITICAL active
    ESTOP    = auto()   # e-stop déclenché, robot immobilisé


class AlertKind(Enum):
    """
    Nature d'une alerte. La sévérité indiquée pour chacune est celle que
    lui donne WatchdogSystem aujourd'hui. Le feedback est la mesure que
    chaque moteur renvoie à chaque cycle.
    """
    # Température moteur au-delà de temp_warning_c (WARNING), puis de
    # temp_critical_c (FATAL).
    TEMPERATURE_WARNING  = auto()
    TEMPERATURE_CRITICAL = auto()
    # Aucun feedback depuis comm_timeout_s (WARNING), puis depuis
    # comm_lost_s (FATAL).
    COMM_TIMEOUT         = auto()
    COMM_LOST            = auto()
    # Couple ou vitesse mesurés au-delà de la limite du joint, sur
    # plusieurs mesures consécutives (CRITICAL).
    TORQUE_OVERLOAD      = auto()
    VELOCITY_OVERRUN     = auto()
    # Déclaré mais pas émis par WatchdogSystem : le courant mesuré est
    # surveillé par JointSafetyGuard, le garde-fou de chaque joint
    # (axis_os/hal/safety/limits.py).
    CURRENT_OVERLOAD     = auto()
    # Écart persistant entre la position commandée et la position
    # mesurée : le moteur ne suit plus sa consigne (CRITICAL).
    POSITION_DRIFT       = auto()
    # Moteur en état FAULT (FATAL). Sert aussi, en CRITICAL et sans
    # joint, à signaler un bug interne du watchdog ou un watchdog trop
    # lent (voir WatchdogSystem._monitor_loop).
    JOINT_FAULT          = auto()
    ESTOP_TRIGGERED      = auto()   # e-stop déclenché (FATAL)
    WATCHDOG_STARTED     = auto()   # INFO
    WATCHDOG_STOPPED     = auto()   # INFO
    HEARTBEAT            = auto()   # signal de vie périodique (INFO)
    # Position mesurée hors des limites [pos_min, pos_max] du joint
    # (CRITICAL). Ajouté à la suite d'un audit de sécurité externe
    # (voir CHANGELOG.md, « audit externe, vague 2 »).
    POSITION_LIMIT_BREACH = auto()


@dataclass
class WatchdogAlert:
    """
    Une alerte émise par le watchdog. joint_id vaut None quand l'alerte
    concerne tout le système (démarrage, signal de vie, bug interne...).
    value et threshold portent la valeur mesurée et le seuil franchi,
    quand ils ont un sens. timestamp_ns est l'heure de création, en
    nanosecondes (time.time_ns()).
    """
    kind:         AlertKind
    severity:     AlertSeverity
    joint_id:     Optional[str]
    message:      str
    value:        Optional[float]
    threshold:    Optional[float]
    timestamp_ns: int = field(default_factory=time.time_ns)

    def __str__(self) -> str:
        """
        Une ligne lisible, par exemple :
        [WDG-FATAL] [left_knee_joint] COMM_LOST val=210.000/limit=200.000
        — Comm perdue depuis 210ms
        """
        joint  = f"[{self.joint_id}]" if self.joint_id else "[SYSTEM]"
        val    = ""
        if self.value is not None and self.threshold is not None:
            val = f" val={self.value:.3f}/limit={self.threshold:.3f}"
        return f"[WDG-{self.severity.name}] {joint} {self.kind.name}{val} — {self.message}"
