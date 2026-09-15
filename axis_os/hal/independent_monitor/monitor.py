"""
AXIS-OS — Moniteur de sécurité indépendant (point 3, recommandation externe)
=================================================================================
Redondance DIVERSE, pas duplication — la distinction compte. Tous les
contrôles existants (JointSafetyGuard, WatchdogSystem) sont construits
par ou dépendent du même chemin logiciel que la commande elle-même
(kinematic_normalizer, G1DDSActuator/G1SimulatedActuator) : si CE
chemin contient un bug de conception, le garde-fou qui en dépend est
faux avec lui. C'est exactement ce qui s'est produit avec C3 (voir
CHANGELOG.md) : JointSafetyGuard validait un couple que
kinematic_normalizer avait déjà silencieusement inversé — le garde
n'a jamais pu détecter cette erreur puisqu'il partage la même vision
faussée du monde que le composant qu'il est censé surveiller.

Ce module lit `feedback.position/velocity/torque` en tant que valeurs
BRUTES (repère mécanique du fabricant), directement depuis
`IRobotHAL.get_full_state()` — sans jamais passer par
`NormalizedRobotHAL` ni `KinematicNormalizer` — et les compare aux
limites du fichier URDF officiel, lues via `urdf_loader.py`
directement (jamais via `robot.py`/`_build_g1_joint_definitions()`,
qui pourrait un jour diverger du fichier source). Aucune dépendance
d'import vers `axis_os.hal.core.kinematic_normalizer` ni vers
`axis_os.hal.drivers.unitree_g1.robot` — vérifié explicitement par
`test_independent_monitor.py` (une assertion sur les imports du
module lui-même, pas seulement un test de comportement), pour qu'un
futur refactor ne puisse pas silencieusement réintroduire ce
couplage et vider ce module de sa valeur de redondance.

Ce moniteur peut déclencher un e-stop directement via
`IRobotHAL.emergency_stop_all()` — SANS jamais consulter
JointSafetyGuard ni WatchdogSystem pour décider de le faire. C'est le
seul type de redondance qui attrape les bugs de CONCEPTION (une
mauvaise hypothèse partagée par tout le chemin de commande), par
opposition à la redondance de DUPLICATION (la même vérification
réécrite deux fois, qui reproduit fidèlement le même bug si
l'hypothèse de départ était fausse).

Limite assumée : ce moniteur ne peut PAS distinguer une position hors
limites due à un vrai problème mécanique (butée franchie) d'une due à
un bug de son PROPRE parsing URDF — il partage cette dépendance avec
tout le reste du système, un choix pragmatique (le fichier URDF
officiel reste la source de vérité la plus fiable disponible), pas un
oubli.

Licence : Apache 2.0
"""

from __future__ import annotations
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Union

from axis_os.hal.core.interfaces import IRobotHAL
from axis_os.hal.core.urdf_loader import load_urdf
from axis_os.hal.core.structured_log import (
    StructuredLogger, LogCategory,
)


@dataclass(frozen=True)
class IndependentMonitorAlert:
    joint_id: str
    reason: str
    measured: float
    limit: float
    timestamp_ns: int


class IndependentSafetyMonitor:
    """
    Tourne dans son propre thread, poll `IRobotHAL.get_full_state()`
    directement — jamais via `NormalizedRobotHAL`. Lit les limites
    depuis le fichier URDF officiel via `urdf_loader.load_urdf()` —
    jamais depuis `axis_os.hal.drivers.unitree_g1.robot.
    _build_g1_joint_definitions()`, qui pourrait diverger du fichier
    source sans que ce moniteur ne le remarque autrement.
    """

    def __init__(
        self,
        robot_hal: IRobotHAL,
        urdf_path: Union[str, Path],
        poll_interval_s: float = 0.05,
        margin_ratio: float = 1.0,
        logger: Optional[StructuredLogger] = None,
    ):
        """
        `margin_ratio` : multiplicateur appliqué aux limites URDF
        (1.0 = strictement les limites officielles, aucune tolérance).
        Volontairement séparé de toute marge appliquée ailleurs dans
        le système (ex: `TORQUE_OVERLOAD` à 110% dans
        `JointSafetyGuard`) — ce moniteur applique SA PROPRE politique,
        décidée indépendamment, une autre facette de la redondance
        diverse recherchée ici.
        """
        self._hal = robot_hal
        self._model = load_urdf(urdf_path)
        self._limits_by_joint: Dict[str, tuple] = {
            j.joint_id: (j.limits.pos_min * margin_ratio if j.limits.pos_min < 0
                        else j.limits.pos_min / margin_ratio,
                        j.limits.pos_max * margin_ratio)
            for j in self._model.get_joints()
        }
        self._poll_interval_s = poll_interval_s
        self._logger = logger or StructuredLogger()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._estop_triggered = False
        self._lock = threading.Lock()
        self._alerts: List[IndependentMonitorAlert] = []
        self._subscribers: List = []

    def subscribe(self, callback) -> None:
        """Callback(IndependentMonitorAlert) -> None, appelé à chaque
        alerte — jamais bloquant pour la boucle de poll elle-même
        (voir _emit())."""
        self._subscribers.append(callback)

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop, daemon=True,
            name="axis-os-independent-monitor",
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    @property
    def estop_triggered(self) -> bool:
        with self._lock:
            return self._estop_triggered

    def get_alerts(self) -> List[IndependentMonitorAlert]:
        with self._lock:
            return list(self._alerts)

    def _poll_loop(self) -> None:
        while self._running:
            try:
                self._check_once()
            except Exception as exc:
                # Ce moniteur ne doit jamais mourir silencieusement —
                # une erreur ici, aussi inattendue soit-elle, doit être
                # visible, pas juste avaler l'exception et continuer
                # comme si de rien n'était (exactement le genre de
                # défaillance silencieuse que ce module existe pour
                # combattre ailleurs dans le système).
                print(f"  [INDEPENDENT MONITOR] Erreur interne (non fatale, "
                      f"continue) : {type(exc).__name__}: {exc}")
            time.sleep(self._poll_interval_s)

    def _check_once(self) -> None:
        feedbacks = self._hal.get_full_state()
        for joint_id, fb in feedbacks.items():
            limits = self._limits_by_joint.get(joint_id)
            if limits is None:
                continue  # joint hors du fichier URDF chargé (ex: capteur monté)
            pos_min, pos_max = limits
            if not (pos_min <= fb.position <= pos_max):
                self._trigger(joint_id, "POSITION_OUT_OF_BOUNDS_INDEPENDENT",
                              fb.position,
                              pos_max if fb.position > pos_max else pos_min)

    def _trigger(self, joint_id: str, reason: str, measured: float, limit: float) -> None:
        alert = IndependentMonitorAlert(
            joint_id=joint_id, reason=reason, measured=measured,
            limit=limit, timestamp_ns=time.time_ns(),
        )
        with self._lock:
            self._alerts.append(alert)
            already_triggered = self._estop_triggered
            self._estop_triggered = True

        self._logger.log(LogCategory.SAFETY_VIOLATION, {
            "source": "independent_monitor", "joint_id": joint_id,
            "reason": reason, "measured": measured, "limit": limit,
        })

        for cb in self._subscribers:
            try:
                cb(alert)
            except Exception:
                pass  # un abonné défaillant ne doit jamais bloquer ce moniteur

        if not already_triggered:
            print(f"  [INDEPENDENT MONITOR] ⚠️  {reason} sur {joint_id} : "
                  f"{measured:.4f} (limite {limit:.4f}) — e-stop déclenché, "
                  f"INDÉPENDAMMENT de JointSafetyGuard/WatchdogSystem.")
            self._hal.emergency_stop_all()
