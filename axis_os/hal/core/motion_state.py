"""
AXIS-OS — Motion State Tracker
=================================
Source de vérité unique pour "l'historique de mouvement" d'un joint.

Contexte : trois revues externes indépendantes (Safety Layer, Watchdog,
Conventions) ont identifié le même trou depuis trois angles différents —
aucun composant d'AXIS-OS ne conservait la commande PRÉCÉDENTE envoyée
à un joint, ce qui bloquait :
  - Limites d'accélération / slew rate (Safety Layer)
  - Détection d'écart commande↔feedback, mouvement inattendu (Watchdog)
  - Cohérence physique entre deux commandes successives (Conventions)

Décision de conception (validée) :
  - Composant partagé plutôt que dupliqué dans chaque consommateur.
  - Vit dans hal/core/ (générique), PAS dans un driver spécifique.
    AXIS-OS vise l'universalité multi-robot ; le G1 est la plateforme
    de référence pour la validation, pas une dépendance du cœur HAL.
    Toute particularité G1 reste dans hal/drivers/unitree_g1/.
  - Un MotionStateTracker par joint, instancié et possédé par IRobotHAL,
    injecté (pas dupliqué) dans SafetyGuard et WatchdogSystem.

Architecture :
    IRobotHAL → MotionStateTracker (1 par joint) → SafetyGuard + Watchdog

Portée volontairement limitée à cette version : ce module fournit
UNIQUEMENT le stockage et le calcul des deltas (position/vitesse/temps
écoulé). Il ne décide de rien — bloquer/rejeter une commande reste la
responsabilité du SafetyGuard, jamais de ce tracker.

Licence : Apache 2.0
"""

from __future__ import annotations
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional, Dict

from .types import ActuatorCommand, ActuatorFeedback


@dataclass
class MotionDelta:
    """
    Écart calculé entre la commande/feedback actuel et le précédent.
    Consommé par SafetyGuard (rate limiting AVANT envoi) et par
    WatchdogSystem (détection d'anomalie APRÈS coup) — même donnée,
    deux usages, zéro duplication de calcul.
    """
    elapsed_s:           float
    position_delta:      Optional[float]   # None si pas de position précédente
    velocity_delta:      Optional[float]
    implied_acceleration: Optional[float]  # (velocity_delta / elapsed_s), None si elapsed_s == 0
    has_history:          bool             # False si c'est la toute première commande


class JointMotionState:
    """
    État de mouvement d'UN joint. Thread-safe (accédé potentiellement
    par le thread applicatif qui envoie des commandes ET par le thread
    du Watchdog qui lit les feedbacks, en parallèle).
    """

    def __init__(self, joint_id: str):
        self.joint_id = joint_id
        self._lock    = threading.Lock()

        self._last_command:      Optional[ActuatorCommand]  = None
        self._last_command_time: Optional[float]             = None  # monotonic
        self._last_feedback:     Optional[ActuatorFeedback]   = None
        self._last_feedback_time: Optional[float]              = None

    def record_command(self, command: ActuatorCommand) -> None:
        with self._lock:
            self._last_command      = command
            self._last_command_time = time.monotonic()

    def record_feedback(self, feedback: ActuatorFeedback) -> None:
        with self._lock:
            self._last_feedback      = feedback
            self._last_feedback_time = time.monotonic()

    def compute_command_delta(self, new_command: ActuatorCommand) -> MotionDelta:
        """
        Calcule l'écart entre new_command et la dernière commande
        enregistrée, SANS enregistrer new_command (lecture seule —
        c'est au SafetyGuard de décider s'il accepte la commande avant
        d'appeler record_command()).
        """
        with self._lock:
            prev = self._last_command
            prev_time = self._last_command_time

        if prev is None or prev_time is None:
            return MotionDelta(
                elapsed_s=0.0, position_delta=None, velocity_delta=None,
                implied_acceleration=None, has_history=False,
            )

        elapsed = time.monotonic() - prev_time
        pos_delta = None
        vel_delta = None
        accel = None

        if new_command.position is not None and prev.position is not None:
            if math.isfinite(new_command.position) and math.isfinite(prev.position):
                pos_delta = new_command.position - prev.position

        if new_command.velocity is not None and prev.velocity is not None:
            if math.isfinite(new_command.velocity) and math.isfinite(prev.velocity):
                vel_delta = new_command.velocity - prev.velocity
                if elapsed > 0:
                    accel = vel_delta / elapsed

        return MotionDelta(
            elapsed_s=elapsed, position_delta=pos_delta, velocity_delta=vel_delta,
            implied_acceleration=accel, has_history=True,
        )

    def compute_feedback_error(self) -> Optional[float]:
        """
        Écart entre la dernière commande de position et le dernier
        feedback de position reçu. None si l'un des deux manque.
        Consommé par le Watchdog pour détecter un moteur qui ne suit
        plus sa consigne.
        """
        with self._lock:
            cmd = self._last_command
            fb  = self._last_feedback

        if cmd is None or fb is None or cmd.position is None:
            return None
        if not (math.isfinite(cmd.position) and math.isfinite(fb.position)):
            return None
        return fb.position - cmd.position

    @property
    def last_command(self) -> Optional[ActuatorCommand]:
        with self._lock:
            return self._last_command

    @property
    def last_feedback(self) -> Optional[ActuatorFeedback]:
        with self._lock:
            return self._last_feedback


class MotionStateTracker:
    """
    Registre de JointMotionState, un par joint d'un robot.
    Instancié et possédé par IRobotHAL, injecté dans SafetyGuard et
    WatchdogSystem — ni l'un ni l'autre ne le crée ni ne le duplique.
    """

    def __init__(self, joint_ids: list[str]):
        self._states: Dict[str, JointMotionState] = {
            jid: JointMotionState(jid) for jid in joint_ids
        }

    def get(self, joint_id: str) -> Optional[JointMotionState]:
        return self._states.get(joint_id)

    def record_command(self, joint_id: str, command: ActuatorCommand) -> None:
        state = self._states.get(joint_id)
        if state is not None:
            state.record_command(command)

    def record_feedback(self, joint_id: str, feedback: ActuatorFeedback) -> None:
        state = self._states.get(joint_id)
        if state is not None:
            state.record_feedback(feedback)

    def compute_command_delta(self, joint_id: str, new_command: ActuatorCommand) -> Optional[MotionDelta]:
        state = self._states.get(joint_id)
        if state is None:
            return None
        return state.compute_command_delta(new_command)

    @property
    def joint_ids(self) -> list[str]:
        return list(self._states.keys())
