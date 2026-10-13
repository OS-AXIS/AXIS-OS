"""
AXIS-OS — Motion State Tracker
=================================
Source de vérité unique pour l'historique de mouvement de chaque joint :
la dernière commande envoyée et le dernier retour capteur (feedback)
reçu.

Pourquoi : trois revues extérieures indépendantes, portant sur la couche
de sécurité (Safety Layer), le chien de garde (Watchdog, qui surveille le
robot en continu) et les conventions articulaires, ont relevé le même
trou sous trois angles différents. Aucun composant d'AXIS-OS ne
conservait la commande précédente envoyée à un joint, ce qui empêchait :
  - de limiter l'accélération ou le slew rate (la vitesse de variation
    d'une consigne d'une commande à la suivante) : couche de sécurité ;
  - de détecter un écart entre commande et feedback, ou un mouvement
    inattendu : watchdog ;
  - de vérifier la cohérence physique entre deux commandes successives :
    conventions.

Décisions de conception (validées) :
  - Un composant partagé plutôt qu'une copie dans chaque consommateur.
  - Il vit dans hal/core/ (code générique), pas dans un pilote
    particulier. AXIS-OS vise à fonctionner sur tous les robots ; le
    Unitree G1 est la plateforme de référence pour la validation, pas une
    dépendance du cœur du HAL (Hardware Abstraction Layer, la couche qui
    cache aux apps les différences entre robots). Toute particularité du
    G1 reste dans hal/drivers/unitree_g1/.
  - Un seul MotionStateTracker par robot, qui tient un JointMotionState
    par joint. Il est créé et possédé par le HAL du robot (une
    implémentation d'IRobotHAL ; aujourd'hui UnitreeG1HAL), puis injecté,
    et non dupliqué, dans SafetyGuard et WatchdogSystem.

Architecture :
    IRobotHAL → MotionStateTracker (un JointMotionState par joint)
              → SafetyGuard + Watchdog

Portée volontairement limitée dans cette version : ce module fournit
uniquement le stockage et le calcul des écarts (position, vitesse, temps
écoulé). Il ne décide de rien : bloquer ou rejeter une commande reste la
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
    Écart entre la commande (ou le feedback) actuelle et la précédente.

    Consommé par SafetyGuard, pour limiter la vitesse de variation avant
    l'envoi, et par WatchdogSystem, pour détecter une anomalie après
    coup : une même donnée, deux usages, aucun calcul en double.

    position_delta et velocity_delta valent None quand l'une des deux
    valeurs comparées manque ou n'est pas finie (NaN, « Not a Number »,
    ou l'infini). implied_acceleration vaut velocity_delta / elapsed_s,
    ou None si velocity_delta manque ou si elapsed_s vaut 0.
    """
    elapsed_s:           float
    position_delta:      Optional[float]   # None sans position précédente
    velocity_delta:      Optional[float]
    implied_acceleration: Optional[float]  # velocity_delta / elapsed_s
    has_history:          bool             # False sans commande précédente


class JointMotionState:
    """
    État de mouvement d'un seul joint.

    Protégé par un verrou (thread-safe) : il peut être utilisé en
    parallèle par le fil d'exécution de l'app, qui envoie des commandes,
    et par celui du Watchdog, qui lit les feedbacks.

    Les instants viennent de time.monotonic(), une horloge qui ne recule
    jamais, même si l'heure du système est modifiée : les durées
    calculées restent justes.
    """

    def __init__(self, joint_id: str):
        self.joint_id = joint_id
        self._lock    = threading.Lock()

        self._last_command:      Optional[ActuatorCommand]  = None
        self._last_command_time: Optional[float]             = None  # monotone
        self._last_feedback:     Optional[ActuatorFeedback]   = None
        self._last_feedback_time: Optional[float]              = None

    def record_command(self, command: ActuatorCommand) -> None:
        """Mémorise la commande et l'instant de son enregistrement."""
        with self._lock:
            self._last_command      = command
            self._last_command_time = time.monotonic()

    def record_feedback(self, feedback: ActuatorFeedback) -> None:
        """Mémorise le feedback et l'instant de son enregistrement."""
        with self._lock:
            self._last_feedback      = feedback
            self._last_feedback_time = time.monotonic()

    def compute_command_delta(self, new_command: ActuatorCommand) -> MotionDelta:
        """
        Calcule l'écart entre new_command et la dernière commande
        enregistrée, sans enregistrer new_command : c'est une lecture
        seule. C'est au SafetyGuard de décider s'il accepte la commande,
        avant que record_command() ne l'enregistre.

        Une valeur absente ou non finie, d'un côté ou de l'autre, donne
        un écart None plutôt qu'un calcul faux.
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
        Écart entre le dernier feedback de position reçu et la dernière
        commande de position (feedback moins commande). None si l'un des
        deux manque ou n'est pas fini.

        Consommé par le Watchdog pour détecter un moteur qui ne suit plus
        sa consigne.
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
        """Dernière commande enregistrée, None avant la première."""
        with self._lock:
            return self._last_command

    @property
    def last_feedback(self) -> Optional[ActuatorFeedback]:
        """Dernier feedback enregistré, None avant le premier."""
        with self._lock:
            return self._last_feedback


class MotionStateTracker:
    """
    Registre des JointMotionState d'un robot, un par joint.

    Créé et possédé par le HAL du robot (IRobotHAL), puis injecté dans
    SafetyGuard et WatchdogSystem : ni l'un ni l'autre ne le crée ni ne
    le duplique.

    Un joint inconnu est ignoré en écriture (record_command,
    record_feedback) et donne None en lecture (get,
    compute_command_delta).
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
