"""
Suiveur de consigne avec échéance de fraîcheur.

Comble l'écart entre une politique lente (GR00T : 2-5 Hz) et la
fréquence de contrôle du HAL, SANS créer de chemin rapide qui
contournerait JointSafetyGuard.

PRINCIPE — une seule boucle de sécurité :

    GR00T (asynchrone, lent)
        |  set_target()      <- non bloquant, depuis n'importe quel thread
        v
    SetpointFollower.tick()  <- à la fréquence du HAL
        |  interpole d'au plus position_delta_max par pas
        v
    ctx.move_arms() / move_legs()
        |
        v
    JointSafetyGuard         <- CHAQUE commande, sans exception

Aucune commande n'atteint les actionneurs sans passer par le garde.
Il n'y a pas de « clamp pré-calculé » : un clamp de vitesse ne
vérifie ni la position absolue, ni la température, ni le couple, ni
la cohérence avec l'état réel du robot. Ce serait une sécurité
différente et appauvrie, pas la même en plus rapide.

DIFFÉRENCE AVEC move_to_pose() : move_to_pose est bloquant et vise
une cible fixe. Ici la consigne est remplacée en cours de route par
la politique, donc le suiveur est non bloquant et reciblable à tout
instant. La logique de pas est la même (relecture de l'état RÉEL
avant chaque pas, routage bras/jambes, arrêt au premier échec).

ÉCHÉANCE DE FRAÎCHEUR : si aucune consigne fraîche n'arrive dans
max_age_s, le suiveur NE CONTINUE PAS vers la dernière cible. Il
gèle sur la position mesurée (HOLD), puis relâche (DAMPING). Une
consigne périmée exécutée indéfiniment est le mode de défaillance
déjà rencontré deux fois sur ce projet.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Optional

from axis_os.hal.core.types import ActuatorCommand, ControlMode, HALStatus


class FollowerState(Enum):
    IDLE = auto()      # aucune consigne reçue
    TRACKING = auto()  # consigne fraîche, on suit
    HOLD = auto()      # consigne périmée, on gèle sur la position mesurée
    DAMPING = auto()   # périmée depuis longtemps, on relâche doucement
    FAULT = auto()     # un pas a été refusé par le garde


@dataclass
class FollowerStats:
    """Mesuré, pas supposé — sert à répondre 'est-ce assez rapide ?'."""
    ticks: int = 0
    commands_sent: int = 0
    commands_rejected: int = 0
    stale_events: int = 0
    last_tick_ms: float = 0.0
    max_tick_ms: float = 0.0
    total_tick_ms: float = 0.0

    @property
    def mean_tick_ms(self) -> float:
        return self.total_tick_ms / self.ticks if self.ticks else 0.0


class SetpointFollower:
    """Suit une consigne mise à jour de façon asynchrone.

    Usage typique (dans une app, via run_mission) :

        follower = SetpointFollower(ctx, joint_limits)
        while mission_active:
            action = policy_service.latest_action()   # non bloquant
            if action is not None:
                follower.set_target(action.target_angles, action.timestamp_ns)
            follower.tick()
            time.sleep(follower.step_s)
    """

    def __init__(
        self,
        ctx,
        delta_max_by_joint: Dict[str, float],
        step_s: float = 0.02,          # 50 Hz
        max_age_s: float = 0.5,        # au-delà : consigne périmée
        damping_after_s: float = 1.5,  # au-delà : on relâche
        safety_factor: float = 0.5,    # marge sur position_delta_max
    ):
        self._ctx = ctx
        self._delta_max = delta_max_by_joint
        self.step_s = step_s
        self._max_age_s = max_age_s
        self._damping_after_s = damping_after_s
        self._safety_factor = safety_factor

        self._lock = threading.Lock()
        self._target: Dict[str, float] = {}
        self._target_ts_ns: int = 0

        self.state = FollowerState.IDLE
        self.stats = FollowerStats()

    # ---- appelé depuis n'importe quel thread -----------------------------

    def set_target(self, target_angles: Dict[str, float],
                   timestamp_ns: Optional[int] = None) -> None:
        """Remplace la consigne courante. Non bloquant.

        timestamp_ns vient de VLAAction.timestamp_ns — c'est l'instant
        où la POLITIQUE a produit l'action, pas celui où on la reçoit.
        Sans lui, l'échéance de fraîcheur mesurerait la latence réseau
        au lieu de l'âge réel de la décision.
        """
        with self._lock:
            self._target = dict(target_angles)
            self._target_ts_ns = timestamp_ns or time.monotonic_ns()

    def _age_s(self) -> float:
        with self._lock:
            ts = self._target_ts_ns
        if ts == 0:
            return float("inf")
        return (time.monotonic_ns() - ts) / 1e9

    # ---- boucle de contrôle ----------------------------------------------

    def tick(self) -> FollowerState:
        """Un pas. Retourne l'état résultant."""
        t0 = time.perf_counter()
        try:
            self._tick_inner()
        finally:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            self.stats.ticks += 1
            self.stats.last_tick_ms = dt_ms
            self.stats.total_tick_ms += dt_ms
            self.stats.max_tick_ms = max(self.stats.max_tick_ms, dt_ms)
        return self.state

    def _tick_inner(self) -> None:
        if self.state is FollowerState.FAULT:
            return

        age = self._age_s()
        with self._lock:
            target = dict(self._target)

        if not target:
            self.state = FollowerState.IDLE
            return

        # --- échéance de fraîcheur ---------------------------------------
        if age > self._damping_after_s:
            if self.state is not FollowerState.DAMPING:
                self.stats.stale_events += 1
                self._ctx.log(
                    f"SetpointFollower: consigne périmée depuis {age:.2f}s "
                    f"— passage en DAMPING."
                )
            self.state = FollowerState.DAMPING
            self._apply_damping()
            return

        if age > self._max_age_s:
            if self.state is not FollowerState.HOLD:
                self.stats.stale_events += 1
                self._ctx.log(
                    f"SetpointFollower: consigne périmée ({age:.2f}s > "
                    f"{self._max_age_s}s) — HOLD sur la position mesurée."
                )
            self.state = FollowerState.HOLD
            # HOLD = ne rien envoyer de nouveau. Le dernier pas envoyé
            # reste la consigne active du PD, qui était à au plus
            # delta_max de la position mesurée — donc borné. On ne
            # progresse PAS vers l'ancienne cible.
            return

        # --- suivi normal -------------------------------------------------
        self.state = FollowerState.TRACKING
        self._step_towards(target)

    def _step_towards(self, target: Dict[str, float]) -> None:
        # Relire l'état RÉEL avant chaque pas — jamais une position
        # supposée. Si un joint a bougé autrement (perturbation, autre
        # app), on repart de sa vraie position.
        state = self._ctx.get_full_state()

        commands: Dict[str, ActuatorCommand] = {}
        for jid, target_pos in target.items():
            fb = state.get(jid)
            if fb is None:
                continue
            delta_max = self._delta_max.get(jid)
            if delta_max is None:
                continue
            budget = delta_max * self._safety_factor
            error = target_pos - fb.position
            if abs(error) <= budget:
                next_pos = target_pos
            else:
                next_pos = fb.position + (budget if error > 0 else -budget)
            commands[jid] = ActuatorCommand(
                mode=ControlMode.POSITION, position=next_pos,
            )

        if not commands:
            return

        arm_cmds = {j: c for j, c in commands.items()
                    if any(k in j for k in ("shoulder", "elbow", "wrist"))}
        leg_cmds = {j: c for j, c in commands.items()
                    if any(k in j for k in ("hip", "knee", "ankle"))}
        unrouted = set(commands) - set(arm_cmds) - set(leg_cmds)
        if unrouted:
            self._ctx.log(f"SetpointFollower: joint(s) non routable(s) : "
                          f"{sorted(unrouted)} — ignorés.")

        results: Dict[str, HALStatus] = {}
        if arm_cmds:
            results.update(self._ctx.move_arms(arm_cmds))
        if leg_cmds:
            results.update(self._ctx.move_legs(leg_cmds))

        self.stats.commands_sent += len(results)
        failed = [j for j, s in results.items() if s != HALStatus.OK]
        if failed:
            self.stats.commands_rejected += len(failed)
            self._ctx.log(
                f"SetpointFollower: pas refusé pour {failed} — FAULT. "
                f"Une commande interpolée sous position_delta_max a quand "
                f"même été rejetée : e-stop, limite de position absolue, "
                f"température, ou couple."
            )
            self.state = FollowerState.FAULT

    def _apply_damping(self) -> None:
        """Relâche : consigne = position mesurée, pour que le PD cesse
        de tirer vers une cible périmée. N'utilise PAS ControlMode.TORQUE
        à zéro — sur un bipède debout, couper le couple est un
        effondrement (cf. suite S4.2)."""
        state = self._ctx.get_full_state()
        commands = {
            jid: ActuatorCommand(mode=ControlMode.POSITION, position=fb.position)
            for jid, fb in state.items()
        }
        arm_cmds = {j: c for j, c in commands.items()
                    if any(k in j for k in ("shoulder", "elbow", "wrist"))}
        leg_cmds = {j: c for j, c in commands.items()
                    if any(k in j for k in ("hip", "knee", "ankle"))}
        if arm_cmds:
            self._ctx.move_arms(arm_cmds)
        if leg_cmds:
            self._ctx.move_legs(leg_cmds)

    def report(self) -> str:
        s = self.stats
        return (
            f"SetpointFollower — {s.ticks} pas | "
            f"moyen {s.mean_tick_ms:.2f} ms | max {s.max_tick_ms:.2f} ms | "
            f"budget {self.step_s * 1000:.1f} ms | "
            f"envoyées {s.commands_sent} | refusées {s.commands_rejected} | "
            f"périmées {s.stale_events} | état final {self.state.name}"
        )
