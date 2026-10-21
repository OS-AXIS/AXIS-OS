"""
Suiveur de consigne avec échéance de fraîcheur.

Le problème : une politique apprise comme GR00T (le modèle de NVIDIA qui
décide des gestes du robot) ne produit une nouvelle consigne que 2 à 5 fois
par seconde, alors que la boucle de contrôle du HAL (couche d'abstraction
matérielle, qui transmet les commandes aux moteurs) tourne bien plus vite.
Ce suiveur comble l'écart en avançant vers la consigne par petits pas, sans
créer de chemin rapide qui contournerait JointSafetyGuard (le garde qui
vérifie chaque commande d'articulation).

Principe : une seule boucle de sécurité.

    GR00T (asynchrone, lent)
        |  set_target()      <- non bloquant, depuis n'importe quel thread
        v
    SetpointFollower.tick()  <- à la fréquence du HAL
        |  interpole d'au plus position_delta_max par pas
        v
    ctx.move_arms() / move_legs()
        |
        v
    JointSafetyGuard         <- chaque commande, sans exception

La politique peut tourner dans un autre thread (fil d'exécution parallèle) :
elle dépose ses consignes par set_target(), et tick() les suit à la cadence
du HAL.

Aucune commande n'atteint les actionneurs sans passer par le garde. Il n'y
a pas de « clamp pré-calculé » (une simple limite de vitesse appliquée à
l'avance) : un tel clamp ne vérifie ni la position absolue, ni la
température, ni le couple, ni la cohérence avec l'état réel du robot. Ce
serait une sécurité différente et appauvrie, pas la même en plus rapide.

Différence avec move_to_pose() : move_to_pose est bloquant et vise une
cible fixe. Ici, la politique remplace la consigne en cours de route ; le
suiveur est donc non bloquant et peut changer de cible à tout instant. La
logique de pas est la même : relecture de l'état réel avant chaque pas,
routage vers les bras ou les jambes, arrêt au premier refus.

Échéance de fraîcheur. Une consigne périmée exécutée indéfiniment est un
mode de défaillance déjà rencontré deux fois sur ce projet : le bug de
publish() jamais appelé, et la raison d'être du moniteur indépendant (voir
docs/groot_architecture.md). Si aucune consigne fraîche n'arrive dans
max_age_s, le suiveur ne continue donc pas vers la dernière cible : il gèle
sur la position mesurée (état HOLD), puis, quand l'âge de la consigne
dépasse damping_after_s, relâche doucement (état DAMPING).

Valeurs non finies et horodatages. Un audit de sécurité du 2026-10-10 a
trouvé deux pièges, couverts par test/test_suiveur_non_fini.py :
  - Cible NaN (« Not a Number », la valeur flottante qui n'est pas un
    nombre). Elle donnait « erreur = NaN » ; or toute comparaison d'ordre
    avec NaN est fausse, donc « abs(erreur) <= budget » et « erreur > 0 »
    étaient faux tous les deux, et le code faisait un pas de -budget (le
    budget est le pas maximal autorisé, delta_max × safety_factor) à
    chaque tick. Chaque pas était fini et sous position_delta_max (le
    déplacement maximal par pas que le JointSafetyGuard accepte) : le
    garde n'avait rien à refuser, et l'articulation dérivait jusqu'à sa
    butée. Une cible non finie (NaN ou infinie) est désormais
    ignorée : l'articulation garde sa consigne active, comme en HOLD.
  - Horodatage dans le futur. Il donnait un âge négatif, donc une consigne
    qui ne périmait jamais. La consigne est désormais tenue pour périmée,
    de même qu'avec un horodatage NaN, infini ou négatif.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Optional

from axis_os.hal.core.types import ActuatorCommand, ControlMode, HALStatus


class FollowerState(Enum):
    """États du suiveur (voir l'en-tête du module)."""
    IDLE = auto()      # aucune consigne reçue
    TRACKING = auto()  # consigne fraîche, on la suit
    HOLD = auto()      # consigne périmée, on gèle sur la position mesurée
    DAMPING = auto()   # périmée depuis longtemps, on relâche doucement
    FAULT = auto()     # un pas a été refusé par le garde ; état définitif


@dataclass
class FollowerStats:
    """Statistiques mesurées, pas supposées : elles servent à répondre à
    la question « le suiveur tient-il la cadence ? ». Les durées sont en
    millisecondes ; report() les affiche à côté du budget d'un pas
    (step_s)."""
    ticks: int = 0
    commands_sent: int = 0
    commands_rejected: int = 0
    stale_events: int = 0
    non_finite_targets: int = 0   # cibles NaN/inf ignorées (voir en-tête)
    last_tick_ms: float = 0.0
    max_tick_ms: float = 0.0
    total_tick_ms: float = 0.0

    @property
    def mean_tick_ms(self) -> float:
        return self.total_tick_ms / self.ticks if self.ticks else 0.0


class SetpointFollower:
    """Suit une consigne mise à jour de façon asynchrone.

    ctx est le contexte d'exécution de l'app (get_full_state, move_arms,
    move_legs, log). delta_max_by_joint donne, pour chaque articulation, le
    déplacement maximal par pas en radians (le position_delta_max du
    JointSafetyGuard) ; une articulation absente ne reçoit aucun pas vers
    la cible.

    Usage typique (dans une app, via run_mission). La source des consignes
    est ici un connecteur VLA (IVLAConnector, voir
    axis_os/store/sdk/ai/connector.py). Son predict_action() attend la
    réponse du modèle : on l'appelle donc dans un autre thread, qui dépose
    chaque action par set_target(), pendant que la boucle de mission
    appelle tick() à cadence fixe.

        follower = SetpointFollower(ctx, delta_max_by_joint)

        # Thread de la politique, en boucle, à son propre rythme :
        action = connecteur.predict_action(image, instruction)
        follower.set_target(action.target_angles, action.timestamp_ns,
                            origine=ORIGINE_IA)

        # Boucle de mission, un pas toutes les step_s secondes :
        while mission_active:
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

        # set_target() peut être appelé depuis un autre thread que tick() :
        # le verrou protège la consigne et son horodatage.
        self._lock = threading.Lock()
        self._target: Dict[str, float] = {}
        self._target_ts_ns: int = 0
        self._origine: Optional[str] = None

        self.state = FollowerState.IDLE
        self.stats = FollowerStats()

    # ---- appelé depuis n'importe quel thread -----------------------------

    def set_target(self, target_angles: Dict[str, float],
                   timestamp_ns: Optional[int] = None,
                   origine: Optional[str] = None) -> None:
        """Remplace la consigne courante. Non bloquant.

        origine : ORIGINE_IA si la consigne vient d'un modèle d'IA ; elle
        est reportée sur chaque commande de tick(), pour que le contexte
        exige ACTUATORS_AI_DRIVEN (voir RobotContext._exiger_pilotage_ia).

        timestamp_ns vient de VLAAction.timestamp_ns (l'action renvoyée par
        le modèle VLA, pour Vision-Language-Action) : c'est l'instant où
        la politique a produit l'action, pas celui où on la reçoit. Sans
        lui, l'échéance de fraîcheur mesurerait la latence réseau au lieu
        de l'âge réel de la décision. Absent (None ou 0), la consigne est
        horodatée à sa réception (comportement d'origine, inchangé).
        """
        if not timestamp_ns:
            ts = time.monotonic_ns()       # None ou 0 : horodatée maintenant
        elif isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, (int, float)) \
                or not math.isfinite(timestamp_ns) or timestamp_ns < 0:
            # Pourquoi : la relecture des corrections de l'audit du
            # 2026-10-10 a relevé que NaN vaut « vrai » en booléen. L'ancien
            # code (« timestamp_ns or time.monotonic_ns() ») le laissait
            # donc passer, et son âge NaN ne dépassait jamais max_age_s : la
            # consigne ne périmait jamais. Un horodatage invalide (NaN,
            # infini, négatif, ou pas un nombre) rend désormais la consigne
            # périmée d'emblée ; ts = 0 signifie « aucune consigne valide ».
            ts = 0
        else:
            ts = int(timestamp_ns)
        with self._lock:
            self._target = dict(target_angles)
            self._target_ts_ns = ts
            self._origine = origine

    def _age_s(self) -> float:
        """Âge de la consigne courante, en secondes. Infini, donc périmé,
        sans consigne ou avec un horodatage invalide."""
        with self._lock:
            ts = self._target_ts_ns
        if ts == 0:
            return float("inf")
        age = (time.monotonic_ns() - ts) / 1e9
        if not math.isfinite(age) or age < 0.0:
            # Horodatage dans le futur : impossible avec time.monotonic_ns
            # du même processus, donc horloge étrangère ou valeur corrompue.
            # Un âge négatif ne dépasserait jamais max_age_s et la consigne
            # ne périmerait jamais : on la traite comme périmée.
            return float("inf")
        return age

    # ---- boucle de contrôle ----------------------------------------------

    def tick(self) -> FollowerState:
        """Un pas de la boucle de contrôle, à appeler toutes les step_s
        secondes. Sa durée est mesurée (stats) même s'il lève une
        exception. Retourne l'état résultant."""
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
        # Après un refus du garde, plus rien n'est envoyé : FAULT est
        # définitif pour ce suiveur.
        if self.state is FollowerState.FAULT:
            return

        age = self._age_s()
        with self._lock:
            target = dict(self._target)
            origine = self._origine

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
            # HOLD consiste à ne rien envoyer de nouveau. Le dernier pas
            # envoyé reste la consigne active du PD (le régulateur
            # proportionnel-dérivé de chaque moteur) ; il était à au plus
            # delta_max de la position mesurée, donc l'écart reste borné.
            # On ne progresse pas vers l'ancienne cible.
            return

        # --- suivi normal -------------------------------------------------
        self.state = FollowerState.TRACKING
        self._step_towards(target, origine)

    def _step_towards(self, target: Dict[str, float],
                      origine: Optional[str] = None) -> None:
        # Relire l'état réel avant chaque pas, jamais une position supposée :
        # si une articulation a bougé autrement (perturbation, autre app),
        # on repart de sa vraie position.
        state = self._ctx.get_full_state()

        commands: Dict[str, ActuatorCommand] = {}
        non_finis = []
        for jid, target_pos in target.items():
            # Articulation inconnue du robot ou sans delta_max : aucun pas,
            # jamais de limite devinée.
            fb = state.get(jid)
            if fb is None:
                continue
            delta_max = self._delta_max.get(jid)
            if delta_max is None:
                continue
            if not (math.isfinite(target_pos) and math.isfinite(fb.position)):
                # Cible (ou mesure) non finie : aucun pas pour cette
                # articulation, qui garde sa consigne active. Sans ce refus,
                # NaN donnait un pas de -budget à chaque tick, jusqu'à la
                # butée (voir l'en-tête).
                non_finis.append(jid)
                continue
            budget = delta_max * self._safety_factor
            error = target_pos - fb.position
            if abs(error) <= budget:
                next_pos = target_pos
            else:
                next_pos = fb.position + (budget if error > 0 else -budget)
            commands[jid] = ActuatorCommand(
                mode=ControlMode.POSITION, position=next_pos, origine=origine,
            )

        if non_finis:
            self.stats.non_finite_targets += len(non_finis)
            self._ctx.log(f"SetpointFollower: cible non finie pour "
                          f"{sorted(non_finis)} — ignorée, joint(s) tenu(s).")

        if not commands:
            return

        # Routage d'après le nom de l'articulation : épaule, coude et
        # poignet vers les bras ; hanche, genou et cheville vers les jambes.
        # Une articulation d'aucun des deux groupes n'est pas envoyée : elle
        # est signalée dans le journal.
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
        # Un pas déjà limité sous position_delta_max peut quand même être
        # refusé par le garde (arrêt d'urgence, limite de position absolue,
        # température, couple) : on s'arrête au premier refus.
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
        """Relâche : la consigne devient la position mesurée, pour que le
        PD cesse de tirer vers une cible périmée.

        N'utilise pas ControlMode.TORQUE à zéro : sur un bipède debout,
        couper le couple, c'est l'effondrement. Voir l'expérience « e-stop
        debout » (arrêt d'urgence, robot en appui sur ses deux jambes) dans
        docs/campagne_validation_isaac.md.
        """
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
        """Résumé d'une ligne : durée moyenne et maximale d'un pas face au
        budget step_s, commandes envoyées et refusées, passages en
        consigne périmée, état final."""
        s = self.stats
        return (
            f"SetpointFollower — {s.ticks} pas | "
            f"moyen {s.mean_tick_ms:.2f} ms | max {s.max_tick_ms:.2f} ms | "
            f"budget {self.step_s * 1000:.1f} ms | "
            f"envoyées {s.commands_sent} | refusées {s.commands_rejected} | "
            f"périmées {s.stale_events} | état final {self.state.name}"
        )
