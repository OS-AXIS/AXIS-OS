"""
App de mesure : combien coûte UNE commande vue depuis le sandbox ?

Répond à la seule question laissée ouverte par test_isaac_follower_latency :
celui-ci mesurait 5.41 ms sur le chemin DIRECT (thread principal, sans
sandbox). Il reste à savoir ce que coûtent, par-dessus :

  - les deux multiprocessing.Queue entre le sous-process et le parent
  - le thread HALBridgeServer qui les consomme
  - l'attente du thread principal via MainThreadSimPump.submit()

Cette app ne fait RIEN d'autre que chronométrer des move_arms() depuis
l'intérieur du sandbox. C'est la mesure de vérité : ce qu'une vraie app
subit réellement, bout en bout.

Volontairement une app séparée plutôt qu'une instrumentation de
package_sorting : on mesure le COÛT DU CHEMIN, pas la performance d'une
app métier dont les temps seraient mêlés à sa propre logique.
"""
from __future__ import annotations

import time
from typing import Any, Dict

from axis_os.hal.core.types import ActuatorCommand, ControlMode
from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext
from axis_os.store.sdk.manifest import (
    AppCategory, AppManifest, Permission, ResourceQuota,
)

def _pos(fb) -> float:
    """Position d'un feedback, quelle que soit sa forme.

    Rend l'app portable entre les deux chemins : ActuatorFeedback
    (direct) ou dict sérialisé (sandbox). Tant que les deux ne sont pas
    unifiés côté OS, toute app qui lit l'état doit faire ça — ce qui
    est précisément le problème à corriger dans le SDK."""
    return fb["position"] if isinstance(fb, dict) else fb.position


N_CALLS = 200
DELTA = 0.01          # minuscule : on mesure le CHEMIN, pas le mouvement
# Uniquement des joints de BRAS : l'app ne déclare que
# ACTUATORS_WRITE_ARMS, et move_arms() filtre sur ces mots-clés
# (shoulder/elbow/wrist). Un joint hors de cette famille serait
# silencieusement écarté du routage, faussant la mesure.
JOINTS = ["left_shoulder_pitch_joint", "left_elbow_joint",
          "right_shoulder_pitch_joint", "right_elbow_joint"]


class LatencyProbeApp(RobotApp):

    def manifest(self) -> AppManifest:
        return AppManifest(
            app_id="com.axisos.latencyprobe",
            name="Latency Probe",
            version="1.0.0",
            author="AXIS-OS",
            author_email="dev@axis-os.local",
            description="Mesure le surcoût du sandbox sur une commande.",
            category=AppCategory.RESEARCH,
            permissions=[
                Permission.ACTUATORS_READ,
                Permission.ACTUATORS_WRITE_ARMS,
            ],
            quota=ResourceQuota(max_memory_mb=8192, max_runtime_s=180.0),
        )

    def on_start(self, ctx: RobotContext) -> bool:
        return True

    def on_stop(self, ctx: RobotContext) -> None:
        pass

    def run_mission(self, ctx: RobotContext, params: Dict[str, Any]) -> MissionResult:
        t_mission = time.perf_counter()

        state_ms, move_ms, total_ms = [], [], []
        rejected = 0

        for i in range(N_CALLS):
            t0 = time.perf_counter()
            state = ctx.get_full_state()
            t1 = time.perf_counter()

            cmds = {}
            for jid in JOINTS:
                fb = state.get(jid)
                if fb is None:
                    continue
                # DIVERGENCE RÉELLE entre les deux chemins, pas un détail
                # de ce test : RobotContext.get_full_state() rend des
                # ActuatorFeedback (fb.position), mais HALBridgeClient —
                # ce que reçoit une app SANDBOXÉE — rend des dicts
                # sérialisés (fb["position"]), puisqu'ils traversent deux
                # multiprocessing.Queue. Une app écrite contre la doc de
                # RobotContext plante donc UNIQUEMENT en sandbox, donc
                # uniquement sur le vrai robot. Déjà contourné au cas par
                # cas dans hal_bridge.move_to_pose(), jamais exposé aux
                # apps. Voir _pos() ci-dessous.
                cmds[jid] = ActuatorCommand(
                    mode=ControlMode.POSITION,
                    position=_pos(fb) + (DELTA if i % 2 == 0 else -DELTA),
                )
            if not cmds:
                return MissionResult(
                    success=False, mission_id="probe",
                    app_id="com.axisos.latencyprobe",
                    duration_s=time.perf_counter() - t_mission,
                    error_message="aucun joint cible trouvé dans get_full_state()",
                )

            results = ctx.move_arms(cmds)
            t2 = time.perf_counter()

            # getattr plutôt que s.name : si move_arms renvoie autre chose
            # qu'un HALStatus pour un joint (None, str), un accès direct à
            # .name ferait planter la mission sur un détail de forme, pas
            # sur ce qu'on mesure.
            rejected += sum(1 for s in results.values()
                            if getattr(s, "name", str(s)) != "OK")
            state_ms.append((t1 - t0) * 1000)
            move_ms.append((t2 - t1) * 1000)
            total_ms.append((t2 - t0) * 1000)

        def stats(xs):
            xs_sorted = sorted(xs)
            n = len(xs_sorted)
            return {
                "mean": sum(xs) / n,
                "p50": xs_sorted[n // 2],
                "p95": xs_sorted[int(n * 0.95)],
                "max": xs_sorted[-1],
            }

        return MissionResult(
            success=True,
            mission_id="probe",
            app_id="com.axisos.latencyprobe",
            duration_s=time.perf_counter() - t_mission,
            items_processed=N_CALLS,
            metrics={
                "n_calls": N_CALLS,
                "joints_per_call": len(JOINTS),
                "rejected": rejected,
                "get_full_state_ms": stats(state_ms),
                "move_arms_ms": stats(move_ms),
                "total_per_cycle_ms": stats(total_ms),
            },
        )
