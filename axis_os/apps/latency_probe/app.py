"""
App de mesure : combien coûte une commande envoyée depuis le sandbox ?

Le sandbox est le processus isolé dans lequel l'OS exécute chaque app.
Cette app répond à la seule question laissée ouverte par
sim/validation/test_isaac_follower_latency.py : ce test mesurait en
moyenne 5.41 ms par cycle sur le chemin direct (thread principal, sans
sandbox). Il reste à savoir ce que coûtent, par-dessus :

  - les deux multiprocessing.Queue (files d'attente entre processus de
    Python) qui relient le sous-processus de l'app au processus de l'OS ;
  - le thread HALBridgeServer qui les consomme, c'est-à-dire le pont qui
    reçoit les commandes de l'app et les transmet à la HAL (la couche
    d'abstraction matérielle, qui parle au robot) ;
  - l'attente du thread principal via MainThreadSimPump.submit() : Isaac
    Sim (le simulateur robotique de NVIDIA) ne peut être piloté que
    depuis son thread principal, auquel les autres threads confient
    leurs appels.

Cette app ne fait rien d'autre que chronométrer des move_arms() depuis
l'intérieur du sandbox. C'est la mesure qui compte : ce qu'une vraie app
subit, de bout en bout. Les résultats et leur interprétation sont dans
docs/latence_chemin_commande.md.

Volontairement une app séparée plutôt qu'une instrumentation de
package_sorting : on mesure le coût du chemin, pas la performance d'une
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

N_CALLS = 200         # nombre de cycles mesurés
DELTA = 0.01          # minuscule : on mesure le CHEMIN, pas le mouvement
# Uniquement des articulations de bras : l'app ne déclare que la
# permission ACTUATORS_WRITE_ARMS, et move_arms() écarte sans erreur
# toute articulation hors du groupe des bras. Ce groupe est déclaré par
# le driver (le pilote logiciel du robot) ou, à défaut, deviné par
# mots-clés dans le nom : shoulder, elbow, wrist (voir _group_of dans
# axis_os/store/sdk/app_base.py). Une articulation hors de cette famille
# serait donc écartée en silence, ce qui fausserait la mesure.
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

        # Chaque cycle : lire l'état des articulations, puis décaler de
        # DELTA chaque articulation de JOINTS par rapport à sa position,
        # en alternant le signe d'un cycle à l'autre pour que le bras
        # oscille sans dériver.
        for i in range(N_CALLS):
            t0 = time.perf_counter()
            state = ctx.get_full_state()
            t1 = time.perf_counter()

            cmds = {}
            for jid in JOINTS:
                fb = state.get(jid)
                if fb is None:
                    continue
                cmds[jid] = ActuatorCommand(
                    mode=ControlMode.POSITION,
                    position=fb.position + (DELTA if i % 2 == 0 else -DELTA),
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

            # getattr plutôt que s.name : si move_arms renvoie pour une
            # articulation autre chose qu'un HALStatus (le statut rendu
            # par la HAL), par exemple None ou une chaîne, un accès direct
            # à .name ferait planter la mission sur un détail de forme, et
            # non sur ce qu'on mesure.
            rejected += sum(1 for s in results.values()
                            if getattr(s, "name", str(s)) != "OK")
            state_ms.append((t1 - t0) * 1000)
            move_ms.append((t2 - t1) * 1000)
            total_ms.append((t2 - t0) * 1000)

        def stats(xs):
            """Moyenne, médiane (p50), 95e centile (p95 : 95 % des mesures
            sont en dessous) et maximum d'une série de durées en ms."""
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
