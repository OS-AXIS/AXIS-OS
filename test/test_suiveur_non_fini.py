"""
SetpointFollower et VLACppConnector — valeurs NON FINIES et horodatages.

Trouvé par l'audit du 2026-10-10 :
  - une cible NaN donnait un pas de -budget à CHAQUE tick (abs(NaN) <= b
    et NaN > 0 sont faux) : l'articulation dérivait jusqu'à sa butée, par
    pas finis et sous position_delta_max — le JointSafetyGuard n'avait
    rien à refuser ;
  - un horodatage dans le FUTUR donnait un âge négatif : la consigne ne
    périmait jamais ;
  - VLACppConnector laissait passer NaN/inf et horodatait l'action à la
    RÉCEPTION (elle paraissait plus jeune que la décision réelle).
"""
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.types import HALStatus
from axis_os.services.policy.setpoint_follower import FollowerState, SetpointFollower

FAIL = 0
NAN = float("nan")


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


@dataclass
class Retour:
    position: float


class FauxContexte:
    """Robot minimal : une commande acceptée DEVIENT la position mesurée
    (suivi parfait), pour voir où le suiveur emmène l'articulation."""

    def __init__(self, positions):
        self.pos = dict(positions)
        self.envois = []
        self.journal = []

    def get_full_state(self):
        return {j: Retour(p) for j, p in self.pos.items()}

    def move_arms(self, cmds):
        self.envois.append({j: c.position for j, c in cmds.items()})
        for j, c in cmds.items():
            self.pos[j] = c.position
        return {j: HALStatus.OK for j in cmds}

    move_legs = move_arms

    def log(self, msg):
        self.journal.append(msg)


DELTA = {"left_elbow_joint": 0.05, "right_elbow_joint": 0.05}

section("1 — Référence : une cible finie est suivie, pas à pas")

ctx = FauxContexte({"left_elbow_joint": 0.0, "right_elbow_joint": 0.0})
f = SetpointFollower(ctx, DELTA, max_age_s=10.0, damping_after_s=20.0)
f.set_target({"left_elbow_joint": 0.1, "right_elbow_joint": -0.1})
for _ in range(10):
    f.tick()
check("Cible finie atteinte", abs(ctx.pos["left_elbow_joint"] - 0.1) < 1e-9
      and abs(ctx.pos["right_elbow_joint"] + 0.1) < 1e-9)

section("2 — Cible NaN : l'articulation NE DÉRIVE PAS")

ctx = FauxContexte({"left_elbow_joint": 0.2, "right_elbow_joint": 0.0})
f = SetpointFollower(ctx, DELTA, max_age_s=10.0, damping_after_s=20.0)
f.set_target({"left_elbow_joint": NAN, "right_elbow_joint": 0.1})
for _ in range(100):
    f.tick()
check(f"Joint à cible NaN resté en place (0.2 rad, mesuré "
      f"{ctx.pos['left_elbow_joint']:.3f}) — avant : -budget à chaque tick, "
      f"soit -1.05 rad après 100 ticks", ctx.pos["left_elbow_joint"] == 0.2)
check("Aucune commande NaN ni dérivée envoyée pour ce joint",
      all("left_elbow_joint" not in e for e in ctx.envois))
check("L'autre joint, à cible finie, est toujours suivi",
      abs(ctx.pos["right_elbow_joint"] - 0.1) < 1e-9)
check("Les cibles non finies sont comptées et journalisées",
      f.stats.non_finite_targets == 100 and any("non finie" in m for m in ctx.journal))
check("Le suiveur ne passe pas en FAULT pour autant (rien n'a été refusé)",
      f.state is FollowerState.TRACKING)

for nom, v in {"+inf": math.inf, "-inf": -math.inf}.items():
    ctx = FauxContexte({"left_elbow_joint": 0.2})
    f = SetpointFollower(ctx, DELTA, max_age_s=10.0, damping_after_s=20.0)
    f.set_target({"left_elbow_joint": v})
    for _ in range(20):
        f.tick()
    check(f"Cible {nom} : joint resté en place", ctx.pos["left_elbow_joint"] == 0.2)

ctx = FauxContexte({"left_elbow_joint": NAN})
f = SetpointFollower(ctx, DELTA, max_age_s=10.0, damping_after_s=20.0)
f.set_target({"left_elbow_joint": 0.1})
f.tick()
check("Position MESURÉE NaN : aucun pas calculé dessus", ctx.envois == [])

section("3 — Horodatage dans le futur : consigne PÉRIMÉE, pas éternelle")

ctx = FauxContexte({"left_elbow_joint": 0.0})
f = SetpointFollower(ctx, DELTA, max_age_s=0.5, damping_after_s=1.5)
futur = time.monotonic_ns() + int(3600e9)          # une heure d'avance
f.set_target({"left_elbow_joint": 0.3}, timestamp_ns=futur)
f.tick()
check(f"Horodatage +1 h : consigne non suivie (état {f.state.name}), "
      f"avant : âge négatif, jamais périmée",
      f.state in (FollowerState.HOLD, FollowerState.DAMPING) and
      ctx.pos["left_elbow_joint"] == 0.0)
f.set_target({"left_elbow_joint": 0.3}, timestamp_ns=time.monotonic_ns())
f.tick()
check("Horodatage correct ensuite : suivi normal", f.state is FollowerState.TRACKING)

# Trouvé à la revue : NaN est « vrai » en booléen, son âge NaN ne dépassait
# jamais max_age_s — la consigne ne périmait JAMAIS.
for nom, ts in {"NaN": NAN, "+inf": math.inf, "négatif": -5, "texte": "12"}.items():
    ctx = FauxContexte({"left_elbow_joint": 0.0})
    f = SetpointFollower(ctx, DELTA, max_age_s=0.05, damping_after_s=0.1)
    f.set_target({"left_elbow_joint": 0.25}, timestamp_ns=ts)
    for _ in range(10):
        f.tick()
    check(f"Horodatage {nom} : consigne PÉRIMÉE d'emblée (état {f.state.name}), "
          f"l'articulation ne la suit pas",
          f.state in (FollowerState.HOLD, FollowerState.DAMPING)
          and ctx.pos["left_elbow_joint"] == 0.0)
ctx = FauxContexte({"left_elbow_joint": 0.0})
f = SetpointFollower(ctx, DELTA, max_age_s=10.0, damping_after_s=20.0)
f.set_target({"left_elbow_joint": 0.1}, timestamp_ns=0)
f.tick()
check("Horodatage 0 (non renseigné) : comportement inchangé, horodaté à la réception",
      f.state is FollowerState.TRACKING)

section("4 — VLACppConnector : NaN rejeté, horodatage à la capture")

try:
    import zmq
    from axis_os.store.sdk.ai.proto import vla_pb2
    from axis_os.store.sdk.ai.connector import VLACppConnector

    port = [5880]

    def serveur(reponse, delai_s=0.0):
        port[0] += 1
        ep = f"tcp://127.0.0.1:{port[0]}"

        def _run():
            s = zmq.Context.instance().socket(zmq.REP)
            s.bind(ep)
            raw = s.recv()
            req = vla_pb2.PredictRequest()
            req.ParseFromString(raw)
            time.sleep(delai_s)
            r = vla_pb2.PredictResponse()
            r.request_id = req.request_id
            r.action_dim, r.chunk_size = len(reponse), 1
            r.action_chunk.extend(reponse)
            s.send(r.SerializeToString())
            s.close()

        threading.Thread(target=_run, daemon=True).start()
        time.sleep(0.2)
        c = VLACppConnector(endpoint=ep, joint_order=list("abc")[:len(reponse)],
                            lang_tokenizer=lambda t: [1], request_timeout_ms=3000)
        c.connect()
        return c

    IMG = {"height": 1, "width": 1, "data": b"\x00\x00\x00"}
    for nom, rep in {"NaN": [0.1, NAN, 0.3], "inf": [0.1, 0.2, math.inf]}.items():
        c = serveur(rep)
        try:
            c.predict_action(IMG, "x")
            check(f"Action contenant {nom} : rejetée", False)
        except RuntimeError as exc:
            check(f"Action contenant {nom} : rejetée ({exc})", "NaN/inf" in str(exc))
        c.disconnect()

    c = serveur([0.1, 0.2, 0.3], delai_s=0.3)
    a = c.predict_action(IMG, "x")
    age = (time.monotonic_ns() - a.timestamp_ns) / 1e9
    check(f"Serveur lent (0.3 s) : l'action a au moins 0.3 s d'âge "
          f"({age:.2f} s) — horodatée à la CAPTURE, pas à la réception",
          age >= 0.29)
    c.disconnect()
except ImportError as exc:
    print(f"  [—] VLACppConnector non testable ici : {exc}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
