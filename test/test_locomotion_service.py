"""
LocomotionService — les trois exigences de sécurité, vérifiées.

1. Une consigne non rafraîchie retombe à zéro.
2. S'arrêter n'est pas couper : l'adaptateur continue de tourner.
3. Après une chute, plus aucune consigne.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.types import HALStatus
from axis_os.services.base import IService
from axis_os.services.locomotion.service import (
    BaseState, ILocomotionAdapter, LocomotionService, LocomotionState, Twist,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FakeAdapter(ILocomotionAdapter):
    def __init__(self):
        self.twists, self.fallen, self.resets = [], False, 0

    @property
    def control_dt(self):
        return 0.02

    @property
    def max_twist(self):
        return Twist(0.8, 0.5, 1.57)

    def reset(self):
        self.resets += 1

    def step(self, state, twist):
        self.twists.append(twist)
        return {"left_knee_joint": 0.3 + twist.vx}

    def is_fallen(self, state):
        return self.fallen


UP = BaseState((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), {}, {})


def make(refuse=False):
    ad, clk, sent = FakeAdapter(), Clock(), []

    def send(targets):
        sent.append(targets)
        return {j: (HALStatus.ERROR_LIMIT if refuse else HALStatus.OK)
                for j in targets}

    svc = LocomotionService(ad, lambda: UP, send, command_timeout_s=0.5, clock=clk)
    return svc, ad, clk, sent


section("0 — Contrat")

svc, ad, clk, sent = make()
check("LocomotionService est un IService", isinstance(svc, IService))
check("Aucune méthode abstraite restante",
      not getattr(svc, "__abstractmethods__", None))
check("Avant start() : tick() refuse et n'envoie rien",
      not svc.tick().ok and sent == [])
check("Avant start() : set_velocity() refuse",
      not svc.set_velocity(0.5).ok)
svc.start()
check("start() réinitialise l'adaptateur", ad.resets == 1)

section("1 — Échéance de fraîcheur")

svc.set_velocity(0.5)
svc.tick()
check("Consigne fraîche transmise", ad.twists[-1] == Twist(0.5, 0, 0))
clk.t = 0.4
svc.tick()
check("À 0.4 s (< 0.5 s) : consigne toujours active", ad.twists[-1].vx == 0.5)
clk.t = 0.6
svc.tick()
check("À 0.6 s sans rafraîchissement : consigne retombée à ZÉRO",
      ad.twists[-1].is_zero)
check("L'expiration est comptée", svc.stats["stale"] == 1)

section("2 — S'arrêter n'est pas couper")

svc, ad, clk, sent = make()
svc.start()
svc.set_velocity(0.5)
svc.tick()
n = len(sent)
svc.halt()
svc.tick()
check("halt() : l'adaptateur CONTINUE de recevoir des pas (avec twist nul)",
      len(sent) == n + 1 and ad.twists[-1].is_zero)
check("halt() n'arrête pas le service : un bipède doit garder son équilibre",
      svc.state is LocomotionState.RUNNING)

clk.t = 5.0
for _ in range(3):
    svc.tick()
check("Consigne expirée depuis longtemps : les pas CONTINUENT, à vitesse nulle",
      len(sent) == n + 4 and all(t.is_zero for t in ad.twists[-3:]))

section("3 — Après une chute, plus rien")

svc, ad, clk, sent = make()
svc.start()
svc.set_velocity(0.5)
svc.tick()
n = len(sent)
ad.fallen = True
r = svc.tick()
check("Chute détectée : tick() signale l'échec", not r.ok and "chute" in r.reason)
check("Chute détectée : AUCUNE consigne envoyée ce pas-là", len(sent) == n)
check("État FALLEN", svc.state is LocomotionState.FALLEN)
for _ in range(5):
    svc.tick()
check("Après la chute : plus aucune consigne, même si on insiste",
      len(sent) == n)
check("set_velocity() refusée après une chute",
      not svc.set_velocity(0.3).ok)
check("health() signale la chute", not svc.health().healthy)

section("4 — Bornage et refus")

svc, ad, clk, sent = make()
svc.start()
r = svc.set_velocity(5.0, -9.0, 9.0)
check("Consigne hors domaine bornée à max_twist",
      r.value == Twist(0.8, -0.5, 1.57))
check("Le bornage est signalé, pas silencieux",
      r.reason is not None and svc.stats["clamped"] == 1)

svc, ad, clk, sent = make(refuse=True)
svc.start()
r = svc.tick()
check("Les refus du garde-fou sont comptés et remontés",
      svc.stats["refused"] == 1 and r.reason is not None)

section("5 — L'adaptateur G1 déclare le repère CORPS")

try:
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        G1LocomotionAdapter, G1LocomotionPolicy,
    )
    import inspect
    src = inspect.getsource(G1LocomotionAdapter.step)
    check("G1LocomotionAdapter transmet ang_vel_frame=\"body\" — BaseState "
          "porte le repère corps par contrat",
          'ang_vel_frame="body"' in src)
    g1 = G1LocomotionAdapter(G1LocomotionPolicy(policy_path=None))
    check("Robot droit : pas de chute", not g1.is_fallen(UP))
    import math
    couche = BaseState((math.cos(math.pi / 4), math.sin(math.pi / 4), 0, 0),
                       (0, 0, 0), {}, {})
    check("Robot couché à 90° : chute détectée", g1.is_fallen(couche))
    check("max_twist = domaine d'entraînement de la politique",
          g1.max_twist == Twist(0.8, 0.5, 1.57))
except ImportError as exc:
    print(f"  [—] adaptateur G1 non testable ici : {exc}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
