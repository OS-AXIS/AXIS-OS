"""
Consignes de vitesse NON FINIES (NaN, inf) — refusées, jamais bornées.

Trouvé par l'audit du 2026-10-10 : le bornage max(-m, min(m, v)) de
LocomotionService.set_velocity transformait NaN en +m (toute comparaison
avec NaN est fausse, donc min(m, nan) rend m). Un ctx.walk(nan) faisait
marcher le robot à PLEINE VITESSE vers l'avant. Même défaut dans le
maintien de cap de l'adaptateur G1 : une orientation NaN commandait la
rotation maximale.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.types import HALStatus
from axis_os.services.locomotion.service import (
    BaseState, ILocomotionAdapter, LocomotionService, Twist,
)

FAIL = 0
NAN, INF = float("nan"), float("inf")


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Horloge:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FauxAdaptateur(ILocomotionAdapter):
    def __init__(self):
        self.twists = []

    @property
    def control_dt(self):
        return 0.02

    @property
    def max_twist(self):
        return Twist(0.8, 0.5, 1.57)

    def reset(self):
        pass

    def step(self, state, twist):
        self.twists.append(twist)
        return {"left_knee_joint": 0.3}

    def is_fallen(self, state):
        return False


DEBOUT = BaseState((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), {}, {})


def service():
    ad, h = FauxAdaptateur(), Horloge()
    svc = LocomotionService(ad, lambda: DEBOUT,
                            lambda cibles: {j: HALStatus.OK for j in cibles},
                            command_timeout_s=0.5, clock=h)
    svc.start()
    return svc, ad, h


section("1 — Le défaut d'origine, reproduit sur le bornage seul")

check("max(-m, min(m, nan)) vaut +m : c'est bien le piège corrigé",
      max(-0.8, min(0.8, NAN)) == 0.8)

section("2 — set_velocity refuse toute consigne non finie")

for nom, (vx, vy, wz) in {
    "vx = NaN": (NAN, 0.0, 0.0),
    "vy = NaN": (0.0, NAN, 0.0),
    "wz = NaN": (0.0, 0.0, NAN),
    "vx = +inf": (INF, 0.0, 0.0),
    "vx = -inf": (-INF, 0.0, 0.0),
    "wz = +inf": (0.0, 0.0, INF),
}.items():
    svc, ad, h = service()
    r = svc.set_velocity(vx, vy, wz)
    svc.tick()
    check(f"{nom} : refusée (ok=False, raison donnée)",
          not r.ok and r.reason is not None and "non finie" in r.reason)
    check(f"{nom} : le robot reçoit une vitesse NULLE, pas la vitesse maximale",
          ad.twists[-1].is_zero)

svc, ad, h = service()
svc.set_velocity(NAN)
check("Le refus est compté (stats['non_finies'])", svc.stats["non_finies"] == 1)

section("3 — Une consigne non finie ANNULE la consigne rapide précédente")

svc, ad, h = service()
svc.set_velocity(0.8, 0.0, 0.0)
svc.tick()
check("Avant : 0.8 m/s transmis", ad.twists[-1].vx == 0.8)
h.t = 0.1
svc.set_velocity(NAN, 0.0, 0.0)
svc.tick()
check("Après un NaN : arrêt immédiat, sans attendre l'échéance de la "
      "consigne précédente", ad.twists[-1].is_zero)

section("4 — Les consignes finies sont inchangées")

svc, ad, h = service()
r = svc.set_velocity(0.3, -0.1, 0.5)
svc.tick()
check("Consigne finie acceptée telle quelle",
      r.ok and ad.twists[-1] == Twist(0.3, -0.1, 0.5))
r = svc.set_velocity(5.0, 0.0, 0.0)
check("Consigne finie trop grande : toujours BORNÉE (comportement inchangé)",
      r.ok and r.value.vx == 0.8 and svc.stats["clamped"] == 1)

section("5 — Une app ne peut pas faire marcher le robot avec un NaN")

try:
    from axis_os.store.sdk.app_base import RobotContext
    import inspect
    src = inspect.getsource(RobotContext.walk)
    check("RobotContext.walk passe par LocomotionService.set_velocity "
          "(donc par ce refus)", "set_velocity(vx, vy, wz)" in src)
except ImportError as exc:
    print(f"  [—] RobotContext non importable ici : {exc}")

section("6 — Maintien de cap du G1 : orientation NaN -> aucune rotation")

try:
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        G1LocomotionAdapter, G1LocomotionPolicy,
    )

    class PolitiqueEspion(G1LocomotionPolicy):
        def __init__(self):
            super().__init__(policy_path=None)
            self.consignes = []

        def step(self, quat, omega, jp, jv, cmd, *, ang_vel_frame):
            self.consignes.append(cmd)
            return {}

    esp = PolitiqueEspion()
    ad = G1LocomotionAdapter(esp)
    ad.step(DEBOUT, Twist())                       # cap de référence : 0
    q_nan = BaseState((NAN, 0.0, 0.0, NAN), (0.0, 0.0, 0.0), {}, {})
    ad.step(q_nan, Twist())
    check(f"Orientation NaN : rotation commandée NULLE "
          f"(wz = {esp.consignes[-1][2]}), pas la rotation maximale",
          esp.consignes[-1][2] == 0.0)
    ad.step(BaseState((math.cos(0.15), 0.0, 0.0, math.sin(0.15)),
                      (0.0, 0.0, 0.0), {}, {}), Twist())
    check("Mesure valide ensuite : nouveau cap de référence repris, pas "
          "de correction brusque", abs(esp.consignes[-1][2]) < 1e-6)
except ImportError as exc:
    print(f"  [—] adaptateur G1 non testable ici : {exc}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
