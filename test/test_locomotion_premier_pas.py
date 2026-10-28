"""
LocomotionService : aucune décision (chute comprise) sur un état que la
physique n'a pas encore mis à jour depuis le démarrage — et, une fois la
physique avancée, la détection de chute s'applique à chaque état, comme
avant.

Constat GPU (RTX PRO 5000, 2026-10-28) : evaluer_vla.sh --reflexes déclarait
une CHUTE à t = 0, avant la première image.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.unitree_g1.locomotion_policy import projected_gravity  # noqa: E402
from axis_os.services.locomotion.service import (  # noqa: E402
    BaseState, ILocomotionAdapter, LocomotionService, LocomotionState, Twist,
)

FAIL = 0
DEBOUT = (1.0, 0.0, 0.0, 0.0)
COUCHE = (math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0)   # 90° de roulis


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


class Adaptateur(ILocomotionAdapter):
    """Chute : la gravité projetée, comme GR00TWBCAdapter (seuil -0,7)."""

    @property
    def control_dt(self) -> float:
        return 0.02

    @property
    def max_twist(self) -> Twist:
        return Twist(0.5, 0.3, 0.6)

    def reset(self) -> None:
        pass

    def step(self, state, twist):
        return {"left_hip_pitch_joint": 0.0}

    def is_fallen(self, state) -> bool:
        return float(projected_gravity(state.quat_wxyz)[2]) > -0.7


def monter(etats):
    envois = []
    suite = iter(etats)
    svc = LocomotionService(Adaptateur(), lambda: next(suite),
                            lambda c: envois.append(c) or {}, clock=lambda: 0.0)
    svc.start()
    return svc, envois


def etat(quat, pas):
    return BaseState(quat, (0.0, 0.0, 0.0), {}, {}, pas_physique=pas)


print("\n  Simulation : premier état pas encore mis à jour par la physique")
svc, envois = monter([etat(COUCHE, 500), etat(DEBOUT, 504), etat(DEBOUT, 508)])
r = svc.tick()
check("État du pas 500 (celui du démarrage), « couché » : ni chute ni consigne",
      not r.ok and svc.state is LocomotionState.RUNNING and not envois
      and svc.stats["attentes_physique"] == 1 and svc.stats["falls"] == 0, r.reason)
r = svc.tick()
check("Physique avancée (pas 504), robot debout : consignes envoyées", r.ok and len(envois) == 1)

print("\n  Une fois la physique avancée, jamais moins sensible")
svc, envois = monter([etat(DEBOUT, 10), etat(DEBOUT, 14), etat(COUCHE, 14)])
svc.tick()
svc.tick()
r = svc.tick()
check("Après le premier pas, un état « couché » même sans nouveau pas : CHUTE",
      svc.state is LocomotionState.FALLEN and svc.stats["falls"] == 1, r.reason)
svc, envois = monter([etat(DEBOUT, 10), etat(COUCHE, 11)])
svc.tick()
svc.tick()
check("Chute au premier état mis à jour par la physique : détectée aussitôt",
      svc.state is LocomotionState.FALLEN)
svc, envois = monter([etat(COUCHE, 10), etat(COUCHE, 10), etat(COUCHE, 12)])
svc.tick()
svc.tick()
check("Physique arrêtée : toujours rien d'envoyé", not envois and svc.state is LocomotionState.RUNNING)
svc.tick()
check("… dès qu'elle avance, le robot couché est vu à terre", svc.state is LocomotionState.FALLEN)

print("\n  Vrai robot (pas de numéro de pas) : inchangé")
svc, envois = monter([etat(COUCHE, None)])
svc.tick()
check("Premier état « couché » : CHUTE aussitôt", svc.state is LocomotionState.FALLEN
      and svc.stats["attentes_physique"] == 0)

print("\n  Scène Isaac Sim T1 (lecture du script, sans Isaac Sim)")
T1 = (Path(__file__).resolve().parents[1] / "sim/validation/test_isaac_t1_tri_usine.py").read_text(
    encoding="utf-8")
boucle = T1.find("while not pilote.termine")
dernier = max(T1.rfind("wait_until_ready", 0, boucle), T1.rfind("rec.start()", 0, boucle))
check("Robot remis debout APRÈS la dernière attente de rendu, juste avant la boucle",
      dernier < T1.rfind("mettre_debout()", 0, boucle) < boucle
      and T1.rfind("loco.start()", 0, boucle) > dernier)
check("L'état lu porte le numéro du pas de physique",
      "pas_physique=getattr(world, \"current_time_step_index\", None)" in T1)

print(f"\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}")
sys.exit(1 if FAIL else 0)
