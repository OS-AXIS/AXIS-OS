"""
Posture : torse penché, bassin abaissé, par la consigne de la marche
(LocomotionService.posture, comme GR00T WBC), pour l'OS seulement.

  1. Disponible sur le G1 29 DOF (sa taille tangue), pas sur le 23 DOF.
  2. Penché de 0,2 rad : accepté, marge d'équilibre écrite, appliqué peu à
     peu (vitesse bornée) ; la cinématique des bras voit la taille penchée.
  3. Penché, le robot ne marche pas ; redressé, il remarche.
  4. Refus : hors des bornes, valeur non finie, robot qui marche.
  5. Gardien de proximité : une personne devant, aucun penché de plus (ni
     demandé, ni en cours : il se fige) ; se redresser reste permis.
  6. Équilibre : une charge lourde loin devant, centre de masse hors de
     l'appui des pieds : refus, la marge écrite.
  7. Aucune app n'y accède : ni le SDK ni le pont des apps n'ont de posture.
  8. Redémarré (après une chute ou un arrêt), le robot repart droit : jamais
     de retour à la posture d'avant, que rien n'a vérifiée de nouveau.
"""
import contextlib
import inspect
import io
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.unitree_g1.gr00t_wbc_policy import (  # noqa: E402
    GR00TWBCAdapter, GR00TWBCPolicy)
from axis_os.store.runtime import hal_bridge  # noqa: E402
from axis_os.store.sdk import app_base  # noqa: E402
from axis_os.testing import SimulatedRobot  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Banc:
    def __init__(self, poignets_29dof=True):
        self.t = [0.0]
        with contextlib.redirect_stdout(io.StringIO()):
            self.r = SimulatedRobot(horloge=lambda: self.t[0], poignets_29dof=poignets_29dof)
            self.r.start(fil=False)
        self.loco = self.r.locomotion
        self.pas(5)

    def pas(self, n=1):
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(n):
                self.t[0] += self.r._dt
                self.r.pas()

    def jusqua_atteinte(self, duree=10.0):
        t0, vus = self.t[0], []
        while not self.loco.posture_atteinte and self.t[0] - t0 < duree:
            avant = self.r.tangage_taille
            self.pas()
            vus.append(abs(self.r.tangage_taille - avant))
        return vus


section("1. Disponible sur le G1 29 DOF seulement")
b = Banc()
check("G1 29 DOF (taille qui tangue) : posture disponible", b.loco.posture_disponible)
b23 = Banc(poignets_29dof=False)
r = b23.loco.posture(0.1)
check("G1 23 DOF : refusée, « ne prend pas de consigne de posture »",
      not r.ok and "ne prend pas de consigne de posture" in r.reason, r.reason)

section("2. Penché de 0,2 rad")
essai = b.loco.posture(0.2, essai=True)
check("Essai : accepté, rien ne change", essai.ok and b.loco.posture_atteinte
      and not b.loco.penche, essai.reason)
r = b.loco.posture(0.2)
check(f"Accepté, marge d'équilibre écrite : « {r.reason} »", r.ok and "cm dans l'appui" in r.reason
      and r.value.marge >= b.loco.bornes_posture.marge_min_m, r.reason)
pas_vus = b.jusqua_atteinte()
vmax = b.loco.bornes_posture.vitesse_tangage_rad_s * b.r._dt
check(f"Appliqué peu à peu : {len(pas_vus)} pas, au plus {max(pas_vus):.4f} rad par pas "
      f"({vmax:.4f} permis)", abs(b.r.tangage_taille - 0.2) < 1e-6 and max(pas_vus) <= vmax + 1e-9
      and len(pas_vus) > 5, f"{b.r.tangage_taille} {max(pas_vus)}")
angles = b.r.manipulation._articulations_fixes()
check("La cinématique des bras de l'OS voit la taille penchée (angles mesurés)",
      abs(angles.get("waist_pitch_joint", 0.0) - 0.2) < 1e-6, str(angles.get("waist_pitch_joint")))

section("3. Penché, le robot ne marche pas")
r = b.loco.set_velocity(0.2, 0.0, 0.0)
check("Marche refusée, « robot penché », à redresser d'abord", not r.ok and "penché" in r.reason,
      r.reason)
x0 = b.r.x
b.pas(20)
check("… et la base n'a pas bougé", abs(b.r.x - x0) < 1e-9, f"{b.r.x} {x0}")
r = b.loco.posture(0.0)
b.jusqua_atteinte()
check("Redressé : posture droite, plus « penché »", r.ok and not b.loco.penche
      and abs(b.r.tangage_taille) < 1e-9, r.reason)
check("… et la marche est de nouveau acceptée", b.loco.set_velocity(0.2, 0.0, 0.0).ok)
b.loco.halt()
b.pas(5)

section("4. Refus : bornes, valeurs non finies, robot qui marche")
for tangage, abaissement, quoi in ((0.5, 0.0, "torse penché de 0,5 rad"),
                                    (-0.1, 0.0, "torse penché en arrière"),
                                    (0.1, 0.2, "bassin abaissé de 20 cm"),
                                    (math.nan, 0.0, "tangage NaN")):
    r = b.loco.posture(tangage, abaissement)
    check(f"{quoi} : refusé ({r.reason})", not r.ok and not b.loco.penche, r.reason)
b.loco.set_velocity(0.2, 0.0, 0.0)
r = b.loco.posture(0.1)
check("Robot qui marche : refusé, « le robot marche »", not r.ok and "marche" in r.reason, r.reason)
b.loco.halt()
b.pas(5)

section("5. Gardien de proximité : une personne devant")
g = Banc()
g.loco.posture(0.2)
g.pas(10)                                   # en cours de penché
deja = g.r.tangage_taille
g.r.add_person(g.r.x + 0.6, g.r.y)
g.pas(5)
fige = g.r.tangage_taille
g.pas(30)
check(f"En cours de penché, la personne arrive : figé à {fige:.3f} rad (pas plus penché)",
      0.0 < deja <= fige + 1e-9 and abs(g.r.tangage_taille - fige) < 1e-9 and fige < 0.2,
      f"{deja} {fige} {g.r.tangage_taille}")
r = g.loco.posture(0.3)
check("Plus penché : refusé, « gardien de proximité »", not r.ok and "gardien" in r.reason, r.reason)
r = g.loco.posture(0.0)
g.jusqua_atteinte()
check("Se redresser reste permis : posture droite", r.ok and abs(g.r.tangage_taille) < 1e-9,
      f"{r.reason} {g.r.tangage_taille}")

section("6. Équilibre : charge lourde loin devant")
e = Banc()
r = e.loco.posture(0.3, charges=[(5.0, (0.7, 0.0, 0.1))])
check(f"5 kg à 70 cm, penché de 0,3 rad : refusé, marge écrite (« {r.reason} »)",
      not r.ok and "équilibre" in r.reason and "HORS de l'appui" in r.reason
      and not e.loco.penche, r.reason)
r = e.loco.posture(0.1, charges=[(1.0, (0.45, 0.0, 0.15))])
check(f"1 kg à 45 cm, penché de 0,1 rad : accepté ({r.reason})", r.ok, r.reason)

section("7. Aucune app n'y accède")
publiques = [n for n, _ in inspect.getmembers(app_base.RobotContext) if "posture" in n.lower()]
check("RobotContext (le SDK des apps) : aucune méthode de posture", not publiques, str(publiques))
check("Pont des apps (hal_bridge) : aucune action de posture",
      "posture" not in inspect.getsource(hal_bridge).lower())

section("8. Redémarré, le robot repart droit")
d = Banc()
d.loco.posture(0.2)
d.jusqua_atteinte()
d.loco.stop()
d.loco.start()
vus = []
for _ in range(20):
    d.pas()
    vus.append(d.r.tangage_taille)
check("Penché de 0,2 rad, arrêté puis redémarré : droit dès le départ, sans retour à la "
      "consigne d'avant", not d.loco.penche and d.loco.equilibre_posture is None
      and max(abs(t) for t in vus) < 1e-9, f"{d.loco.posture_actuelle} {max(vus)}")
check("… et il remarche", d.loco.set_velocity(0.2, 0.0, 0.0).ok)
wbc = GR00TWBCAdapter(GR00TWBCPolicy())
wbc.posture(hauteur=0.64, rpy=(0.0, 0.2, 0.0))
wbc.reset()
check("GR00T WBC réinitialisé (start) : consigne de torse droite, hauteur par défaut",
      wbc.hauteur is None and wbc.rpy == (0.0, 0.0, 0.0), f"{wbc.hauteur} {wbc.rpy}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
