"""
Se rapprocher de la table (décision de revue : « ce qui reste des 11 cm,
tant que la garde des avant-bras au-dessus du plateau est tenue, 3 cm »).

distance_de_prise (distance_ideale), robot simulé léger, G1 29 DOF, mains
du G1, carton de la scène T1 posé de plus en plus loin du bord de la table :
  1. À sa place, au bord de la table : la distance d'avant (premier
     palier, inchangé : 0,36 m, le bassin à 26 cm du bord).
  2. À 25 cm, hors de portée ainsi : le robot se rapproche, pieds contre la
     table au plus, l'erreur d'approche de la marche comprise (bord_min_m
     plus la marge des pieds, du vrai bord).
  3. De 0 à 30 cm du bord : jamais plus près du bord que cette limite ; et la
     prise, planifiée le robot placé à cette distance, la confirme (avant-
     bras à marge_plateau_m au moins du plateau) : l'approche ne contredit
     pas la prise.
  4. À 36 cm : aucune distance, la raison écrite.
"""
import contextlib
import io
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import Objet  # noqa: E402
from axis_os.testing import cartons as K  # noqa: E402
from axis_os.testing.robot_simule import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
R = ReglagesPrise()


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Banc:
    """Le carton T1 à `retrait` du bord de la table (x = 0), robot démarré."""

    def __init__(self, retrait):
        self.t = [0.0]
        c = replace(K.CARTON_T1, retrait_m=retrait)
        self.c = c
        cx, cy, cz = K.place_sur_table(c)
        carton = SimulatedObject("carton", (cx, cy, cz), size=(c.largeur, c.profondeur, c.hauteur),
                                 masse_kg=c.masse_kg)
        with contextlib.redirect_stdout(io.StringIO()):
            self.r = SimulatedRobot(objects=[carton], horloge=lambda: self.t[0],
                                    contact_main_m=0.11, poignets_29dof=True, plateaux=[K.TABLE])
            self.r.start(fil=False)
        self.m = self.r.manipulation
        self.objet = Objet((cx, cy, cz), c.largeur, c.profondeur, c.hauteur,
                           bord_m=cx - c.profondeur / 2.0)

    def distance(self):
        """(distance de prise, bassin au bord de la table, raison)."""
        d, raison = self.m.distance_de_prise(self.objet)
        bord = None if d is None else d - self.c.profondeur / 2.0 - self.objet.bord_m
        return d, bord, raison

    def planifier(self, d):
        """La prise planifiée, le robot placé à `d` du carton : (ok, raison, garde)."""
        self.r.x, self.r.y, self.r.heading = self.objet.centre[0] - d, self.objet.centre[1], 0.0
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(5):
                self.t[0] += self.r._dt
                self.r.pas()
        ok, raison = self.m._planifier(self.objet)
        return ok, raison, (self.m._plan or {}).get("garde") if ok else None


section("1. À sa place, au bord de la table : la distance d'avant")
d, bord, raison = Banc(K.CARTON_T1.retrait_m).distance()
check(f"Distance {d} m, bassin à {bord and round(100 * bord)} cm du bord (0,36 m et 26 cm, comme "
      f"avant)", d is not None and abs(d - 0.36) < 1e-6 and abs(bord - 0.26) < 0.005, raison)

section("2. À 25 cm, hors de portée ainsi : pieds contre la table au plus")
b = Banc(0.25)
# Pieds contre la table, erreur d'approche de la marche comprise (position et cap).
LIMITE = R.bord_min_m + b.m._pieds_au_bord(0.0)[1]
d, bord, raison = b.distance()
check(f"Atteint : distance {d} m, bassin à {bord and round(100 * bord)} cm du bord "
      f"(bord_min_m {100 * R.bord_min_m:.0f} cm plus l'erreur d'approche : "
      f"{100 * LIMITE:.1f} cm, pas plus près)",
      d is not None and LIMITE - 1e-9 <= bord < LIMITE + 0.005, raison)

section("3. De 0 à 30 cm du bord : jamais plus près, et la prise le confirme")
trop_pres, contredits, vus = [], [], 0
for cm in range(0, 31, 2):
    b = Banc(cm / 100.0)
    d, bord, _ = b.distance()
    if d is None:
        continue
    vus += 1
    if bord < LIMITE - 1e-9:
        trop_pres.append(f"{cm} cm : bassin à {100 * bord:.1f} cm")
    ok, raison, garde = b.planifier(d)
    if not ok or garde < R.marge_plateau_m:
        contredits.append(f"{cm} cm : {raison or garde}")
check(f"{vus} places atteintes sur 16 : bassin jamais à moins de {100 * LIMITE:.1f} cm du "
      f"bord", vus >= 14 and not trop_pres, "; ".join(trop_pres))
check("La prise planifiée à cette distance la confirme : avant-bras à 3 cm au moins du plateau",
      not contredits, "; ".join(contredits[:3]))

section("4. À 36 cm : aucune distance, la raison écrite")
d, _, raison = Banc(0.36).distance()
check(f"Refusé : « {raison} »", d is None and "hors de portée" in raison, raison)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
