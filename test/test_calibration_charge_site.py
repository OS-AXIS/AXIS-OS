"""
Calibration de charge sur une table que le site connaît, sans affaiblir le
gardien de proximité.

Pourquoi : essai GPU T1 du 20 novembre 2026, 20 h 27. La masse de référence
était posée sur un plateau ajouté devant le robot, que la carte du site ne
connaissait pas : le gardien a vu « obstacle inexpliqué à 0,49 m » et le
robot ne s'est jamais mis en route. Désormais, la masse est posée par
l'opérateur sur un emplacement libre de la table de prise (dans la carte),
et prise exactement comme un carton (ManipulationService.calibrer_charge :
la prise de T1, la même exemption du gardien) ; une personne qui approche
reste détectée et arrête le robot.

Robot simulé léger, G1 29 DOF, mains du G1 ; la table de prise est dans la
carte du site (son rectangle explique ce qui repose dessus), le lidar voit
les personnes :
  1. la masse sur la table connue n'arrête pas le robot ; la calibration se
     fait, l'exemption levée à la fin ;
  2. une personne à 0,5 m pendant la calibration : le gardien arrête le
     robot, les bras ne bougent plus tant qu'elle est là ;
  3. le banc T1 calibre par défaut, sur la table de prise (texte du banc).
"""
import contextlib
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.service import Etat, Objet  # noqa: E402
from axis_os.services.navigation.service import Carte, Rectangle  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
RACINE = Path(__file__).resolve().parents[1]


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


# La table de prise, dans la carte du site (comme scene_demo.TABLE_PRISE) :
# son bord à 0,31 m devant le bassin, plateau à 0,80 m.
TABLE = Rectangle(0.31, -0.8, 0.91, 0.8, zmax=0.80 + 0.25)
CENTRE = (0.41, 0.0, 0.90)


class Banc:
    def __init__(self):
        self.t = [0.0]
        self.r = SimulatedRobot(objects=[SimulatedObject("masse_reference", CENTRE)],
                                horloge=lambda: self.t[0], contact_main_m=0.11,
                                poignets_29dof=True,
                                carte=Carte(-5.0, -5.0, 5.0, 5.0, obstacles=[TABLE]),
                                plateaux=[(0.31, 0.91, -0.8, 0.8, 0.80)])
        self.r.start(fil=False)
        self.m = self.r.manipulation
        self.journal = []
        self.m._journal = self.journal.append
        self.sortie = io.StringIO()
        self.pas(20)

    def pas(self, n=1):
        with contextlib.redirect_stdout(self.sortie):
            for _ in range(n):
                self.t[0] += self.r._dt
                self.r.pas()

    def calibrer(self):
        return self.m.calibrer_charge(Objet(CENTRE, 0.24, 0.20, 0.20), 1.0)


section("1. La masse de référence sur la table connue du site")
b = Banc()
g = b.r.guard
v = g.evaluer(b.t[0])
check("Avant de commencer : la masse sur la table de prise est expliquée par la carte, le "
      "gardien laisse passer", not g.arrete, str(v))
res = b.calibrer()
arrets, exemptee = 0, False
t0 = b.t[0]
while b.m.etat not in (Etat.LIBRE, Etat.ECHEC) and b.t[0] - t0 < 60.0:
    b.pas()
    arrets += g.arrete
    exemptee = exemptee or g._exemption is not None
check(f"Calibrée sur la table connue, jamais arrêtée par le gardien ({arrets} pas arrêtés) ; "
      f"exemption pendant la prise, comme pour un carton, levée à la fin",
      res.ok and b.m.etat is Etat.LIBRE and b.m.calibration_charge is not None
      and arrets == 0 and exemptee and g._exemption is None,
      f"{res} {b.m.etat} {b.m.raison} {arrets}")

section("2. Une personne pendant la calibration : le robot s'arrête")
b = Banc()
g = b.r.guard
b.calibrer()
t0 = b.t[0]
while b.m.etat not in (Etat.FERMETURE, Etat.SERRAGE, Etat.LEVAGE) and b.t[0] - t0 < 30.0:
    b.pas()
etat_avant = b.m.etat
p = b.r.add_person(0.5, 0.3)
b.pas(5)
q0 = dict(b.m._q)
arretee = g.arrete
b.pas(100)
bouge = max(abs(b.m._q[j] - q0[j]) for j in q0)
check(f"Personne à 0,5 m pendant la calibration ({etat_avant.name}) : le gardien arrête le "
      f"robot ; bras immobiles 2 s ({1000 * bouge:.2f} mrad au plus)",
      arretee and g.arrete and bouge < 1e-6, f"{arretee} {g.arrete} {bouge}")
b.r.clear_people()
t0 = b.t[0]
while b.m.etat not in (Etat.LIBRE, Etat.ECHEC) and b.t[0] - t0 < 60.0:
    b.pas()
check("Elle s'en va : le gardien laisse repartir, la calibration finit (ou échoue avec sa "
      "raison), exemption levée", not g.arrete and b.m.etat in (Etat.LIBRE, Etat.ECHEC)
      and g._exemption is None, f"{b.m.etat} {b.m.raison}")

section("3. Le banc T1 calibre par défaut, sur la table de prise")
T1 = (RACINE / "sim/validation/test_isaac_t1_tri_usine.py").read_text(encoding="utf-8")
check("Plus de plateau ajouté devant le robot : la masse de référence posée sur un emplacement "
      "de la table de prise, accostée et prise par la façade (facade.calibrer_charge)",
      "plateau_reference" not in T1 and "facade.calibrer_charge(" in T1
      and "facade.accoster(" in T1 and "sd.EMPLACEMENTS_X" in T1)
check("Calibration par défaut ; --sans-calibration-charge gardé pour les essais ; l'opérateur "
      "ne remplit la table qu'une fois la calibration finie",
      "--sans-calibration-charge" in T1 and "args.sans_calibration_charge" in T1
      and 'and etat_charge.get("fini")' in T1)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
