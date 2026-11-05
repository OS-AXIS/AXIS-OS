"""
Couples lus dans Isaac Sim : l'effort MESURÉ par la physique, plus les
erreurs d'estimation du G1, au lieu des efforts imposés.

Constat (essai GPU T1 du 5 novembre 2026) : le pont, le backend et le banc
lisaient get_applied_joint_efforts, les efforts passés à set_joint_efforts,
nuls quand les articulations sont commandées en position. La prise ne
recevait que du bruit, et les couples vérifiés contre les limites de l'URDF
(garde-fou, campagne de validation S1.2) valaient zéro.

Désormais (bruit_couples.source_couples) : get_measured_joint_efforts (la
force de l'articulation projetée sur son axe) s'il rend une valeur finie
par articulation, sinon les efforts imposés ; le texte dit laquelle, et
pourquoi. Le pont (read_feedback, read_all_raw : garde-fou, pompe), le
backend et le banc T1 lisent la même.

Sans Isaac Sim : articulations factices aux méthodes d'Isaac Sim.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.isaac_sim.backend import IsaacSimBackend  # noqa: E402
from axis_os.hal.drivers.isaac_sim.bridge import IsaacSimBridge  # noqa: E402
from axis_os.hal.drivers.isaac_sim.bruit_couples import BruitCouples, source_couples  # noqa: E402

FAIL = 0
RACINE = Path(__file__).resolve().parents[1]


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Articulation:
    """Articulation factice : `mesure` (couples de la physique), ou une
    méthode qui manque, lève, ou rend trop peu de valeurs ; les imposés sont
    nuls (commande en position). `vue` : rend (1, n), comme une vue Isaac."""

    def __init__(self, mesure, vue=False, panne=None):
        self.mesure, self.vue, self.panne = np.asarray(mesure, dtype=float), vue, panne
        if panne != "absente":
            self.get_measured_joint_efforts = self._mesure

    def _mesure(self):
        if self.panne == "erreur":
            raise RuntimeError("vue physique non initialisée")
        v = self.mesure[:-1] if self.panne == "courte" else self.mesure
        if self.panne == "nan":
            v = v.copy()
            v[0] = math.nan
        return v[None, :] if self.vue else v

    def get_applied_joint_efforts(self):
        z = np.zeros_like(self.mesure)
        return z[None, :] if self.vue else z

    def get_joint_positions(self):
        z = np.zeros_like(self.mesure)
        return z[None, :] if self.vue else z

    get_joint_velocities = get_joint_positions


VRAIS = np.array([5.3, -2.1, 0.4, 12.0])     # couples de la physique (N·m)

section("1. Le choix de la source, et ce qu'en dit le journal")
lire, texte = source_couples(Articulation(VRAIS), 4)
check("Effort mesuré disponible : c'est lui qu'on lit, et on le dit",
      np.allclose(lire(), VRAIS) and texte == "couples lus : effort MESURÉ par la physique "
      "(get_measured_joint_efforts)", texte)
lire, texte = source_couples(Articulation(VRAIS, vue=True), 4)
check("Vue (1, n) d'Isaac : mise à plat", lire().shape == (4,) and np.allclose(lire(), VRAIS))
for panne, raison in (("absente", "méthode absente de cette version d'Isaac Sim"),
                      ("erreur", "lecture impossible (RuntimeError : vue physique non "
                                 "initialisée)"),
                      ("courte", "3 valeur(s) pour 4 articulations"),
                      ("nan", "ou des valeurs non finies")):
    lire, texte = source_couples(Articulation(VRAIS, panne=panne), 4)
    check(f"Effort mesuré {panne} : efforts imposés (nuls), raison dite, NON FIABLES annoncés",
          np.allclose(lire(), 0.0) and "efforts IMPOSÉS (get_applied_joint_efforts), nuls en "
          "commande de position" in texte and raison in texte and "NON FIABLES" in texte, texte)


class Rien:
    pass


check("Ni l'un ni l'autre : rien à lire (None), et on le dit",
      source_couples(Rien(), 4)[0] is None and "NON FIABLES" in source_couples(Rien(), 4)[1])

section("2. Le pont : garde-fou et pompe lisent l'effort mesuré")
b = IsaacSimBridge(variante="29dof")
noms = [d.joint_id for d in b._joint_defs][:4]
b._robot = Articulation(VRAIS)
b._joint_index = {j: i for i, j in enumerate(noms)}
b._choisir_source_couples(4)
fb = b.read_feedback(noms[3])
check("read_feedback (JointSafetyGuard) : le couple de la physique, 12 N·m (avant : 0)",
      fb.torque == 12.0 and b.source_couples.startswith("couples lus : effort MESURÉ"),
      str(fb.torque))
check("read_all_raw (pompe du mode sandbox) : les mêmes couples",
      [b.read_all_raw()[j][2] for j in noms] == list(VRAIS))
b.bruit_couples = BruitCouples(4, graine=2)
ecarts = [b.read_feedback(noms[0]).torque - VRAIS[0] for _ in range(400)]
check("Avec les erreurs d'estimation du G1 : autour du couple mesuré (gain ±5 %, biais "
      "±0,5, bruit 0,3 N·m)", abs(np.mean(ecarts)) < 0.8 and 0.2 < np.std(ecarts) < 0.4,
      f"{np.mean(ecarts):+.2f} ± {np.std(ecarts):.2f}")
b = IsaacSimBridge(variante="29dof")
b._robot = Articulation(VRAIS, panne="absente")
b._joint_index = {j: i for i, j in enumerate(noms)}
b._choisir_source_couples(4)
check("Effort mesuré absent : couple imposé (0), comme avant, et dit",
      b.read_feedback(noms[3]).torque == 0.0 and "efforts IMPOSÉS" in b.source_couples)
b = IsaacSimBridge(variante="29dof")
b._robot, b._joint_index = Rien(), {}
check("Avant setup() : rien de lu (NaN), la source le dit",
      b.couples() is None and "pas encore lus" in b.source_couples)

section("3. Le backend Isaac (vue (1, n)) : même source")
k = IsaacSimBackend(force_stub=True)
k._articulation = Articulation(VRAIS, vue=True)
k._joint_indices = {"a": 0, "b": 3}
k._lire_couples, k.source_couples = source_couples(k._articulation, 4)
check("État d'une articulation : le couple de la physique (avant : l'imposé, 0)",
      k._get_joint_state_production("b").torque == 12.0
      and k._get_joint_state_production("a").torque == 5.3)

section("4. Le banc T1 lit la même source, et la dit au journal")
T1 = (RACINE / "sim/validation/test_isaac_t1_tri_usine.py").read_text(encoding="utf-8")
check("Couples de la prise : bridge.couples() (source du pont + erreurs d'estimation), "
      "plus get_applied_joint_efforts",
      "e = bridge.couples()" in T1 and "get_applied_joint_efforts" not in T1)
check("Au démarrage, le journal dit la source",
      'journaliser(f"[{sim_t[0]:7.1f}s] {bridge.source_couples}")' in T1)
for nom in ("bridge.py", "backend.py"):
    src = (RACINE / "axis_os/hal/drivers/isaac_sim" / nom).read_text(encoding="utf-8")
    lus = [l for l in src.splitlines() if "get_applied_joint_efforts()" in l
           and "= self._articulation.get_applied_joint_efforts()" not in l]
    check(f"{nom} : plus aucun couple lu par get_applied_joint_efforts (sauf l'écriture du "
          f"mode couple du backend, qui relit ce qu'il impose)", not lus, str(lus))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
