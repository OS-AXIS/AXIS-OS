"""
Arrêt de sécurité qui dure pour la même cause : l'app le signale à
l'opérateur, avec la cause, au bout de 20 s.

Constat GPU (essai T1 du 5 novembre 2026) : un carton poussé de 5 cm hors de
la table de prise arrête le robot pendant 15 à 19 s, quatre fois (« obstacle
inexpliqué à 0.50 m — ARRÊT »). Le gardien a raison d'arrêter ; on n'y touche
pas. Mais l'app attendait en silence : l'arrêt allait par à-coups (le gardien
relâche un instant, puis arrête de nouveau), et chaque reprise remettait à
zéro le compte de l'alerte des 60 s.

Désormais (logique.py, facade.py) : après ConfigTri.alerte_meme_cause_s
(20 s) d'arrêt pour la même cause (même point à rayon_ignore_m près,
reprises brèves comprises), une alerte donne la cause : ce que dit le
gardien, le point, de combien il dépasse le bord de quelle table (carte),
et le carton suivi le plus proche. Rien que ce que l'OS perçoit.
"""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from axis_os.apps.tri_colis import scene_demo  # noqa: E402
from axis_os.apps.tri_colis.facade import FacadeServices  # noqa: E402
from axis_os.apps.tri_colis.logique import ConfigTri, TriColis  # noqa: E402
from axis_os.services.locomotion.proximite import Verdict  # noqa: E402
from axis_os.services.navigation.service import Pose2D  # noqa: E402
from monde_tri_factice import REGLES, MondeFactice, executer  # noqa: E402

FAIL = 0
MEME = "toujours pour la même cause"
RAISON = "obstacle inexpliqué à 0.50 m — ARRÊT"
POINT = (3.53, 1.93)            # le point du gardien dans l'essai GPU


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def facade(verdict, horloge=lambda: 0.0):
    """Façade sur la carte de la scène T1, avec les cartons qu'elle suit
    après sa dernière recherche (bleu et rouge, à leur place)."""
    f = FacadeServices(SimpleNamespace(carte=scene_demo.carte()),
                       SimpleNamespace(dernier_verdict=verdict), SimpleNamespace(),
                       SimpleNamespace(), lambda: Pose2D(3.61, 1.44, scene_demo.CAP_PRISE),
                       horloge, scene_demo.CAP_PRISE, journal=lambda m: None)
    f._suivis = [{"couleur": "rouge", "centre": (4.40, 2.10, 0.90)},
                 {"couleur": "bleu", "centre": (3.60, 2.10, 0.90)}]
    return f


section("1. La façade dit ce qui arrête le robot")
point, texte = facade(Verdict(0.0, 0.50, RAISON, POINT)).cause_arret()
check("Carton qui dépasse : la raison du gardien, le point, la table, le carton",
      point == POINT and texte.startswith(RAISON) and "point en (3.53, 1.93)" in texte
      and "il dépasse de 7 cm le bord de la table du poste « prise »" in texte
      and "carton bleu à 0.18 m (vu en dernier en (3.60, 2.10))" in texte
      and "sans doute ce carton, à remettre en place" in texte, texte)
_, texte = facade(Verdict(0.0, 0.40, "personne à 0.40 m — ARRÊT", (2.0, 1.0))).cause_arret()
check("Personne dans l'allée : ni table ni carton", texte ==
      "personne à 0.40 m — ARRÊT, point en (2.00, 1.00)", texte)
_, texte = facade(Verdict(0.0, 0.40, RAISON, (3.53, 1.70))).cause_arret()
check("Point à 30 cm du bord (plus loin qu'un carton ne peut dépasser) : pas de table",
      "table" not in texte and "carton" not in texte, texte)
check("Sans mesure du point : la raison seule",
      facade(Verdict(0.0, None, "lidar muet — ARRÊT")).cause_arret() == (None, "lidar muet — ARRÊT"))


class Monde(MondeFactice):
    """Monde factice dont la façade dit la cause de l'arrêt : `causes`, une
    liste de (début, point, texte), la dernière commencée s'applique."""

    def __init__(self, humains, causes):
        super().__init__([("rouge", (0.0, 0.0, 0.8))], humains=humains)
        self.causes = causes

    def cause_arret(self):
        _, p, txt = [c for c in self.causes if c[0] <= self.t][-1]
        return p, txt


def a_coups(debut, fin):
    """Arrêts de 1.2 s séparés d'une reprise de 0.1 s, comme dans l'essai."""
    return [(t, min(t + 1.2, fin)) for t in [debut + 1.3 * k for k in range(1000)] if t < fin]


def alertes(m):
    return [(float(l[1:8]), l) for l in m.logs if "ALERTE" in l and MEME in l]


section("2. L'app signale l'arrêt qui dure, même par à-coups")
m = Monde(a_coups(5.0, 45.0), [(0.0, POINT, "carton bleu qui dépasse")])
tri = executer(m, 60.0)
a = alertes(m)
check("UNE alerte, au bout de 20 s, avec la cause donnée par la façade",
      len(a) == 1 and 24.9 <= a[0][0] <= 25.5 and "carton bleu qui dépasse ; merci d'intervenir"
      in a[0][1] and "depuis 20 s" in a[0][1], str(a))
check("… (l'alerte des 60 s, elle, ne part pas : chaque reprise la remet à zéro)",
      not any("arrêt de sécurité depuis" in l for l in m.logs))
check("… aucune commande pendant les arrêts", not m.violations, str(m.violations[:3]))

section("3. Même cause seulement")
m = Monde(a_coups(5.0, 45.0), [(0.0, POINT, "carton bleu"), (15.0, (4.44, 1.95), "carton rouge")])
executer(m, 60.0)
a = alertes(m)
check("La cause change à 15 s (autre point) : le compte repart, alerte à 35 s, carton rouge",
      len(a) == 1 and 34.9 <= a[0][0] <= 35.5 and "carton rouge" in a[0][1], str(a))
m = Monde(a_coups(5.0, 45.0), [(0.0, POINT, "carton bleu"), (15.0, (3.55, 1.95), "carton bleu")])
executer(m, 60.0)
a = alertes(m)
check("Le point bouge de 3 cm (mesure) : même cause, alerte à 25 s",
      len(a) == 1 and 24.9 <= a[0][0] <= 25.5, str(a))
m = Monde(a_coups(5.0, 17.0) + a_coups(27.0, 40.0), [(0.0, POINT, "carton bleu")])
executer(m, 60.0)
check("12 s d'arrêt, 10 s de reprise, 13 s d'arrêt : pas d'alerte (deux arrêts courts)",
      not alertes(m), str(alertes(m)))
m = MondeFactice([("rouge", (0.0, 0.0, 0.8))], humains=[(5.0, 125.0)])
executer(m, 200.0, ConfigTri(regles=dict(REGLES), duree_mission_s=200.0))
check("Robot qui ne dit pas la cause : seule l'alerte des 60 s, comme avant",
      not alertes(m) and sum("arrêt de sécurité depuis" in l for l in m.logs) == 1)

section("4. De bout en bout : la façade et l'app")
t = [0.0]
f = facade(Verdict(0.0, 0.50, RAISON, POINT), horloge=lambda: t[0])
journal = []
f._journal = journal.append
tri = TriColis(f, ConfigTri(regles={"bleu": "C"}, duree_mission_s=1e6))
while t[0] < 30.0:
    tri.pas()
    t[0] += 0.05
a = [l for l in journal if MEME in l]
check("Carton bleu sorti de 7 cm de la table : l'opérateur le sait au bout de 20 s",
      len(a) == 1 and "il dépasse de 7 cm le bord de la table du poste « prise »" in a[0]
      and "carton bleu" in a[0] and tri.bilan.alertes and MEME in tri.bilan.alertes[-1], str(a))
print("  " + (a[0] if a else ""))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
