"""
Les lignes réelles de l'essai GPU T1 du 20 novembre 2026, 21 h 37 (le port :
t1_prise_20261120_2137.csv réduit au ramené et au port, et à l'orientation
du carton donnée par le juge), rejouées dans les fonctions de l'OS.

Ce qui s'est passé : levé, le carton a tourné entre les paumes de -4° à -50°
autour de la ligne des mains, et glissé de ~8 cm vers le bas ; les mains
mesurées n'ont pas tourné de plus de 3,8°. L'OS n'a vu que « mains écartées
de 5 à 13 mm », 8 fois, chaque fois resserrées, puis « ne pèse plus ». La
caméra de tête ne mesurait le carton qu'en entier dans l'image : coupé par
son bord bas, rotation_vue est restée vide.

Avec les corrections :
  1. la caméra de tête mesure la pente du dessus vu en partie
     (rotation_dessus, banc T1) ; l'OS arrête et fait reposer à
     rotation_vue_max_deg (6°). Rejouée sur la vérité du juge, une vue en
     retard de 0,1 s et 2° sous la vérité : arrêt avant 10° de rotation ;
  2. sans la caméra (aveugle), les écartements que l'OS a vus à l'essai :
     au 2e en 10 s, transport interrompu (bien plus tôt que « ne pèse plus »,
     mais après 10° : la caméra est le premier signal, les écartements le
     filet).
Ce que la vue du dessus donnera sur GPU reste à vérifier : ici, sa mesure
est la vérité du juge, dégradée.
"""
import csv
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import ManipulationService  # noqa: E402

FAIL = 0
R = ReglagesPrise()
DONNEES = Path(__file__).resolve().parent / "donnees"
# La caméra de profondeur, lue à chaque pas de l'OS au banc (rotation_tete) :
# une image de retard au plus (0,1 s, prudent), 2° sous la vérité.
LATENCE_S, ERREUR_DEG, PERIODE_S = 0.1, 2.0, 0.1
# « port : mains écartées de … mm de leur serrage » au journal de l'essai.
ECARTEMENTS_ESSAI = (52.1, 55.0, 58.2, 60.7, 63.4, 65.2, 65.8, 66.7, 71.8)
# L'écart lu au plus bas entre les deux premiers (t1_prise_20261120_2137.csv,
# faces commandées et mesurées, arrondies au mm : 0 mm à 52,9 s).
CREUX_ENTRE_DEUX_MM = 0.0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


L = [(float(r["t_s"]), r["phase"], float(r["carton_roulis_deg"]))
     for r in csv.DictReader(open(DONNEES / "t1_port_20261120_2137.csv"))]


def vrai(t):
    """Roulis du carton (juge) à l'instant t, la ligne la plus proche avant."""
    return next((v for tt, _, v in reversed(L) if tt <= t + 1e-9), L[0][2])


section("1. La vérité : le carton tourne entre les paumes")
t10 = next(t for t, _, v in L if abs(v) > 10.0)
ram = [abs(v) for _, ph, v in L if ph == "RAMENER"]
debut_port = next(t for t, ph, _ in L if ph == "TENU")
check(f"Tourné de {max(abs(v) for _, _, v in L):.0f}° au plus ; 10° passés à {t10:.2f} s, "
      f"{t10 - debut_port:.1f} s après le début du port ; au ramené, {max(ram):.1f}° au plus",
      max(abs(v) for _, _, v in L) > 40.0 and max(ram) < R.rotation_vue_max_deg)

section("2. La caméra de tête, dessus vu en partie : arrêt avant 10°")
m = ManipulationService.__new__(ManipulationService)
m.r = R
m._init_mesure()
vue = [None]
m.mesures = SimpleNamespace(rotation=lambda: vue[0])
arret, t_vue = None, L[0][0]
for t, ph, _ in L:
    if t + 1e-9 >= t_vue:
        v = vrai(t - LATENCE_S)
        vue[0] = (abs(v) - ERREUR_DEG) * (1 if v > 0 else -1)
        t_vue += PERIODE_S
    raison = m._surveiller_rotation(None)
    if raison:
        arret = (t, ph, raison)
        break
check(f"Vue en retard de {LATENCE_S:g} s, {ERREUR_DEG:g}° sous la vérité, toutes les "
      f"{PERIODE_S:g} s : arrêt à {arret[0] if arret else float('nan'):.2f} s, carton tourné "
      f"de {abs(vrai(arret[0])) if arret else float('nan'):.1f}° (au plus 10°)",
      arret is not None and arret[0] < t10 and abs(vrai(arret[0])) < 10.0
      and "il tourne entre les paumes" in arret[2], str(arret))
check("… jamais au ramené (pas de fausse alerte avant le port)",
      arret is not None and arret[1] == "TENU", str(arret))
vue[0] = 0.0
check(f"La vue n'autorise rien : son seuil ({R.rotation_vue_max_deg:g}°) ne dépasse pas "
      f"rotation_max_deg ({R.rotation_max_deg:g}°) ; un carton vu droit ne fait que se taire",
      R.rotation_vue_max_deg <= R.rotation_max_deg and m._surveiller_rotation(None) == "")

section("3. Caméra aveugle : les écartements vus par l'OS à l'essai")
deuxieme = next(t for i, t in enumerate(ECARTEMENTS_ESSAI) if i >= R.ecartements_max - 1
                and t - ECARTEMENTS_ESSAI[i - R.ecartements_max + 1] <= R.fenetre_ecartements_s)
check(f"Au {R.ecartements_max}e écartement en {R.fenetre_ecartements_s:g} s : transport "
      f"interrompu à {deuxieme:.1f} s (carton tourné de {abs(vrai(deuxieme)):.0f}°), au lieu de "
      f"marcher jusqu'à « ne pèse plus » (~72 s, {abs(vrai(71.8)):.0f}°)",
      deuxieme < 60.0 and abs(vrai(deuxieme)) < abs(vrai(71.8)))
bas = 1000 * (R.ecart_face_port_m - R.hysteresis_ecart_m)
check(f"… avec l'hystérésis des écartements ({1000 * R.hysteresis_ecart_m:g} mm) : entre les deux "
      f"premiers, l'écart lu est redescendu à {CREUX_ENTRE_DEUX_MM:g} mm (arrondi au mm), sous "
      f"{bas:g} : le 2e compte, au même instant", CREUX_ENTRE_DEUX_MM + 1.0 < bas)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
