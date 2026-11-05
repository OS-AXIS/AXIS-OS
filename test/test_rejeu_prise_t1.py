"""
Les lignes réelles de l'essai GPU T1 du 5 novembre 2026 (5 prises, 2 mains,
t1_prise_20261105_1250.csv réduit à ses colonnes utiles), rejouées dans la
logique de décision de la prise : signal_contact (la fermeture) et
main_retenue (le serrage jugé en géométrie), celles qu'appelle
ManipulationService.

Constat : 5 prises tentées, 0 triée, toutes « serrage insuffisant (-2.9 à
-3.4 N à la main au plus serré, 5 au moins) » ; juge (vérité simulation) :
10 « contacts manqués », force réelle de 6 à 9.5 N par main.

Ce que disent les lignes :
  1. le contact, l'OS l'avait vu, en géométrie, sur les 10 mains, juste
     avant de passer au serrage (recalage au toucher à chaque essai) ; le
     juge du banc le perdait (voir test_banc_t1_verite.py) ;
  2. la force à la main ne suivait pas la force réelle : les couples lus
     n'étaient que le bruit du banc (voir CalibrationCouples.fiabilite) ;
  3. le serrage, jugé sur cette force, était refusé ; jugé en géométrie, les
     10 mains étaient retenues par le carton, retard tenu.

Approximations du rejeu : la hausse du couple du roulis est prise sans la
variation de pesanteur du bras pendant la fermeture (le CSV n'a pas les
angles), et le roulis n'est pas vu en retard (pas dans le CSV).
"""
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.prise import (  # noqa: E402
    ReglagesPrise, main_retenue, signal_contact,
)

FAIL = 0
R = ReglagesPrise()
LARGEUR = 0.24                     # le carton de la scène T1
CSV = Path(__file__).resolve().parent / "donnees" / "t1_prise_20261105_1250.csv"


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def essais():
    """Les prises : lignes de chaque main, de la sous-phase « bruit » à la
    fin du serrage."""
    sortie, cour = [], None
    for row in csv.DictReader(open(CSV)):
        if row["phase"] == "FERMETURE" and row["sous_phase"] == "bruit" and (
                cour is None or any(x["phase"] == "SERRAGE" for x in cour)):
            cour = []
            sortie.append(cour)
        cour.append(row)
    return [{m: [x for x in e if x["main"] == m] for m in ("left", "right")} for e in sortie]


def f(x, c):
    return float(x[c]) if x[c] else 0.0


def contact(lignes, sens, fiables):
    """Rejoue la fermeture : (instant, signal) du contact confirmé (2 lignes
    de suite), ou None."""
    bruit = [x for x in lignes if x["sous_phase"] == "bruit"]
    tau0 = np.mean([f(x, "couple_shoulder_roll_nm") for x in bruit])
    ferm = [x for x in lignes if x["phase"] == "FERMETURE" and not x["sous_phase"]]
    e0 = f(ferm[0], "face_mesuree_m") - f(ferm[0], "face_commandee_m")
    compte = 0
    for x in ferm:
        fc, fm = f(x, "face_commandee_m"), f(x, "face_mesuree_m")
        s = signal_contact(R, abs(f(x, "couple_shoulder_roll_nm") - tau0), 0.0,
                           f(x, "force_normale_n"), R.force_contact_n,
                           sens * ((fm - fc) - e0), abs(fm - sens * LARGEUR / 2), fiables)
        compte = compte + 1 if s else 0
        if compte >= 2:
            return float(x["t_s"]), s, e0
    return None


E = essais()
section(f"1. Contact : {len(E)} prises, {2 * len(E)} mains, les lignes de la fermeture")
vus = {}
for fiables in (True, False):
    for n, e in enumerate(E, 1):
        for main, sens in (("left", 1.0), ("right", -1.0)):
            c = contact(e[main], sens, fiables)
            t_serrage = next(float(x["t_s"]) for x in e[main] if x["phase"] == "SERRAGE")
            vus[(fiables, n, main)] = (c, t_serrage)
for fiables, nom in ((False, "couples non fiables (le verdict de cet essai)"),
                     (True, "couples crus fiables (main)")):
    ok = [k for k, (c, t) in vus.items() if k[0] == fiables and c and c[1] == "géométrie"
          and c[0] <= t]
    check(f"{nom} : contact en géométrie sur les {2 * len(E)} mains, avant le passage au "
          f"serrage", len(ok) == 2 * len(E),
          str([(k, c) for k, (c, t) in vus.items() if k[0] == fiables]))

section("2. La force à la main ne suivait pas la force réelle")
contacts = [x for e in E for m in e.values() for x in m if x["phase"] == "SERRAGE"]
juge = np.array([f(x, "force_juge_n") for x in contacts])
estimee = np.array([f(x, "force_normale_n") for x in contacts])
roulis = np.array([f(x, "couple_shoulder_roll_nm") for x in contacts])
check(f"Serrage : force réelle {juge.mean():.1f} N en moyenne (juge), force estimée "
      f"{estimee.mean():.1f} N, corrélation {np.corrcoef(juge, estimee)[0, 1]:+.2f}",
      juge.mean() > 4.0 and estimee.mean() < 0.0
      and abs(np.corrcoef(juge, estimee)[0, 1]) < 0.5)
check(f"… couple lu du roulis : {roulis.mean():+.2f} ± {roulis.std():.2f} N·m, corrélation avec "
      f"la force réelle {np.corrcoef(juge, roulis)[0, 1]:+.2f} (attendu : plus d'1 N·m de hausse)",
      roulis.std() < 0.5 and abs(np.corrcoef(juge, roulis)[0, 1]) < 0.2)

section("3. Serrage : jugé en géométrie, les mains étaient retenues")
for n, e in enumerate(E, 1):
    for main, sens in (("left", 1.0), ("right", -1.0)):
        c, _ = vus[(False, n, main)]
        serr = [x for x in e[main] if x["phase"] == "SERRAGE"]
        touchee = f(serr[0], "face_mesuree_m")             # face touchée (recalage)
        fin = serr[-1]
        t_fin = float(fin["t_s"])
        maintien = [x for x in serr if float(x["t_s"]) >= t_fin - R.duree_maintien_s]
        retards = [sens * ((f(x, "face_mesuree_m") - f(x, "face_commandee_m")) - c[2])
                   for x in maintien]
        ecart = abs(f(fin, "face_mesuree_m") - touchee)
        retenue = main_retenue(R, retards[-1], ecart)
        faces = [f(x, "face_mesuree_m") for x in maintien]      # main bloquée ?
        stable = max(faces) - min(faces) <= R.glissement_serrage_m
        force_ok = f(fin, "force_normale_n") >= R.force_min_n
        check(f"prise {n} main {'G' if main == 'left' else 'D'} : retenue à "
              f"{100 * retards[-1]:.1f} cm de sa consigne, à {100 * ecart:.1f} cm de la face "
              f"touchée, main immobile à {1000 * (max(faces) - min(faces)):.0f} mm près "
              f"(force réelle {f(fin, 'force_juge_n'):.1f} N) ; jugée sur la force "
              f"({f(fin, 'force_normale_n'):+.1f} N) : refusée", retenue and stable and not force_ok)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
