"""
Les lignes réelles de l'essai GPU T1 du 6 novembre 2026, 18 h 21 (une prise,
deux mains ; t1_prise_20261106_1821.csv réduit à ses colonnes utiles),
rejouées dans les fonctions de l'OS. Couples fiables, prise parfaite,
« serrage d'après le poids : G 14.6 N, D 14.4 N par main pour 11.4 N de
poids (requis 14.3, visé 15.3) », carton ramené ; au port, marge de
frottement 1,58 puis 1,93 malgré le resserrage : à reposer. Le carton n'est
pas tombé.

Ce que disent les lignes :
  1. les 11,4 N : la toute première lecture après la remise à zéro de la
     moyenne glissante, à la confirmation de la prise (une lecture seule,
     bruitée) ; le poids de référence était le plus grand lu depuis le
     levage, il l'a gardé. Les quinze lectures qui suivent, bras immobiles :
     médiane 9,85 N, pour 9,81 en vrai ;
  2. au retour, les deux mains ne bougent pas comme un seul corps : la
     cinématique inverse ne tenait que la POSITION des points de contact ;
     les paumes ont piqué de ~15° autour de la ligne des mains, la gauche a
     roulé de ~10° de plus que la droite : le carton s'est tordu entre elles
     (9,7° de rotation, 4,6 cm de glissement à la main gauche, la force du
     juge à gauche de 14,6 à 11,5 N), et l'estimation du poids a monté
     jusqu'à 11,8 N.

Avec les corrections (service.py) : poids pesé une fois, bras immobiles, en
haut du levage (médiane d'une fenêtre) ; retour en corps rigide, chemin
vérifié d'avance ; roulis et tangage du carton surveillés pendant le retour.
"""
import csv
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.cinematique import ModeleBras  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import (  # noqa: E402
    ManipulationService, _rotation_vecteur,
)

FAIL = 0
R = ReglagesPrise()
CSV = Path(__file__).resolve().parent / "donnees" / "t1_prise_20261106_1821.csv"
ART = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch",
       "wrist_yaw")
VRAI = 9.81                               # carton de 1 kg (banc T1)
MU, S = R.frottement_serrage, R.securite_serrage


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def f(r, c):
    return float(r[c]) if r[c] else np.nan


LIGNES = list(csv.DictReader(open(CSV)))
PAS = {}                                   # t -> {main: ligne}
for r in LIGNES:
    PAS.setdefault(f(r, "t_s"), {})[r["main"]] = r
PAS = {t: d for t, d in sorted(PAS.items()) if len(d) == 2}
PHASE = {t: d["left"]["phase"] for t, d in PAS.items()}
LEVAGE = [t for t in PAS if PHASE[t] == "LEVAGE"]
RAMENER = [t for t in PAS if PHASE[t] == "RAMENER"]
TENU = [t for t in PAS if PHASE[t] == "TENU"]


def angles(t):
    return {f"{c}_{a}_joint": f(PAS[t][c], f"angle_{a}_rad") for c in ("left", "right") for a in ART}


def os_():
    """Le service de manipulation réduit à ce que rejouent ces lignes : ses
    réglages, le modèle des bras du G1 et ses 7 articulations par bras
    commandées (celles du banc T1)."""
    m = ManipulationService.__new__(ManipulationService)
    m.r, m._fixes, m._delta = R, {}, {"left": 0.0, "right": 0.0}
    m.mesures = SimpleNamespace(commandables=tuple(angles(LEVAGE[-1])), modele=ModeleBras(),
                                contact_main_m=None)
    return m


section("1. Le constat : un poids gardé trop haut, un carton tordu au retour")
# La moyenne glissante de l'OS (poids_os_n) repart de zéro à la confirmation
# de la prise ; ses valeurs successives rendent les lectures une à une.
moy = [f(PAS[t]["left"], "poids_os_n") for t in LEVAGE]
k0 = next(i for i in range(1, len(moy)) if abs(moy[i] - moy[i - 1]) > 0.8
          and LEVAGE[i] > LEVAGE[0] + 1.0)
n = 3 * R.filtre_force
fen = moy[k0:k0 + n]
lectures = [fen[0]] + [(k + 1) * fen[k] - k * fen[k - 1] for k in range(1, len(fen))]
check(f"Poids de référence de l'OS : {lectures[0]:.2f} N, la première lecture après la remise à "
      f"zéro de la moyenne, à la confirmation (t = {LEVAGE[k0]:.2f} s), gardée comme le plus grand "
      f"lu depuis le levage ({100 * (lectures[0] / VRAI - 1):+.0f} % du vrai) ; requis « 14.3 »",
      abs(lectures[0] - 11.44) < 0.05 and abs(S * lectures[0] / (2 * MU) - 14.3) < 0.05)
pr = [f(PAS[t]["left"], "poids_os_n") for t in RAMENER]
check(f"Au retour, l'estimation du poids monte jusqu'à {max(pr):.1f} N (bras qui bougent, carton "
      f"tordu)", max(pr) > 1.15 * VRAI)
roulis = [f(PAS[t]["left"], "carton_roulis_deg") for t in RAMENER]
cz = [f(PAS[t]["left"], "contact_rel_z_m") for t in RAMENER]
juge = [f(PAS[t]["left"], "force_juge_n") for t in RAMENER]
check(f"Retour (vérité du banc) : carton tourné de {roulis[0]:+.1f}° à {roulis[-1]:+.1f}°, point de "
      f"contact gauche {100 * (cz[-1] - cz[0]):+.1f} cm sur sa face, force du juge à gauche de "
      f"{juge[0]:.1f} à {min(juge):.1f} N", roulis[-1] - roulis[0] > 8.0
      and cz[-1] - cz[0] < -0.03 and min(juge) < juge[0] - 2.5)
M = ModeleBras().au_contact(R.contact_main_m)
p0, p1 = (os_()._poses(angles(t)) for t in (RAMENER[0], RAMENER[-1]))
rot = {c: _rotation_vecteur(p1[c][1] @ p0[c][1].T) for c in ("left", "right")}
relatif = math.degrees(float(np.linalg.norm(rot["left"] - rot["right"])))
check(f"Mains mesurées : paumes tournées de {math.degrees(rot['left'][1]):+.0f}° et "
      f"{math.degrees(rot['right'][1]):+.0f}° autour de la ligne des mains, et l'une par rapport "
      f"à l'autre de {relatif:.0f}° : pas un seul corps rigide (la cinématique inverse ne tenait "
      f"que la position des points de contact)", abs(math.degrees(rot["left"][1])) > 10
      and relatif > 6)

section("2. Avec les corrections")
pese = float(np.median(lectures))
check(f"Poids pesé : médiane des {len(lectures)} lectures qui suivent la confirmation, bras "
      f"immobiles en haut du levage : {pese:.2f} N, à 5 % du vrai ({VRAI}) ; requis "
      f"{S * pese / (2 * MU):.1f} N par main au lieu de 14.3", abs(pese - VRAI) <= 0.05 * VRAI)
m = os_()
m._poses_mesurees0 = m._poses(angles(RAMENER[0]))
seuil = None
for t in RAMENER:
    m._angles_mesures = lambda t=t: angles(t)
    est = m._rotation_estimee()
    if max(abs(v) for v in est) > R.rotation_ramener_max_deg:
        seuil = (t, est, f(PAS[t]["left"], "carton_roulis_deg") - roulis[0])
        break
check(f"Surveillance du retour (_rotation_estimee, sur les mains mesurées) : seuil de "
      f"{R.rotation_ramener_max_deg:g}° passé à t = {seuil[0]:.2f} s (roulis {seuil[1][0]:+.1f}°, "
      f"tangage {seuil[1][1]:+.1f}°), quand le carton n'avait tourné que de {seuil[2]:.1f}° : "
      f"geste arrêté là, resserrage", seuil is not None and abs(seuil[2]) < 3.0, str(seuil))
# Le retour en corps rigide, depuis la posture réelle de la fin du levage.
m = os_()
m._q = angles(LEVAGE[-1])
poses = m._poses(m._q)
u = poses["right"][0] - poses["left"][0]
u = u / np.linalg.norm(u)
c0 = (poses["left"][0] + poses["right"][0]) / 2.0
m._rigide = (c0, {"left": (*poses["left"], u), "right": (*poses["right"], -u)}, dict(m._delta),
             float(np.linalg.norm(poses["right"][0] - poses["left"][0])))
plateau = c0[2] - R.hauteur_test_m - 0.10          # le dessous du carton, levé de 4 cm
vers = c0 + np.array([-0.105, 0.0, R.hausse_transport_m])  # face arrière à 22 cm, comme l'essai
q = m._chemin_rigide(c0, vers, plateau)
check("Retour en corps rigide (_chemin_rigide), de la posture réelle de la fin du levage au "
      "retour de l'essai (10,5 cm vers le buste, 5 cm plus haut) : tout le chemin passe — "
      "étapes atteintes, sans saut de posture, avant-bras au-dessus du plateau, dans les bornes",
      q is not None)
if q is not None:
    fin = m._poses(q)
    ecart = max(math.degrees(float(np.linalg.norm(_rotation_vecteur(fin[c][1] @ poses[c][1].T))))
                for c in ("left", "right"))
    trans = [fin[c][0] - poses[c][0] for c in ("left", "right")]
    check(f"… mains au bout du chemin : orientation inchangée ({ecart:.2f}° au plus), même "
          f"translation pour les deux à {1000 * float(np.linalg.norm(trans[0] - trans[1])):.1f} mm "
          f"près : le carton ne roule ni ne tangue", ecart <= R.tol_rigide_deg
          and float(np.linalg.norm(trans[0] - trans[1])) <= 2 * R.tol_rigide_m)
def somme(t):
    return f(PAS[t]["left"], "force_juge_n") + f(PAS[t]["right"], "force_juge_n")


check(f"Retour d'origine : forces du juge de {somme(RAMENER[0]):.1f} N (somme, début du retour) à "
      f"{somme(TENU[0]):.1f} N (début du port) : {somme(RAMENER[0]) - somme(TENU[0]):.1f} N perdus "
      f"en tordant le carton ; marge {MU * somme(TENU[0]) / lectures[0]:.2f} sur la référence "
      f"d'avant", somme(RAMENER[0]) - somme(TENU[0]) > 5.0)
vise = S * pese / (2 * MU) + R.marge_serrage_n
haut = max(min(f(PAS[t][c], "force_juge_n") for c in ("left", "right")) for t in LEVAGE)
# Le port rejoué : le carton y entre serré au visé, gardé par le retour rigide
# (sans les 6 N perdus) ; les baisses des forces du juge au port sont
# reprises, leurs hausses (le resserrage de l'essai) non.
marges = []
for t in TENU:
    n_ = [vise + min(0.0, f(PAS[t][c], "force_juge_n") - f(PAS[TENU[0]][c], "force_juge_n"))
          for c in ("left", "right")]
    marges.append(MU * sum(n_) / pese)
check(f"Port rejoué avec les corrections : serrage visé d'après le poids pesé, {vise:.1f} N par "
      f"main (les mains en ont donné {haut:.1f} pendant l'essai), gardé au retour rigide, baisses "
      f"du port reprises : marge de frottement {min(marges):.2f} à {max(marges):.2f}, au moins "
      f"{S:g} pendant tout le port", haut >= vise and min(marges) >= S)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
