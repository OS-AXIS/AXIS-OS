"""
Les lignes réelles de l'essai GPU T1 du 5 novembre 2026, 16 h 06 (une
prise, deux mains ; t1_prise_20261105_1606.csv réduit à ses colonnes
utiles), rejouées dans la cinématique et l'estimation de la force de l'OS.

Constat : couples mesurés, jugés fiables (pente 1,00 / 0,99) ; contact vu
par la force sur les deux mains, serrage, levage ; puis le carton levé a
PIVOTÉ autour de la ligne des mains et il est tombé. Force estimée en fin de
serrage 8 à 10 N, celle du juge 6,3 à 6,8 N (35 % de trop).

Ce que disent les lignes (posture reconstruite par cinématique inverse : ce
relevé n'avait pas les angles ; le banc les relève désormais) :
  1. la surestimation est un gain (estimée ≈ 1,3 × juge, sans décalage) : ni
     le frottement, ni la force totale contre la normale (le juge somme déjà
     les impulsions, frottement compris) ; c'est le bras de levier : estimée
     au repère de la main, la force rejoint celle du juge avec le contact
     11 cm plus loin le long de la main (ReglagesPrise.contact_main_m) ;
  2. la ligne qui joignait les points de contact passait donc à ~11 cm du
     centre de gravité, en profondeur : 6 fois la tolérance (tolerances_cg),
     d'où le basculement ; la prise visant désormais le point de contact au
     milieu des faces, la ligne y passe.
"""
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.cinematique import ModeleBras  # noqa: E402
from axis_os.services.manipulation.prise import (  # noqa: E402
    ReglagesPrise, cibles_mains, ecart_ligne_cg, resoudre, tolerances_cg,
)

FAIL = 0
R = ReglagesPrise()
CSV = Path(__file__).resolve().parent / "donnees" / "t1_prise_20261105_1606.csv"
# Le carton bleu dans le repère du bassin, au toucher (journal de l'essai :
# « bleu réel (0.331, 0.014, 0.140) »), et sa largeur.
CENTRE = np.array([0.331, 0.015, 0.140])
ART = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch",
       "wrist_yaw")


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
MODELE = ModeleBras()


def posture(chaine, sens, face):
    """Angles du bras dont le repère de la main est là où la prise l'avait
    mis : au centre du carton en profondeur et en hauteur, sa face
    intérieure à `face` du centre (poignets et lacet d'épaule à zéro)."""
    q, _ = chaine.inverse(CENTRE + np.array([0.0, face + sens * R.demi_main_m, 0.0]), {})
    return {**{j: 0.0 for j in chaine.bornes}, **q}


def force(chaine, q, joints, dtau, bout):
    """La force à la main de l'OS (pseudo-inverse amortie de Jᵀ), le contact
    à `bout` le long de la main."""
    ch = MODELE.au_contact(bout)
    ch = ch.gauche if chaine.cote == "left" else ch.droite
    jac = ch.jacobienne(q, joints)
    return np.linalg.solve(jac @ jac.T + R.amorti_force_m ** 2 * np.eye(3), jac @ dtau)


section("1. La force à la main : un gain, celui du bras de levier")
resultats = {}
for cote, sens, chaine in (("left", 1.0, MODELE.gauche), ("right", -1.0, MODELE.droite)):
    joints = [f"{cote}_{a}_joint" for a in ART]
    L = [r for r in LIGNES if r["main"] == cote]
    bruit = [r for r in L if r["sous_phase"] == "bruit"]
    serr = [r for r in L if r["phase"] == "SERRAGE" and f(r, "force_juge_n") > 5.0]
    tout = [r for r in L if r["phase"] in ("FERMETURE", "SERRAGE") and not r["sous_phase"]
            and f(r, "force_juge_n") > 1.0]
    juge = np.array([f(r, "force_juge_n") for r in tout])
    est = np.array([f(r, "force_normale_n") for r in tout])
    a, b = np.linalg.lstsq(np.vstack([juge, np.ones_like(juge)]).T, est, rcond=None)[0]
    check(f"main {'G' if sens > 0 else 'D'} : estimée = {a:.2f} × juge {b:+.2f} N, corrélation "
          f"{np.corrcoef(juge, est)[0, 1]:.2f} — un gain (1,2 à 1,5), pas un biais (< 1 N)",
          1.2 <= a <= 1.5 and abs(b) < 1.0 and np.corrcoef(juge, est)[0, 1] > 0.9)
    q = posture(chaine, sens, np.mean([f(r, "face_mesuree_m") for r in serr]))
    q0 = posture(chaine, sens, np.mean([f(r, "face_mesuree_m") for r in bruit]))
    tau0 = np.array([np.mean([f(r, f"couple_{x}_nm") for r in bruit]) for x in ART])
    dg = chaine.couples_gravite(q, joints) - chaine.couples_gravite(q0, joints)
    dtau = np.mean([[f(r, f"couple_{x}_nm") for x in ART] for r in serr], axis=0) - tau0 - dg
    n = np.array([0.0, -sens, 0.0])                       # vers le carton
    os_ = np.mean([f(r, "force_normale_n") for r in serr])
    j_ = np.mean([f(r, "force_juge_n") for r in serr])
    f0, f11 = float(force(chaine, q, joints, dtau, 0.0) @ n), \
        float(force(chaine, q, joints, dtau, R.contact_main_m) @ n)
    resultats[cote] = q
    check(f"… posture reconstruite : au repère de la main, la force rejouée ({f0:.2f} N) est "
          f"celle de l'OS pendant l'essai ({os_:.2f} N) à 5 % près", abs(f0 - os_) <= 0.05 * os_)
    check(f"… au point de contact ({100 * R.contact_main_m:.0f} cm le long de la main) : "
          f"{f11:.2f} N, celle du juge ({j_:.2f} N) à 5 % près (au repère : "
          f"{100 * (f0 / j_ - 1):+.0f} %)", abs(f11 - j_) <= 0.05 * j_)

section("2. Où passait la ligne des mains : à 11 cm du centre de gravité")
contacts = MODELE.au_contact(R.contact_main_m)
cg_ = contacts.gauche.main(resultats["left"])
cd_ = contacts.droite.main(resultats["right"])
prof, haut = ecart_ligne_cg(cg_, cd_, CENTRE)
tp, th = tolerances_cg(R)
check(f"Repères des mains au centre du carton : ligne des points de contact à "
      f"{100 * prof:+.1f} cm du centre de gravité en profondeur (derrière lui), "
      f"{100 * haut:+.1f} cm en hauteur ; tolérance {100 * tp:.1f} et {100 * th:.1f} cm : "
      f"refusée désormais avant de lever", 0.09 <= prof <= 0.13 and abs(prof) > tp)
check(f"… couple de basculement du carton de 1 kg, levé : {9.81 * prof:.2f} N·m, 5 fois et plus "
      f"ce que retiennent les mains caoutchouc serrées à 8 N "
      f"({9.81 * tp * R.securite_basculement:.2f} N·m)",
      9.81 * prof > 5 * 9.81 * tp * R.securite_basculement)
q, _ = resoudre(contacts, *cibles_mains(CENTRE, 0.24, 0.0, R.demi_main_m),
                {**resultats["left"], **resultats["right"]})
check("Prise d'aujourd'hui (points de contact visés au milieu des faces) : la ligne passe par "
      "le centre de gravité, aux tolérances près",
      q is not None and all(abs(v) <= t for v, t in zip(
          ecart_ligne_cg(*contacts.mains({**resultats["left"], **resultats["right"], **q}),
                         CENTRE), (tp, th))))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
