"""
Les lignes réelles de l'essai GPU T1 du 25 novembre 2026, 16 h 43
(t1_prise_20261125_1643.csv réduit à la dernière seconde du retour et au
port), rejouées dans la règle du port de l'OS.

Ce qui s'est passé : prise en pendule, carton tenu sans tourner ; calibration
de charge non faite (le banc avait posé la masse de référence sur un carton) ;
au port, « transport interrompu : marge 1.89 … sur les 11.0 N lus sur la
foulée, plus que les 9.3 pesés », le carton parfaitement tenu (vrai 9,81 N).

Décision : le poids de la marge n'est jamais lu en marchant (securite_serrage
couvre déjà les charges de la marche) ; avec une calibration de charge valide,
la pesée seule ; sans, le plus grand de la pesée et du poids lu base arrêtée
(ici : la médiane de la dernière demi-seconde du retour, ReglagesPrise.
fin_retour_s).

Ce que disent les lignes :
  1. le poids lu en marchant n'est pas plus haut que base arrêtée : 10,3 N en
     marchant, 10,7 base arrêtée pour resserrer, 10,4 à la fin du retour
     (base arrêtée, avant de marcher), pour 9,81 vrais et 9,28 pesés. L'excès
     (+6 %) est un biais de l'estimation sans calibration dans la posture de
     port, pas les secousses ;
  2. la marge réelle (forces du juge, vrai poids) reste de 2,06 à 2,19 sur
     1 s : tenu, au ras du coefficient de sécurité ;
  3. rejouée sur la règle du port de l'OS (la marge du relevé ramenée au
     poids retenu) : sans calibration, la règle donne 10,4 N, l'issue de
     l'essai ne change pas (dernier arrêt jamais repris) ; avec la
     calibration de charge (la pesée seule, juste à 2 % près), chaque arrêt
     est repris tant que la pesée n'est pas au-dessus du vrai poids ; à +2 %,
     le dernier arrêt est encore en cours à la fin du relevé.
"""
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.unitree_g1.robot import build_g1_joint_definitions  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402

FAIL = 0
R = ReglagesPrise()
DONNEES = Path(__file__).resolve().parent / "donnees"
VRAI = 9.81                                   # carton de 1 kg
# Au journal de l'essai : chaque arrêt de la base au port, « port : marge de
# frottement M sur la dernière foulée (… sur les W N lus sur la foulée, plus
# que les 9.3 pesés) : base arrêtée, on resserre », et la reprise qui le suit.
ARRETS = ((152.0, 2.00, 10.2), (152.2, 1.98, 10.4), (152.4, 1.98, 10.4), (153.5, 1.95, 10.8),
          (154.0, 1.97, 10.8), (154.3, 1.97, 10.8), (155.1, 2.00, 10.5), (155.3, 1.98, 10.8),
          (155.5, 1.97, 10.8), (155.7, 2.00, 10.6))
REPRISES = (152.1, 152.3, 153.1, 153.9, 154.1, 155.0, 155.2, 155.4, 155.6)
INTERRUPTION = (156.7, 1.89, 11.0)            # « transport interrompu » : marge sur 0,3 s, poids lu
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
    return float(r[c]) if r.get(c) else np.nan


P = {}
for r_ in csv.DictReader(open(DONNEES / "t1_port_20261125_1643.csv", encoding="utf-8")):
    P.setdefault(f(r_, "t_s"), {})[r_["main"]] = r_
T = [t for t in sorted(P) if len(P[t]) == 2]
RAM = [t for t in T if P[t]["left"]["phase"] == "RAMENER"]
TENU = [t for t in T if P[t]["left"]["phase"] == "TENU"]
LU = {t: f(P[t]["left"], "poids_os_n") for t in T}
PESE = f(P[T[0]]["left"], "poids_pese_n")
ARRETEE = [(a[0], b) for a, b in zip(ARRETS, REPRISES + (INTERRUPTION[0],))]

section("1. Le poids lu : en marchant, base arrêtée, à la fin du retour")
fin_retour = [LU[t] for t in RAM if t >= RAM[-1] - R.fin_retour_s + 1e-9]
statique = float(np.median(fin_retour))
au_port_arretee = [LU[t] for t in TENU if any(a <= t <= b for a, b in ARRETEE)]
au_port_marche = [LU[t] for t in TENU if not any(a <= t <= b for a, b in ARRETEE)]
check(f"Pesé {PESE:.2f} N ({100 * (PESE / VRAI - 1):+.1f} %) ; base arrêtée à la fin du retour "
      f"({len(fin_retour)} lectures en {R.fin_retour_s:g} s) : {statique:.2f} N "
      f"({100 * (statique / VRAI - 1):+.1f} %), pour {VRAI} vrais",
      PESE < 0.97 * VRAI and statique > 1.03 * VRAI and len(fin_retour) >= 3 * R.filtre_force)
check(f"Au port, le poids lu en marchant ({np.median(au_port_marche):.2f} N, médiane de "
      f"{len(au_port_marche)} relevés) n'est pas plus haut que base arrêtée pour resserrer "
      f"({np.median(au_port_arretee):.2f}, {len(au_port_arretee)}) : l'excès sur le vrai poids est "
      f"un biais de l'estimation sans calibration dans la posture de port, pas les secousses",
      np.median(au_port_marche) <= np.median(au_port_arretee)
      and abs(np.median(au_port_marche) - statique) < 0.2)
somme = np.array([f(P[t]["left"], "force_juge_n") + f(P[t]["right"], "force_juge_n") for t in TENU])
reelle = [R.frottement_serrage * somme[i:i + 10].mean() / VRAI for i in range(len(somme) - 9)]
check(f"Marge réelle (forces du juge, vrai poids, sur 1 s) : de {min(reelle):.2f} à "
      f"{max(reelle):.2f} — tenu, au ras du coefficient de sécurité",
      R.securite_serrage <= min(reelle) < 1.1 * R.securite_serrage)

section("2. La règle du port de l'OS, rejouée selon le poids de la marge")
MARGE = {t: f(P[t]["left"], "marge_frottement") for t in TENU}
LIM = {d.joint_id: min(R.effort_max_nm, R.part_couple_garde * d.limits.torque_max)
       for d in build_g1_joint_definitions("29dof")}
PLAFONDS = {t: any(abs(f(P[t][c], f"couple_{a}_nm")) > LIM[f"{c}_{a}_joint"] - 0.3
                   for c in ("left", "right") for a in ART) for t in TENU}
# Le poids sur lequel l'OS calculait sa marge à chaque relevé : le plus grand
# de la pesée et du poids lu sur la foulée, celui du journal aux arrêts,
# interpolé entre eux (avant le premier : la médiane des relevés de la
# dernière seconde).
TJ = [a[0] for a in ARRETS] + [INTERRUPTION[0]]
WJ = [a[2] for a in ARRETS] + [INTERRUPTION[2]]


def poids_essai(t):
    if t < TJ[0]:
        return max(PESE, float(np.median([LU[x] for x in T if t - 1.0 < x <= t])))
    return max(PESE, float(np.interp(t, TJ, WJ)))


def regle(poids):
    """La règle du port de l'OS, sur la marge de l'essai ramenée à `poids`
    (None : celui de l'essai) : arrêt sous securite_serrage ; reprise
    au-dessus de securite_reprise après delai_reprise_s, ou dès
    securite_serrage aux plafonds ; abandon sans reprise en
    duree_resserrage_s. Rend (arrêts, début de l'arrêt encore en cours à la
    fin du relevé ou None, marge au plus bas)."""
    n, t0, bas = 0, None, float("inf")
    for t in TENU:
        if np.isnan(MARGE[t]):
            continue
        mg = MARGE[t] if poids is None else MARGE[t] * poids_essai(t) / poids
        bas = min(bas, mg)
        if t0 is None:
            if mg < R.securite_serrage:
                t0, n = t, n + 1
        elif (mg > R.securite_reprise and t - t0 >= R.delai_reprise_s) or \
                (R.reprise_aux_plafonds and PLAFONDS[t] and mg >= R.securite_serrage):
            t0 = None
        elif t - t0 >= R.duree_resserrage_s:
            break
    return n, t0, bas


ecart_journal = max(abs(np.median([LU[x] for x in T if t - 1.0 < x <= t]) - w) for t, _, w in ARRETS)
check(f"Le poids de la marge de l'essai, pris au journal à chacun de ses {len(ARRETS)} arrêts "
      f"(de {min(WJ):.1f} à {max(WJ):.1f} N) ; la médiane des relevés, à 0,1 s, s'en écarte de "
      f"{ecart_journal:.2f} N au plus (on prend le journal)",
      min(WJ) > PESE and ecart_journal < 0.5)
n_e, ouvert_e, bas_e = regle(None)
check(f"Rejoué tel quel : {n_e} arrêts, le dernier (depuis {ouvert_e or float('nan'):.2f} s) "
      f"jamais repris jusqu'au bout du relevé ({TENU[-1]:.2f} s ; transport interrompu à "
      f"{INTERRUPTION[0]} s), marge au plus bas {bas_e:.2f}",
      n_e >= 5 and ouvert_e is not None and INTERRUPTION[0] - ouvert_e <= R.duree_resserrage_s + 0.05)
sans_cal = max(PESE, statique)
n_s, ouvert_s, bas_s = regle(sans_cal)
check(f"Sans calibration, la règle : le plus grand de la pesée et du poids lu base arrêtée, "
      f"{sans_cal:.2f} N ; l'issue ne change pas : {n_s} arrêts, le dernier (depuis "
      f"{ouvert_s or float('nan'):.2f} s) jamais repris ; marge au plus bas {bas_s:.2f}",
      sans_cal == statique and ouvert_s is not None and n_s >= 5)
issues = {k: regle(VRAI * k) for k in (0.98, 1.0, 1.02)}
check(f"Avec la calibration de charge (la pesée seule, juste à 2 % près) : pesée au vrai poids "
      f"({VRAI} N), {issues[1.0][0]} arrêt(s), tous repris ; 2 % en dessous, "
      f"{issues[0.98][0]} ; marge au plus bas {issues[1.0][2]:.2f} et {issues[0.98][2]:.2f} : le "
      f"port continue",
      issues[1.0][1] is None and issues[0.98][1] is None and issues[1.0][0] <= 2)
check(f"… 2 % au-dessus ({VRAI * 1.02:.2f} N) : {issues[1.02][0]} arrêt, encore en cours à la fin "
      f"du relevé (depuis {issues[1.02][1] or float('nan'):.2f} s ; marge au plus bas "
      f"{issues[1.02][2]:.2f}) : ce carton de 1 kg, aux plafonds des bras, ne se porte qu'avec un "
      f"poids juste",
      issues[1.02][1] is not None)
n_p, ouvert_p, bas_p = regle(PESE)
check(f"La pesée seule SANS calibration ({PESE:.2f} N, {100 * (PESE / VRAI - 1):+.1f} %) : "
      f"{n_p} arrêt, marge au plus bas {bas_p:.2f} — optimiste de {100 * (VRAI / PESE - 1):.0f} %, "
      f"jamais retenue sans calibration",
      n_p == 0 and bas_p >= R.securite_serrage and PESE < VRAI)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
