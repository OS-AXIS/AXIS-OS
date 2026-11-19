"""
Les lignes réelles de l'essai GPU T1 du 19 novembre 2026, 17 h 16 (le port :
t1_prise_20261119_1716.csv réduit au port et à ses colonnes utiles),
rejouées dans les fonctions de l'OS. Le meilleur essai : prise parfaite,
poids pesé 9,8 N (9,8 en vrai), serrage 12,5 N par main, carton porté 20 s
sur ~3 m, jamais tombé ; mais « ALERTE colis mal tenu » à 66,2 s, pas de
dépose.

Ce que disent les lignes :
  1. le carton a tourné de 25 à 28° autour de la ligne des mains (le « roulis » du
     juge : son lacet de -88° met son axe propre le long d'elle), sans que
     l'OS ne le voie ; la force du juge reste à ~12 N par main ;
  2. les faces COMMANDÉES des deux mains glissent ensemble de 2,2 cm vers la
     droite, leur écart constant : le resserrage n'a pas serré, il a fait
     glisser la paire (une main poussée, l'autre reculée par le plafond du
     tangage du poignet droit, dépassé une ligne sur cinq) ;
  3. les paumes MESURÉES, elles, n'ont pas tourné de plus de 3° (repère du
     torse) : le carton a tourné entre elles. C'est pourquoi les contrôles
     d'alors ne l'ont pas vu : la rotation estimée sur les mains (4°) ne
     surveillait que le retour vers le buste ; l'inclinaison (la droite qui
     joint les mains, dans le repère du monde) ne voit pas, par
     construction, un carton qui tourne autour d'elle ; la caméra de tête ne
     le mesure qu'en entier dans l'image (il y était coupé : rotation_vue
     vide au port) ; et les couples ne voient pas le couple de basculement
     (sous leur bruit).

Avec les corrections (service.py) :
  - la dérive des mains est vue pendant tout le port, sur les angles
    mesurés, quand le carton n'avait tourné que de ~2° (avant 4°), et
    ramenée vers la pose figée ;
  - le resserrage et le desserrage ne bougent les mains que le long de la
    normale, ensemble : sur la posture réelle du début du port, la paire
    serrée ne glisse pas ;
  - hystérésis, sur la marge de frottement de l'OS reconstruite d'après les
    couples de l'essai : stricte, bien moins d'arrêts de la base, et, la
    marge ne dépassant presque jamais 2,2 (le poignet droit à son plafond),
    le port se serait arrêté proprement au premier creux, au lieu de marcher
    20 s avec un carton tenu au ras du coefficient de sécurité ; par défaut
    (reprise_aux_plafonds), les bras à leurs plafonds repartent au seuil :
    presque autant d'arrêts qu'avant, ici.
"""
import csv
import math
import sys
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.unitree_g1.robot import build_g1_joint_definitions  # noqa: E402
from axis_os.services.manipulation.cinematique import ModeleBras  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import (  # noqa: E402
    ManipulationService, _rotation_vecteur,
)

FAIL = 0
R = ReglagesPrise()
CSV = Path(__file__).resolve().parent / "donnees" / "t1_prise_20261119_1716.csv"
ART = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch",
       "wrist_yaw")
POIDS = 9.8                                   # pesé par l'OS (9,8 en vrai)
# Biais des couples, calibration de l'essai (journal : manipulation, biais_couples_nm).
BIAIS = {"left_shoulder_pitch_joint": 0.36, "left_shoulder_roll_joint": 0.0,
         "left_shoulder_yaw_joint": 0.31, "left_elbow_joint": -0.03,
         "left_wrist_roll_joint": 0.2, "left_wrist_pitch_joint": 0.35,
         "left_wrist_yaw_joint": 0.1, "right_shoulder_pitch_joint": 0.18,
         "right_shoulder_roll_joint": 0.28, "right_shoulder_yaw_joint": -0.04,
         "right_elbow_joint": -0.34, "right_wrist_roll_joint": -0.21,
         "right_wrist_pitch_joint": -0.26, "right_wrist_yaw_joint": -0.05}
RESSERRAGES_ESSAI = 32                        # « on resserre » au journal de l'essai


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def f(r, c):
    return float(r[c]) if r[c] else np.nan


PAS = {}                                   # t -> {main: ligne}
for r in csv.DictReader(open(CSV)):
    PAS.setdefault(f(r, "t_s"), {})[r["main"]] = r
PAS = {t: d for t, d in sorted(PAS.items()) if len(d) == 2}
TENU = list(PAS)
T0 = TENU[0]


def angles(t):
    return {f"{c}_{a}_joint": f(PAS[t][c], f"angle_{a}_rad") for c in ("left", "right") for a in ART}


def couples(t):
    return {f"{c}_{a}_joint": f(PAS[t][c], f"couple_{a}_nm") for c in ("left", "right") for a in ART}


def vrai(t, c="carton_roulis_deg"):
    return f(PAS[t]["left"], c) - f(PAS[T0]["left"], c)


def os_():
    """Le service de manipulation réduit à ce que rejouent ces lignes : ses
    réglages, le modèle des bras du G1, ses 7 articulations par bras
    commandées (celles du banc T1), les angles mesurés de la ligne `t`."""
    m = ManipulationService.__new__(ManipulationService)
    m.r, m._fixes = R, {}
    m._init_mesure()
    m.ligne = [T0]
    m.mesures = SimpleNamespace(commandables=tuple(angles(T0)), modele=ModeleBras(),
                                contact_main_m=None, positions=lambda: angles(m.ligne[0]))
    m._horloge = lambda: m.ligne[0]
    m.stats, m.journal = {}, []
    m._journal = m.journal.append
    m.biais_couples = BIAIS
    m._plan = {"lacet": 0.0}
    m._tau0, m._g0 = {}, {}
    return m


section("1. Le constat")
fin = TENU[-1]
juge = [f(PAS[t][c], "force_juge_n") for t in TENU for c in ("left", "right")]
check(f"Vérité du banc : carton tourné de {vrai(fin):+.1f}° autour de la ligne des mains de "
      f"t = {T0:.1f} à {fin:.1f} s ; force du juge {np.mean(juge):.1f} N par main en moyenne",
      vrai(fin) < -20.0 and 11.0 < float(np.mean(juge)) < 14.0)
fc = {c: [f(PAS[t][c], "face_commandee_m") for t in TENU] for c in ("left", "right")}
milieu = [(g + d) / 2.0 for g, d in zip(fc["left"], fc["right"])]
ecart = [g - d for g, d in zip(fc["left"], fc["right"])]
check(f"Faces commandées : la paire glisse de {100 * (milieu[-1] - milieu[0]):+.1f} cm de côté, "
      f"son écart change de {100 * (ecart[-1] - ecart[0]):+.1f} cm : le resserrage n'a pas serré",
      milieu[-1] - milieu[0] < -0.015 and abs(ecart[-1] - ecart[0]) < 0.005)
lim = {d.joint_id: min(R.effort_max_nm, R.part_couple_garde * d.limits.torque_max)
       for d in build_g1_joint_definitions("29dof")}
j = "right_wrist_pitch_joint"
au_dela = sum(1 for t in TENU if abs(couples(t)[j]) > lim[j])
check(f"Tangage du poignet droit au-delà de sa limite d'effort ({lim[j]:.1f} N·m) sur "
      f"{au_dela} lignes du port sur {len(TENU)} : son plafond faisait reculer la main droite",
      au_dela >= len(TENU) // 5, str(au_dela))
m = os_()
p0 = m._poses_torse(m._angles_mesures())
rot_mains = 0.0
for t in TENU:
    m.ligne[0] = t
    p = m._poses_torse(m._angles_mesures())
    rot_mains = max(rot_mains, max(math.degrees(float(np.linalg.norm(
        _rotation_vecteur(p[c][1] @ p0[c][1].T)))) for c in ("left", "right")))
check(f"Paumes mesurées (cinématique, repère du torse) : {rot_mains:.1f}° au plus pendant tout "
      f"le port, quand le carton en tournait de {abs(vrai(fin)):.0f} : il a tourné ENTRE elles",
      rot_mains < 0.2 * abs(vrai(fin)))

section("2. Avec les corrections")
# Dérive des mains : le service, pose figée au début du port, sur les angles
# mesurés de chaque ligne (sans corriger : on regarde ce qu'il voit).
m = os_()
m._q = angles(T0)
poses = m._poses(m._q)
u = poses["right"][0] - poses["left"][0]
e0 = float(np.linalg.norm(u))
u = u / e0
c0 = (poses["left"][0] + poses["right"][0]) / 2.0
m._rigide = (c0, {"left": (*poses["left"], u), "right": (*poses["right"], -u)},
             dict(m._delta), e0)
m._centre_porte, m._ecart_mains = c0, e0
m._figer_port()
vue, rot_os = None, 0.0
for t in TENU:
    m.ligne[0] = t
    m._surveiller_mains_port(corriger=False)
    rot_os = max(rot_os, max(map(abs, m.rotation_port)))
    if vue is None and m.derive_port[0] > R.derive_port_m:
        vue = (t, m.derive_port[0], vrai(t))
check(f"Dérive des mains vue sur les angles mesurés (moyenne sur {R.fenetre_derive_s:g} s, au-delà "
      f"de {100 * R.derive_port_m:g} cm) à t = {vue and vue[0]:.1f} s ({vue and 100 * vue[1]:.1f} cm), "
      f"quand le carton n'avait tourné que de {vue and vue[2]:+.1f}° : avant "
      f"{R.rotation_ramener_max_deg:g}°, base arrêtée et mains ramenées vers leur pose figée",
      vue is not None and abs(vue[2]) < R.rotation_ramener_max_deg, str(vue))
check(f"… sa rotation, elle, reste à {rot_os:.1f}° au plus : un carton qui tourne entre les "
      f"paumes ne se voit pas sur les mains (la caméra de tête, en entier dans l'image)",
      rot_os < R.rotation_ramener_max_deg)
# Resserrage du port : sur la posture réelle du début du port, 5 mm de serrage
# de chaque côté, le long de la normale, depuis la posture figée.
q = m._q_port({"left": 0.005, "right": 0.005})
pq = m._poses(q) if q is not None else None
glisse = None if pq is None else float(np.linalg.norm(
    (pq["left"][0] + pq["right"][0]) / 2.0 - c0))
serre = None if pq is None else e0 - float(np.linalg.norm(pq["left"][0] - pq["right"][0]))
tourne = None if pq is None else max(math.degrees(float(np.linalg.norm(
    _rotation_vecteur(pq[c][1] @ poses[c][1].T)))) for c in ("left", "right"))
check(f"Resserrage du port sur la posture réelle : écart des mains -{serre and 1000 * serre:.1f} mm, "
      f"la paire glisse de {glisse and 1000 * glisse:.2f} mm (essai : "
      f"{100 * abs(milieu[-1] - milieu[0]):.1f} cm), paumes tournées de {tourne and tourne:.2f}°",
      q is not None and abs(serre - 0.010) < 0.002 and glisse < 0.0005
      and tourne <= R.tol_rigide_deg, f"{serre} {glisse} {tourne}")
# Marge de frottement de l'OS, reconstruite d'après les couples de l'essai
# (_force_main, biais de la calibration, pesanteur verticale ; moyenne de
# 0,3 s comme _hist_normales), et les arrêts que chaque règle en tire.
hist, marges = deque(maxlen=3), []
for t in TENU:
    a, tau = angles(t), couples(t)
    n = sum(float(m._force_main(c, tau, a, np.array([0.0, 0.0, -9.81]), depuis_fermeture=False)
                  @ m._vers_objet(c)) for c in ("left", "right"))
    hist.append(n)
    marges.append((t, R.frottement_serrage * float(np.mean(hist)) / POIDS))


# Aux plafonds : une articulation d'un bras à moins de 0,3 N·m du sien
# (_marge_plafonds, sur les couples tels que l'OS les lit).
LIM = {d.joint_id: min(R.effort_max_nm, R.part_couple_garde * d.limits.torque_max)
       for d in build_g1_joint_definitions("29dof")}
plafonds = {t: any(abs(v) > LIM[j_] - 0.3 for j_, v in couples(t).items()) for t in TENU}


def arrets(reprise, delai, aux_plafonds):
    """Arrêts de la base que la règle tire de la marge de l'essai (sans
    l'abandon au bout de duree_resserrage_s)."""
    n, t0 = 0, None
    for t, mg in marges:
        if t0 is None:
            if mg < R.securite_serrage:
                t0, n = t, n + 1
        elif mg > reprise and t - t0 >= delai:
            t0 = None
        elif aux_plafonds and plafonds[t] and mg >= R.securite_serrage:
            t0 = None
    return n


moy = float(np.mean([mg for _, mg in marges]))
check(f"Marge de frottement de l'OS reconstruite : {moy:.2f} en moyenne (de "
      f"{min(mg for _, mg in marges):.2f} à {max(mg for _, mg in marges):.2f}), au ras du "
      f"coefficient de sécurité ({R.securite_serrage:g}) comme au journal de l'essai",
      abs(moy - R.securite_serrage) < 0.15)
avant = arrets(R.securite_serrage, 0.0, False)
stricte = arrets(R.securite_reprise, R.delai_reprise_s, False)
defaut = arrets(R.securite_reprise, R.delai_reprise_s, True)
check(f"Arrêts de la base : {avant} en repartant au seuil ({RESSERRAGES_ESSAI} au journal de "
      f"l'essai, l'OS y tournant à 50 Hz) ; hystérésis stricte (au-dessus de "
      f"{R.securite_reprise:g}, après {R.delai_reprise_s:g} s) : {stricte}",
      stricte * 3 <= avant)
check(f"… par défaut (reprise_aux_plafonds) : {defaut}, jamais plus qu'avant — les bras sont à "
      f"leurs plafonds sur {sum(plafonds.values())} lignes sur {len(TENU)}, et y repartent au "
      f"seuil", defaut <= avant)
dessus = sum(1 for _, mg in marges if mg > R.securite_reprise)
check(f"La marge ne passe {R.securite_reprise:g} que sur {dessus} lignes sur {len(marges)} : en "
      f"stricte, le premier resserrage n'y arrive pas en {R.duree_resserrage_s:g} s, le port "
      f"s'arrête proprement, carton tenu, au lieu de marcher 20 s au ras du coefficient de "
      f"sécurité", dessus <= len(marges) // 10)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
