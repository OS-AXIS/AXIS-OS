"""
Les lignes réelles de l'essai GPU T1 du 19 novembre 2026, 22 h 04
(t1_prise_20261119_2204.csv, réduit au levage et au port) et le levage de
celui de 17 h 16 (t1_levage_20261119_1716.csv), rejoués dans les fonctions de
l'OS. Prise parfaite, puis « transport interrompu : serrage perdu au port
malgré le resserrage (marge de frottement 1.94 (au moins 2, 2.2 pour
repartir)) » après 4 resserrages et 2 s de port, le carton pourtant tenu
(rotation de 1° au plus, force du juge de 12 à 13,3 N par main).

Ce que disent les lignes :
  1. la pesée : 10,6 N pour 9,8 (+8 %), médiane de 15 lectures prises bras
     immobiles (1 mrad au plus), à la fin du levage commandé. Aux mêmes
     instants, les couples du relevé (une autre lecture du même état
     physique : le banc tire un bruit neuf à chaque lecture) pèsent 9,65 N :
     ce n'est pas une charge réelle (une base qui accélère aurait relevé les
     deux), c'est le bruit d'une lecture (~1 N d'écart type) sur une fenêtre
     trop courte ;
  2. la marge du port se calcule sur ce poids pesé (10,6 N), pas sur le
     poids lu en marchant (de 9,6 à 11,2 N) : 1,94 sur 10,6 N, ~2,1 sur les
     9,81 vrais ;
  3. le serrage réel varie à la marche (juge de 11,4 à 14,3 N par main) ;
     sur 1 s, la marge réelle reste au-dessus de 2 : le carton était tenu,
     au ras du coefficient de sécurité.

Avec les corrections (prise.poids_pese, ManipulationService._peser) :
  - la pesée sur lectures_pesee lectures bras immobiles (100, 2 s),
    moyenne tronquée : à ±2 % près (95 %), contre ±5 % sur 15 lectures ; il
    reste un biais de +2 à +5 % (côté
    lourd) : sans les erreurs de gain des couples estimés du banc (connues,
    graine 3), les couples du relevé pèsent le carton à 1,5 % près, et
    presque tout l'écart vient des deux coudes ;
  - rejoué sur la règle du port de l'OS : avec les 10,6 N de l'essai, la
    base reste arrêtée jusqu'au bout du relevé (abandon de l'essai à 48,4 s) ;
    avec le vrai poids, chaque arrêt est repris ; avec la pesée longue
    (+4,7 %), le dernier arrêt ne l'est pas encore à la fin du relevé. Ce
    carton de 1 kg, poignets à leurs plafonds, ne se porte qu'avec un poids
    juste à ~2 % près.
"""
import csv
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.isaac_sim.bruit_couples import BruitCouples  # noqa: E402
from axis_os.hal.drivers.unitree_g1.robot import build_g1_joint_definitions  # noqa: E402
from axis_os.services.manipulation.cinematique import ModeleBras  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise, poids_pese  # noqa: E402
from axis_os.services.manipulation.service import ManipulationService  # noqa: E402

FAIL = 0
R = ReglagesPrise()
DONNEES = Path(__file__).resolve().parent / "donnees"
VRAI = 9.81                                   # carton de 1 kg
PESE_ESSAI = 10.6                             # « poids du carton : 10.6 N » (journal)
INTERRUPTION_ESSAI = 48.4                     # « transport interrompu » (journal)
ART = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch",
       "wrist_yaw")
# Biais des couples, calibration des deux essais (journal : biais_couples_nm).
BIAIS = {"left_shoulder_pitch_joint": 0.36, "left_shoulder_roll_joint": 0.0,
         "left_shoulder_yaw_joint": 0.31, "left_elbow_joint": -0.03,
         "left_wrist_roll_joint": 0.2, "left_wrist_pitch_joint": 0.35,
         "left_wrist_yaw_joint": 0.1, "right_shoulder_pitch_joint": 0.18,
         "right_shoulder_roll_joint": 0.28, "right_shoulder_yaw_joint": -0.04,
         "right_elbow_joint": -0.34, "right_wrist_roll_joint": -0.21,
         "right_wrist_pitch_joint": -0.26, "right_wrist_yaw_joint": -0.05}
# Ordre des articulations dans Isaac Sim (celui du bruit des couples du banc,
# bruit_couples.BruitCouples, graine 3 par défaut) ; vérifié plus bas : les
# biais du banc retrouvent ceux de la calibration.
ORDRE_ISAAC = ("left_hip_pitch", "right_hip_pitch", "waist_yaw", "left_hip_roll",
               "right_hip_roll", "waist_roll", "left_hip_yaw", "right_hip_yaw", "waist_pitch",
               "left_knee", "right_knee", "left_shoulder_pitch", "right_shoulder_pitch",
               "left_ankle_pitch", "right_ankle_pitch", "left_shoulder_roll",
               "right_shoulder_roll", "left_ankle_roll", "right_ankle_roll",
               "left_shoulder_yaw", "right_shoulder_yaw", "left_elbow", "right_elbow",
               "left_wrist_roll", "right_wrist_roll", "left_wrist_pitch", "right_wrist_pitch",
               "left_wrist_yaw", "right_wrist_yaw")


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def f(r, c):
    return float(r[c]) if r.get(c) else np.nan


def charger(nom):
    pas = {}
    for r in csv.DictReader(open(DONNEES / nom)):
        pas.setdefault(f(r, "t_s"), {})[r["main"]] = r
    return {t: d for t, d in sorted(pas.items()) if len(d) == 2}


def angles(d):
    q = {f"{c}_{a}_joint": f(d[c], f"angle_{a}_rad") for c in ("left", "right") for a in ART}
    if d["left"].get("taille_tangage_rad"):
        for a, col in (("yaw", "taille_lacet_rad"), ("roll", "taille_roulis_rad"),
                       ("pitch", "taille_tangage_rad")):
            q[f"waist_{a}_joint"] = f(d["left"], col)
    return q


def couples(d):
    return {f"{c}_{a}_joint": f(d[c], f"couple_{a}_nm") for c in ("left", "right") for a in ART}


def os_():
    """Le service de manipulation réduit à l'estimation des forces : ses
    réglages, le modèle des bras du G1, les biais de la calibration."""
    m = ManipulationService.__new__(ManipulationService)
    m.r, m._fixes = R, {}
    m._init_mesure()
    m.mesures = SimpleNamespace(modele=ModeleBras(), contact_main_m=None)
    m.biais_couples = dict(BIAIS)
    m.gains_couples = {}               # sans calibration de charge, comme à l'essai
    m._tau0, m._g0 = {}, {}
    return m


M = os_()
G = np.array([0.0, 0.0, -9.81])


def poids_csv(d, conv=None):
    """Poids lu sur les couples du relevé (une autre lecture que celle de
    l'OS) : somme des forces des deux mains vers le haut, pesanteur
    verticale (le relevé n'a pas la centrale inertielle)."""
    tau = couples(d) if conv is None else conv(couples(d))
    return sum(float(M._force_main(c, tau, angles(d), G, depuis_fermeture=False)[2])
               for c in ("left", "right"))


def lectures_os(moy, debut, fin):
    """Les lectures une à une de l'OS, d'après sa moyenne glissante
    (poids_os_n) repartie de zéro à `debut`."""
    n, lec = 3 * R.filtre_force, []
    for k, i in enumerate(range(debut, fin)):
        lec.append(moy[i] if k == 0 else (k + 1) * moy[i] - k * moy[i - 1] if k < n
                   else lec[k - n] + n * (moy[i] - moy[i - 1]))
    return np.array(lec)


def levage(nom):
    """Lectures du levage : (instants, lectures de l'OS, indice de la
    confirmation, palier au serrage de prise, juge par main)."""
    P = charger(nom)
    lev = [t for t in P if P[t]["left"]["phase"] == "LEVAGE"]
    moy = [f(P[t]["left"], "poids_os_n") for t in lev]
    d0 = next(i for i, v in enumerate(moy) if not np.isnan(v))
    k0 = next(i for i in range(d0 + 1, len(moy)) if abs(moy[i] - moy[i - 1]) > 0.8
              and lev[i] > lev[0] + 1.0)
    lec = np.concatenate([lectures_os(moy, d0, k0), lectures_os(moy, k0, len(moy))])
    t = np.array(lev[d0:])
    juge = np.array([(f(P[x]["left"], "force_juge_n") + f(P[x]["right"], "force_juge_n")) / 2
                     for x in t])
    # Palier : le carton levé, au serrage de prise, bras immobiles — des
    # 0,3 dernières secondes du levage commandé jusqu'à la montée du serrage.
    prise = float(np.median(juge[:20]))
    monte = int(np.argmax(juge > prise + 0.4))
    palier = [i for i in range(len(t)) if t[i] >= lev[0] + R.duree_levage_test_s - 0.3 - 1e-6
              and i < monte]
    return P, lev, t, lec, k0 - d0, palier, juge


section("1. La pesée de l'essai : 10,6 N pour 9,8")
P, LEV, T, LEC, KC, PALIER, JUGE = levage("t1_prise_20261119_2204.csv")
fen = LEC[KC:KC + 3 * R.filtre_force]
check(f"Rejouée : médiane des {len(fen)} lectures qui suivent la confirmation, "
      f"{np.median(fen):.2f} N ({100 * (np.median(fen) / VRAI - 1):+.1f} %), le « {PESE_ESSAI} » du "
      f"journal", abs(np.median(fen) - PESE_ESSAI) < 0.05)
A = np.array([[f(P[x][c], f"angle_{a}_rad") for c in ("left", "right") for a in ART]
              for x in T[KC:KC + len(fen)]])
bouge = float(np.abs(A - A[0]).max())
check(f"Bras immobiles : {1000 * bouge:.1f} mrad au plus pendant la fenêtre ; confirmation à "
      f"{T[KC] - LEV[0]:.2f} s du début du levage, la fin du levage commandé "
      f"({R.duree_levage_test_s:g} s, profil adouci)",
      bouge <= 0.0015 and abs(T[KC] - LEV[0] - R.duree_levage_test_s) <= 0.04)
csv15 = np.array([poids_csv(P[x]) for x in T[KC:KC + len(fen)]])
check(f"Aux mêmes instants, les couples du relevé (bruit neuf à chaque lecture) pèsent "
      f"{csv15.mean():.2f} N ({100 * (csv15.mean() / VRAI - 1):+.1f} %), l'OS {fen.mean():.2f} en "
      f"moyenne : pas une charge réelle (une base qui accélère aurait relevé les deux)",
      csv15.mean() < VRAI + 0.1 and fen.mean() - csv15.mean() > 0.5)
os_p = LEC[PALIER]
csv_p = np.array([poids_csv(P[x]) for x in T[PALIER]])
sigma = float(np.mean([os_p.std(), csv_p.std()]))
check(f"Une lecture : {os_p.std():.2f} N d'écart type (OS), {csv_p.std():.2f} (relevé), sur le "
      f"palier au serrage de prise ; 15 lectures ne sont sûres qu'à ±{100 * sigma / np.sqrt(15) / VRAI:.1f} % "
      f"(un écart type) : la fenêtre d'avant ne pouvait pas peser à 3 % près",
      0.6 <= sigma <= 1.4 and sigma / np.sqrt(15) / VRAI > 0.02)

section(f"2. La pesée longue : {R.lectures_pesee} lectures bras immobiles, moyenne tronquée")
pese, incertitude = poids_pese(os_p, R.part_ecartee_pesee)
check(f"Sur le palier de l'essai ({len(os_p)} lectures de l'OS, bras immobiles, serrage de "
      f"prise) : {pese:.2f} N ({100 * (pese / VRAI - 1):+.1f} %) ±{2 * incertitude:.2f} (deux écarts "
      f"types), côté lourd, à 5 % du vrai",
      VRAI * 0.99 <= pese <= VRAI * 1.05)
# Reproductibilité : tirages de n lectures parmi les deux flux du palier (celui
# du relevé recalé sur celui de l'OS de leur écart médian sur tout le levage :
# la pesanteur de la centrale inertielle, que le relevé n'a pas), AVEC remise :
# des lectures indépendantes, comme celles de l'OS (un bruit neuf à chaque
# pas). Sans remise, n lectures prises parmi les 70 du palier variaient de
# moins en moins à mesure que n approchait 70 (100 n'y tient plus) : c'était
# optimiste. L'incertitude que l'OS annonce (poids_pese, deux écarts types)
# doit tenir 2 %, et le tirage la retrouver.
recal = float(np.median(LEC - np.array([poids_csv(P[x]) for x in T])))
tous = np.concatenate([os_p, csv_p + recal])
rng = np.random.default_rng(2204)


def demi_largeur(n):
    t_ = [poids_pese(rng.choice(tous, n, replace=True), R.part_ecartee_pesee)[0]
          for _ in range(4000)]
    return (np.percentile(t_, 97.5) - np.percentile(t_, 2.5)) / 2.0 / VRAI


d15, dn = demi_largeur(15), demi_largeur(R.lectures_pesee)
annonce = 2.0 * float(np.std(tous)) / np.sqrt(R.lectures_pesee) / VRAI
check(f"Reproductible : de tirage en tirage du palier, ±{100 * dn:.1f} % avec "
      f"{R.lectures_pesee} lectures (95 %), contre ±{100 * d15:.1f} % avec 15 ; l'OS annonce "
      f"±{100 * annonce:.2f} % (deux écarts types), à 2 % près",
      annonce <= 0.02 and dn <= 0.5 * d15 and abs(dn - annonce) <= 0.003,
      f"{d15} {dn} {annonce}")
# Le biais qui reste : les erreurs de gain des couples estimés (bruit_couples,
# ±5 % par moteur, comme l'estimation du vrai G1 par le courant), connues au
# banc. L'ordre des articulations se vérifie sur les biais : ceux du banc
# retrouvent ceux de la calibration.
banc = BruitCouples(len(ORDRE_ISAAC), graine=3)
GAIN = {f"{j}_joint": g for j, g in zip(ORDRE_ISAAC, banc._gain)}
BIAIS_BANC = {f"{j}_joint": b for j, b in zip(ORDRE_ISAAC, banc._biais)}
ecart_biais = max(abs(BIAIS_BANC[j] - BIAIS[j]) for j in BIAIS)
# (couple - biais) / gain : couples et biais calibrés ramenés au gain 1.
M.biais_couples = {j: BIAIS[j] / GAIN[j] for j in BIAIS}
sans_gain = np.array([poids_csv(P[x], lambda tau: {j: v / GAIN[j] for j, v in tau.items()})
                      for x in T[PALIER]])
M.biais_couples = dict(BIAIS)
# Les coudes portent presque tout l'effet (le poids du carton passe par eux) ;
# leur pesanteur varie trop peu d'une posture de calibration à l'autre pour
# en mesurer le gain (CalibrationCouples.fiabilite : 1 N·m au moins).
M.biais_couples = {**BIAIS, **{j: BIAIS[j] / GAIN[j] for j in BIAIS if "elbow" in j}}
coudes = np.array([poids_csv(P[x], lambda tau: {j: v / GAIN[j] if "elbow" in j else v
                                                for j, v in tau.items()}) for x in T[PALIER]])
M.biais_couples = dict(BIAIS)
check(f"Le biais qui reste vient des couples estimés : sans leurs erreurs de gain (banc, "
      f"graine 3 ; ses biais à {ecart_biais:.2f} N·m de ceux de la calibration), les couples "
      f"du relevé pèsent {sans_gain.mean():.2f} N ({100 * (sans_gain.mean() / VRAI - 1):+.1f} %), "
      f"contre {csv_p.mean():.2f} avec ; les coudes seuls (gains "
      f"{GAIN['left_elbow_joint']:.3f} et {GAIN['right_elbow_joint']:.3f}) en font "
      f"{csv_p.mean() - coudes.mean():+.2f} N",
      ecart_biais <= 0.1 and abs(sans_gain.mean() / VRAI - 1) <= 0.015
      and csv_p.mean() > sans_gain.mean() + 0.1
      and abs((csv_p.mean() - coudes.mean()) - (csv_p.mean() - sans_gain.mean())) < 0.05)

section("3. Le port : la marge sur le poids pesé, le carton tenu")
TENU = [t for t in P if P[t]["left"]["phase"] == "TENU"]
lu = [f(P[t]["left"], "poids_os_n") for t in TENU]
marge = {t: f(P[t]["left"], "marge_frottement") for t in TENU}
fin = [marge[t] for t in TENU if not np.isnan(marge[t])][-8:]
check(f"Poids lu au port de {min(lu):.1f} à {max(lu):.1f} N ; la marge de l'OS, sur les "
      f"{PESE_ESSAI} N pesés, finit de {min(fin):.2f} à {max(fin):.2f} : de "
      f"{min(fin) * PESE_ESSAI / VRAI:.2f} à {max(fin) * PESE_ESSAI / VRAI:.2f} sur les 9,81 vrais",
      max(lu) - min(lu) > 1.0 and min(fin) < R.securite_serrage
      and min(fin) * PESE_ESSAI / VRAI > 1.95)
jg = [f(P[t][c], "force_juge_n") for t in TENU for c in ("left", "right")]
# Angles du carton (juge, repère du bassin) : ils oscillent au pas de la
# marche ; une rotation, elle, dériverait d'une seconde à l'autre.
ang = {c: [f(P[t]["left"], c) for t in TENU] for c in ("carton_roulis_deg", "carton_tangage_deg")}
oscille = max(abs(v) for vs in ang.values() for v in vs)
derive = max(abs(np.mean(vs[-10:]) - np.mean(vs[:10])) for vs in ang.values())
mains = [abs(f(P[t]["left"], c)) for t in TENU for c in ("port_roulis_deg", "port_tangage_deg",
                                                         "port_lacet_deg")
         if P[t]["left"].get(c)]
check(f"Carton tenu : force du juge de {min(jg):.1f} à {max(jg):.1f} N par main ; ses angles "
      f"(juge) oscillent de ±{oscille:.1f}° au pas, sans tourner ({derive:.1f}° de la première à "
      f"la dernière seconde) ; mains mesurées tournées de {max(mains):.1f}° au plus",
      min(jg) > 10.0 and oscille < 3.0 and derive < 0.5 and max(mains) < 1.5)
somme = np.array([f(P[t]["left"], "force_juge_n") + f(P[t]["right"], "force_juge_n")
                  for t in TENU])
reelle = [R.frottement_serrage * somme[i:i + 10].mean() / VRAI for i in range(len(somme) - 9)]
check(f"Marge réelle (juge, vrai poids, sur 1 s) : de {min(reelle):.2f} à {max(reelle):.2f} — tenu, "
      f"au ras du coefficient de sécurité ; le serrage réel varie à la marche (juge de "
      f"{min(jg):.1f} à {max(jg):.1f} N par main)",
      min(reelle) >= R.securite_serrage and max(jg) - min(jg) > 2.0)
LIM = {d.joint_id: min(R.effort_max_nm, R.part_couple_garde * d.limits.torque_max)
       for d in build_g1_joint_definitions("29dof")}
plafonds = {t: any(abs(v) > LIM[j] - 0.3 for j, v in couples(P[t]).items()) for t in TENU}


def regle(poids):
    """La règle du port de l'OS, sur la marge de l'OS au relevé ramenée à
    `poids` : arrêt sous securite_serrage ; reprise au-dessus de
    securite_reprise après delai_reprise_s, ou dès securite_serrage aux
    plafonds ; abandon sans reprise en duree_resserrage_s. Rend (arrêts,
    début de l'arrêt encore en cours à la fin du relevé ou None, abandon
    dans le relevé ou None)."""
    n, t0 = 0, None
    for t in TENU:
        if np.isnan(marge[t]):
            continue
        mg = marge[t] * PESE_ESSAI / poids
        if t0 is None:
            if mg < R.securite_serrage:
                t0, n = t, n + 1
        elif (mg > R.securite_reprise and t - t0 >= R.delai_reprise_s) or \
                (R.reprise_aux_plafonds and plafonds[t] and mg >= R.securite_serrage):
            t0 = None
        elif t - t0 >= R.duree_resserrage_s:
            return n, t0, t
    return n, t0, None


n_essai, ouvert_essai, ab_essai = regle(PESE_ESSAI)
n_long, ouvert_long, ab_long = regle(pese)
n_vrai, ouvert_vrai, ab_vrai = regle(VRAI)
# Le relevé finit à 48,34 s, l'abandon de l'essai vient à 48,4 : avec 10,6 N,
# le dernier arrêt est encore en cours à la fin du relevé, 1 s avant l'abandon.
check(f"Rejoué sur la règle de l'OS : avec les {PESE_ESSAI} N de l'essai, base arrêtée depuis "
      f"{ouvert_essai or float('nan'):.2f} s, sans reprise jusqu'au bout du relevé ({TENU[-1]:.2f} s ; "
      f"abandon de l'essai à {INTERRUPTION_ESSAI} s, {R.duree_resserrage_s:g} s après) ; avec le "
      f"vrai poids, chaque arrêt repris dans le relevé ({n_vrai})",
      ab_essai is None and ouvert_essai is not None
      and abs(ouvert_essai + R.duree_resserrage_s - INTERRUPTION_ESSAI) <= 0.1
      and ab_vrai is None and ouvert_vrai is None,
      f"{n_essai} {ouvert_essai} {ab_essai} / {ouvert_vrai} {ab_vrai}")
check(f"… mais avec la pesée longue ({pese:.2f} N, {100 * (pese / VRAI - 1):+.1f} %), le dernier "
      f"arrêt (depuis {ouvert_long or float('nan'):.2f} s) n'est toujours pas repris à la fin du "
      f"relevé : ce carton de 1 kg, poignets à leurs plafonds (marge réelle de {min(reelle):.2f} à "
      f"{max(reelle):.2f}), ne se porte qu'avec un poids juste à ~2 % près",
      ab_long is None and ouvert_long is not None and min(reelle) < 1.04 * R.securite_serrage,
      f"{ouvert_long} {ab_long}")

section("4. L'essai de 17 h 16 : pesé juste, rien ne change")
P2, LEV2, T2, LEC2, KC2, PALIER2, _ = levage("t1_levage_20261119_1716.csv")
fen2 = LEC2[KC2:KC2 + 3 * R.filtre_force]
pese2 = poids_pese(LEC2[PALIER2], R.part_ecartee_pesee)[0]
check(f"Médiane des 15 lectures : {np.median(fen2):.2f} N (le « 9.8 » du journal) ; pesée longue "
      f"sur son palier ({len(PALIER2)} lectures) : {pese2:.2f} N ({100 * (pese2 / VRAI - 1):+.1f} %)",
      abs(np.median(fen2) - 9.8) < 0.05 and abs(pese2 / VRAI - 1) <= 0.03)
P3 = charger("t1_prise_20261119_1716.csv")
lu3 = [f(d["left"], "poids_os_n") for d in P3.values()]
check(f"Son port : poids lu de {np.nanmin(lu3):.1f} à {np.nanmax(lu3):.1f} N ; la marge, sur les "
      f"9,8 N pesés, n'en dépend pas (test_port_sans_derive, §6) — ses arrêts venaient du serrage "
      f"réel au ras de 2 et du carton qui tournait entre les paumes (test_rejeu_port_t1716)",
      np.nanmax(lu3) - np.nanmin(lu3) > 1.0)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
