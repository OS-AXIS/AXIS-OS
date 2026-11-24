"""
Calibration de charge : une fois par robot, une masse connue, le gain des
coudes ; enregistrée au profil du robot avec sa date ; sans elle, l'OS le dit
et pèse comme avant.

Pourquoi : la pesée de l'OS (au levage, bras immobiles) gardait un biais de +2
à +5 % côté lourd sur les essais GPU T1 du 19 novembre 2026 (17 h 16 et
22 h 04) : les gains des couples estimés (±5 % par moteur, au banc comme sur le
vrai G1), et presque tout par les deux coudes (test_rejeu_port_t2204, §2). Un
carton de 1 kg, poignets à leurs plafonds, ne se porte qu'avec un poids juste à
~2 % près.

La calibration (ManipulationService.calibrer_charge) : la masse de référence
prise comme un carton (la prise de T1), levée, pesée bras immobiles plus
longtemps (250 lectures), puis reposée, ni serrée plus, ni portée. Son poids,
partagé moitié-moitié entre les mains (charge centrée, prise par ses deux
faces), donne le gain du couple estimé de chaque coude (prise.gain_coude) :
une équation par main, une inconnue par bras. Hors de [0,85 ; 1,15] : refusée
(masse décentrée ou mal déclarée), rien n'est changé.

Vérifié ici :
  1. le profil du robot (profil_robot) : absent, écrit d'un coup, relu ; d'un
     autre robot ou illisible, refusé sans être écrasé ;
  2. gain_coude, à la main ;
  3. sur le robot simulé de la CI, gains des coudes faussés comme ceux du banc
     Isaac (1,047 et 0,980) : sans calibration, l'OS le dit ; la calibration
     retrouve les gains, repose la masse là où elle était, rend les gains de
     prise et l'exemption, s'enregistre datée ; relue au démarrage suivant,
     la pesée de cartons de 0,5 à 1,2 kg tombe à 2 % près ; masse mal
     déclarée, ou hors bornes : refusée, la calibration d'avant reste ;
  4. sur les CSV des essais GPU (même robot simulé : même graine des erreurs
     de gain), avec le calcul de l'OS : les gains des coudes retrouvés sur un
     palier de 35 lectures ; la pesée de l'OS rejouée, calibrée sur les AUTRES
     essais, à 2 % près (sans : +1,6 et +4,7 %).
"""
import contextlib
import csv
import io
import json
import os
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.isaac_sim.bruit_couples import BruitCouples  # noqa: E402
from axis_os.services.manipulation.cinematique import ModeleBras  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise, gain_coude, poids_pese  # noqa: E402
from axis_os.services.manipulation.service import Etat, ManipulationService, Objet  # noqa: E402
from axis_os.services.profil_robot import (  # noqa: E402
    FORMAT, ProfilRobotErreur, enregistrer, lire_profil,
)
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
R = ReglagesPrise()
VRAI = 9.81


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


section("1. Le profil du robot : un fichier JSON par robot, écrit d'un coup")
with tempfile.TemporaryDirectory() as d:
    chemin = os.path.join(d, "robots", "g1_0042.json")
    vide = lire_profil(chemin, "g1_0042")
    check("Absent : un profil vide (robot jamais calibré), rien d'écrit",
          vide == {"format": FORMAT, "robot": "g1_0042"} and not os.path.exists(chemin))
    fiche = {"date": "2026-11-20T10:00:00+00:00", "masse_kg": 1.0,
             "gains_couples": {"left_elbow_joint": 1.05, "right_elbow_joint": 0.98}}
    enregistrer(chemin, "g1_0042", "calibration_charge", fiche)
    doc = json.load(open(chemin, encoding="utf-8"))
    check("Écrit : format, robot, calibration avec sa date ; aucun fichier temporaire laissé",
          doc["format"] == FORMAT and doc["robot"] == "g1_0042"
          and doc["calibration_charge"]["date"] == fiche["date"]
          and os.listdir(os.path.dirname(chemin)) == ["g1_0042.json"])
    check("Relu tel quel", lire_profil(chemin, "g1_0042")["calibration_charge"] == fiche)
    try:
        lire_profil(chemin, "g1_0043")
        autre = ""
    except ProfilRobotErreur as e:
        autre = str(e)
    check(f"Celui d'un autre robot : refusé (« {autre} »)", "g1_0043" in autre)
    with open(chemin, "w", encoding="utf-8") as f:
        f.write('{"format": "axis-profil-robot", "robot": "g1_0042", "calib')
    try:
        enregistrer(chemin, "g1_0042", "calibration_charge", fiche)
        illisible = ""
    except ProfilRobotErreur as e:
        illisible = str(e)
    check("Illisible : refusé, et pas écrasé par un profil neuf (on ne devine pas)",
          "illisible" in illisible and open(chemin, encoding="utf-8").read().endswith('"calib'))

section("2. gain_coude : une équation par main, une inconnue par bras")
# Main qui porte la moitié de 9,81 N ; le coude en donne 7,9 N (palier de 2204).
part = 7.9
for k in (0.95, 1.0, 1.047):
    lu = VRAI / 2 + (1.0 / k - 1.0) * -part        # ce que lit un coude de gain k
    check(f"Coude de gain {k} : retrouvé {gain_coude(lu, part, VRAI):.4f}",
          abs(gain_coude(lu, part, VRAI) - k) < 1e-9)
check("Coude qui ne porte rien : pas de gain (NaN)", np.isnan(gain_coude(4.9, 0.0, VRAI)))


class GainsCoudes:
    """Couples estimés du robot simulé : ceux des coudes faussés d'un gain,
    comme ceux du banc Isaac (bruit_couples, graine 3)."""

    def __init__(self, noms, gains):
        self.noms, self.gains = noms, gains

    def __call__(self, valeurs):
        return [v * self.gains.get(n, 1.0) for n, v in zip(self.noms, valeurs)]


GAINS_BANC = {"left_elbow_joint": 1.047, "right_elbow_joint": 0.980}
X = 0.41


class Banc:
    def __init__(self, masse=1.0, gains=GAINS_BANC):
        self.t = [0.0]
        self.r = SimulatedRobot(objects=[SimulatedObject("carton", (X, 0.0, 0.90))],
                                horloge=lambda: self.t[0], contact_main_m=0.11,
                                poignets_29dof=True)
        self.r.MASSE_OBJET_KG = masse
        self.r.start(fil=False)
        self.r.bruit_couples = GainsCoudes(sorted(list(self.r._BRAS) + list(self.r._poignets)),
                                           gains)
        self.m = self.r.manipulation
        # Les biais que la calibration des couples trouve au démarrage (comme
        # au banc et sur le robot) : le couple de repos des roulis du robot
        # simulé. Sans eux, il pèserait 0,25 N de trop, quelle que soit la masse.
        c = self.r.COUPLE_REPOS_NM
        self.m.biais_couples = {"left_shoulder_roll_joint": c, "right_shoulder_roll_joint": -c}
        self.journal = []
        self.m._journal = self.journal.append
        self.sortie = io.StringIO()
        self.pas(20)

    def pas(self, n=1):
        with contextlib.redirect_stdout(self.sortie):
            for _ in range(n):
                self.t[0] += self.r._dt
                self.r.pas()

    def attendre(self, fin, duree=40.0):
        t0 = self.t[0]
        while self.m.etat not in fin and self.t[0] - t0 < duree:
            self.pas()
        return self.t[0] - t0

    def prendre(self):
        self.m.prendre(Objet((X, 0.0, 0.90), 0.24, 0.20, 0.20))
        self.attendre((Etat.TENU, Etat.ECHEC))
        return self.m.poids_carton

    def ligne(self, debut):
        return next((x for x in reversed(self.journal) if x.startswith(debut)), "")


section("3. Robot simulé, coudes faussés comme au banc Isaac (1,047 et 0,980)")
# Prise en pendule : la pesée de ce robot simulé, aux gains vrais, lit 1 % de
# moins (poignets très souples, efforts calculés sur les bras non déformés) ;
# coudes faussés, elle s'écarte de la vérité dans un sens ou dans l'autre, et
# la calibration en retrouve les gains à 3 % près. Ce qui compte, la pesée
# une fois calibrée, reste jugé à 2 % près (plus bas).
b0 = Banc()
p0 = b0.prendre()
check(f"Sans calibration : pesé {p0:.2f} N ({100 * (p0 / VRAI - 1):+.1f} %), et l'OS le dit "
      f"(« …{b0.ligne('poids du carton').split(' ; ')[-1]} »)",
      b0.m.etat is Etat.TENU and "sans calibration de charge" in b0.ligne("poids du carton")
      and abs(p0 / VRAI - 1.0) > 0.005)
with tempfile.TemporaryDirectory() as d:
    chemin = os.path.join(d, "simule.json")
    b = Banc()
    texte = b.m.appliquer_calibration_charge(lire_profil(chemin, "simule").get("calibration_charge"))
    check(f"Profil sans calibration, au démarrage : « {texte[:72]}… »",
          texte.startswith("pas de calibration de charge") and b.m.gains_couples == {}
          and "PREMIERE_MISE_EN_ROUTE" in texte)
    raideur0 = len(b.r.changements_raideur)
    res = b.m.calibrer_charge(Objet((X, 0.0, 0.90), 0.24, 0.20, 0.20), 1.0,
                              enregistrer=lambda f: enregistrer(chemin, "simule",
                                                                "calibration_charge", f))
    duree = b.attendre((Etat.LIBRE, Etat.ECHEC))
    ligne = b.ligne("calibration de charge du")
    o = b.r.objects[0]
    k = b.m.gains_couples
    check(f"Calibration faite en {duree:.1f} s, revenue en LIBRE (« {ligne[:90]}… »)",
          res.ok and b.m.etat is Etat.LIBRE and "250 lectures" in ligne, b.m.raison)
    check(f"Gains retrouvés : G {k.get('left_elbow_joint', 0):.3f}, D "
          f"{k.get('right_elbow_joint', 0):.3f} (faussés 1,047 et 0,980), à 3 % près",
          all(abs(k.get(j, 0) / v - 1) <= 0.03 for j, v in GAINS_BANC.items()), str(k))
    check(f"Masse reposée là où elle était ({o.position[0]:.3f}, {o.position[1]:.3f}, "
          f"{o.position[2]:.3f}), lâchée, jamais tombée ; gains de prise rendus, exemption levée",
          b.r.held is None and not b.r.chutes and abs(o.position[0] - X) < 0.01
          and abs(o.position[2] - 0.90) < 0.01 and len(b.r.changements_raideur) > raideur0
          and b.r.changements_raideur[-1] is False and not b.m.raideur_active
          and (b.m._gardien is None or b.m._gardien._exemption is None), str(b.r.chutes))
    profil = lire_profil(chemin, "simule")
    cal = profil.get("calibration_charge", {})
    check(f"Enregistrée au profil du robot, datée ({cal.get('date')}), 1 kg, 250 lectures",
          cal.get("date", "").startswith("20") and cal.get("masse_kg") == 1.0
          and cal.get("lectures") == 250
          and set(cal.get("gains_couples", {})) == {"left_elbow_joint", "right_elbow_joint"})
    # Démarrage suivant : un robot neuf relit son profil.
    for masse in (1.0, 0.5, 1.2):
        b2 = Banc(masse=masse)
        texte = b2.m.appliquer_calibration_charge(
            lire_profil(chemin, "simule").get("calibration_charge"))
        p = b2.prendre()
        check(f"Relue au démarrage (« {texte[:60]}… ») : carton de {masse:g} kg pesé {p:.2f} N "
              f"({100 * (p / (VRAI * masse) - 1):+.1f} %), à 2 % près ; l'OS dit d'où vient la "
              f"correction",
              texte.startswith("calibration de charge du 20") and b2.m.etat is Etat.TENU
              and abs(p / (VRAI * masse) - 1) <= 0.02
              and "gains des coudes de la calibration de charge du 20" in b2.ligne("poids du carton"))
    # Refus : masse mal déclarée (0,7 kg réels pour 1 kg déclaré), hors bornes.
    b3 = Banc(masse=0.7)
    b3.m.appliquer_calibration_charge(lire_profil(chemin, "simule").get("calibration_charge"))
    avant = dict(b3.m.gains_couples)
    b3.m.calibrer_charge(Objet((X, 0.0, 0.90), 0.24, 0.20, 0.20), 1.0,
                         enregistrer=lambda f: enregistrer(chemin, "simule", "calibration_charge", f))
    b3.attendre((Etat.LIBRE, Etat.ECHEC))
    check(f"Masse mal déclarée (0,7 kg pour 1) : refusée, reposée (« {b3.m.raison[:110]}… ») ; "
          f"la calibration d'avant reste, au profil aussi",
          b3.m.etat is Etat.ECHEC and "calibration de charge refusée" in b3.m.raison
          and b3.m.gains_couples == avant and b3.r.held is None and not b3.r.chutes
          and lire_profil(chemin, "simule")["calibration_charge"] == cal)
    for m_ in (0.3, 2.0, float("nan")):
        r_ = b3.m.calibrer_charge(Objet((X, 0.0, 0.90), 0.24, 0.20, 0.20), m_)
        check(f"Masse de référence de {m_:g} kg : refusée sans bouger (« {r_.reason[:80]}… »)",
              not r_.ok and "calibration de charge refusée" in r_.reason
              and b3.m.gains_couples == avant)
    b4 = Banc()
    b4.r.grasp_succeeds = False
    b4.m.appliquer_calibration_charge(lire_profil(chemin, "simule").get("calibration_charge"))
    b4.m.calibrer_charge(Objet((X, 0.0, 0.90), 0.24, 0.20, 0.20), 1.0)
    b4.attendre((Etat.LIBRE, Etat.ECHEC))
    check("Calibration qui échoue en route : gains d'avant rendus, dit au journal",
          b4.m.gains_couples == avant and any("calibration de charge non faite" in x
                                             for x in b4.journal))
    faux = dict(cal, gains_couples={"left_elbow_joint": 1.4, "right_elbow_joint": 1.0})
    texte = b4.m.appliquer_calibration_charge(faux)
    check(f"Profil aux gains hors bornes : pas appliqué, dit (« {texte[:60]}… »)",
          "NON APPLIQUÉE" in texte and b4.m.gains_couples == {})

section("4. Les CSV des essais GPU : la pesée de l'OS à 2 % près, calibrée sur les autres")
DONNEES = Path(__file__).resolve().parent / "donnees"
ESSAIS = {"1716": "t1_levage_20261119_1716.csv", "2204": "t1_prise_20261119_2204.csv",
          "2157": "t1_prise_20261105_2157.csv"}
ART = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch",
       "wrist_yaw")
# Biais de la calibration des couples des essais (journal : biais_couples_nm).
BIAIS = {"left_shoulder_pitch_joint": 0.36, "left_shoulder_roll_joint": 0.0,
         "left_shoulder_yaw_joint": 0.31, "left_elbow_joint": -0.03,
         "left_wrist_roll_joint": 0.2, "left_wrist_pitch_joint": 0.35,
         "left_wrist_yaw_joint": 0.1, "right_shoulder_pitch_joint": 0.18,
         "right_shoulder_roll_joint": 0.28, "right_shoulder_yaw_joint": -0.04,
         "right_elbow_joint": -0.34, "right_wrist_roll_joint": -0.21,
         "right_wrist_pitch_joint": -0.26, "right_wrist_yaw_joint": -0.05}
# Erreurs de gain du banc (graine 3, ordre des articulations d'Isaac Sim :
# test_rejeu_port_t2204) : les vraies, pour juger ce que la calibration retrouve.
ORDRE_ISAAC = ("left_hip_pitch", "right_hip_pitch", "waist_yaw", "left_hip_roll",
               "right_hip_roll", "waist_roll", "left_hip_yaw", "right_hip_yaw", "waist_pitch",
               "left_knee", "right_knee", "left_shoulder_pitch", "right_shoulder_pitch",
               "left_ankle_pitch", "right_ankle_pitch", "left_shoulder_roll",
               "right_shoulder_roll", "left_ankle_roll", "right_ankle_roll",
               "left_shoulder_yaw", "right_shoulder_yaw", "left_elbow", "right_elbow",
               "left_wrist_roll", "right_wrist_roll", "left_wrist_pitch", "right_wrist_pitch",
               "left_wrist_yaw", "right_wrist_yaw")
GAIN = {f"{j}_joint": g for j, g in zip(ORDRE_ISAAC, BruitCouples(len(ORDRE_ISAAC), graine=3)._gain)}
G = np.array([0.0, 0.0, -9.81])


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


M = ManipulationService.__new__(ManipulationService)     # l'OS réduit à ses forces
M.r, M._fixes = R, {}
M._init_mesure()
M.mesures = SimpleNamespace(modele=ModeleBras(), contact_main_m=None)
M.biais_couples, M.gains_couples = dict(BIAIS), {}
M._tau0, M._g0 = {}, {}


def palier(nom):
    """Le carton levé, au serrage de prise, bras immobiles : des 0,3 dernières
    secondes du levage commandé jusqu'à la montée du serrage (la pesée de
    l'OS) ; et les lectures une à une de l'OS (sa moyenne glissante
    poids_os_n, repartie de zéro à la confirmation), sauf à 21 h 57 (pesée
    d'alors)."""
    P = charger(ESSAIS[nom])
    lev = [t for t in P if P[t]["left"]["phase"] == "LEVAGE"]
    juge = np.array([(f(P[t]["left"], "force_juge_n") + f(P[t]["right"], "force_juge_n")) / 2
                     for t in lev])
    haut = juge > float(np.median(juge[:20])) + 0.4
    monte = int(np.argmax(haut)) if haut.any() else len(lev)
    pal = [i for i, t in enumerate(lev) if t >= lev[0] + R.duree_levage_test_s - 0.3 - 1e-6
           and i < monte]
    rel = [(couples(P[lev[i]]), angles(P[lev[i]]), G) for i in pal]
    lec = None
    if nom != "2157":
        moy = [f(P[t]["left"], "poids_os_n") for t in lev]
        d0 = next(i for i, v in enumerate(moy) if not np.isnan(v))
        k0 = next(i for i in range(d0 + 1, len(moy)) if abs(moy[i] - moy[i - 1]) > 0.8
                  and lev[i] > lev[0] + 1.0)
        n, tout = 3 * R.filtre_force, []
        for debut, fin in ((d0, k0), (k0, len(moy))):
            lec_ = []
            for k, i in enumerate(range(debut, fin)):
                lec_.append(moy[i] if k == 0 else (k + 1) * moy[i] - k * moy[i - 1] if k < n
                            else lec_[k - n] + n * (moy[i] - moy[i - 1]))
            tout += lec_
        lec = np.array([tout[i - d0] for i in pal])
    return rel, lec


def poids_releve(rel, gains):
    """Poids lu sur les couples du relevé (une autre lecture que celle de
    l'OS), lecture par lecture, avec ces gains des coudes."""
    M.gains_couples = dict(gains)
    w = np.array([sum(float(M._force_main(c, tau, q, g, depuis_fermeture=False)[2])
                      for c in ("left", "right")) for tau, q, g in rel])
    M.gains_couples = {}
    return w


def calibrer(noms):
    M.gains_couples = {}
    mes = M._gains_coudes([x for n in noms for x in PAL[n][0]], VRAI)
    return {j: v[0] for j, v in mes.items()}, mes


PAL = {n: palier(n) for n in ESSAIS}
for nom in ("1716", "2204"):
    k, mes = calibrer([nom])
    check(f"Calibré sur le palier de {nom} ({len(PAL[nom][0])} lectures du relevé) : coudes G "
          f"{k['left_elbow_joint']:.3f} ±{mes['left_elbow_joint'][2]:.3f}, D "
          f"{k['right_elbow_joint']:.3f} ±{mes['right_elbow_joint'][2]:.3f} ; ceux du banc : "
          f"{GAIN['left_elbow_joint']:.3f} et {GAIN['right_elbow_joint']:.3f}, à 2 % près",
          all(abs(k[j] / GAIN[j] - 1) <= 0.02 for j in k), str(k))
# La pesée de l'OS rejouée (ses lectures, moyenne tronquée), corrigée de ce que
# la calibration change à chaque lecture (calculé sur les couples du relevé aux
# mêmes instants : la correction est linéaire en couple), calibrée sur les
# AUTRES essais : la calibration, faite une fois, sert aux cartons suivants.
for nom, autres in (("1716", ("2204", "2157")), ("2204", ("1716", "2157"))):
    k, _ = calibrer(autres)
    rel, lec = PAL[nom]
    corr = poids_releve(rel, k) - poids_releve(rel, {})
    avant, _ = poids_pese(lec, R.part_ecartee_pesee)
    apres, u = poids_pese(lec + corr, R.part_ecartee_pesee)
    check(f"Pesée de l'OS de {nom} ({len(lec)} lectures), calibrée sur {' et '.join(autres)} : "
          f"{apres:.2f} N ({100 * (apres / VRAI - 1):+.1f} %), à 2 % près ; sans : {avant:.2f} N "
          f"({100 * (avant / VRAI - 1):+.1f} %) ; bruit de ces lectures ±{100 * 2 * u / VRAI:.1f} %",
          abs(apres / VRAI - 1) <= 0.02 and abs(apres / VRAI - 1) < abs(avant / VRAI - 1))
k, _ = calibrer(("1716", "2204"))
w = poids_releve(PAL["2157"][0], k)
check(f"Essai de 21 h 57 (couples du relevé, {len(w)} lectures ; pesée de l'OS d'alors, autre), "
      f"calibré sur 1716 et 2204 : {poids_pese(w, R.part_ecartee_pesee)[0]:.2f} N "
      f"({100 * (poids_pese(w, R.part_ecartee_pesee)[0] / VRAI - 1):+.1f} %), à 2 % près",
      abs(poids_pese(w, R.part_ecartee_pesee)[0] / VRAI - 1) <= 0.02)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
