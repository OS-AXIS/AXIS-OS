"""
Biais des couples estimés : calibration au démarrage, et filet « main déjà
en appui » sur la force corrigée et sur la géométrie.

Constat GPU (essai T1 de main, novembre 2026) : « main déjà en appui » à
4.6 N puis 3.9 N (main droite) avant la fermeture, alors que le juge (vérité
de la simulation) ne voyait aucun contact ; main gauche entre 1.1 et 2.3 N.
Le bruit au repos était de 0.6 à 1.05 N : un biais, pas du bruit.

Désormais : au démarrage, bras en l'air dans les postures de la calibration
des mains, le service de manipulation relève couples estimés et pesanteur du
modèle, et en tire un biais par articulation (moindres carrés) et le résidu
(journal). La force à la main se calcule sur les couples corrigés de ces
biais ; le filet « déjà en appui » garde son seuil (max(3 N, 4 × bruit)) et
s'ajoute une géométrie : la main mesurée retenue en dehors de sa consigne à
la pose d'ouverture.

Et la dérive de signe pendant la fermeture (constat GPU : contact réel de
2.4 à 4.0 N, aucun signal de l'OS, puis « serrage insuffisant » à -3 N) : un
biais constant s'annule dans la force relative (zéro pris avant de fermer).
Ce qui la décale, c'est un couple pris dans ce zéro et qui disparaît
ensuite : les gains étaient raidis au début même de la fenêtre du zéro, et
le bras, moins affaissé, bougeait pendant qu'on la mesurait. Section 5.

Robot simulé léger (axis_os.testing) : `bruit_couples` reçoit les couples
de toutes les articulations des bras (ordre alphabétique) et rend ceux que
lit l'OS.
"""
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.isaac_sim.bruit_couples import BruitCouples  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import (  # noqa: E402
    Etat, Objet, Postures, posture_bras,
)
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


P = Postures()
# Les postures de la calibration des mains du banc T1
# (sim/validation/test_isaac_t1_tri_usine.py, postures_calib).
POSTURES = [("port", posture_bras(P.roulis_ouvert, P.tangage_port, P.coude)),
            ("ouverte", posture_bras(P.roulis_presaisie, P.tangage_port, P.coude)),
            ("basse", posture_bras(0.25, -0.15, 0.50)),
            ("haute", posture_bras(0.25, -0.70, 0.20)),
            ("port", posture_bras(P.roulis_ouvert, P.tangage_port, P.coude))]
# Biais choisis pour reproduire le constat : ~5 N à la main droite au repos.
BIAIS = {"right_shoulder_roll_joint": 1.0, "right_elbow_joint": -1.0,
         "right_shoulder_yaw_joint": 1.0, "left_shoulder_pitch_joint": 0.6,
         "left_elbow_joint": 0.4, "left_wrist_roll_joint": -0.3}
CARTON = Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20)


class Biais:
    """Couple estimé = couple vrai + un biais fixe par articulation."""

    def __init__(self, r, biais, suite=None):
        self.noms = sorted(list(r._BRAS) + list(r._poignets))
        self.b = np.array([biais.get(j, 0.0) for j in self.noms])
        self.suite = suite

    def __call__(self, v):
        v = np.asarray(v, dtype=float) + self.b
        return v if self.suite is None else self.suite(v)


def robot(y=0.0, biais=None, suite=None, cedent=True):
    r = SimulatedRobot(objects=[SimulatedObject("carton", (0.30, y, 0.90))], time_scale=2.0)
    r.__enter__()
    r.poignets_cedent = cedent
    time.sleep(0.2)
    if biais is not None:
        r.bruit_couples = Biais(r, biais, suite)
    return r


def calibrer(r, echantillons=10):
    """Ce que fait le banc T1 au démarrage : chaque posture, au plus
    0,02 rad par pas, puis des relevés bras immobiles. Comme sur le banc,
    loin de tout : le carton est écarté le temps de la calibration."""
    m, journal = r.manipulation, []
    m._journal = journal.append
    place = r.objects[0].position
    r.objects[0].position = (3.0, 3.0, 0.90)
    for nom, cible in POSTURES:
        while True:
            reste = 0.0
            for j, v in cible.items():
                e = v - r.adapter.pose_bras.get(j, v)
                r.adapter.pose_bras[j] = r.adapter.pose_bras.get(j, v) + max(-0.02, min(0.02, e))
                reste = max(reste, abs(e))
            if reste < 1e-6:
                break
            time.sleep(0.01)
        time.sleep(0.3)
        for _ in range(echantillons):
            m.echantillon_calibration_couples(nom)
            time.sleep(0.02)
    texte = m.terminer_calibration_couples()
    m.resynchroniser()
    m._journal = None
    r.objects[0].position = place
    time.sleep(0.2)
    return texte, journal


def prendre(r, objet=CARTON, duree=15.0):
    m = r.manipulation
    m.prendre(objet)
    t0 = time.monotonic()
    while m.etat not in (Etat.TENU, Etat.ECHEC) and time.monotonic() - t0 < duree:
        time.sleep(0.005)
    return m


def verite(r):
    """Biais vrais vus par l'OS : ceux injectés, plus le couple de repos
    (frottement) que le robot simulé ajoute aux roulis d'épaule."""
    v = {j: BIAIS.get(j, 0.0) for j in sorted(list(r._BRAS) + list(r._poignets))}
    v["left_shoulder_roll_joint"] += r.COUPLE_REPOS_NM
    v["right_shoulder_roll_joint"] -= r.COUPLE_REPOS_NM
    return v


section("1. Le constat GPU : un biais des couples fait croire à une main en appui")
r = robot(biais=BIAIS)
m = prendre(r)
check("Sans calibration : « main D déjà en appui » à ~5 N, sans aucun contact réel",
      m.etat is Etat.ECHEC and "main D déjà en appui" in m.raison
      and m.force_avant_fermeture["right"] > 4.0 and r.forces_contact["right"] == 0.0, m.raison)
r.__exit__(None, None, None)

section("2. Calibration au démarrage : un biais par articulation, et le résidu")
r = robot(biais=BIAIS)
texte, journal = calibrer(r)
m = r.manipulation
vrai = verite(r)
ecarts = {j: abs(m.biais_couples.get(j, math.inf) - b) for j, b in vrai.items()}
check("Biais trouvés à 0.05 N·m près, pour chacune des 12 articulations mesurées",
      len(ecarts) == 12 and max(ecarts.values()) < 0.05,
      f"{ {j: round(e, 3) for j, e in ecarts.items()} }")
check("Résidu faible (le modèle explique le reste) : moins de 0.05 N·m",
      m.stats["residu_couples_nm"] < 0.05, str(m.stats["residu_couples_nm"]))
check("Journal : biais trouvés et résidu, une ligne par articulation, puis le verdict (les "
      "couples suivent la pesanteur : fiables)",
      journal and journal[0].startswith("calibration des couples (4 postures)")
      and "résidu max" in journal[0] and len(journal) == 14
      and any("right_elbow_joint" in l and "biais -1.0" in l for l in journal)
      and journal[-1].startswith("couples estimés fiables"),
      "\n".join(journal[:3] + journal[-1:]))
m = prendre(r)
check("Après calibration : plus de fausse alarme, la prise tient",
      m.etat is Etat.TENU, m.raison)
check("… force absolue avant de fermer proche de zéro (moins de 1 N)",
      max(abs(v) for v in m.force_avant_fermeture.values()) < 1.0,
      str(m.force_avant_fermeture))
r.__exit__(None, None, None)

section("3. Bruit, biais et gains du banc (BruitCouples) : la calibration tient")
# Poignets raides, comme test_force_main.py section 6 : poignets mous et
# couples bruités, le serrage frôle ses 5 N (déjà le cas avant ce changement).
for graine in (1, 2, 3):
    r = robot(biais=BIAIS, suite=BruitCouples(12, graine=graine), cedent=False)
    texte, journal = calibrer(r, echantillons=25)
    m = r.manipulation
    m = prendre(r)
    check(f"Graine {graine} : prise tenue, force avant de fermer à "
          f"{max(abs(v) for v in m.force_avant_fermeture.values()):.1f} N au plus (sous le "
          f"seuil de 3 N)", m.etat is Etat.TENU
          and max(abs(v) for v in m.force_avant_fermeture.values()) < 3.0,
          f"{m.raison} {m.force_avant_fermeture} {journal[0]}")
    r.__exit__(None, None, None)

section("4. Une vraie main en appui reste refusée, par la force corrigée et par la géométrie")
# Carton 6.5 cm à gauche de sa perception : l'ouverture (5 cm) ne suffit
# pas, la main gauche repose sur lui à la pose d'ouverture.
for nom, reglages, signal in (("force et géométrie", ReglagesPrise(), "déjà en appui"),
                              ("force seule", ReglagesPrise(retard_appui_m=1.0), "N, seuil"),
                              ("géométrie seule", ReglagesPrise(force_appui_n=1e3),
                               "retenue à")):
    r = robot(y=0.065, biais=BIAIS)
    calibrer(r)
    r.manipulation.r = reglages
    m = prendre(r)
    check(f"Main G sur le carton ({nom}) : « main G déjà en appui », refusée avant de fermer",
          m.etat is Etat.ECHEC and "main G déjà en appui" in m.raison and signal in m.raison
          and Etat.FERMETURE is not None and not any(m.contacts_estimes.values()), m.raison)
    r.__exit__(None, None, None)
check("Seuil inchangé : max(3 N, 4 × bruit au repos)",
      ReglagesPrise().force_appui_n == 3.0 and ReglagesPrise().sigmas_appui == 4.0)

section("5. Zéro des couples pris bras immobile, après le raidissement")


class Transitoire:
    """Après le raidissement, le bras moins affaissé remonte : pendant ce
    mouvement, le couple du coude et du lacet d'épaule s'écarte de la
    pesanteur, puis y revient (1,5 N·m, constante de temps 0,15 s)."""

    def __init__(self, r, amplitude=1.5, tau=0.15):
        self.r, self.a, self.tau = r, amplitude, tau
        self.noms = sorted(list(r._BRAS) + list(r._poignets))
        self.t_raidi, self.prec = None, False

    def __call__(self, v):
        if self.r.raideur_active and not self.prec:
            self.t_raidi = self.r.clock()
        self.prec = self.r.raideur_active
        v = np.asarray(v, dtype=float).copy()
        if self.t_raidi is not None:
            k = self.a * math.exp(-(self.r.clock() - self.t_raidi) / self.tau)
            for i, j in enumerate(self.noms):
                if j.endswith(("elbow_joint", "shoulder_yaw_joint")):
                    v[i] += -k if j.startswith("left") else k
        return v


r = robot()
r.bruit_couples = Transitoire(r)
m = r.manipulation
m.prendre(CARTON)
t_raidi, t0_zero, forces_avant_contact = None, None, []
t0 = time.monotonic()
while m.etat not in (Etat.TENU, Etat.ECHEC) and time.monotonic() - t0 < 15:
    if r.raideur_active and t_raidi is None:
        t_raidi = r.clock()
    if m.etat is Etat.FERMETURE and m._sous != "bruit" and t0_zero is None:
        t0_zero = r.clock()
    if m.etat is Etat.FERMETURE and m._sous != "bruit" and r.forces_contact["left"] == 0.0 \
            and m.forces["left"] is not None:
        forces_avant_contact.append(float(m.forces["left"] @ m._vers_objet("left")))
    time.sleep(0.002)
check("Gains raidis dès le placement, zéro des couples pris au moins 0.5 s après",
      t_raidi is not None and t0_zero is not None
      and t0_zero - t_raidi >= m.r.duree_raidissement_s, f"{t_raidi} {t0_zero}")
check("… force relative non décalée avant le contact (moins de 1 N, aucun mauvais signe)",
      forces_avant_contact and max(abs(f) for f in forces_avant_contact) < 1.0,
      f"{min(forces_avant_contact, default=None)} {max(forces_avant_contact, default=None)}")
check("… et la prise tient", m.etat is Etat.TENU, m.raison)
r.__exit__(None, None, None)

section("6. Référence du contrôle d'appui prise bras posés, pas à l'appel de prendre()")
# Constat (CI de la PR 24, machine chargée) : prendre() appelé bras encore en
# mouvement ; la référence de l'écart main-consigne, prise à cet instant,
# était fausse, et « main G déjà en appui (retenue à 1.6 cm) » refusait la
# prise sans aucun objet. Ici, le mouvement est simulé : pendant 0.3 s après
# l'appel, le roulis gauche mesuré est en retard de 0.06 rad sur sa consigne.
r = SimulatedRobot(objects=[], time_scale=2.0)
r.__enter__()
time.sleep(0.2)
m = r.manipulation
vrai, debut = m._angles_mesures, []


def en_mouvement():
    a = vrai()
    if a is not None and debut and r.clock() - debut[0] < 0.3:
        a = dict(a)
        a["left_shoulder_roll_joint"] -= 0.06
    return a


m._angles_mesures = en_mouvement
debut.append(r.clock())
m = prendre(r)
check("Bras encore en mouvement à l'appel, aucun objet : pas de fausse alarme d'appui, la "
      "prise va jusqu'à « rien entre les mains »", m.etat is Etat.ECHEC
      and "rien entre les mains" in m.raison and "déjà en appui" not in m.raison, m.raison)
check("… référence prise bras posés : retard de la main à l'ouverture sous 1 mm",
      max(abs(v) for v in m.stats["retard_ouverture_cm"].values()) < 0.1,
      str(m.stats.get("retard_ouverture_cm")))
r.__exit__(None, None, None)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
