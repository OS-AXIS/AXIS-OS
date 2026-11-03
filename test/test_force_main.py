"""
Force estimée à la main, et serrage décidé sur elle (en newtons).

Constat GPU (2026-10-29, PR 16, 3 essais) : fermeture vue des deux mains,
puis 3 fois « serrage insuffisant (0.1 / 0.4 / 0.1 N·m au plus serré) ». Le
serrage ne bougeait que le roulis d'épaule et ne mesurait que son couple.

On vérifie ici, sans GPU :
  - la jacobienne de la main (ModeleBras) contre des différences finies, et
    contre Pinocchio s'il est installé ; le poids des bras de même ;
  - sur le robot simulé léger (bras souples : chaque articulation un ressort
    autour de sa consigne, une main ne traverse pas l'objet) : la force
    estimée par l'OS (pseudo-inverse amortie de Jᵀ sur tous les couples du
    bras) contre la vraie force de contact du monde simulé ; la force passe
    par le lacet d'épaule autant que par le roulis ; le serrage atteint la
    force visée ; plafond de couple de chaque articulation ; carton qui
    tourne ; couples bruités sans faux contact ni faux serrage ; lacet
    d'épaule et poignets commandés pendant le serrage, rendus ensuite.
"""
import math
import sys
import time
from pathlib import Path

import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE))

from axis_os.hal.drivers.isaac_sim.bruit_couples import BruitCouples  # noqa: E402
from axis_os.services.manipulation.cinematique import URDF_G1_29DOF, ModeleBras  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import Etat, Objet, posture_bras  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
R = ReglagesPrise()
CARTON = Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20)


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def bras(c):
    return [f"{c}_{a}" for a in ("shoulder_pitch_joint", "shoulder_roll_joint",
                                 "shoulder_yaw_joint", "elbow_joint", "wrist_roll_joint",
                                 "wrist_pitch_joint", "wrist_yaw_joint")]


def prendre(r, objet=CARTON, jusqua=(Etat.TENU, Etat.ECHEC), duree=15.0, pendant=None):
    m = r.manipulation
    m.prendre(objet)
    etats, t0 = [], time.monotonic()
    while m.etat not in jusqua and time.monotonic() - t0 < duree:
        if not etats or etats[-1] is not m.etat:
            etats.append(m.etat)
        if pendant is not None:
            pendant(r, m)
        time.sleep(0.005)
    etats.append(m.etat)
    return etats


section("1. Jacobienne et poids des bras (cinématique de l'URDF)")
modele = ModeleBras()
rng = np.random.default_rng(3)
pire_j = pire_g = 0.0
for _ in range(12):
    q = posture_bras(rng.uniform(0.1, 0.4), rng.uniform(-0.7, -0.1), rng.uniform(-0.2, 0.9))
    q.update({j: float(rng.uniform(-0.4, 0.4)) for c in ("left", "right") for j in bras(c)[2:]})
    for c, chaine in (("left", modele.gauche), ("right", modele.droite)):
        js = bras(c)
        jac = chaine.jacobienne(q, js)
        g = chaine.couples_gravite(q, js)
        for i, j in enumerate(js):
            q2 = dict(q)
            q2[j] += 1e-6
            pire_j = max(pire_j, float(np.abs((chaine.main(q2) - chaine.main(q)) / 1e-6
                                              - jac[:, i]).max()))
            # Couple de maintien = dérivée de l'énergie potentielle des liens.
            v = [sum(l[3] * 9.81 * float(l[2][2]) for l in chaine._reperes(qq)[0])
                 for qq in (q, q2)]
            pire_g = max(pire_g, abs((v[1] - v[0]) / 1e-6 - float(g[i])))
check(f"Jacobienne de la main = différences finies, 7 articulations par bras, 12 postures "
      f"(écart {pire_j:.1e} m/rad)", pire_j < 1e-5)
check(f"Couple de maintien contre le poids = dérivée de l'énergie potentielle (écart "
      f"{pire_g:.1e} N·m)", pire_g < 1e-4)
try:
    import pinocchio as pin
except ImportError:
    pin = None
    print("  (Pinocchio absent : comparaison de référence non faite)")
if pin is not None:
    mod_p = pin.buildModelFromUrdf(str(URDF_G1_29DOF))
    don = mod_p.createData()
    q = posture_bras(0.25, -0.3, 0.5)
    q.update({"left_shoulder_yaw_joint": 0.2, "left_wrist_yaw_joint": -0.3})
    qp = pin.neutral(mod_p)
    for nom, v in q.items():
        jid = mod_p.getJointId(nom)
        qp[mod_p.joints[jid].idx_q] = v
    pin.computeJointJacobians(mod_p, don, qp)
    pin.updateFramePlacements(mod_p, don)
    fid = mod_p.getFrameId("left_rubber_hand")
    jp = pin.getFrameJacobian(mod_p, don, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3]
    cols = [mod_p.joints[mod_p.getJointId(j)].idx_v for j in bras("left")]
    gp = pin.computeGeneralizedGravity(mod_p, don, qp)
    check("Pinocchio : même jacobienne de la main gauche",
          np.allclose(jp[:, cols], modele.gauche.jacobienne(q, bras("left")), atol=1e-6))
    check("Pinocchio : même couple de maintien contre le poids",
          np.allclose(gp[cols], modele.gauche.couples_gravite(q, bras("left")), atol=1e-4))

section("2. Force estimée contre la vraie force de contact ; le lacet d'épaule en porte")
journal = []
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=2.0) as r:
    m = r.manipulation
    m._journal = journal.append
    time.sleep(0.2)
    vu = {}

    def relever(r_, m_):
        if m_.etat is Etat.LEVAGE and "levage" not in vu:
            vu["levage"] = (dict(r_.forces_contact), dict(m_.force_serrage),
                            dict(m_._q), {j: v for j, v in m_._libres0.items()})
            etat = r_.raw_hal.get_full_state()
            q = r_._angles_bras()
            vu["contact"] = {j: etat[j].torque - g for j, g in zip(
                r_._BRAS[:5], r_.modele_bras.gauche.couples_gravite(q, r_._BRAS[:5]))}
    prendre(r, pendant=relever)
    check("Prise tenue", m.etat is Etat.TENU, m.raison)
    vraie, estimee, q_serre, libres0 = vu["levage"]
    check(f"Au levage : force réelle {vraie['left']:.1f} / {vraie['right']:.1f} N, estimée par "
          f"l'OS {estimee['left']:.1f} / {estimee['right']:.1f} N (10 % près)",
          all(abs(estimee[c] - vraie[c]) <= 0.1 * vraie[c] for c in vraie), f"{vraie} {estimee}")
    check(f"Serrage arrivé à la force visée ({R.force_serrage_n:.0f} N, 1 N près)",
          min(vraie.values()) >= R.force_serrage_n - 1.0, str(vraie))
    lacet, roulis = abs(vu["contact"]["left_shoulder_yaw_joint"]), \
        abs(vu["contact"]["left_shoulder_roll_joint"]) - r.COUPLE_REPOS_NM
    total = sum(abs(v) for v in vu["contact"].values())
    check(f"La force passe par tout le bras : lacet d'épaule {lacet:.2f} N·m, roulis "
          f"{roulis:.2f} N·m ; le roulis seul ne voit que {100 * roulis / total:.0f} % du couple "
          f"de contact du bras", lacet >= roulis and roulis < 0.5 * total, str(vu["contact"]))
    check("Lacet d'épaule et poignet commandés pendant le serrage (plus figés)",
          "left_shoulder_yaw_joint" in libres0 and "left_wrist_roll_joint" in libres0
          and abs(q_serre["left_shoulder_yaw_joint"] - libres0["left_shoulder_yaw_joint"]) > 1e-4,
          f"{libres0} {q_serre.get('left_shoulder_yaw_joint')}")
    check("Bruit au repos donné au journal avant la fermeture",
          any(l.startswith("bruit de la force estimée au repos") for l in journal), str(journal))
    m.poser(0.70)
    t0 = time.monotonic()
    while m.etat is not Etat.LIBRE and time.monotonic() - t0 < 10:
        time.sleep(0.01)
    check("Après le dépôt : lacet d'épaule et poignets rendus à leur consigne d'avant",
          m.etat is Etat.LIBRE and not m._libres0
          and all(abs(m._bras[j] - v) < 1e-6 for j, v in libres0.items()),
          f"{m.etat} {[(j, m._bras.get(j), v) for j, v in libres0.items()]}")

section("3. Plafond de couple de chaque articulation du bras")
journal = []
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                    time_scale=2.0) as r:
    m = r.manipulation
    m._journal = journal.append
    m.mesures.couple_max["left_shoulder_yaw_joint"] = 3.0         # limite 1,5 N·m
    time.sleep(0.2)
    pic = [0.0]

    def suivre(r_, m_):
        if m_.etat is Etat.SERRAGE:
            pic[0] = max(pic[0], abs(r_.raw_hal.get_full_state()["left_shoulder_yaw_joint"].torque))
    prendre(r, pendant=suivre)
    check(f"Lacet d'épaule limité à 1,5 N·m : au plus {pic[0]:.2f} N·m pendant le serrage (un "
          f"pas de dépassement au plus)", pic[0] <= 1.5 + 0.1, str(pic[0]))
    time.sleep(1.0)
    fin = abs(r.raw_hal.get_full_state()["left_shoulder_yaw_joint"].torque)
    check(f"… objet tenu : sous la limite ({fin:.2f} N·m) une fois les bras arrêtés",
          m.etat is not Etat.TENU or fin <= 1.5 + 0.05, str(fin))
    check("… le serrage s'est arrêté sur cette limite (prise tenue, ou refus clair)",
          m.stats.get("serrages_limites", 0) > 0 and (m.etat is Etat.TENU
                                                      or "serrage insuffisant" in m.raison),
          f"{m.etat} {m.raison} {m.stats}")

section("4. Échec : force estimée par main et couple de chaque articulation au journal")
journal = []
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90), raideur_n_m=400.0)],
                    time_scale=2.0) as r:
    m = r.manipulation
    m._journal = journal.append
    time.sleep(0.2)
    prendre(r)
    ligne = next((l for l in journal if l.startswith("prise ratée")), "")
    check("Carton mou : « serrage insuffisant » en newtons à la main",
          m.etat is Etat.ECHEC and "serrage insuffisant" in m.raison and " N à la main" in m.raison,
          m.raison)
    check("… journal : force estimée à chaque main et couple de chaque articulation des bras",
          "forces estimées à la main : G (" in ligne and "D (" in ligne
          and all(f"{c}_{a}" in ligne for c in ("left", "right")
                  for a in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                            "wrist_roll", "wrist_yaw")), ligne)

section("5. Carton qui tourne pendant la prise")
for lacet in (8.0, 13.0):
    with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                        time_scale=2.0) as r:
        m = r.manipulation
        time.sleep(0.2)
        tourne = [False]

        def tourner(r_, m_, lacet=lacet):
            if not tourne[0] and any(m_.contacts_estimes.values()):
                r_.objects[0].lacet = math.radians(lacet)       # il tourne au contact
                tourne[0] = True
        prendre(r, pendant=tourner)
        check(f"Carton tourné de {lacet:.0f}° au contact : prise tenue ou refus clair, sans "
              f"consigne refusée par le garde-fou", tourne[0] and (
                  m.etat is Etat.TENU or "prise refusée" in m.raison)
              and r.locomotion.stats["refused"] == 0, f"{m.etat} {m.raison}")
        check("… jamais plus de 10 N·m à une articulation des bras",
              m.effort_max_mesure <= R.effort_max_nm, f"{m.effort_max_mesure:.1f}")

section("6. Couples bruités (estimés, comme le vrai G1) : aucun faux contact, aucun faux serrage")
for i in range(3):
    journal = []
    with SimulatedRobot(objects=[], time_scale=2.0) as r:
        r.bruit_couples = BruitCouples(12, graine=10 + i)
        m = r.manipulation
        m._journal = journal.append
        time.sleep(0.2)
        etats = prendre(r)
        check(f"Rien entre les mains (graine {10 + i}) : ÉCHEC « rien entre les mains », aucun "
              f"contact estimé, aucun serrage", m.etat is Etat.ECHEC
              and "rien entre les mains" in m.raison and not any(m.contacts_estimes.values())
              and Etat.SERRAGE not in etats, f"{m.raison} {m.contacts_estimes}")
bruits = []
for i in range(3):
    journal = []
    with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))],
                        time_scale=2.0) as r:
        r.bruit_couples = BruitCouples(12, graine=20 + i)
        m = r.manipulation
        m._journal = journal.append
        time.sleep(0.2)
        prendre(r)
        bruits += list(m.bruit_force.values())
        check(f"Carton (graine {20 + i}) : prise tenue, force réelle d'au moins "
              f"{R.force_min_n:.0f} N à chaque main", m.etat is Etat.TENU
              and min(r.forces_contact.values()) >= R.force_min_n,
              f"{m.raison} {r.forces_contact}")
check(f"Bruit de la force mesuré au repos : {min(bruits):.2f} à {max(bruits):.2f} N par lecture "
      f"(~0,85 attendus pour 0,3 N·m par couple)", 0.3 <= min(bruits) and max(bruits) <= 1.6,
      str(bruits))
check(f"Seuil de contact jamais sous {R.sigmas_contact:.0f} fois le bruit filtré",
      m._seuil_contact >= R.sigmas_contact * max(m.bruit_force.values())
      / math.sqrt(R.filtre_force) - 1e-9)

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
