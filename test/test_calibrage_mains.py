"""
Calibrage des mains de la prise mesurée.

Constat GPU (2026-10-28, evaluer_vla.sh 5 1 --reflexes) : 0/5, « rien entre
les mains » ×5 ; à la fermeture, les mains étaient contre la face AVANT du
carton, ~10 cm du milieu des faces latérales. On vérifie ici :
  - CalibrationMains : écart entre la main calculée par la cinématique
    (angles mesurés) et la main mesurée ; au-delà de 1,5 cm, refus ;
  - ManipulationService : prise mesurée refusée tant que la calibration
    a échoué ; avant de fermer, les mains vues par le robot et celles du
    modèle doivent concorder, et être au milieu des faces ;
  - chaque échec de prise est journalisé avec les deux mains et l'objet ;
  - le banc T1 fait la calibration avant de laisser la main à la mission.
"""
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.cinematique import ModeleBras  # noqa: E402
from axis_os.services.manipulation.prise import CalibrationMains  # noqa: E402
from axis_os.services.manipulation.service import Etat, Objet, posture_bras  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
RACINE = Path(__file__).resolve().parents[1]


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def attendre(m, etats, duree=15.0):
    t0 = time.monotonic()
    while m.etat not in etats and time.monotonic() - t0 < duree:
        time.sleep(0.01)


section("1. CalibrationMains : modèle contre mains mesurées, sur quelques postures")
modele = ModeleBras()
postures = [("port", posture_bras(0.20, -0.30, 0.0)), ("basse", posture_bras(0.25, -0.15, 0.5)),
            ("haute", posture_bras(0.25, -0.70, 0.2))]
juste = CalibrationMains(modele)
for nom, q in postures:
    juste.ajouter(nom, q, modele.mains(q))
ok, texte = juste.bilan()
check("Mains mesurées = modèle : écart nul, prise mesurée autorisée", ok and juste.ecart_max < 1e-9)
fausse = CalibrationMains(modele)
for nom, q in postures:
    g, d = modele.mains(q)
    fausse.ajouter(nom, q, (g + np.array([-0.10, 0.0, 0.0]), d + np.array([-0.10, 0.0, 0.0])))
ok, texte = fausse.bilan()
check("Mains réelles 10 cm en avant du modèle : REFUSÉE", not ok and "REFUSÉE" in texte)
check("… le bilan dit l'écart de chaque main, posture par posture (-10,0 cm en x)",
      texte.count("-10.0, +0.0, +0.0") == 2 * len(postures), texte)
limite = CalibrationMains(modele)
q = postures[0][1]
g, d = modele.mains(q)
limite.ajouter("port", q, (g + np.array([0.0, 0.0, 0.016]), d))
check("1,6 cm d'écart sur une seule main : refusée (limite 1,5 cm)", not limite.bilan()[0])
nan = CalibrationMains(modele)
nan.ajouter("port", q, (np.full(3, math.nan), d))
check("Main mesurée non finie : refusée", not nan.bilan()[0] and nan.ecart_max == math.inf)
check("Aucun relevé : refusée", not CalibrationMains(modele).bilan()[0])

section("2. Calibration ratée : aucune prise mesurée")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))], time_scale=2.0) as r:
    m = r.manipulation
    m.calibration_refus = "cinématique des bras non calibrée : main réelle à 10.0 cm du modèle"
    time.sleep(0.2)
    avant = r._angles_bras()
    res = m.prendre(Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20))
    time.sleep(0.3)
    apres = r._angles_bras()
    check("prendre() refusé, avec la raison de la calibration", not res.ok
          and "non calibrée" in res.reason, res.reason)
    check("… bras immobiles, aucune exemption", max(abs(apres[j] - avant[j]) for j in avant) < 1e-3
          and r.guard._exemption is None)

section("3. Avant de fermer : mains mesurées et modèle d'accord, au milieu des faces")
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))], time_scale=2.0) as r:
    m = r.manipulation
    journal = []
    m._journal = journal.append
    vraies = m._mains

    def decalees():
        g, d = vraies()
        return g + np.array([0.03, 0.0, 0.0]), d + np.array([0.03, 0.0, 0.0])
    m._mains = decalees            # le robot voit ses mains 3 cm ailleurs que le modèle
    etats = []
    m.prendre(Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20))
    t0 = time.monotonic()
    while m.etat not in (Etat.ECHEC, Etat.TENU) and time.monotonic() - t0 < 10:
        etats.append(m.etat)
        time.sleep(0.005)
    check("Désaccord de 3 cm : ÉCHEC « mains mesurées à 3.0 cm du modèle »",
          m.etat is Etat.ECHEC and "3.0 cm du modèle" in m.raison, m.raison)
    check("… avant toute fermeture, exemption levée", Etat.FERMETURE not in etats
          and r.guard._exemption is None)
    check("… journalisé : mains mesurées, modèle, cibles, objet visé",
          len(journal) == 1 and all(k in journal[0] for k in
                                    ("prise ratée", "main G (", "modèle G (", "cibles G (",
                                     "objet visé (")), journal)
with SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))], time_scale=2.0) as r:
    m = r.manipulation
    res = m.prendre(Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20))
    attendre(m, (Etat.TENU, Etat.ECHEC))
    check("Mains et modèle d'accord : la prise va jusqu'au bout", m.etat is Etat.TENU, m.raison)

section("4. Chaque échec de prise est journalisé (mains et objet)")
with SimulatedRobot(objects=[], time_scale=2.0) as r:
    m = r.manipulation
    journal = []
    m._journal = journal.append
    m.prendre(Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20))
    attendre(m, (Etat.ECHEC,))
    check("Rien entre les mains : ÉCHEC journalisé avec les mains et l'objet visé",
          m.etat is Etat.ECHEC and journal and "rien entre les mains" in journal[-1]
          and "main D (" in journal[-1] and "objet visé (0.300, 0.000, 0.180)" in journal[-1],
          journal)
with SimulatedRobot(objects=[], time_scale=2.0) as r:
    m = r.manipulation
    journal = []
    m._journal = journal.append
    res = m.prendre(Objet((0.56, 0.0, 0.90), 0.24, 0.20, 0.20))
    check("Refus « hors de portée » : journalisé aussi, avec l'objet visé",
          not res.ok and journal and "hors de portée" in journal[-1]
          and "objet visé (0.560" in journal[-1], journal)

section("5. Banc T1 (lecture du script, sans Isaac Sim)")
T1 = (RACINE / "sim/validation/test_isaac_t1_tri_usine.py").read_text(encoding="utf-8")
check("Calibration des mains avant que la mission ne commande quoi que ce soit",
      "if not etat_calib[\"fini\"]:\n            etat_calib[\"fini\"] = pas_calibration()\n"
      "        else:\n            pilote.pas()" in T1)
check("Mains réelles (main_monde) comparées au modèle sur les angles mesurés, 4 postures",
      "calib.ajouter(nom, {j: st_.joint_pos[j] for j in chaine_bras}" in T1
      and T1.count('posture_bras(') >= 5)
check("Calibration ratée : bilan au journal, prise mesurée refusée",
      "manip.calibration_refus = (" in T1 and "journaliser(ligne_)" in T1)
check("Échecs de prise journalisés avec la position réelle des cartons",
      "journal=lambda m: journaliser(" in T1 and "cartons_reels()" in T1)
check("Repère complet du bassin pour les cibles et les mains (vers_bassin)",
      "vers_bassin=vers_robot" in T1)

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
