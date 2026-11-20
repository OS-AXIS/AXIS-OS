"""
Port sans dérive, carton surveillé pendant tout le port (essai GPU T1 du
19 novembre 2026, 17 h 16 ; test_rejeu_port_t1716.py) : carton porté 20 s,
jamais tombé, mais tourné de 24° sans aucune alerte de l'OS ; le resserrage
poussait une main pendant que le plafond du poignet droit faisait reculer
l'autre : la paire a glissé de côté sans serrer plus, et la base s'est
arrêtée 32 fois sur le seuil de la marge de frottement.

Robot simulé léger, horloge simulée, G1 29 DOF (7 articulations par bras),
mains du G1 :
  1. Pose figée au début du port : graine des bras, paumes et ligne des
     mains mesurées ; carton droit, mains immobiles.
  2. Un poignet à son plafond et la marge sous le seuil (le cas de
     l'essai) : resserrage et desserrage symétriques, le long de la normale,
     depuis la posture figée : la paire ne glisse pas de côté, les paumes ne
     tournent pas, le même serrage donne les mêmes bras. Le chemin d'avant
     (une main à la fois, bras sans pose figée) faisait glisser la paire.
  3. Mains mesurées qui dérivent (lacet d'épaule) : dérive vue en moyenne
     glissante, ramenées vers la pose figée.
  4. Carton qui tourne (paumes mesurées) : au-delà de 4°, base arrêtée et
     correction ; au-delà de 10°, transport interrompu, à reposer, la
     raison au journal avec le nombre de resserrages du port.
  5. Hystérésis : marge au ras du seuil, couples bruités comme au banc.
     Stricte : la base ne repart qu'au-dessus de securite_reprise, après
     delai_reprise_s (bien moins d'arrêts qu'avant, en repartant au seuil
     sur la marge de 0,3 s ; la foulée seule en retire déjà), et
     s'arrête proprement si les bras n'y arrivent pas ; le bilan du port au
     journal. Par défaut (reprise_aux_plafonds), les bras à leurs plafonds :
     elle repart dès que la marge tient securite_serrage, jamais en dessous.
  6. La marge sur le poids le plus prudent (essai du 20 novembre, 21 h 37 :
     pesé 8,4 N pour 9,8, lu de 10,1 à 11,4 au port) : le plus grand de la
     pesée et de la médiane du poids lu sur la foulée ; une pesée trop légère
     ne rend plus la marge optimiste ; le journal dit sur quel poids.
"""
import contextlib
import inspect
import io
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.drivers.isaac_sim.bruit_couples import BruitCouples  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import Etat, Objet, _rotation_vecteur  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
R = ReglagesPrise()


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Banc:
    """Le carton T1 (1 kg) pris et tenu, robot simulé léger."""

    def __init__(self, masse=1.0, bruit=False):
        self.t = [0.0]
        self.r = SimulatedRobot(objects=[SimulatedObject("carton", (0.41, 0.0, 0.90))],
                                horloge=lambda: self.t[0], contact_main_m=0.11,
                                poignets_29dof=True)
        self.r.MASSE_OBJET_KG = masse
        if bruit:
            self.r.bruit_couples = BruitCouples(len(self.r._BRAS) + len(self.r._poignets),
                                                graine=1716)
        self.r.start(fil=False)
        self.m = self.r.manipulation
        self.journal = []
        self.m._journal = self.journal.append
        self.sortie = io.StringIO()
        self.depasse = 0.0                  # couple estimé au-delà de son plafond (N·m)
        self.depasse_vrai = 0.0             # couple VRAI (monde simulé) au-delà de son plafond
        self.pas(20)
        self.m.prendre(Objet((0.41, 0.0, 0.90), 0.24, 0.20, 0.20))
        t0 = self.t[0]
        while self.m.etat not in (Etat.TENU, Etat.ECHEC) and self.t[0] - t0 < 40.0:
            self.pas()

    def pas(self, n=1):
        with contextlib.redirect_stdout(self.sortie):
            for _ in range(n):
                self.t[0] += self.r._dt
                self.r.pas()
                tau, _ = self.m._efforts()
                if tau:
                    self.depasse = max([self.depasse] + [abs(v) - self.m._limite_effort(j)
                                                         for j, v in tau.items()])
                    self.depasse_vrai = max([self.depasse_vrai] + [
                        abs(self.r.couples_vrais.get(j, 0.0)) - self.m._limite_effort(j)
                        for j in tau])

    def lignes(self, debut):
        return [l_ for l_ in self.journal if l_.startswith(debut)]

    def mains_commandees(self, debut=None):
        """(milieu des mains, rotation des paumes par rapport aux poses
        commandées `debut` en degrés, écart des mains) commandés, repère du
        torse."""
        p = self.m._poses(self.m._q)
        o = debut or p
        rot = max(math.degrees(float(np.linalg.norm(_rotation_vecteur(p[c][1] @ o[c][1].T))))
                  for c in p)
        return ((p["left"][0] + p["right"][0]) / 2.0, rot,
                float(np.linalg.norm(p["left"][0] - p["right"][0])))

    def refus_garde_fou(self):
        return self.sortie.getvalue().count("SAFETY VIOLATION")


def poignet_au_plafond(b):
    """Le cas de l'essai : le tangage du poignet droit à son plafond (moitié
    du couple max du garde-fou), et le coefficient de sécurité un peu
    au-dessus de la marge : le resserrage est demandé pendant que le plafond
    fait reculer la main droite."""
    tau, _ = b.m._efforts()
    j = "right_wrist_pitch_joint"
    b.m.mesures.couple_max[j] = 2.0 * (abs(tau[j]) - 0.05)
    b.m.r = replace(b.m.r, securite_serrage=(b.m.marge_frottement or 2.0) + 0.05,
                    duree_resserrage_s=1e9)        # on regarde la consigne, pas l'abandon


section("1. Pose figée au début du port")
b = Banc()
b.pas(60)
m = b.m
check("Carton pris et tenu", m.etat is Etat.TENU, m.raison)
check("Pose figée au début du port : graine des bras, paumes et ligne des mains mesurées, "
      "position commune de référence", m._port0 is not None
      and {"q", "ori0", "ligne0", "pos0"} <= set(m._port0), str(m._port0 and m._port0.keys()))
check(f"Carton droit, mains immobiles : rotation {m.rotation_port and max(map(abs, m.rotation_port)):.2f}° "
      f"(roulis, tangage, lacet), dérive {m.derive_port and 1000 * m.derive_port[0]:.1f} mm",
      m.rotation_port is not None and max(map(abs, m.rotation_port)) < 0.5
      and m.derive_port is not None and m.derive_port[0] < 0.003 and not m.penche)

section("2. Un poignet à son plafond, la marge sous le seuil (le cas de l'essai)")
resultats = {}
for nom, fige in (("pose figée", True), ("une main à la fois (avant)", False)):
    b = Banc()
    b.pas(60)
    if not fige:
        b.m._port0 = None               # bras sans pose figée : le chemin d'avant
    debut = b.m._poses(b.m._q)
    c0, _, e0 = b.mains_commandees()
    poignet_au_plafond(b)
    b.pas(400)
    c1, rot, e1 = b.mains_commandees(debut)
    resultats[nom] = (float(np.linalg.norm(c1 - c0)), rot, e1 - e0, b)
glisse, rot, de, b = resultats["pose figée"]
glisse_av, _, de_av, _ = resultats["une main à la fois (avant)"]
check(f"Pose figée : la paire commandée glisse de {1000 * glisse:.2f} mm, paumes tournées de "
      f"{rot:.2f}° (au plus {R.tol_rigide_deg:g}), écart des mains changé de {1000 * de:+.1f} mm "
      f"(serrage et desserrage, le long de la normale)",
      glisse < 0.001 and rot <= R.tol_rigide_deg, f"{glisse} {rot}")
check(f"Le chemin d'avant, même scénario : la paire glisse de {1000 * glisse_av:.1f} mm de côté "
      f"(écart changé de {1000 * de_av:+.1f} mm)", glisse_av > 5 * max(glisse, 0.0005),
      f"{glisse_av}")
m = b.m
d = dict(m._delta)
q1, q2 = m._q_port(d), m._q_port(d)
check("Le même serrage donne les mêmes bras (calculés depuis la posture figée, pas depuis la "
      "consigne du moment)", q1 is not None and max(abs(q1[j] - q2[j]) for j in q1) < 1e-9)
check(f"Plafonds tenus : au plus {max(0.0, b.depasse):.2f} N·m au-delà, aucune consigne refusée "
      f"par le garde-fou", b.depasse <= 0.1 and b.refus_garde_fou() == 0,
      f"{b.depasse} {b.refus_garde_fou()}")

section("3. Mains mesurées qui dérivent : ramenées vers la pose figée")
b = Banc()
b.pas(60)
m = b.m
lire = m.mesures.positions
derive_rad = {"left_shoulder_yaw_joint": 0.06, "right_shoulder_yaw_joint": 0.06}
m.mesures.positions = lambda: {j: v + derive_rad.get(j, 0.0) for j, v in lire().items()}
b.pas(150)
corr = b.lignes("port : mains dérivées de")
check(f"Lacet des deux épaules mesuré 0,06 rad à côté : dérive vue en moyenne sur "
      f"{R.fenetre_derive_s:g} s, « {corr[:1] and corr[0][:100]}… »",
      corr and m.corrections_ce_port >= 1, str(b.journal[-4:]))
b.pas(200)
check(f"… et ramenées : dérive restante {m.derive_port and 1000 * m.derive_port[0]:.1f} mm "
      f"(au plus {1000 * R.derive_port_m:.0f}), correction {1000 * float(np.linalg.norm(m._correction_port[0])):.1f} mm "
      f"(au plus {1000 * R.correction_port_max_m:.0f}), carton tenu, marche libre",
      m.derive_port is not None and m.derive_port[0] <= R.derive_port_m
      and m.etat is Etat.TENU and not m.penche and not m.a_reposer, str(m.derive_port))
m.mesures.positions = lire

section("4. Carton qui tourne : base arrêtée, puis à reposer")
b = Banc()
b.pas(60)
m = b.m
lire = m.mesures.positions
tourne = {"left_wrist_pitch_joint": 0.09, "right_wrist_pitch_joint": 0.09}
m.mesures.positions = lambda: {j: v + tourne.get(j, 0.0) for j, v in lire().items()}
for _ in range(5):
    b.pas()
    alarme = b.lignes("port : carton tourné de")
    if alarme:
        break
check(f"Paumes mesurées tournées de ~5° : « {alarme[:1] and alarme[0][:110]}… », base arrêtée",
      alarme and m.penche and not m.a_reposer, f"{m.rotation_port} {b.journal[-3:]}")
b.pas(150)
check(f"… mains ramenées vers la pose figée : carton revenu ({m.rotation_port and max(map(abs, m.rotation_port)):.1f}°), "
      f"la marche peut reprendre", b.lignes("port : carton revenu") and not m.penche
      and m.etat is Etat.TENU, str(b.journal[-3:]))
tourne.update(left_wrist_pitch_joint=0.35, right_wrist_pitch_joint=0.35)
b.pas(3)
check(f"Tournées de ~20° : transport interrompu, à reposer : « {m.raison[:110]}… »",
      m.a_reposer and m.penche and "carton tourné de" in m.raison and "à reposer" in m.raison,
      m.raison)
fin = b.lignes("fin du port (interrompu)")
check(f"Au journal, le bilan du port : « {fin[:1] and fin[0]} »",
      fin and "resserrage(s)" in fin[0] and "correction(s)" in fin[0])
m.mesures.positions = lire

section("5. Hystérésis du resserrage")
# Couples bruités comme au banc : le desserrage aux plafonds (inchangé) fait
# varier le serrage et la marge de quelques centièmes ; le seuil d'arrêt est
# mis juste sous la marge du début du port. Le VRAI couple sous le plafond, à
# l'erreur d'estimation du banc près (son biais au plus), comme la campagne de
# cartons (violations).
tol = 0.1 + inspect.signature(BruitCouples).parameters["biais_max_nm"].default
# « Avant » : la règle d'avant l'hystérésis, sur la marge de 0,3 s d'alors
# (fenetre_marge_s=0,3) ; les autres, la règle d'aujourd'hui (foulée de 1 s,
# reprise jugée sur 0,3 s base arrêtée).
essais = {}
for nom, reprise, delai, plafonds, fenetre in (
        ("reprise au seuil (avant)", 0.0, 0.0, True, 0.3),
        ("stricte, reprise 0,1 au-dessus", 0.1, R.delai_reprise_s, False, R.fenetre_marge_s),
        # Hors de portée des bras : 0,5 au-dessus (0,2 l'était selon le tirage
        # du bruit : resserrés, les bras gagnaient jusqu'à 0,21).
        ("stricte, reprise hors de portée", 0.5, R.delai_reprise_s, False, R.fenetre_marge_s),
        ("reprise 0,2 au-dessus, aux plafonds au seuil", 0.2, R.delai_reprise_s, True,
         R.fenetre_marge_s),
        ("reprise au seuil, sur la foulée", 0.0, 0.0, True, R.fenetre_marge_s)):
    b = Banc(bruit=True)
    m = b.m
    m.r = replace(m.r, fenetre_marge_s=fenetre)
    b.pas(60)
    seuil = (m.marge_frottement or 2.0) - 0.01
    m.r = replace(m.r, securite_serrage=seuil, securite_reprise=seuil + reprise,
                  delai_reprise_s=delai, reprise_aux_plafonds=plafonds)
    n0 = m.resserrages_ce_port
    b.pas(1000)
    essais[nom] = (m.resserrages_ce_port - n0, b)
avant = essais["reprise au seuil (avant)"][0]
apres, b = essais["stricte, reprise 0,1 au-dessus"]
check(f"Marge au ras du seuil, 20 s de port : {avant} arrêts de la base en repartant au seuil, "
      f"{apres} en ne repartant qu'à 0,1 au-dessus, après {R.delai_reprise_s:g} s",
      avant >= 2 * max(1, apres), f"{avant} {apres}")
m = b.m
retrouve = b.lignes("port : serrage retrouvé")
# Chaque reprise : (marge dite, reprise), arrondie au centième au journal.
reprises = [(float(l_.split("marge de frottement ")[1].split(",")[0]),
             float(l_.split("au-dessus de ")[1].split(")")[0]))
            for l_ in retrouve if "au-dessus de " in l_]
# Au ras du seuil, avec le bruit du banc, la stricte s'arrête proprement quand
# les bras n'atteignent pas la reprise à temps (3 tirages du bruit sur 8) :
# c'est son rôle. Ce qu'elle garantit : jamais de reprise sous securite_reprise,
# carton toujours tenu, plafonds et garde-fou respectés.
check(f"… chaque reprise dit sa marge, toujours au-dessus de la reprise ({len(reprises)} "
      f"reprises, « {retrouve[:1] and retrouve[0]} ») ; carton tenu ("
      + ("porté jusqu'au bout" if not m.a_reposer else f"arrêt propre : « {m.raison[:70]}… »")
      + f") ; couples vrais au plus {max(0.0, b.depasse_vrai):.2f} N·m au-delà des plafonds "
      f"(erreur d'estimation du banc : {tol:.1f}) ; aucune consigne refusée par le garde-fou",
      reprises and len(reprises) == len(retrouve) and all(x >= y - 0.005 for x, y in reprises)
      and m.etat is Etat.TENU and (not m.a_reposer or m.penche)
      and b.depasse_vrai <= tol and b.refus_garde_fou() == 0,
      f"{reprises} {m.raison} {b.depasse_vrai}")
n, b = essais["stricte, reprise hors de portée"]
m = b.m
check(f"Stricte, reprise hors de portée des bras (0,5 au-dessus) : la base ne repart pas sous elle ; "
      f"{n} arrêt(s), puis arrêt propre, carton tenu : « {m.raison[:150]}… »",
      m.etat is Etat.TENU and m.a_reposer and m.penche and b.depasse_vrai <= tol
      and b.refus_garde_fou() == 0, m.raison)
fin = b.lignes("fin du port (interrompu)")
check(f"Au journal, les resserrages du port : « {fin[:1] and fin[0]} »",
      fin and f"{m.resserrages_ce_port} resserrage(s)" in fin[0])
n, b = essais["reprise 0,2 au-dessus, aux plafonds au seuil"]
m = b.m
plafonds = [l_ for l_ in b.journal if "hors de portée des bras, à leurs plafonds" in l_]
check(f"Par défaut (reprise_aux_plafonds), la même reprise hors de portée, bras à leurs "
      f"plafonds : la base repart dès que la marge tient securite_serrage, jamais en dessous, et "
      f"le dit (« {plafonds[:1] and plafonds[0][:100]}… ») ; {n} arrêts ; carton porté",
      plafonds and m.etat is Etat.TENU and not m.a_reposer and b.depasse_vrai <= tol
      and b.refus_garde_fou() == 0
      and all(float(l_.split("marge de frottement ")[1].split(",")[0]) >= m.r.securite_serrage
              for l_ in plafonds), f"{m.raison} {b.journal[-3:]}")

foulee = essais["reprise au seuil, sur la foulée"][0]
check(f"La foulée seule (1 s), toujours en repartant au seuil : {foulee} arrêts, au plus ceux de "
      f"la marge sur 0,3 s ({avant}) : le bruit des couples arrête moins la base",
      foulee <= avant, f"{foulee} {avant}")

section("6. La marge sur le poids le plus prudent : la pesée, ou le poids lu sur la foulée")
# Essai GPU du 20 novembre 2026, 21 h 37 : pesé 8,4 N pour 9,8 (le couple que
# le centre de gravité, hors de la ligne des mains, fait passer par elles, lu
# comme une force), lu de 10,1 à 11,4 N au port. Sur la seule pesée, la marge
# était optimiste de 17 %.
b = Banc()
b.pas(60)
m = b.m
lu = float(np.median([w for _, w in m._poids_port]))
m.poids_carton = 0.85 * lu                # une pesée trop légère, comme à 21 h 37
b.pas(5)
somme = float(np.mean(m._hist_normales))
check(f"Pesée trop légère ({m.poids_carton:.1f} N pour {lu:.1f} lus sur la foulée) : marge "
      f"{m.marge_frottement:.2f} = 0,8 × somme des normales / {lu:.1f} N lus, pas "
      f"{R.frottement_serrage * somme / m.poids_carton:.2f} sur la pesée : jamais optimiste",
      abs(m.marge_frottement - R.frottement_serrage * somme
          / float(np.median([w for _, w in m._poids_port]))) < 1e-9
      and m.marge_frottement < R.frottement_serrage * somme / m.poids_carton - 0.1)
m.poids_carton = 1.15 * lu                # une pesée trop lourde : c'est elle qui compte
b.pas(5)
check(f"Pesée plus lourde que le poids lu ({m.poids_carton:.1f} N) : la marge se calcule sur elle",
      abs(m.marge_frottement - R.frottement_serrage * float(np.mean(m._hist_normales))
          / m.poids_carton) < 1e-9)
m.poids_carton = lu
b.r.MASSE_OBJET_KG = 11.2 / 9.81         # le carton pèse plus sur les mains, comme en marchant
b.pas(60)
lu1 = float(np.median([w for _, w in m._poids_port]))
m.r = replace(m.r, securite_serrage=m.marge_frottement + 0.3, duree_resserrage_s=1e9)
b.pas(5)
arret = b.lignes("port : marge de frottement")
check(f"Le poids lu monte à {lu1:.1f} N : la marge le suit, et le journal le dit à l'arrêt de la "
      f"base : « {arret[-1] if arret else ''} »",
      lu1 > lu + 1.0 and arret and "lus sur la foulée, plus que les" in arret[-1] and m.penche)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
