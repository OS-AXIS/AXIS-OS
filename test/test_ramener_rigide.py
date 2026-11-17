"""
Ramener le carton vers le buste sans le tordre : les deux mains bougent comme
un seul corps rigide avec lui.

Constat GPU (essai T1 du 6 novembre 2026, 18 h 21 ; test_rejeu_port_t1821.py) :
la cinématique inverse du retour ne tenait que la POSITION des points de
contact ; les paumes ont piqué de ~15° autour de la ligne des mains, la
gauche a roulé de ~10° de plus que la droite, et le carton s'est tordu entre
elles (9,7°), en perdant 6 N de serrage.

Désormais (service.py, ReglagesPrise) :
  - chaque main garde son orientation et suit la même translation (à
    tol_rigide_deg et tol_rigide_m près), sur tout le chemin vérifié d'avance
    (sans saut de posture, avant-bras au-dessus du plateau) ; sinon on
    rapproche moins, ou pas du tout : le carton reste là où le levage l'a
    laissé ;
  - la pose entière d'une main demande six articulations commandées au moins
    (le tangage du poignet) : un bras à cinq ne ramène pas le carton ;
  - pendant le retour, le roulis et le tangage du carton estimés sur les
    mains mesurées ne passent pas rotation_ramener_max_deg : au-delà, le
    geste s'arrête et l'on resserre avant de marcher.

Robot simulé léger, horloge simulée, mains du G1 (contact 11 cm le long de la
main) ; bras à 7 articulations (poignets_29dof), sauf mention.
"""
import contextlib
import io
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

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
    def __init__(self, poignets_29dof=True, reglages=None, x=0.41):
        self.t = [0.0]
        self.x = x
        self.r = SimulatedRobot(objects=[SimulatedObject("carton", (x, 0.0, 0.90))],
                                horloge=lambda: self.t[0], contact_main_m=0.11,
                                poignets_29dof=poignets_29dof)
        self.r.start(fil=False)
        self.m = self.r.manipulation
        if reglages is not None:
            self.m.r = reglages
        self.journal = []
        self.m._journal = self.journal.append
        self.etats, self.sortie = [], io.StringIO()
        self.depasse = 0.0
        self.depart = None                 # centre de l'objet et poses au début du retour
        self.pas(20)

    def pas(self, n=1):
        with contextlib.redirect_stdout(self.sortie):
            for _ in range(n):
                self.t[0] += self.r._dt
                self.r.pas()
                if self.m.etat is Etat.RAMENER and self.depart is None:
                    self.depart = (np.array(self.m._c_de), self.m._poses(self.m._q_de))
                if not self.etats or self.etats[-1] is not self.m.etat:
                    self.etats.append(self.m.etat)
                tau, _ = self.m._efforts()
                if tau:
                    self.depasse = max([self.depasse] + [abs(v) - self.m._limite_effort(j)
                                                         for j, v in tau.items()])

    def prendre(self, duree=40.0):
        self.m.prendre(Objet((self.x, 0.0, 0.90), 0.24, 0.20, 0.20))
        t0 = self.t[0]
        while self.m.etat not in (Etat.TENU, Etat.ECHEC) and self.t[0] - t0 < duree:
            self.pas()
        return self

    def ligne(self, debut):
        return next((l for l in self.journal if l.startswith(debut)), "")


def ecarts(b):
    """Au bout du retour : la plus grande rotation d'une main COMMANDÉE
    depuis le début du geste (degrés), et l'écart des translations des deux
    mains, la variation du serrage (le long de la ligne des mains) retirée (m)."""
    c0, p0 = b.depart
    p1 = b.m._poses(b.m._q)
    rot = max(math.degrees(float(np.linalg.norm(_rotation_vecteur(p1[c][1] @ p0[c][1].T))))
              for c in ("left", "right"))
    u = p0["right"][0] - p0["left"][0]
    u = u / np.linalg.norm(u)
    d = (p1["left"][0] - p0["left"][0]) - (p1["right"][0] - p0["right"][0])
    return rot, float(np.linalg.norm(d - (d @ u) * u))


section("1. Retour en corps rigide : le carton ramené, ni roulé ni tangué")
b = Banc().prendre()
m, r = b.m, b.r
check("Carton pris et tenu", m.etat is Etat.TENU, m.raison)
rot, trans = ecarts(b)
recul = float(b.depart[0][0] - m._centre_porte[0])
check(f"Ramené de {100 * recul:.1f} cm vers le buste (au journal)", recul >= 0.05
      and b.ligne("carton ramené sans toucher le torse"), b.journal[-3:])
check(f"… mains commandées : orientation gardée ({rot:.2f}° au plus, tolérance "
      f"{R.tol_rigide_deg:g}), même translation pour les deux ({1000 * trans:.1f} mm près)",
      rot <= R.tol_rigide_deg and trans <= 2 * R.tol_rigide_m, f"{rot} {trans}")
check(f"… mains mesurées : roulis et tangage estimés du carton {m.rotation_carton} (au plus "
      f"{R.rotation_ramener_max_deg:g}°), vrai carton à plat (monde simulé)",
      m.rotation_carton is not None and max(abs(v) for v in m.rotation_carton)
      < R.rotation_ramener_max_deg and r.rotation_objet_deg == 0.0, str(m.rotation_carton))
check(f"… sans consigne refusée par le garde-fou, plafonds tenus ({max(0.0, b.depasse):.2f} N·m "
      f"au-delà au plus)", b.sortie.getvalue().count("SAFETY VIOLATION") == 0
      and b.depasse <= 0.1, f"{b.depasse}")
b.pas(250)
check("Cinq secondes de port : tenu, rien de signalé", m.etat is Etat.TENU and not m.a_reposer
      and not m.penche and r.held is not None, m.raison)

section("2. Pas de chemin rigide : rapprocher moins, ou pas du tout")
c = Banc(reglages=replace(R, saut_rigide_rad=1e-4)).prendre()
check("Aucun chemin sans saut de posture : « porté là où le levage l'a laissé » (au journal), le "
      "carton n'a pas bougé au retour", c.m.etat is Etat.TENU
      and "porté là où le levage l'a laissé" in c.ligne("retour : aucun chemin")
      and float(np.linalg.norm(c.m._centre_porte - c.depart[0])) < 1e-6,
      f"{c.m.raison} {c.journal[-3:]}")
c.pas(150)
check("… tenu au port, rien de signalé", c.m.etat is Etat.TENU and not c.m.a_reposer, c.m.raison)
d = Banc(poignets_29dof=False).prendre()
check("Bras à 5 articulations (HAL 23 DOF, sans tangage du poignet) : la pose d'une main ne se "
      "tient pas, carton porté là où le levage l'a laissé (au journal)", d.m.etat is Etat.TENU
      and "5 articulations commandées par bras" in d.ligne("retour : ")
      and d.m._rigide is None, f"{d.m.raison} {d.journal[-3:]}")

section("3. Le carton tourne au retour : geste arrêté, on resserre")
e = Banc()
contact, offset = e.r._contact, [0.0]


def poignet_qui_cede():
    """Le poignet gauche mesuré tourne pendant le retour (paume qui cède),
    0,01 rad par pas : l'OS le voit sur les angles mesurés."""
    contact()
    if e.m.etat is Etat.RAMENER:
        offset[0] += 0.01
    e.r._poignets["left_wrist_pitch_joint"] += offset[0] if e.m.etat is Etat.RAMENER else 0.0


e.r._contact = poignet_qui_cede
e.prendre()
check("Rotation estimée au-delà du seuil : « retour arrêté : carton tourné de … on resserre » (au "
      "journal), le geste s'arrête en chemin", e.ligne("retour arrêté : carton tourné de")
      and e.m.stats.get("retours_arretes") == 1 and e.m.etat is Etat.TENU,
      f"{e.m.raison} {e.journal[-3:]}")
penche = False
for _ in range(120):
    e.pas()
    penche = penche or e.m.penche
check("Au port : base arrêtée, resserrage au moins jusqu'au serrage visé (au journal), puis la "
      "marche reprend, plafonds tenus", penche and e.ligne("port : carton tourné au retour")
      and not e.m.penche and not e.m.a_reposer and e.m.etat is Etat.TENU and e.depasse <= 0.1,
      f"{e.journal[-4:]} {e.depasse}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
