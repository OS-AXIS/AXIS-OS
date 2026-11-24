"""
Se pencher pour atteindre plus loin (ManipulationService.plan_penche,
changer_tangage), par la consigne de posture de la marche (bornée,
équilibre vérifié, gardée par le gardien de proximité).

Robot simulé léger, G1 29 DOF (sa taille tangue), mains du G1, carton de
la scène T1 posé de plus en plus loin du bord de la table :
  1. Plan : à portée debout, pas de penche ; un peu plus loin, le plus petit
     tangage qui suffit, la marge d'équilibre écrite ; trop loin, refus pour
     l'équilibre (carton de masse_max_kg aux mains), la marge écrite.
  2. Penché, pris (0,5 kg : bras ainsi tendus, 1 kg dépasse les plafonds
     des poignets), puis redressé le carton tenu : le carton ne bouge pas
     pendant le redressement (ni en place ni en orientation), le robot finit
     droit, rien ne tombe.
  3. Refus de changer_tangage : rien en main ; bras qui ne tiendraient plus
     le carton en chemin (vérifié avant de bouger : rien ne bouge).
"""
import contextlib
import io
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.service import Etat, Objet  # noqa: E402
from axis_os.testing import cartons as K  # noqa: E402
from axis_os.testing.robot_simule import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Banc:
    """Le carton T1 à `retrait` du bord de la table, robot démarré."""

    def __init__(self, retrait, masse=1.0, hauteur=None, table=K.PLATEAU_M):
        self.t = [0.0]
        c = replace(K.CARTON_T1, retrait_m=retrait, masse_kg=masse,
                    hauteur=K.CARTON_T1.hauteur if hauteur is None else hauteur)
        self.c = c
        cx, cy, cz = K.place_sur_table(c)
        cz += table - K.PLATEAU_M                  # la table à `table` m
        self.carton = SimulatedObject("carton", (cx, cy, cz), size=(c.largeur, c.profondeur,
                                                                     c.hauteur), masse_kg=c.masse_kg)
        with contextlib.redirect_stdout(io.StringIO()):
            self.r = SimulatedRobot(objects=[self.carton], horloge=lambda: self.t[0],
                                    contact_main_m=0.11, poignets_29dof=True,
                                    plateaux=[K.TABLE[:4] + (table,)])
            self.r.start(fil=False)
        self.m = self.r.manipulation
        self.objet = Objet((cx, cy, cz), c.largeur, c.profondeur, c.hauteur,
                           bord_m=cx - c.profondeur / 2.0)

    def pas(self, n=1):
        with contextlib.redirect_stdout(io.StringIO()):
            for _ in range(n):
                self.t[0] += self.r._dt
                self.r.pas()

    def jusqua(self, fini, duree):
        t0 = self.t[0]
        while not fini() and self.t[0] - t0 < duree:
            self.pas()

    def prendre_penche(self):
        """Plan, penche, pré-saisie, insertion, prise : (d, tangage, raison)."""
        d, tangage, raison = self.m.plan_penche(self.objet)
        if d is None:
            return d, tangage, raison
        x_prise = self.objet.centre[0] - d
        self.r.x, self.r.y, self.r.heading = x_prise - K.RECUL_INSERTION_M, self.objet.centre[1], 0.0
        self.pas(20)
        if tangage:
            self.r.locomotion.posture(tangage)
            self.jusqua(lambda: self.r.locomotion.posture_atteinte, 10.0)
        self.m.presaisir(self.objet)
        self.jusqua(lambda: self.m.etat is Etat.PRET, 5.0)
        while self.r.x < x_prise - 1e-9:
            self.r.x = min(x_prise, self.r.x + K.VITESSE_INSERTION_M_S * self.r._dt)
            self.pas()
        self.pas(10)
        self.m.prendre(self.objet)
        self.jusqua(lambda: self.m.etat in (Etat.TENU, Etat.ECHEC), 40.0)
        return d, tangage, raison


section("1. Plan : debout, penché juste assez, ou refus pour l'équilibre")
d, t, raison = Banc(0.15).m.plan_penche(Banc(0.15).objet)
check(f"Carton à 15 cm du bord : à portée debout (d = {d}), pas de penche", d is not None and t == 0.0,
      raison)
# Prise en pendule (mains ~5 cm au-dessus du centre) : sur la table de 80 cm,
# le carton qu'il faut se pencher pour atteindre (31 cm du bord) est pris,
# mais les bras, au plus haut de leur portée, ne le ramènent plus ni ne le
# tiennent quand le torse se redresse. Sur une table de 70 cm (mains plus bas
# par rapport aux épaules) : debout jusqu'à 27 cm, penché à 28 et 29 cm,
# refusé pour l'équilibre au-delà.
b = Banc(0.29, table=0.70)
d, t, raison = b.m.plan_penche(b.objet)
check(f"Table de 70 cm, carton à 29 cm : penché de {math.degrees(t):.1f}°, la marge d'équilibre écrite (« {raison} »)",
      d is not None and 0.0 < t <= 0.10 and "cm dans l'appui" in raison, raison)
d, t, raison = Banc(0.32).m.plan_penche(Banc(0.32).objet)
check(f"À 32 cm : refusé pour l'équilibre, la marge écrite (« {raison[-110:]} »)",
      d is None and "équilibre" in raison and "appui des pieds" in raison, raison)

section("2. Penché, pris, redressé le carton tenu (0,5 kg)")
# 1 kg, bras ainsi tendus : « trop lourd pour ces mains » (les plafonds des
# poignets). Le plan, lui, juge l'équilibre avec masse_max_kg aux mains.
b = Banc(0.29, masse=0.5, table=0.70)
d, t, raison = b.prendre_penche()
check(f"Penché de {math.degrees(t):.1f}°, carton pris et tenu (TENU)",
      t > 0.0 and b.m.etat is Etat.TENU and b.r.held is b.carton, f"{b.m.etat} {b.m.raison}")
avant = np.array(b.carton.position)
r = b.m.changer_tangage(0.0)
ecart, tourne = 0.0, 0.0
while b.m.tangage_en_cours and b.t[0] < 120.0:
    b.pas()
    ecart = max(ecart, float(np.linalg.norm(np.array(b.carton.position) - avant)))
    tourne = max(tourne, abs(b.r.rotation_objet_deg))
check(f"Redressement accepté, marge écrite (« {r.reason} »)", r.ok and "cm dans l'appui" in r.reason,
      r.reason)
check(f"Le carton ne bouge pas pendant le redressement : {1000 * ecart:.1f} mm au plus, tourné de "
      f"{tourne:.1f}° au plus", ecart <= 0.005 and tourne <= 1.0, f"{ecart} {tourne}")
check("Le robot finit droit, le carton toujours tenu, rien n'est tombé",
      abs(b.r.tangage_taille) < 1e-3 and not b.r.locomotion.penche and b.r.held is b.carton
      and not b.r.chutes, f"{b.r.tangage_taille} {b.r.chutes}")

section("3. Refus de changer_tangage")
v = Banc(0.15)
r = v.m.changer_tangage(0.1)
check("Rien en main : refusé, rien ne bouge", not r.ok and "objet tenu" in r.reason
      and not v.r.locomotion.penche, r.reason)
b = Banc(0.15)
b.prendre_penche()
q_rigide = b.m._q_rigide


def coupe(centre, ecart=None, graine=None, delta=None):
    """Les bras ne tiennent plus le carton en place, torse penché de plus de 0,05 rad."""
    if b.m._fixes.get("waist_pitch_joint", 0.0) > 0.05:
        return None
    return q_rigide(centre, ecart, graine, delta)


b.m._q_rigide = coupe
q0 = dict(b.m._q)
r = b.m.changer_tangage(0.15)
b.pas(20)
check(f"Bras qui ne tiendraient plus le carton en chemin : refusé, vérifié avant de bouger "
      f"(« {r.reason} »)", not r.ok and "avant de bouger" in r.reason and not b.r.locomotion.penche
      and b.m._q == q0 and b.m.etat is Etat.TENU, r.reason)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
