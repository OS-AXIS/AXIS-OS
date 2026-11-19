"""
Planificateur de prise (axis_os/services/manipulation/planificateur.py,
ManipulationService.planifier_prise et evaluer_prise), robot simulé léger,
G1 29 DOF, mains du G1, table de la scène T1 (bord en x = 0) :

  1. Géométrie : la boîte d'après ses 8 coins, et retour ; les deux paires
     de faces, chacune face à la table ; les positions de base, de 2 en 2 cm.
  2. Carton T1 de face : retenu face à lui, toutes les marges positives,
     chemins vérifiés, les pieds derrière le bord (erreur d'approche
     comprise), et rien n'a bougé pendant la planification.
  3. Carton tourné de 20° : retenu face à lui (cap +20°).
  4. Chaque contrôle refuse au-delà de son seuil : pieds (bassin trop près
     du bord), largeur (carton trop large), portée (trop loin), couples
     (carton lourd, bras tendus).
  5. Refus : la raison de la meilleure candidate, celle qui va le plus loin
     (« même la meilleure prise : … »).
  6. Journal (refus par contrôle) et vue plongeante (PNG).
  7. Campagne : le carton T1 tourné de 30°, pris face à lui et déposé ;
     pieds jamais au-delà du bord.
  8. Rapport : les cartons impossibles par construction (trop lourd, trop
     large, hors de toute portée même pieds contre la table) à part des
     faisables, et la réussite sur les faisables.
"""
import contextlib
import io
import math
import sys
import tempfile
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.locomotion.equilibre import pieds_debout  # noqa: E402
from axis_os.services.manipulation import planificateur as P  # noqa: E402
from axis_os.testing import cartons as K  # noqa: E402
from axis_os.testing.robot_simule import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
BORD = P.BordTable((0.0, 0.0), 0.0, K.PLATEAU_M)


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Banc:
    """Un carton (T1 modifié) sur la table, robot démarré, à l'écart."""

    def __init__(self, **modifs):
        self.t = [0.0]
        c = replace(K.CARTON_T1, **modifs)
        self.c = c
        self.lacet = math.radians(c.lacet_deg)
        self.centre = K.place_sur_table(c)
        carton = SimulatedObject("carton", self.centre, size=(c.largeur, c.profondeur, c.hauteur),
                                 lacet=self.lacet, masse_kg=c.masse_kg)
        with contextlib.redirect_stdout(io.StringIO()):
            self.r = SimulatedRobot(objects=[carton], horloge=lambda: self.t[0],
                                    contact_main_m=0.11, poignets_29dof=True, plateaux=[K.TABLE])
            self.r.start(fil=False)
        self.m = self.r.manipulation
        self.journal = []
        self.m._journal = self.journal.append

    def coins(self):
        return P.coins_boite(self.centre, self.lacet, self.c.profondeur, self.c.largeur,
                             self.c.hauteur)

    def planifier(self):
        return self.m.planifier_prise(self.coins(), BORD)


def pied_au_dela(c):
    """Le pied le plus avancé, au-delà du bord (x = 0) ; négatif : devant."""
    return max(c.x + px * math.cos(c.cap) - py * math.sin(c.cap) + r
               for px, py, r in pieds_debout())


section("1. Géométrie")
coins = P.coins_boite((0.3, -0.1, 0.9), math.radians(20.0), 0.20, 0.24, 0.20)
b = P.boite_depuis_coins(coins)
check("8 coins, et retour : centre, côtés, hauteur ; lacet à un quart de tour près",
      np.allclose(b.centre, (0.3, -0.1, 0.9)) and abs(b.hauteur - 0.20) < 1e-9
      and sorted((round(b.a, 6), round(b.b, 6))) == [0.2, 0.24]
      and min(abs(math.remainder(b.lacet - math.radians(20.0), math.pi / 2)), 1.0) < 1e-6,
      f"{b}")
pp = P.paires(b, BORD)
check("Deux paires, chacune face à la table (cap à 90° au plus de sa normale), largeur et "
      "profondeur échangées",
      len(pp) == 2 and all(abs(math.remainder(p.cap - BORD.normale, 2 * math.pi)) <= math.pi / 2
                           for p in pp)
      and {round(pp[0].largeur, 6), round(pp[1].largeur, 6)} == {0.2, 0.24}, str(pp))
pos = P.positions(b, pp[0])
d = sorted({round(p_[3], 6) for p_ in pos})
check(f"Positions : {len(pos)} pour une paire, de {100 * P.PAS_M:.0f} en "
      f"{100 * P.PAS_M:.0f} cm en profondeur et en travers",
      len(pos) > 50 and all(abs(d2 - d1 - P.PAS_M) < 1e-9 for d1, d2 in zip(d, d[1:])))

section("2. Carton T1 de face")
banc = Banc()
avant = (banc.r.x, banc.r.y, banc.r.heading, dict(banc.m._q), banc.m.etat, banc.m._plan)
plan = banc.planifier()
ch = plan.choisie
check(f"Retenu : {ch and ch.explication()}", ch is not None and ch.ok and ch.complete
      and abs(ch.cap) < 1e-9 and all(m.relative >= 0 for m in ch.marges), plan.raison)
check("Contrôles faits : pieds, largeur, portée, garde, ligne, couples, équilibre, chemins",
      ch is not None and [m.nom for m in ch.marges] == ["pieds", "largeur", "portée", "garde",
                                                         "ligne", "couples", "équilibre",
                                                         "chemins"],
      str(ch and [m.nom for m in ch.marges]))
check(f"Pieds derrière le bord, erreur d'approche comprise : "
      f"{ch and round(100 * pied_au_dela(ch), 1)} cm",
      ch is not None and pied_au_dela(ch) <= -banc.m.r.erreur_approche_m)
check("Rien n'a bougé pendant la planification (base, bras, état de la prise)",
      (banc.r.x, banc.r.y, banc.r.heading, dict(banc.m._q), banc.m.etat, banc.m._plan) == avant)

section("3. Carton tourné de 20°")
plan = Banc(lacet_deg=20.0).planifier()
check(f"Retenu face à lui : {plan.choisie and plan.choisie.explication()}",
      plan.choisie is not None and abs(math.degrees(plan.choisie.cap) - 20.0) < 1.0, plan.raison)

section("4. Chaque contrôle refuse au-delà de son seuil")
banc = Banc()
objet = banc.m._objet_paire(P.boite_depuis_coins(banc.coins()), P.paires(
    P.boite_depuis_coins(banc.coins()), BORD)[0], BORD)
cx = banc.centre[0]
for quoi, pose, nom in (("bassin à 5 cm du bord", (-0.05, 0.0, 0.0), "pieds"),
                        ("carton à 70 cm du bassin", (cx - 0.70, 0.0, 0.0), "portée")):
    m = banc.m.evaluer_prise(objet, pose)
    check(f"{quoi} : refus « {nom} » ({m[-1].texte[:70]}…)", m[-1].nom == nom
          and m[-1].relative < 0)
large = replace(objet, largeur=0.46)
m = banc.m.evaluer_prise(large, (cx - 0.40, 0.0, 0.0))
check(f"Carton de 46 cm : refus « largeur » ({m[-1].texte[:70]}…)", m[-1].nom == "largeur"
      and m[-1].relative < 0)
m = banc.m.evaluer_prise(objet, (cx - 0.40, 0.0, 0.0), masse_kg=3.0)
check(f"3 kg estimés : refus « couples » ({m[-1].texte[:80]}…)", m[-1].nom == "couples"
      and m[-1].relative < 0 and "N·m pour" in m[-1].texte)

section("5. Refus : la raison de la meilleure candidate")
banc = Banc(largeur=0.33)
plan = banc.planifier()
m_ = plan.meilleure
check(f"Carton de 33 cm : refusé, « {plan.raison[:110]}… »", plan.choisie is None
      and plan.raison.startswith("même la meilleure prise : ") and m_ is not None
      and m_.refus is not None and m_.refus.texte in plan.raison
      and len(m_.marges) == max(len(c.marges) for c in plan.candidates), plan.raison)

section("6. Journal et vue plongeante")
check(f"Journal : « {banc.journal[-1][:120]}… »", banc.journal and "candidates" in banc.journal[-1]
      and "refusées" in banc.journal[-1] and "même la meilleure prise" in banc.journal[-1])
with tempfile.TemporaryDirectory() as d_:
    chemin = Path(d_) / "vue.png"
    P.dessiner(plan, chemin)
    from PIL import Image
    img = Image.open(chemin)
    check(f"Vue plongeante : PNG {img.size[0]} × {img.size[1]}", img.size == (900, 700))

section("7. Campagne : carton T1 tourné de 30°")
c = replace(K.CARTON_T1, numero=1300, lacet_deg=30.0)
x = K.essayer_carton(c)
check(f"Pris face à lui (cap {x['plan']['cap_deg']}°) et déposé ; pieds "
      f"{100 * x['pied_au_dela_m']:.1f} cm devant le bord",
      x["issue"] == "déposée" and abs(x["plan"]["cap_deg"] - 30.0) < 1.0
      and x["pied_au_dela_m"] < 0 and not K.violations(x), f"{x['issue']} {x['raison']}")

section("8. Rapport : impossibles par construction, à part des faisables")
for quoi, modifs, attendu in (("carton T1", {}, ""), ("1,5 kg", {"masse_kg": 1.5}, "trop lourd"),
                              ("46 cm de large, au bord", {"largeur": 0.46}, "trop large"),
                              ("à 40 cm du bord", {"retrait_m": 0.40}, "hors de toute portée")):
    b_ = Banc(**modifs)
    v = K.impossible_par_construction(b_.c, b_.m)
    check(f"{quoi} : {v or 'faisable'}", v == attendu, v)
lourd = {**x, "impossible": "trop lourd", "issue": "refusée", "raison": "trop lourd : 1.5 kg pesés"}
texte = K.rapport([x, lourd])
check("Le carton de la campagne (§7) est faisable ; le rapport met l'impossible à part et donne "
      "la réussite sur les faisables",
      x["impossible"] == "" and "impossibles par construction : 1 (trop lourd 1), 0 déposés" in texte
      and "faisables : 1 cartons, 1 tenus et déposés (100 %)" in texte, texte[:400])

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
