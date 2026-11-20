"""
Au port, la marge de frottement jugée sur une foulée (1 s), et un vrai
desserrage toujours vu en 0,3 s au plus.

Pourquoi : le serrage réel varie au pas de la marche (essai GPU T1 du
19 novembre 2026, 22 h 04 : juge de 10,5 à 14,3 N par main) ; la marge de
frottement, jugée sur 0,3 s, le suivait et arrêtait la base au ras du
coefficient de sécurité (2), le carton tenu. Jugée sur une foulée
(ReglagesPrise.fenetre_marge_s), à condition que la surveillance directe du
glissement reste à 0,3 s au plus :
  - mains écartées de leur serrage (le carton qui bouge entre elles) : à
    chaque pas, inchangé ;
  - carton qui tourne (paumes mesurées, caméra de tête) : à chaque pas,
    inchangé ;
  - poids qui chute : jugé sur les 5 dernières lectures (0,1 s), pendant
    duree_perte_s (0,15 s) — avant, une moyenne de 0,3 s puis 0,3 s, ~0,5 s ;
  - et la marge sur les 15 dernières lectures (0,3 s), qui ne doit jamais
    passer sous securite_glissement (1 : le serrage ne porte plus le carton au
    frottement prudent ; la marche ne la fait varier que de quelques
    centièmes).
Coefficient de sécurité 2, frottement 0,8, plafonds de couple : inchangés.

Robot simulé léger, horloge simulée, G1 29 DOF, mains du G1, carton de 1 kg
porté ; les pannes sont celles du monde simulé, jamais lues par l'OS
(SimulatedRobot.decalage_articulations : un moteur qui glisse ;
raideur_articulations : un moteur qui faiblit) :
  1. de vrais desserrages, vus en 0,3 s au plus, et par quoi ;
  2. chaque contrôle direct, seul, en 0,3 s au plus ;
  3. la foulée : une variation du serrage au pas de la marche n'arrête plus
     la base (la marge sur 0,3 s l'aurait arrêtée) ; une main desserrée sans
     que le carton bouge est jugée sur la foulée, en ~1 s (le compromis
     accepté).
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
from axis_os.services.manipulation.service import Etat, Objet  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
R = ReglagesPrise()
DELAI = 0.3                                   # « vu en 0,3 s au plus »


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


# Ce que dit le journal de l'OS, contrôle par contrôle.
CONTROLES = {"mains écartées": "mains écartées", "rotation": "carton tourné de",
             "poids qui chute": "ne pèse plus sur les mains",
             "marge sur 0,3 s": "marge de frottement sur 0,3 s",
             "marge sur la foulée": "sur la dernière foulée"}


class Banc:
    """Le carton de 1 kg pris, porté 3 s (la foulée pleine)."""

    def __init__(self, masse=1.0):
        self.t = [0.0]
        self.r = SimulatedRobot(objects=[SimulatedObject("carton", (0.41, 0.0, 0.90))],
                                horloge=lambda: self.t[0], contact_main_m=0.11,
                                poignets_29dof=True)
        self.r.MASSE_OBJET_KG = masse
        self.r.start(fil=False)
        self.m = self.r.manipulation
        self.journal = []
        self.m._journal = self.journal.append
        self.sortie = io.StringIO()
        self.pas(20)
        self.m.prendre(Objet((0.41, 0.0, 0.90), 0.24, 0.20, 0.20))
        t0 = self.t[0]
        while self.m.etat not in (Etat.TENU, Etat.ECHEC) and self.t[0] - t0 < 40.0:
            self.pas()
        self.pas(150)

    def pas(self, n=1, chaque=None):
        with contextlib.redirect_stdout(self.sortie):
            for _ in range(n):
                self.t[0] += self.r._dt
                if chaque is not None:
                    chaque(self)
                self.r.pas()

    def essai(self, panne, duree=1.0, chaque=None):
        """La panne au temps 0 : (vu, lâché, {contrôle : premier instant au
        journal}) en secondes depuis la panne ; vu : base arrêtée (penche) ou
        transport interrompu."""
        m, r = self.m, self.r
        self.avant = (m.etat, m.penche, m.a_reposer, m.marge_frottement)
        n0 = len(self.journal)
        panne(self)
        t0, vu, lache, premiers = self.t[0], None, None, {}
        while self.t[0] - t0 < duree - 1e-9:
            self.pas(chaque=chaque)
            dt = round(self.t[0] - t0, 3)
            if vu is None and (m.penche or m.a_reposer):
                vu = dt
            if lache is None and r.held is None:
                lache = dt
            for ligne in self.journal[n0:]:
                if ligne.startswith(("port : ", "transport interrompu : ")):
                    for nom, cle in CONTROLES.items():
                        if cle in ligne.split(" ; fin du port")[0]:
                            premiers.setdefault(nom, dt)
            n0 = len(self.journal)
        return vu, lache, premiers


def lacets(droit=0.0, gauche=0.0):
    """Lacet d'épaule qui glisse de `droit` (main droite) et `gauche` rad,
    vers l'extérieur : la main s'ouvre de ~29 mm par 0,1 rad dans la posture
    de port (le lacet d'épaule y ferme la main)."""
    def panne(b):
        if droit:
            b.r.decalage_articulations["right_shoulder_yaw_joint"] = -droit
        if gauche:
            b.r.decalage_articulations["left_shoulder_yaw_joint"] = gauche
    return panne


def bras_droit_faiblit(part):
    def panne(b):
        for j in list(b.r._BRAS) + list(b.r._poignets):
            if j.startswith("right"):
                b.r.raideur_articulations[j] = part
    return panne


def carton_vide(b):
    b.r.MASSE_OBJET_KG = 0.1                   # le fond lâche, le contenu tombe


def carton_ecrase(b):
    o = b.r.held
    o.size = (o.size[0] - 0.01, o.size[1], o.size[2])


def resume(premiers):
    return ", ".join(f"{k} {v:.2f} s" for k, v in sorted(premiers.items(), key=lambda kv: kv[1]))


section("1. De vrais desserrages, vus en 0,3 s au plus")
CAS = [
    ("la main droite lâche le carton (son lacet d'épaule glisse de 0,10 rad : 2,9 cm)",
     lacets(droit=0.10)),
    ("le bras droit faiblit (moteurs à 30 % de leur raideur)", bras_droit_faiblit(0.3)),
    ("les deux mains s'ouvrent de 2,3 cm ensemble (lacets 0,08 rad) : rien ne tourne, rien ne "
     "s'écarte, le carton est moins serré", lacets(droit=0.08, gauche=0.08)),
    ("les deux mains lâchent ensemble (lacets 0,09 rad)", lacets(droit=0.09, gauche=0.09)),
    ("le carton se vide : il ne pèse plus que 0,1 kg", carton_vide),
    ("le carton s'écrase de 1 cm entre les mains", carton_ecrase),
]
RES = {}
for nom, panne in CAS:
    b = Banc()
    vu, lache, premiers = b.essai(panne)
    RES[nom] = (vu, lache, premiers)
    porte = b.avant[0] is Etat.TENU and not b.avant[1] and not b.avant[2]
    check(f"{nom} : vu en {vu if vu is None else f'{vu:.2f}'} s" +
          (f" (carton lâché à {lache:.2f} s)" if lache is not None else " (carton tenu)") +
          f" — {resume(premiers)}",
          porte and vu is not None and vu <= DELAI and premiers
          and min(premiers.values()) <= DELAI, f"{b.avant} {premiers}")

section("2. Chaque contrôle direct, seul, en 0,3 s au plus")
vu, lache, premiers = RES["le carton se vide : il ne pèse plus que 0,1 kg"]
check(f"Poids qui chute, seul (le carton se vide, rien ne bouge) : vu en "
      f"{premiers.get('poids qui chute', math.nan):.2f} s (avant : moyenne de 0,3 s puis 0,3 s, "
      f"~0,5 s)", set(premiers) <= {"poids qui chute", "rotation"}
      and premiers.get("poids qui chute", 9) <= DELAI)
vu, lache, premiers = RES[CAS[0][0]]
check(f"Poids qui chute, carton lâché : vu {premiers.get('poids qui chute', math.nan) - lache:.2f} s "
      f"après qu'il a quitté les mains", premiers.get("poids qui chute", 9) - lache <= DELAI)
vu, lache, premiers = RES["le carton s'écrase de 1 cm entre les mains"]
check(f"Mains écartées, seules (carton écrasé, tenu) : vu en "
      f"{premiers.get('mains écartées', math.nan):.2f} s, au pas suivant",
      list(premiers) == ["mains écartées"] and premiers["mains écartées"] <= 2 * 0.02 + 1e-9)
vu, lache, premiers = RES[CAS[1][0]]
check(f"Rotation (paumes mesurées) : vue en {premiers.get('rotation', math.nan):.2f} s",
      premiers.get("rotation", 9) <= 2 * 0.02 + 1e-9)
vu, lache, premiers = RES[CAS[2][0]]
check(f"Marge sur 0,3 s, seule (les deux mains desserrées ensemble, rien ne tourne ni ne "
      f"s'écarte) : sous {R.securite_glissement:g} en {premiers.get('marge sur 0,3 s', math.nan):.2f} s",
      list(premiers) == ["marge sur 0,3 s"] and premiers["marge sur 0,3 s"] <= DELAI)

section("3. La marge sur une foulée")
# Le serrage qui varie au pas de la marche, les mains immobiles (essai de
# 22 h 04 : forces du juge de 10,5 à 14,3 N par main, faces mesurées décalées
# ensemble, leur écart constant à 2 mm près ; l'estimation de l'OS les
# suivait) : injecté dans l'estimation de la force à chaque main, sa seule
# composante normale (le poids n'y change pas), ±30 % au pas (0,8 s) ; carton
# de 1,2 kg (marge ~2,3, au ras du coefficient).
b = Banc(masse=1.2)
AMPL, T0 = 0.3, b.t[0]
courtes, longues = [], []
force_main = b.m._force_main


def force_au_pas(cote, tau, angles, g, depuis_fermeture=True):
    f = force_main(cote, tau, angles, g, depuis_fermeture)
    if f is None:
        return f
    n = b.m._vers_objet(cote)
    k = 1.0 + AMPL * math.sin(2 * math.pi * (b.t[0] - T0) / 0.8)
    return f + (k - 1.0) * float(f @ n) * n


def relever(b_):
    if b_.m.marge_courte is not None:
        courtes.append(b_.m.marge_courte)
        longues.append(b_.m.marge_frottement)


def marche(b_):
    b_.m._force_main = force_au_pas


vu, lache, premiers = b.essai(marche, duree=4.0, chaque=relever)
check(f"Serrage estimé qui varie au pas (±{100 * AMPL:.0f} %, un pas en 0,8 s, mains immobiles) : "
      f"la marge sur 0,3 s descend à {min(courtes):.2f}, sous {R.securite_serrage:g} (elle aurait "
      f"arrêté la base) ; sur la foulée, elle reste de {min(longues[60:]):.2f} à "
      f"{max(longues[60:]):.2f} : la base n'est pas arrêtée, carton tenu",
      min(courtes) < R.securite_serrage and min(longues[60:]) >= R.securite_serrage
      and vu is None and lache is None and min(courtes) > R.securite_glissement,
      f"{vu} {premiers}")
# Une main desserrée sans que le carton bouge : jugée sur la foulée.
b = Banc()
vu, lache, premiers = b.essai(lacets(droit=0.06), duree=1.5)
check(f"La main droite s'ouvre de 1,7 cm, le carton ne bouge pas (rien ne tourne, rien ne "
      f"s'écarte, il pèse toujours) : marge {b.m.marge_frottement:.2f} sur la foulée, vue en "
      f"{vu if vu is None else f'{vu:.2f}'} s (sur 0,3 s : ~0,3 s) — le compromis accepté",
      vu is not None and vu <= R.fenetre_marge_s + 0.1 and lache is None
      and list(premiers) == ["marge sur la foulée"], f"{premiers}")
b = Banc()                                   # porté 3 s, sans panne
n = len(b.m._hist_normales)
check(f"La foulée : {n} lectures sur {b.m._t_normales[-1] - b.m._t_normales[0]:.2f} s ; la marge "
      f"sur toutes, le plancher sur les {3 * R.filtre_force} dernières",
      abs(b.m._t_normales[-1] - b.m._t_normales[0] - R.fenetre_marge_s) <= b.r._dt + 1e-9
      and abs(b.m.marge_frottement - R.frottement_serrage * float(np.mean(b.m._hist_normales))
              / b.m.poids_carton) < 1e-9
      and abs(b.m.marge_courte - R.frottement_serrage
              * float(np.mean(list(b.m._hist_normales)[-3 * R.filtre_force:]))
              / b.m.poids_carton) < 1e-9)
check(f"Rien d'affaibli : coefficient {R.securite_serrage:g}, frottement {R.frottement_serrage:g}, "
      f"reprise {R.securite_reprise:g}",
      R.securite_serrage == 2.0 and R.frottement_serrage == 0.8 and R.securite_reprise == 2.2)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
