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
     accepté) ;
  4. le carton qui tourne entre les paumes (essai GPU du 20 novembre 2026,
     21 h 37) : au 2e écartement des mains en 10 s, ou dès que l'écart
     grandit malgré le resserrage, transport interrompu, à reposer ;
  5. l'hystérésis des écartements : le bruit de mesure de l'écart n'en fait
     pas compter un 2e, un vrai 2e écartement compte toujours.
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


def carton_allege(b):
    # Il ne pèse plus que 0,35 kg (sous part_poids_min du poids de référence) :
    # prise en pendule, le carton vidé à 0,1 kg (ou à 0,3) relâche assez les
    # bras pour que les mains s'écartent aussi ; allégé à 0,35, seul le poids
    # le dit.
    b.r.MASSE_OBJET_KG = 0.35


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
    ("le carton s'allège : il ne pèse plus que 0,35 kg", carton_allege),
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
vu, lache, premiers = RES["le carton s'allège : il ne pèse plus que 0,35 kg"]
check(f"Poids qui chute, seul (le carton s'allège, rien ne bouge) : vu en "
      f"{premiers.get('poids qui chute', math.nan):.2f} s (avant : moyenne de 0,3 s puis 0,3 s, "
      f"~0,5 s)", set(premiers) <= {"poids qui chute", "rotation"}
      and premiers.get("poids qui chute", 9) <= DELAI)
# Prise en pendule : la main droite qui glisse de 2,9 cm, ou les deux mains
# de 0,09 rad, ne lâchent plus le carton (le transport est interrompu avant) ;
# le bras droit qui faiblit, si.
vu, lache, premiers = RES[CAS[1][0]]
check(f"Poids qui chute, carton lâché : vu {premiers.get('poids qui chute', math.nan) - lache:.2f} s "
      f"après qu'il a quitté les mains", lache is not None
      and premiers.get("poids qui chute", 9) - lache <= DELAI, f"{lache} {premiers}")
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
# Des mains desserrées sans que le carton bouge : jugées sur la foulée. Prise
# en pendule (mains 6 cm au-dessus du centre) : une seule main qui s'ouvre de
# 1,7 cm (lacet 0,06 rad) laisse la marge à 2,09, au-dessus du coefficient ; à
# 0,065 rad, le carton tourne (vu en 0,06 s). Les deux mains de 1,2 cm chacune
# (0,04 rad) : rien ne tourne, rien ne s'écarte.
b = Banc()
vu, lache, premiers = b.essai(lacets(droit=0.04, gauche=0.04), duree=1.5)
check(f"Les deux mains s'ouvrent de 1,2 cm, le carton ne bouge pas (rien ne tourne, rien ne "
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

section("4. Carton qui tourne entre les paumes : mains écartées à répétition")
# Essai GPU du 20 novembre 2026, 21 h 37 : le carton a tourné de 50° autour de
# la ligne des mains ; l'OS n'en a vu que « mains écartées de 5 à 13 mm »,
# 8 fois, chaque fois resserrées. Ici, l'écart vu par l'OS (sa géométrie des
# mains, _serrage_port) est imposé ; tout le reste de l'OS tourne tel quel.


def ecart_impose(b):
    e = [0.0]
    b.m._serrage_port = lambda angles: b.m._serrage_port_ref - e[0]
    return e


def ecartement(b, e, mm, duree=0.1):
    e[0] = mm / 1000.0
    b.pas(round(duree / b.r._dt))
    e[0] = 0.0


def interrompu(b):
    return next((ligne for ligne in b.journal if ligne.startswith("transport interrompu")), "")


b = Banc()
e = ecart_impose(b)
ecartement(b, e, 7)
b.pas(round(2.0 / b.r._dt))
check("Un écartement de 7 mm, resserré : base arrêtée le temps de resserrer, le port continue",
      b.m.etat is Etat.TENU and not b.m.a_reposer and b.r.held is not None
      and any("mains écartées de 7 mm" in ligne for ligne in b.journal) and not interrompu(b),
      f"{interrompu(b)} {b.journal[-3:]}")
ecartement(b, e, 7)
b.pas(5)
check(f"Le 2e en moins de {R.fenetre_ecartements_s:g} s : transport interrompu, à reposer, "
      f"« carton qui tourne entre les paumes »",
      b.m.a_reposer and b.r.held is not None
      and f"carton qui tourne entre les paumes : mains écartées 2 fois en "
          f"{R.fenetre_ecartements_s:g} s" in interrompu(b), interrompu(b))
b = Banc()
e = ecart_impose(b)
ecartement(b, e, 7)
b.pas(round((R.fenetre_ecartements_s + 1.0) / b.r._dt))
ecartement(b, e, 7)
b.pas(round(2.0 / b.r._dt))
check(f"Deux écartements à plus de {R.fenetre_ecartements_s:g} s l'un de l'autre : chacun "
      f"resserré, le port continue", not b.m.a_reposer and not interrompu(b), interrompu(b))
b = Banc()
e = ecart_impose(b)
e[0] = 0.006
t0 = b.t[0]
while not b.m.a_reposer and b.t[0] - t0 < 2.0:
    b.pas()
    e[0] += 0.0005                           # 2,5 cm/s : le carton tourne encore
check(f"L'écart qui grandit malgré le resserrage (de plus de {1000 * R.croissance_ecart_m:g} mm "
      f"après {R.delai_croissance_s:g} s) : transport interrompu en "
      f"{b.t[0] - t0:.2f} s, avant l'abandon du resserrage ({R.duree_resserrage_s:g} s)",
      b.m.a_reposer and b.t[0] - t0 < R.duree_resserrage_s
      and "malgré le resserrage" in interrompu(b)
      and "carton qui tourne entre les paumes" in interrompu(b), interrompu(b))

section("5. Hystérésis des écartements : le bruit de mesure n'en fait pas compter un 2e")
# Le bruit de l'écart lu en marchant : 0,2 à 0,45 mm par lecture (essais GPU
# 17 h 16, 18 h 21, 21 h 37 ; ReglagesPrise.hysteresis_ecart_m). Sans
# hystérésis, l'écart qui repassait le seuil comptait un écartement de plus :
# au 2e en 10 s, transport interrompu, le carton tenu.
BAS = 1000 * (R.ecart_face_port_m - R.hysteresis_ecart_m)
OSCILLE = (5.2, 4.8, 5.3, 4.6, 5.1, 4.9, 5.2)


def oscille(b, e):
    """L'écart lu qui oscille autour du seuil (toutes les 0,1 s), puis revenu
    sous la bande ; rend le nombre d'écartements comptés pendant l'oscillation."""
    for mm in OSCILLE:
        e[0] = mm / 1000.0
        b.pas(round(0.1 / b.r._dt))
    compte = len(b.m._ecartements)
    e[0] = (BAS - 0.5) / 1000.0
    b.pas(round(1.0 / b.r._dt))
    return compte


b = Banc()
e = ecart_impose(b)
compte = oscille(b, e)
check(f"Écart qui oscille autour du seuil ({', '.join(f'{v:g}' for v in OSCILLE)} mm), jamais "
      f"sous {BAS:g} mm : un seul écartement compté, fini l'écart revenu sous {BAS:g} mm ; le "
      f"port continue", compte == 1 and not interrompu(b) and not b.m.a_reposer
      and b.m._ecart_debut is None and b.r.held is not None, f"{compte} {interrompu(b)}")
b = Banc()
b.m.r = replace(R, hysteresis_ecart_m=0.0)
e = ecart_impose(b)
oscille(b, e)
check("… la même oscillation sans hystérésis (avant) : un écartement à chaque passage du seuil, "
      "transport interrompu « mains écartées 2 fois en 10 s », le carton tenu",
      b.m.a_reposer and "mains écartées 2 fois en" in interrompu(b), interrompu(b))
ecartement(b := Banc(), e := ecart_impose(b), 7, duree=0.1)
e[0] = (BAS - 0.5) / 1000.0
b.pas(round(1.0 / b.r._dt))
ecartement(b, e, 7)
b.pas(5)
check(f"Un vrai 2e écartement, l'écart revenu sous {BAS:g} mm entre les deux, en moins de "
      f"{R.fenetre_ecartements_s:g} s : compté, transport interrompu",
      b.m.a_reposer and "mains écartées 2 fois en" in interrompu(b), interrompu(b))


def ecart_resserre(b, mm, sigma_mm=0.0, graine=0):
    """Un vrai écartement de `mm`, que le resserrage résorbe : l'écart vrai
    baisse de ce que la consigne des deux mains avance (carton rigide) ; l'OS
    le lit avec un bruit gaussien de `sigma_mm` par lecture. Rend la lecture
    de l'écart par l'OS."""
    rng = np.random.default_rng(graine)
    d0 = sum(b.m._delta.values())
    b.m._serrage_port = lambda angles: b.m._serrage_port_ref - (
        mm / 1000.0 - (sum(b.m._delta.values()) - d0) + rng.normal(0.0, sigma_mm / 1000.0))
    return lambda: 1000 * (b.m._serrage_port_ref - b.m._serrage_port(None))


# Carton de 0,5 kg : les bras ont de la marge sous leurs plafonds pour resserrer
# (1 N·m ; à 1 kg, prise en pendule, les lacets des poignets en sont à 0,1).
repart = {}
for h in (R.hysteresis_ecart_m, 0.0):
    b = Banc(masse=0.5)
    b.m.r = replace(R, hysteresis_ecart_m=h)
    lire = ecart_resserre(b, 7.5)
    arretee = False
    while b.t[0] < 60.0:
        b.pas()
        arretee = arretee or b.m.penche
        if arretee and not b.m.penche:
            break
    repart[h] = lire()
check(f"Un vrai écartement de 7,5 mm, resserré : la base repart l'écart à "
      f"{repart[R.hysteresis_ecart_m]:.1f} mm, sous {BAS:g} (le resserrage l'y a amené) ; sans "
      f"hystérésis, elle repartait à {repart[0.0]:.1f} mm, au ras du seuil, là où le bruit le "
      f"refait passer", repart[R.hysteresis_ecart_m] < BAS
      and BAS < repart[0.0] <= 1000 * R.ecart_face_port_m, str(repart))
SIGMA = 0.45
for h, attendu in ((R.hysteresis_ecart_m, 0), (0.0, 3)):
    raisons = []
    for graine in range(3):
        b = Banc(masse=0.5)
        b.m.r = replace(R, hysteresis_ecart_m=h)
        ecart_resserre(b, 7.0, SIGMA, graine)
        b.pas(round(10.0 / b.r._dt))
        raisons.append(interrompu(b))
    check(f"Un vrai écartement de 7 mm, puis 10 s d'écart lu à ±{SIGMA:g} mm (écart-type, le "
          f"plus bruité des essais GPU), 3 tirages : " + (
              f"hystérésis de {1000 * h:g} mm, jamais interrompu" if not attendu else
              "sans hystérésis (avant), interrompu à tort à chaque tirage (2e écartement compté)"),
          sum(bool(x) for x in raisons) == attendu
          and all("mains écartées 2 fois en" in x for x in raisons if x), str(raisons))

b = Banc()
e = ecart_impose(b)
ecartement(b, e, 7, duree=0.1)
e[0] = 0.004
b.pas(round(0.7 / b.r._dt))
repartie = not b.m.penche and any("hors de portée des bras, à leurs plafonds" in ligne
                                  for ligne in b.journal)
e[0] = 0.0053
b.pas(round(0.1 / b.r._dt))
e[0] = 0.004
b.pas(round(1.0 / b.r._dt))
check(f"Carton de 1 kg, bras à leurs plafonds (rien de plus à serrer) : la base repart l'écart "
      f"revenu sous le seuil (4 mm), comme avant l'hystérésis ; l'écartement reste en cours : "
      f"un nouveau passage du seuil (5,3 mm) arrête la base, sans compter un 2e écartement",
      repartie and len(b.m._ecartements) == 1 and not interrompu(b) and not b.m.a_reposer,
      f"{repartie} {len(b.m._ecartements)} {interrompu(b)} {b.journal[-3:]}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
