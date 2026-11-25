"""
Le poids qui chute, au port, jugé depuis le niveau lu au début du port,
dans la même posture (décision sur la PR #42).

Avant : « moins de 40 % du plus grand poids lu depuis le levage ». Ce plus
grand se prenait dans d'autres postures (levage, retour) ; le décalage des
couples, qui change avec la posture, ne s'y annulait pas : prise en pendule,
un carton de 0,22 kg tenu était lu sous ce seuil (campagne, n° 105 : reposé
à tort, 5 tirages du bruit des couples sur 16). Maintenant : le poids lu sur
0,1 s plus bas que le niveau du début du port de plus de 60 % du poids pesé,
pendant 0,15 s ; vu en 0,3 s au plus.

  1. le niveau : la moyenne des lectures de la première foulée du port (1 s),
     sans les 0,3 s les plus récentes tant qu'elle se remplit, puis figée ;
     repris à chaque nouvelle posture ;
  2. un vrai lâcher total (le carton quitte les mains) : vu par le poids
     0,3 s au plus après, prise normale (mains au centre de gravité) et en
     pendule ;
  3. un lâcher partiel (le carton glisse sans tomber) : le poids lu ne baisse
     que de 4 à 22 % pendant le glissement, ni l'ancienne règle ni la
     nouvelle ne le voient ; la marge de frottement sur 0,3 s le voit, et en
     combien de temps ;
  4. les fausses alertes sur le bruit réel des essais GPU (17 h 16, 22 h 04,
     18 h 21) : le poids tel que l'OS l'a relevé (moyenne de 0,3 s, plus
     lisse que la règle : borne optimiste) et recalculé à chaque lecture
     depuis les couples relevés (une lecture : borne pessimiste), pour des
     cartons de 2,2 à 10 N ; la règle de l'OS rejouée, l'ancienne aussi.

Robot simulé léger, horloge simulée, G1 29 DOF, mains du G1, carton de 1 kg
(frottement 1,2) ; les pannes sont celles du monde simulé, jamais lues par
l'OS (SimulatedRobot.decalage_articulations : un lacet d'épaule qui glisse,
la main s'ouvre ; raideur_articulations : un bras qui faiblit).
"""
import contextlib
import csv
import io
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.cinematique import ModeleBras  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import Etat, ManipulationService, Objet  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
R = ReglagesPrise()
DELAI = 0.3                                   # « vu en 0,3 s au plus »
DONNEES = Path(__file__).resolve().parent / "donnees"
# Prise normale : les mains visées au centre de gravité (comme avant la prise
# en pendule), la ligne admise jusqu'à 3 cm dessous (elles se posent plus bas).
NORMALE = replace(R, marge_pendule_m=-0.03, affaissement_prise_m=0.03)


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


CONTROLES = {"poids qui chute": "ne pèse plus sur les mains", "mains écartées": "mains écartées",
             "rotation": "carton tourné de", "marge sur 0,3 s": "marge de frottement sur 0,3 s",
             "marge sur la foulée": "sur la dernière foulée"}


class Banc:
    """Le carton de 1 kg pris (prise en pendule, ou normale), porté `port_s`."""

    def __init__(self, normale=False, port_s=3.0):
        self.t = [0.0]
        self.r = SimulatedRobot(objects=[SimulatedObject("carton", (0.41, 0.0, 0.90))],
                                horloge=lambda: self.t[0], contact_main_m=0.11,
                                poignets_29dof=True)
        self.r.start(fil=False)
        self.m = self.r.manipulation
        if normale:
            self.m.r = NORMALE
        self.journal = []
        self.m._journal = self.journal.append
        self.sortie = io.StringIO()
        self.pas(20)
        self.m.prendre(Objet((0.41, 0.0, 0.90), 0.24, 0.20, 0.20))
        t0 = self.t[0]
        while self.m.etat not in (Etat.TENU, Etat.ECHEC) and self.t[0] - t0 < 40.0:
            self.pas()
        self.t_port = self.t[0]
        self.pas(round(port_s / self.r._dt))

    def pas(self, n=1, chaque=None):
        with contextlib.redirect_stdout(self.sortie):
            for _ in range(n):
                self.t[0] += self.r._dt
                if chaque is not None:
                    chaque(self)
                self.r.pas()

    def ligne(self):
        return next((l_ for l_ in self.journal if l_.startswith("ligne des mains au serrage")), "")

    def essai(self, panne, duree=1.5):
        """La panne au temps 0 : (vu, lâché, début du glissement, glissé au
        plus (m), {contrôle : premier instant au journal}, poids lu sur 0,1 s
        au plus bas le carton tenu), en secondes depuis la panne ; vu : base
        arrêtée (penche) ou transport interrompu."""
        m, r = self.m, self.r
        self.avant = (m.etat, m.penche, m.a_reposer)
        n0 = len(self.journal)
        panne(self)
        t0 = self.t[0]
        vu = lache = glisse = None
        glisse_max, premiers, bas = 0.0, {}, float("inf")
        while self.t[0] - t0 < duree - 1e-9:
            self.pas()
            dt = round(self.t[0] - t0, 3)
            if vu is None and (m.penche or m.a_reposer):
                vu = dt
            if lache is None and r.held is None:
                lache = dt
            if glisse is None and r.glissement_m > 0.0:
                glisse = dt
            glisse_max = max(glisse_max, r.glissement_m)
            if r.held is not None and len(m._hist_charge) >= R.filtre_force:
                bas = min(bas, float(np.mean([v[0] for v in
                                              list(m._hist_charge)[-R.filtre_force:]])))
            for ligne in self.journal[n0:]:
                if ligne.startswith(("port : ", "transport interrompu : ")):
                    for nom, cle in CONTROLES.items():
                        if cle in ligne.split(" ; fin du port")[0]:
                            premiers.setdefault(nom, dt)
            n0 = len(self.journal)
        return vu, lache, glisse, glisse_max, premiers, bas


def lacets(rad):
    """Les deux lacets d'épaule qui glissent de `rad`, vers l'extérieur : les
    deux mains s'ouvrent ensemble (~29 mm par 0,1 rad dans la posture de port)."""
    def panne(b):
        b.r.decalage_articulations["right_shoulder_yaw_joint"] = -rad
        b.r.decalage_articulations["left_shoulder_yaw_joint"] = rad
    return panne


def bras_droit_faiblit(part):
    def panne(b):
        for j in list(b.r._BRAS) + list(b.r._poignets):
            if j.startswith("right"):
                b.r.raideur_articulations[j] = part
    return panne


def resume(premiers):
    return ", ".join(f"{k} {v:.2f} s" for k, v in sorted(premiers.items(), key=lambda kv: kv[1]))


def posture(normale):
    return "prise normale" if normale else "prise en pendule"


section("1. Le niveau du début du port : la première foulée, dans sa posture")
b = Banc(port_s=0.2)
m = b.m
check(f"Porté depuis {b.t[0] - b.t_port:.1f} s : pas encore de niveau (au moins 0,1 s de "
      f"lectures plus vieilles que {R.recentes_niveau_s:g} s), rien à comparer",
      m.etat is Etat.TENU and m._niveau_debut_port(b.t[0], m._hist_charge[-1][0]) is None)
b.pas(round(0.6 / b.r._dt))
provisoire, incertain = m._niveau_debut_port(b.t[0], m._hist_charge[-1][0])
anciennes = [w for tt, w in m._niveau_port if tt <= b.t[0] - R.recentes_niveau_s + 1e-9]
check(f"Porté depuis {b.t[0] - b.t_port:.1f} s : niveau provisoire {provisoire:.2f} N "
      f"(± {incertain:.2f}), la moyenne des {len(anciennes)} lectures plus vieilles que "
      f"{R.recentes_niveau_s:g} s (une chute en cours n'abaisse pas sa propre référence)",
      abs(provisoire - float(np.mean(anciennes))) < 1e-9
      and abs(incertain - float(np.std(anciennes, ddof=1)) / np.sqrt(len(anciennes))) < 1e-9
      and len(anciennes) < len(m._niveau_port))
b.pas(round(1.0 / b.r._dt))
fige = m._niveau_port_fige
lues = m._niveau_port
b.pas(round(1.0 / b.r._dt))
check(f"Après la première foulée : niveau figé à {fige[0]:.2f} N (± {fige[1]:.2f}), la moyenne de "
      f"ses {len(lues)} lectures ({lues[-1][0] - lues[0][0]:.2f} s), le carton pesé "
      f"{m.poids_carton:.2f} N ; il ne bouge plus",
      fige is not None and abs(fige[0] - float(np.mean([w for _, w in lues]))) < 1e-9
      and fige[1] < incertain and lues[-1][0] - lues[0][0] <= R.fenetre_marge_s + 1e-9
      and m._niveau_port_fige == fige and len(m._niveau_port) == len(lues))
m._port_actif = False                       # fin d'un geste du port : nouvelle posture
b.pas()
check("Nouvelle posture (fin d'un geste du port) : le niveau est repris",
      m._niveau_port_fige is None and len(m._niveau_port) == 1)

section("2. Lâcher total : vu par le poids 0,3 s au plus après que le carton a quitté les mains")
for normale in (False, True):
    for nom, panne in (("les deux mains s'ouvrent de 2,9 cm (lacets 0,10 rad)", lacets(0.10)),
                       ("le bras droit faiblit (moteurs à 30 % de leur raideur)",
                        bras_droit_faiblit(0.3))):
        b = Banc(normale=normale)
        vu, lache, glisse, gmax, premiers, bas = b.essai(panne)
        poids = premiers.get("poids qui chute")
        check(f"{posture(normale)}, {nom} : carton lâché à "
              f"{lache if lache is None else f'{lache:.2f}'} s, poids qui chute vu "
              f"{poids - lache if poids is not None and lache is not None else float('nan'):.2f} s "
              f"après ; premier vu en {vu if vu is None else f'{vu:.2f}'} s — {resume(premiers)}",
              b.avant[0] is Etat.TENU and lache is not None and poids is not None
              and poids - lache <= DELAI and vu is not None and vu <= DELAI,
              f"{b.ligne()} {premiers}")
# Au tout début du port (0,1 s), le niveau n'est pas encore lu (0,4 s) : le
# lâcher est vu par les autres contrôles, et par le poids une fois le niveau
# lu (sur les lectures d'avant le lâcher : une chute en cours n'abaisse pas sa
# propre référence).
for normale in (False, True):
    b = Banc(normale=normale, port_s=0.1)
    vu, lache, glisse, gmax, premiers, bas = b.essai(lacets(0.10))
    poids = premiers.get("poids qui chute")
    check(f"{posture(normale)}, les deux mains s'ouvrent 0,1 s après le début du port, avant que "
          f"le niveau soit lu : carton lâché à {lache if lache is None else f'{lache:.2f}'} s, vu en "
          f"{vu if vu is None else f'{vu:.2f}'} s ; le poids, {poids - lache if poids is not None and lache is not None else float('nan'):.2f} s "
          f"après — {resume(premiers)}",
          lache is not None and vu is not None and vu <= DELAI and poids is not None
          and poids - lache <= 2 * DELAI, f"{premiers}")

section("3. Lâcher partiel : le carton glisse sans tomber ; vu par la marge de frottement")
# Le poids lu pendant le glissement n'est que ce que le frottement des mains
# retient encore : 4 à 22 % de moins, sous les 60 % de la règle (ni sous
# l'ancienne : 40 % du plus grand lu). Ce qui le voit : la marge de
# frottement sur 0,3 s (les forces normales, avec le frottement prudent 0,8 ;
# ce carton-ci en a 1,2), sous 1, « un vrai desserrage ». Un carton plus
# glissant que 0,8 (0,7) ne laisse pas ce temps : il tombe 0,2 à 0,3 s après
# que les mains s'ouvrent, vu en même temps (mains écartées, marge), puis par
# le poids 0,22 s après (docs/validation_port_carton.md, §15.11).
for normale, rad in ((False, 0.089), (True, 0.083)):
    b = Banc(normale=normale)
    niveau = b.m._niveau_port_fige[0]
    vu, lache, glisse, gmax, premiers, bas = b.essai(lacets(rad))
    chute = (niveau - bas) / b.m.poids_carton
    marge = premiers.get("marge sur 0,3 s")
    check(f"{posture(normale)}, les deux mains s'ouvrent ({rad:g} rad) : le carton glisse de "
          f"{100 * gmax:.1f} cm sans tomber (glisse dès {glisse if glisse is None else f'{glisse:.2f}'} s)",
          glisse is not None and gmax > 0.01 and lache is None and b.r.held is not None
          and not b.r.chutes, f"{gmax} {lache} {b.r.chutes}")
    check(f"… poids lu au plus bas {100 * chute:.0f} % sous le niveau du début du port (la règle : "
          f"{100 * (1 - R.part_poids_min):.0f} %) ; vu par la marge de frottement sur 0,3 s en "
          f"{marge if marge is None else f'{marge:.2f}'} s, "
          f"{marge - glisse if marge is not None and glisse is not None else float('nan'):.2f} s "
          f"après le début du glissement — {resume(premiers)}",
          0.0 < chute < 1 - R.part_poids_min and "poids qui chute" not in premiers
          and marge is not None and marge <= DELAI and marge - glisse <= DELAI,
          f"{chute} {premiers}")

section("4. Fausses alertes, sur le bruit réel des essais GPU")
MODELE = ModeleBras().au_contact(R.contact_main_m)
ART = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch",
       "wrist_yaw")
TAILLE = (("taille_lacet_rad", "waist_yaw_joint"), ("taille_roulis_rad", "waist_roll_joint"),
          ("taille_tangage_rad", "waist_pitch_joint"))


def lectures_gpu(nom):
    """Au port (TENU) : (pas en s, poids relevé par l'OS, poids recalculé à
    chaque lecture comme _force_main, depuis les couples et angles relevés :
    pesanteur verticale, biais des couples inconnus, un décalage constant)."""
    lignes = {}
    for r_ in csv.DictReader(open(DONNEES / nom, encoding="utf-8")):
        lignes.setdefault(float(r_["t_s"]), {})[r_["main"]] = r_
    T = [t_ for t_ in sorted(lignes) if len(lignes[t_]) == 2
         and lignes[t_]["left"]["phase"] == "TENU"]
    os_, recalc = [], []
    for t_ in T:
        angles, tau = {}, {}
        for cote in ("left", "right"):
            l_ = lignes[t_][cote]
            for a in ART:
                angles[f"{cote}_{a}_joint"] = float(l_[f"angle_{a}_rad"])
                tau[f"{cote}_{a}_joint"] = float(l_[f"couple_{a}_nm"])
            for cle, j in TAILLE:
                if l_.get(cle):
                    angles[j] = float(l_[cle])
        total = 0.0
        for cote, chaine in (("left", MODELE.gauche), ("right", MODELE.droite)):
            js = [f"{cote}_{a}_joint" for a in ART]
            jac = chaine.jacobienne(angles, js)
            d = np.array([tau[j] for j in js]) - chaine.couples_gravite(angles, js,
                                                                         (0.0, 0.0, -9.81))
            lam = R.amorti_force_m
            total += float(np.linalg.solve(jac @ jac.T + lam * lam * np.eye(3), jac @ d)[2])
        os_.append(float(lignes[t_]["left"]["poids_os_n"]))
        recalc.append(total)
    return float(np.median(np.diff(T))), np.array(os_), np.array(recalc)


def rejouer(lus, dt, pese, bruit_pesee):
    """La règle de l'OS (_surveiller_port), une lecture par relevé
    (filtre_force 1), le bruit d'une lecture à la pesée `bruit_pesee` : la
    première alerte (s depuis le début du port), ou None. pese None :
    l'ancienne règle (40 % du plus grand lu)."""
    m_ = ManipulationService.__new__(ManipulationService)
    m_.r = replace(R, filtre_force=1)
    m_._init_mesure()
    m_._bruit_pesee = bruit_pesee
    t = [0.0]
    alerte = []
    m_._horloge = lambda: t[0]
    m_._efforts = lambda: ({}, "")
    m_._surveiller_rotation = lambda tau: ""
    m_._interrompre_port = lambda raison: alerte.append(t[0])
    m_.poids_carton = pese
    for i, v in enumerate(lus):
        t[0] = i * dt
        m_._hist_charge.append((float(v), 0.0, None))
        m_.charge = (float(np.mean([h[0] for h in m_._hist_charge])), 0.0)
        m_._poids_ref = max(m_._poids_ref or 0.0, m_.charge[0])
        m_._surveiller_port()
        if alerte:
            return alerte[0]
    return None


ESSAIS = ("t1_prise_20261119_1716.csv", "t1_prise_20261119_2204.csv",
          "t1_prise_20261106_1821.csv")
# Le bruit d'une lecture bras immobiles, en haut du levage, recalculé comme
# ci-dessous : 0,95 N (17 h 16, t1_levage_20261119_1716.csv), 1,02 N (21 h 37) ;
# celui de la moyenne de 0,3 s relevée par l'OS, divisé par la racine de 15.
BRUIT_PESEE = 1.0
lus = {nom: lectures_gpu(nom) for nom in ESSAIS}
duree = sum(len(v[1]) * v[0] for v in lus.values())
for k, borne, bruit_pesee in ((1, "poids relevé par l'OS (moyenne de 0,3 s)", BRUIT_PESEE / np.sqrt(15)),
                              (2, "poids recalculé à chaque lecture", BRUIT_PESEE)):
    bruit = np.concatenate([v[k] - np.median(v[k]) for v in lus.values()])
    resultats = {}
    for W in (2.2, 3.0, 5.0, 10.0):
        nouvelle = ancienne = ports = 0
        for nom, (dt, *series) in lus.items():
            x = W + series[k - 1] - np.median(series[k - 1])
            for d in range(0, max(1, len(x) - 12), round(1.0 / dt)):
                tranche = x[d:d + round(10.0 / dt)]
                ports += 1
                nouvelle += rejouer(tranche, dt, W, bruit_pesee) is not None
                ancienne += rejouer(tranche, dt, None, bruit_pesee) is not None
        resultats[W] = (nouvelle, ancienne, ports)
    texte = " ; ".join(f"{W:g} N : {n}/{p} (ancienne {a}/{p})"
                       for W, (n, a, p) in resultats.items())
    # Relevé par l'OS (0,3 s) : aucune ; recalculé à chaque lecture (une
    # lecture pour 0,1 s, un niveau de quelques lectures au début du port :
    # bien plus bruité que l'OS à 50 Hz), jamais plus que l'ancienne règle,
    # aucune à 5 N et plus (pour 2,2 et 3 N, surtout dans la première seconde
    # du port, le niveau encore jeune).
    check(f"{borne}, bruit {np.std(bruit):.2f} N d'écart-type sur {duree:.0f} s de port : ports de "
          f"10 s au plus, commencés chaque seconde, en fausse alerte — {texte}",
          all(n == 0 for W, (n, a, p) in resultats.items() if k == 1 or W >= 5.0)
          and all(n <= a for n, a, p in resultats.values()), str(resultats))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
