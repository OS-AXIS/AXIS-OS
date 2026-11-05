"""
Couples estimés non fiables : la force à la main ne décide plus, la
géométrie si.

Constat GPU (essai T1 du 5 novembre 2026) : 5 prises, 0 triée, toutes
« serrage insuffisant (-2.9 à -3.4 N) » alors que le juge mesurait 6 à
9.5 N par main. Le banc lisait les efforts imposés aux articulations (nuls :
elles sont commandées en position), pas le couple des moteurs ; l'OS ne
recevait que les erreurs d'estimation ajoutées par le banc. À la calibration,
les « biais » valaient l'opposé de la pesanteur.

Désormais (prise.py, service.py) : la calibration des couples juge s'ils
suivent la pesanteur du modèle (CalibrationCouples.fiabilite). S'ils ne la
suivent pas, la force à la main ne décide plus ni du contact, ni du filet
d'appui avant de fermer, ni du serrage, ni de la confirmation au levage :
contact en géométrie (et retard du roulis), serrage jugé sur la main
retenue par l'objet (retard tenu, main bloquée). Les plafonds de couple
restent jugés sur le couple. Rien entre les mains : toujours refusé.

Robot simulé léger, sur une horloge simulée (le test fait chaque pas).
`bruit_couples` remplace les couples lus par ceux du banc Isaac : du bruit
autour d'un décalage par articulation, sans la pesanteur ni le contact.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.prise import CalibrationCouples, ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import (  # noqa: E402
    Etat, Objet, Postures, posture_bras,
)
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
P, R = Postures(), ReglagesPrise()
# Les postures de la calibration du banc T1 (postures_calib).
POSTURES = [("port", posture_bras(P.roulis_ouvert, P.tangage_port, P.coude)),
            ("ouverte", posture_bras(P.roulis_presaisie, P.tangage_port, P.coude)),
            ("basse", posture_bras(0.25, -0.15, 0.50)),
            ("haute", posture_bras(0.25, -0.70, 0.20)),
            ("port", posture_bras(P.roulis_ouvert, P.tangage_port, P.coude))]
CARTON = Objet((0.30, 0.0, 0.90), 0.24, 0.20, 0.20)


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class CouplesIsaac:
    """Couples lus comme sur le banc Isaac de l'essai : les efforts imposés
    (nuls) plus les erreurs d'estimation (décalage par articulation, bruit
    de 0.3 N·m). Ni pesanteur, ni contact."""

    def __init__(self, n, graine=1):
        self.rng = np.random.default_rng(graine)
        self.decalage = self.rng.uniform(-0.5, 0.5, n)

    def __call__(self, v):
        return self.decalage + self.rng.normal(0.0, 0.3, len(v))


class Banc:
    def __init__(self, objets=True, isaac=True):
        self.t = [0.0]
        self.r = SimulatedRobot(objects=[SimulatedObject("carton", (0.30, 0.0, 0.90))]
                                if objets else [], horloge=lambda: self.t[0])
        self.r.start(fil=False)
        self.r.poignets_cedent = True
        if isaac:
            self.r.bruit_couples = CouplesIsaac(len(self.r._BRAS) + len(self.r._poignets))
        self.m = self.r.manipulation
        self.journal = []
        self.m._journal = self.journal.append
        self.pas(20)

    def pas(self, n=1):
        for _ in range(n):
            self.t[0] += self.r._dt
            self.r.pas()

    def calibrer(self):
        """Ce que fait le banc T1 au démarrage, carton écarté : chaque
        posture à 0,02 rad par pas au plus, puis des relevés bras immobiles."""
        o = self.r.objects[0] if self.r.objects else None
        place = o.position if o else None
        if o:
            o.position = (3.0, 3.0, 0.90)
        pose = self.r.adapter.pose_bras
        for nom, cible in POSTURES:
            while True:
                reste = 0.0
                for j, v in cible.items():
                    e = v - pose.get(j, v)
                    pose[j] = pose.get(j, v) + max(-0.02, min(0.02, e))
                    reste = max(reste, abs(e))
                self.pas()
                if reste < 1e-6:
                    break
            self.pas(15)
            for _ in range(10):
                self.m.echantillon_calibration_couples(nom)
                self.pas()
        texte = self.m.terminer_calibration_couples()
        self.m.resynchroniser()
        if o:
            o.position = place
        self.pas(10)
        return texte

    def prendre(self, duree=30.0):
        self.m.prendre(CARTON)
        etats, t0 = [], self.t[0]
        while self.m.etat not in (Etat.TENU, Etat.ECHEC) and self.t[0] - t0 < duree:
            if not etats or etats[-1] is not self.m.etat:
                etats.append(self.m.etat)
            self.pas()
        etats.append(self.m.etat)
        return etats


section("1. La calibration juge si les couples suivent la pesanteur")
b = Banc()
texte = b.calibrer()
check("Couples du banc Isaac (bruit, ni pesanteur ni contact) : NON FIABLES, et dit au journal",
      b.m.couples_fiables is False and "couples estimés NON FIABLES" in texte
      and "ne décide plus ni du contact ni du serrage" in texte, texte.splitlines()[-1])
_, pentes = b.m.calibration_couples.fiabilite()
check("… pente des couples contre la pesanteur, au tangage d'épaule : près de 0 (attendu 1)",
      pentes and all(abs(v) < 0.3 for v in pentes.values()), str(pentes))
vrai = Banc(isaac=False)
texte_vrai = vrai.calibrer()
check("Couples justes (robot simulé : pesanteur et contact) : fiables, pente près de 1",
      vrai.m.couples_fiables is True and "couples estimés fiables" in texte_vrai
      and all(abs(v - 1) < 0.2 for v in vrai.m.calibration_couples.fiabilite()[1].values()),
      texte_vrai.splitlines()[-1])
check("Sans relevé, ou sans posture où la pesanteur varie : pas jugé (None)",
      CalibrationCouples().fiabilite()[0] is None)

section("2. Couples non fiables : la prise se fait en géométrie")
etats = b.prendre()
m = b.m
check("Carton pris et tenu (avant, sur ces couples : « serrage insuffisant », force estimée "
      "fausse)", m.etat is Etat.TENU and Etat.SERRAGE in etats, f"{m.etat} {m.raison}")
check("… contact vu en géométrie (ni couple, ni force) sur les deux mains",
      all(m.contacts_estimes[c] and m.contacts_estimes[c]["signal"] in ("géométrie",
                                                                       "retard du roulis")
          for c in ("left", "right")), str(m.contacts_estimes))
jugement = next((l for l in b.journal if l.startswith("serrage jugé en géométrie")), "")
check("… serrage jugé sur les mains retenues par l'objet, au journal",
      "main G retenue à" in jugement and "main D retenue à" in jugement, jugement)
check("… sans consigne refusée par le garde-fou", b.r.locomotion.stats["refused"] == 0)

section("3. Rien entre les mains : toujours refusé")
v = Banc(objets=False)
v.calibrer()
etats = v.prendre()
check("Aucun carton, couples non fiables : refus « rien entre les mains », jamais tenu",
      v.m.etat is Etat.ECHEC and "rien entre les mains" in v.m.raison
      and Etat.TENU not in etats, f"{v.m.etat} {v.m.raison}")

section("4. Couples fiables : rien ne change")
etats = vrai.prendre()
check("Couples justes : prise tenue, serrage jugé sur la force (comme avant)",
      vrai.m.etat is Etat.TENU and not any(l.startswith("serrage jugé en géométrie")
                                           for l in vrai.journal), f"{vrai.m.etat} {vrai.m.raison}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
