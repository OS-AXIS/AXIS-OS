"""
Repose sûre : le carton tenu est reposé là où il a été pris, jamais lâché en
chemin.

Avant : poser() sans plateau, en prise mesurée, ramenait le carton à sa place
en interpolant les angles des bras (état POSE) : les mains ne restaient pas
un corps rigide avec lui, le serrage se relâchait en chemin. Campagne de
cartons variés, le n° 1004 du balayage (30 cm de large), l'essai de dépose du
planificateur fidèle (mains au contact) : pris, porté, la dépose à sa place
trouvée impossible en bas (les mains ne s'y ouvraient pas sur place), puis
reposé par POSE : « mains qui ne serrent plus », tombé. Le planificateur
l'écarte aujourd'hui par prudence (« dépose impossible ensuite »).

Désormais (ManipulationService._redescendre, _reposer_a_sa_place) :
  1. le chemin de la dépose, en corps rigide (devant, à la hauteur où il est,
     puis vers le bas), jusqu'à sa place : le milieu des mains au serrage ;
     tout le chemin vérifié avant de bouger ;
  2. serrage gardé jusqu'à sa place sur la table ; arrêté en chemin, il
     n'ouvre pas : tenu, à reposer ;
  3. l'ouverture de la dépose, sur place (serrage relâché, mains écartées de
     jeu_ouverture_m, sans remonter), vérifiée avant de bouger ; plus
     l'ouverture du placement, qui remontait les mains de ~2 cm encore au
     contact (la fermeture en pendule descend en fermant) ;
  4. le robot a un peu bougé depuis la prise (l'erreur d'approche, 2 cm) : sa
     place, revue d'ici, est rattrapée ; plus loin, aucune table n'est connue
     sous les mains : posé devant lui à la hauteur de sa prise, comme avant
     (ctx.place(None)), par le même geste, et le journal le dit ;
  5. le robot simulé porte le carton pendant la descente de la repose : il y
     glisse, s'y place entre les mains, et le perdre est une chute (cas
     interdit de la campagne).

Robot simulé léger, horloge simulée, mains du G1 (contact 11 cm le long de la
main), bras à 7 articulations, table de prise à 80 cm.
"""
import contextlib
import io
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402
from axis_os.services.manipulation.service import (  # noqa: E402
    Etat, ManipulationService, Objet,
)
from axis_os.store.sdk.app_base import RobotContext  # noqa: E402
from axis_os.store.sdk.results import RefusalCode  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402
from axis_os.testing import cartons as K  # noqa: E402

FAIL = 0
R = ReglagesPrise()
CENTRE = (0.41, 0.0, K.PLATEAU_M + 0.10)       # carton T1 (24 × 20 × 20 cm) sur la table


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


class Banc:
    """Le carton T1 pris sur sa table, puis porté (TENU)."""

    def __init__(self):
        self.t = [0.0]
        self.carton = SimulatedObject("carton", CENTRE)
        self.r = SimulatedRobot(objects=[self.carton], horloge=lambda: self.t[0],
                                contact_main_m=0.11, poignets_29dof=True, plateaux=[K.TABLE])
        self.r.start(fil=False)
        self.m = self.r.manipulation
        self.journal = []
        self.m._journal = self.journal.append
        self.etats, self.sortie = [], io.StringIO()
        self.lache = None             # (instant, état, sous-phase, z du carton) quand il quitte les mains
        self.forces_min = np.inf      # pendant la descente de la repose
        self.montee = 0.0             # des mains commandées, à l'ouverture, encore au contact
        self.pas(20)

    def pas(self, n=1):
        m, r = self.m, self.r
        with contextlib.redirect_stdout(self.sortie):
            for _ in range(n):
                tenu = r.held is not None
                self.t[0] += r._dt
                r.pas()
                if not self.etats or self.etats[-1] is not m.etat:
                    self.etats.append(m.etat)
                if m.etat is Etat.REPOSE and m._sous == "redescendre" and r.held is not None:
                    self.forces_min = min(self.forces_min, *r.forces_contact.values())
                if m.etat is Etat.REPOSE and m._sous == "desserrer" and r.held is not None:
                    z = sum(float(p[0][2]) for p in m._poses(m._q).values()) / 2.0
                    self.z_bas = getattr(self, "z_bas", z)
                    self.montee = max(self.montee, z - self.z_bas)
                if tenu and r.held is None and self.lache is None:
                    self.lache = (self.t[0], m.etat, m._sous, self.carton.position)

    def jusqua(self, fini, duree):
        t0 = self.t[0]
        while not fini() and self.t[0] - t0 < duree:
            self.pas()

    def prendre(self):
        self.m.prendre(Objet(CENTRE, 0.24, 0.20, 0.20))
        self.jusqua(lambda: self.m.etat in (Etat.TENU, Etat.ECHEC), 40.0)
        self.pas(100)                                  # deux secondes de port
        return self

    def ligne(self, debut):
        return next((l for l in self.journal if l.startswith(debut)), "")


section("1. poser() sans plateau : reposé là où il a été pris, sur sa table")
b = Banc().prendre()
m, r = b.m, b.r
check("Carton pris et porté", m.etat is Etat.TENU and r.held is not None, m.raison)
res = m.poser()
b.jusqua(lambda: m.etat is Etat.LIBRE, 20.0)
x, y, z = b.carton.position
check(f"Reposé : bras libres, carton sur la table à sa place ({100 * (x - CENTRE[0]):+.1f} cm, "
      f"{100 * (y - CENTRE[1]):+.1f} cm ; à plat à {z:.3f} m)", res.ok and m.etat is Etat.LIBRE
      and r.held is None and abs(x - CENTRE[0]) < 0.01 and abs(y - CENTRE[1]) < 0.01
      and abs(z - CENTRE[2]) < 1e-6, f"{res.reason} {m.etat} {m.raison}")
check(f"… jamais par l'ancienne interpolation des angles (états : "
      f"{' → '.join(e.name for e in b.etats[-4:])}), sans chute ni consigne refusée",
      Etat.POSE not in b.etats and not r.chutes
      and b.sortie.getvalue().count("SAFETY VIOLATION") == 0, str(r.chutes))
check(f"… serrage gardé jusqu'à sa place : les deux mains serrent pendant toute la descente "
      f"({b.forces_min:.1f} N au plus faible), lâché à l'ouverture seulement, en bas",
      b.forces_min > 0.5 and b.lache is not None and b.lache[2] == "desserrer"
      and b.lache[3][2] < CENTRE[2] + R.hauteur_test_m / 4, str(b.lache))
check(f"… l'ouverture sur place ne remonte pas les mains encore au contact ({1000 * b.montee:.1f} "
      f"mm, au plus 2)", b.montee < 0.002, f"{b.montee}")
check("Au journal : « repose : l'objet redescend là où il a été pris … »",
      b.ligne("repose : l'objet redescend là où il a été pris") != "", b.journal[-3:])

section("2. Rien de vérifié, rien ne bouge : refusé, l'objet reste tenu")
for nom, panne, attendu in (
        ("chemin", lambda m_: setattr(m_, "_chemin_rigide", lambda *a, **k: None),
         "hors d'atteinte des bras"),
        ("ouverture", None, "ne pourraient pas s'y ouvrir sur place")):
    c = Banc().prendre()
    if panne is not None:
        panne(c.m)
    else:
        q_rigide = c.m._q_rigide

        def sans_ouverture(centre, ecart=None, graine=None, delta=None, rotation=None,
                           _q=q_rigide):
            if delta is not None and all(v == 0.0 for v in delta.values()):
                return None                    # l'ouverture sur place, serrage relâché
            return _q(centre, ecart, graine, delta, rotation)
        c.m._q_rigide = sans_ouverture
    q0, centre0 = dict(c.m._q), np.array(c.m._centre_porte)
    res = c.m.poser()
    c.pas(50)
    check(f"{nom.capitalize()} qui ne passe pas : « repose impossible : … {attendu} … l'objet reste "
          f"tenu », rien n'a bougé", not res.ok and attendu in str(res.reason)
          and "repose impossible" in c.m.raison and c.m.etat is Etat.TENU
          and c.r.held is not None and Etat.REPOSE not in c.etats
          and float(np.linalg.norm(c.m._centre_porte - centre0)) < 1e-9
          and max(abs(c.m._q[j] - v) for j, v in q0.items()) < 1e-3,
          f"{res.reason} {c.m.etat}")
    check("… et pour une app (ctx.place(None)) : OUT_OF_REACH, comme « dépose impossible »",
          RobotContext._code_refus_manipulation(str(res.reason)) is RefusalCode.OUT_OF_REACH,
          str(res.reason))

section("3. Le robot a bougé depuis la prise")
e = Banc().prendre()
e.r.x += 0.01                                     # 1 cm : la place, revue d'ici, est rattrapée
res = e.m.poser()
e.jusqua(lambda: e.m.etat is Etat.LIBRE, 20.0)
x = e.carton.position[0]
check(f"Base à 1 cm de là (au plus {100 * R.erreur_approche_m:g}) : reposé à sa place sur la "
      f"table, le décalage rattrapé ({100 * (x - CENTRE[0]):+.1f} cm)", res.ok
      and e.m.etat is Etat.LIBRE and e.r.held is None and not e.r.chutes
      and abs(x - CENTRE[0]) < 0.005, f"{res.reason} {e.m.raison} {e.r.chutes}")
d = Banc().prendre()
d.r.x += 0.05                                     # la base a bougé de 5 cm
res = d.m.poser()
d.jusqua(lambda: d.m.etat is Etat.LIBRE, 20.0)
x = d.carton.position[0]
check(f"Base à 5 cm de là : aucune table connue sous les mains, posé devant le robot à la hauteur "
      f"de sa prise, comme avant (ctx.place(None)), par le même geste vérifié ; le journal le dit "
      f"(ici, sur la table, {100 * (x - CENTRE[0]):+.1f} cm de sa place)", res.ok
      and d.m.etat is Etat.LIBRE and Etat.POSE not in d.etats and not d.r.chutes
      and d.ligne("repose : l'objet redescend devant le robot, à la hauteur de sa prise : le "
                  "robot n'est plus là où il l'a pris") != ""
      and abs(x - (CENTRE[0] + 0.05)) < 0.01, f"{res.reason} {d.m.raison} {d.journal[-2:]}")

section("4. Arrêtée en chemin, la repose n'ouvre pas en l'air")
f = Banc().prendre()
res = f.m.poser()
f.pas(10)
f.m._q_mains = lambda centre, ecart: None        # la pose ne se tient plus, en plein geste
f.jusqua(lambda: f.m.etat is not Etat.REPOSE, 20.0)
f.pas(50)
check("Pose qui ne se tient plus en chemin : arrêtée au dernier point tenu, l'objet tenu (à "
      "reposer), jamais ouvert en l'air", res.ok and f.m.etat is Etat.TENU and f.m.a_reposer
      and "repose interrompue" in f.m.raison and f.r.held is not None and not f.r.chutes,
      f"{f.m.etat} {f.m.raison} {f.r.chutes}")

section("5. Repose pendant le retour (le carton tourne) : même chemin vérifié")


def tourne_au_retour(banc, apres=40):
    """Le carton « tourne » au 41e pas du retour (0,8 s, en chemin vers le
    buste) : _pas_ramener le fait reposer (_reposer)."""
    n, defaut = [0], banc.m._surveiller_rotation

    def surveiller(tau):
        if banc.m.etat is Etat.RAMENER:
            n[0] += 1
            if n[0] > apres:
                return "carton tourné (essai)"
        return defaut(tau)
    banc.m._surveiller_rotation = surveiller


g = Banc()
tourne_au_retour(g)
g.m.prendre(Objet(CENTRE, 0.24, 0.20, 0.20))
g.jusqua(lambda: g.m.etat in (Etat.ECHEC, Etat.LIBRE), 60.0)
check("Reposé à sa place, puis la prise échoue avec la raison (« carton reposé sur la table »), "
      "sans chute", Etat.RAMENER in g.etats and g.m.etat is Etat.ECHEC
      and "carton reposé sur la table" in g.m.raison and g.r.held is None and not g.r.chutes
      and abs(g.carton.position[2] - CENTRE[2]) < 1e-6, f"{g.m.raison} {g.r.chutes}")
h = Banc()
tourne_au_retour(h)
h.m.prendre(Objet(CENTRE, 0.24, 0.20, 0.20))
h.jusqua(lambda: h.m.etat is Etat.RAMENER, 40.0)
h.m._chemin_rigide = lambda *a, **k: None         # le chemin de la repose ne passe pas
h.jusqua(lambda: h.m.etat is not Etat.RAMENER, 10.0)
h.pas(30)
check("Chemin qui ne passe pas : l'objet reste tenu là où il est, à reposer (« repose "
      "impossible »), jamais lâché", h.m.etat is Etat.TENU and h.m.a_reposer
      and "repose impossible" in h.m.raison and h.r.held is not None and not h.r.chutes,
      f"{h.m.etat} {h.m.raison}")

section("6. Le robot simulé porte le carton pendant la descente de la repose")
check("Descente de la repose (REPOSE, « redescendre ») : un geste du port pour le robot simulé ; "
      "l'ouverture qui suit (« desserrer »), non", all((
          SimulatedRobot._porte(type("R", (), {"manipulation": type("M", (), {
              "etat": e_, "_sous": s_})(), "_ETATS_PORT": SimulatedRobot._ETATS_PORT})()) == v)
          for e_, s_, v in ((Etat.REPOSE, "redescendre", True), (Etat.REPOSE, "desserrer", False),
                            (Etat.TENU, "", True), (Etat.OUVERTURE, "", False))))

section("7. Campagne : le n° 1004, l'essai de dépose du planificateur fidèle")
essai = ManipulationService._essai_chemins


def essai_fidele(self, *a, **k):
    """L'essai de dépose du planificateur, mains au contact (et non serrées
    au plus de leurs bornes) : le n° 1004 est pris."""
    r_ = self.r
    self.r = replace(r_, penetration_poids_max_m=-r_.penetration_max_m)
    try:
        return essai(self, *a, **k)
    finally:
        self.r = r_


ManipulationService._essai_chemins = essai_fidele
try:
    n1004 = next(c for _, _, c in K.balayage() if c.numero == 1004)
    x = K.essayer_carton(n1004)
finally:
    ManipulationService._essai_chemins = essai
check(f"30 cm de large : pris, porté, dépose à sa place impossible en bas (« {x['raison'][:60]}… »)",
      x["tenu"] and "dépose impossible" in x["raison"] and "DESCENTE" in x["etats"], x["raison"])
check(f"… reposé à sa place : sur la table, sans chute, aucun cas interdit (états : "
      f"{' → '.join(x['etats'][-4:])})", x["issue"] == "reposée" and x["sur_table"]
      and not x["chutes"] and not K.violations(x) and "POSE" not in x["etats"],
      f"{x['issue']} {x['chutes']} {K.violations(x)}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
