"""
Porter le carton sans qu'il tourne autour de la ligne des mains.

Constat GPU (essai T1 du 5 novembre 2026, 16 h 06) : carton serré, levé,
puis il PIVOTE autour de la ligne qui joint les mains, bascule, glisse et
tombe. Les mains caoutchouc touchent presque en un point, 11 cm plus loin que
leur repère le long de la main (test_rejeu_prise_t1606.py) : visant le centre
du carton avec leur repère, elles le touchaient près de sa face arrière.

Désormais (prise.py, service.py) :
  - le point de contact de la main (MesuresBras / ReglagesPrise.contact_main_m)
    est la « main » de toute la géométrie de la prise : il vise le milieu des
    faces, et la force s'y estime ;
  - avant de lever, la ligne des points de contact mesurés doit passer par
    le centre de gravité (tolerances_cg, du couple de basculement que les
    mains retiennent) ;
  - levage lent et progressif, carton ramené vers le torse au plus près que
    les bras le permettent sans changer de posture en chemin, arrêt au
    toucher du torse ;
  - le carton qui tourne (caméra de tête, quand elle le voit) est reposé
    doucement sur la table ; au port, celui qui ne pèse plus sur les mains
    arrête la marche.

Robot simulé léger, horloge simulée ; ses mains : celles du G1
(contact_main_m=0.11), carton 11 cm plus loin que pour des mains idéales
(même posture des bras).
"""
import contextlib
import io
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.manipulation.prise import ReglagesPrise, tolerances_cg  # noqa: E402
from axis_os.services.manipulation.service import Etat, Objet  # noqa: E402
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
    def __init__(self, x=0.41, contact=0.11, os_contact=None, reglages=None, objets=True):
        self.t = [0.0]
        self.x = x
        self.r = SimulatedRobot(objects=[SimulatedObject("carton", (x, 0.0, 0.90))]
                                if objets else [], horloge=lambda: self.t[0],
                                contact_main_m=contact)
        self.r.start(fil=False)
        self.m = self.r.manipulation
        if os_contact is not None:
            # L'OS croit ses mains ailleurs que là où elles touchent.
            self.m.mesures.contact_main_m = os_contact
        if reglages is not None:
            self.m.r = reglages
        self.journal = []
        self.m._journal = self.journal.append
        self.etats, self.levage, self.sortie = [], [], io.StringIO()
        self.serrage = None                 # (force estimée, vraie) à la fin du serrage
        self.res = None
        self.pas(20)

    def pas(self, n=1):
        with contextlib.redirect_stdout(self.sortie):
            for _ in range(n):
                self.t[0] += self.r._dt
                self.r.pas()
                if not self.etats or self.etats[-1] is not self.m.etat:
                    self.etats.append(self.m.etat)
                if self.m.etat is Etat.SERRAGE:
                    self.serrage = (min(self.m.force_serrage.values()),
                                    min(self.r.forces_contact.values()))
                if self.m.etat is Etat.LEVAGE and self.m._c_actuel is not None:
                    self.levage.append((self.t[0], float(self.m._c_actuel[2])))

    def prendre(self, jusqua=(Etat.TENU, Etat.ECHEC), duree=30.0):
        self.res = self.m.prendre(Objet((self.x, 0.0, 0.90), 0.24, 0.20, 0.20))
        t0 = self.t[0]
        while self.m.etat not in jusqua and self.t[0] - t0 < duree:
            self.pas()
        return self

    def refus_garde_fou(self):
        return self.sortie.getvalue().count("SAFETY VIOLATION")

    def ligne(self, debut):
        return next((l for l in self.journal if l.startswith(debut)), "")


section("1. Mains du G1 : la ligne des points de contact passe par le centre de gravité")
b = Banc().prendre()
m, r = b.m, b.r
tp, th = tolerances_cg(R)
check("Carton pris et tenu", m.etat is Etat.TENU, m.raison)
check(f"… au serrage, ligne des mains à moins de {100 * tp:.1f} cm du centre de gravité en "
      f"profondeur, {100 * th:.0f} cm en hauteur (au journal)",
      m.ligne_cg is not None and abs(m.ligne_cg[0]) <= tp and abs(m.ligne_cg[1]) <= th
      and b.ligne("ligne des mains au serrage"), b.ligne("ligne des mains au serrage"))
check("… levé et ramené, il ne tourne pas (vérité du monde simulé)",
      r.held is not None and r.rotation_objet_deg == 0.0, str(r.rotation_objet_deg))
f_os, f_vrai = b.serrage
check(f"… à la fin du serrage, force à la main estimée au point de contact : {f_os:.1f} N, la "
      f"vraie {f_vrai:.1f} N, à 10 % près (au repère de la main, 35 % de trop sur GPU)",
      abs(f_os - f_vrai) <= 0.10 * f_vrai, f"{f_os} {f_vrai}")
check(f"… poids porté, d'après les couples : {m.charge[0]:.1f} N (carton de 1 kg : 9,8)",
      m.charge is not None and abs(m.charge[0] - 9.81) <= 1.5, str(m.charge))
check("… aucune consigne refusée par le garde-fou, avant-bras jamais sur le plateau",
      b.refus_garde_fou() == 0 and r.forearm_table_contacts == 0,
      f"{b.refus_garde_fou()} {r.forearm_table_contacts}")
b.pas(250)
check("Cinq secondes de port : tenu, rien de signalé", m.etat is Etat.TENU and not m.a_reposer
      and r.held is not None, m.raison)

section("2. Levage lent et progressif, carton vers le torse")
t_lev = b.levage[-1][0] - b.levage[0][0]
dz = np.diff([z for _, z in b.levage])
check(f"Levage de {100 * R.hauteur_test_m:.0f} cm en {t_lev:.1f} s (au moins "
      f"{R.duree_levage_test_s:.0f} : 2 cm/s), sans à-coup : départ et arrivée en douceur",
      t_lev >= R.duree_levage_test_s - 0.05 and dz[0] < 0.25 * dz.max() and dz[-2] < 0.25 * dz.max(),
      f"{t_lev} {dz[:3]} {dz.max()}")
check("Mains du G1 : ramené au plus près que les bras le permettent sans changer de posture en "
      "chemin ; le journal dit où, et que le torse n'est pas touché",
      "carton ramené sans toucher le torse : face arrière à" in "".join(b.journal), b.journal[-3:])
c = Banc(x=0.30, contact=0.0).prendre()
check("Mains idéales (contact à leur repère) : carton contre le torse, arrêt au toucher (au "
      "journal), poussée du torse modérée",
      c.m.etat is Etat.TENU and c.ligne("carton contre le torse") and 0.0 < c.r.force_torse < 15.0,
      f"{c.m.raison} {c.r.force_torse} {c.journal[-3:]}")
c.pas(250)
check("… et il ne bouge plus pendant le port", c.m.etat is Etat.TENU and not c.m.a_reposer)

section("3. Le défaut de l'essai GPU : la ligne des mains loin du centre de gravité")
# Carton là où l'OS d'avant le prenait (son repère de main au centre) ; les
# mains touchent 11 cm plus loin, près de la face arrière.
d = Banc(x=0.30, os_contact=0.0).prendre(jusqua=(Etat.ECHEC,))
check("L'OS croit toucher avec le repère de ses mains (le code d'avant) : le carton levé tourne, "
      "la caméra de tête le voit, il est reposé doucement, la prise échoue avec la raison",
      d.m.etat is Etat.ECHEC and "carton tourné de" in d.m.raison
      and "carton reposé sur la table" in d.m.raison and Etat.TENU not in d.etats
      and Etat.REPOSE in d.etats, f"{d.m.raison} {d.etats}")
o = d.r.objects[0]
check("… reposé : sur la table, à sa place, mains ouvertes", d.r.held is None
      and abs(o.position[2] - 0.90) < 0.03 and abs(o.position[0] - 0.30) < 0.03
      and d.r.rotation_objet_deg < d.r.ROTATION_CHUTE_DEG,
      f"{o.position} {d.r.rotation_objet_deg}")
e = Banc(x=0.30, os_contact=0.0)
e.r.objet_visible = False
e.prendre(jusqua=(Etat.ECHEC,), duree=40.0)
check("Caméra aveugle : le carton tombe ; jamais de marche avec lui (prise non confirmée, ou "
      "port interrompu)", Etat.TENU not in e.etats or e.m.a_reposer, f"{e.m.raison} {e.etats}")
g = Banc(reglages=replace(R, rayon_contact_main_m=0.0005)).prendre()
raison = g.m.raison or (g.res.reason if g.res is not None and not g.res.ok else "")
check("Tolérance au millimètre (mains qui ne retiendraient rien) : refus AVANT de lever (au "
      "plan ou au serrage), « ligne des mains … le carton tournerait autour d'elle »",
      "le carton tournerait autour d'elle" in raison and Etat.LEVAGE not in g.etats,
      f"{raison} {g.etats}")

section("4. Au port, le carton qui ne pèse plus sur les mains arrête la marche")
h = Banc().prendre()
h.pas(50)
check("Tenu, poids de référence pris au début du port", h.m.etat is Etat.TENU
      and h.m._poids_ref is not None and h.m._poids_ref > 5.0, str(h.m._poids_ref))
o = h.r.objects[0]
h.r.held, h.r._centre_tenu = None, None
o.position = (o.position[0], o.position[1], 0.10)          # échappé, au sol
h.pas(60)
check("Carton échappé : « ne pèse plus sur les mains », à reposer (la façade arrête la base et "
      "le signale)", h.m.a_reposer and "ne pèse plus sur les mains" in h.m.raison, h.m.raison)
h.pas(50)
check("… l'alerte tient (pas de reprise de la marche)", h.m.a_reposer)

section("5. Rien entre les mains : toujours refusé")
v = Banc(objets=False).prendre()
check("Aucun carton : « rien entre les mains », jamais tenu", v.m.etat is Etat.ECHEC
      and "rien entre les mains" in v.m.raison and Etat.TENU not in v.etats, v.m.raison)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
