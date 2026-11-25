"""
Prise en pendule : la ligne des mains au-dessus du centre de gravité du
carton, d'une marge écrite dans les réglages (ReglagesPrise,
marge_pendule_m, affaissement_prise_m, bord_face_m ; justification en
commentaire).

Pourquoi : essai GPU T1 du 20 novembre 2026, 21 h 37. La règle d'avant
admettait la ligne « à 8,3 cm au plus » du centre, dessus comme dessous ;
les mains, visées au milieu des faces, se sont posées 0,2 à 1 cm au-dessus du
centre, décalées de 0,9 cm en profondeur, et le frottement en rotation du
caoutchouc ne retenait presque rien : le carton a tourné de 50° entre les
paumes (l'équilibre d'un pendule presque sans bras, tan θ = e / d).

Robot simulé léger, G1 29 DOF, mains du G1 ; la rotation du carton autour de
la ligne des mains y est modélisée (SimulatedRobot._bras_levier : le poids le
tourne dans le sens qui annule son bras de levier, au-delà de ce que le
frottement en rotation des mains retient) :
  1. les réglages, et les cartons trop bas refusés avec la raison ;
  2. l'OS vise les mains au-dessus du centre, et le vérifie au serrage sur
     les mains mesurées ;
  3. rejeu de la géométrie de 21 h 37 (centre 0,9 cm devant la ligne, ligne
     0,5 cm au-dessus, frottement en rotation de 1 mm de rayon effectif : sous 5 mm à l'essai) : le
     carton tourne ; ligne sous le centre, il bascule ; ligne 6 cm au-dessus,
     écarté de 20°, il revient.
"""
import contextlib
import io
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from axis_os.services.manipulation.prise import ReglagesPrise, hauteur_prise  # noqa: E402
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
    """Un carton de 1 kg, de `hauteur` m, pris par les mains du G1, porté."""

    def __init__(self, hauteur=0.20):
        self.t = [0.0]
        z = 0.80 + hauteur / 2.0
        self.r = SimulatedRobot(objects=[SimulatedObject("carton", (0.41, 0.0, z),
                                                         size=(0.24, 0.20, hauteur))],
                                horloge=lambda: self.t[0], contact_main_m=0.11,
                                poignets_29dof=True)
        self.r.start(fil=False)
        self.m = self.r.manipulation
        self.journal = []
        self.m._journal = self.journal.append
        self.sortie = io.StringIO()
        self.pas(20)
        self.res = self.m.prendre(Objet((0.41, 0.0, z), 0.24, 0.20, hauteur))
        t0 = self.t[0]
        while self.m.etat not in (Etat.TENU, Etat.ECHEC) and self.t[0] - t0 < 40.0:
            self.pas()

    def pas(self, n=1):
        with contextlib.redirect_stdout(self.sortie):
            for _ in range(n):
                self.t[0] += self.r._dt
                self.r.pas()


section("1. Les réglages ; un carton trop bas refusé, avec la raison")
VISEE = R.marge_pendule_m + R.affaissement_prise_m
check(f"Marge {100 * R.marge_pendule_m:g} cm au serrage (estimée par l'OS), affaissement des mains "
      f"{100 * R.affaissement_prise_m:g} cm : visée {100 * VISEE:g} cm ; contact à "
      f"{100 * R.bord_face_m:g} cm au moins sous l'arête",
      (R.marge_pendule_m, R.affaissement_prise_m, R.bord_face_m) == (0.03, 0.03, 0.03))
check("Carton de 20 cm ou de 18 cm : mains 6 cm au-dessus du centre ; de 17 cm : aucune prise en "
      "pendule (contact à moins de 3 cm de l'arête)",
      abs(hauteur_prise(R, 0.20) - VISEE) < 1e-12 and abs(hauteur_prise(R, 0.18) - VISEE) < 1e-12
      and hauteur_prise(R, 0.17) is None)
b = Banc(hauteur=0.15)
check("Carton de 15 cm : refusé avant de bouger les bras, « trop bas pour une prise en "
      "pendule », jamais tenu",
      "trop bas pour une prise en pendule" in f"{b.m.raison} {b.res}" and b.r.held is None
      and b.m.etat in (Etat.ECHEC, Etat.LIBRE), f"{b.m.etat} {b.m.raison} {b.res}")

section("2. L'OS vise les mains au-dessus du centre, et le vérifie au serrage")
b = Banc()
serrage = next((ligne for ligne in b.journal if ligne.startswith("ligne des mains au serrage")), "")
check(f"Tenu ; ligne des mains au serrage (mains mesurées) {100 * b.m.ligne_cg[1]:+.1f} cm "
      f"au-dessus du centre (au moins {100 * R.marge_pendule_m:g})",
      b.m.etat is Etat.TENU and b.m.ligne_cg[1] >= R.marge_pendule_m and serrage,
      f"{b.m.raison} {b.m.ligne_cg}")
centre = np.asarray(b.r._centre_tenu, dtype=float)
check(f"Vérité du monde simulé : centre du carton {100 * -centre[2]:.1f} cm sous la ligne des "
      f"mains, {100 * abs(centre[0]):.1f} cm en profondeur ; porté 3 s, il ne tourne pas",
      centre[2] <= -R.marge_pendule_m and (b.pas(150) or True)
      and b.r.held is not None and abs(b.r.rotation_objet_deg) < 1.0,
      f"{centre} {b.r.rotation_objet_deg}")
b.m.ligne_cg = None
ecart = b.m._ecart_cg((np.array([0.0, 0.1, 0.01]), np.array([0.0, -0.1, 0.01])),
                      np.zeros(3))
check("Ligne 1 cm au-dessus du centre (ce qu'admettait la règle d'avant) : refusée, « prise "
      "en pendule »", "prise en pendule" in ecart and "+1.0 cm en hauteur" in ecart, ecart)
check("… 4 cm au-dessus et 1 cm en profondeur : admise",
      b.m._ecart_cg((np.array([0.01, 0.1, 0.04]), np.array([0.01, -0.1, 0.04])),
                    np.zeros(3)) == "")

section("3. Rejeu de la géométrie de 21 h 37 dans le robot simulé")


def rejeu(e, z, depart_deg=0.0, duree=3.0):
    """Le carton porté, son centre à e devant et z au-dessus de la ligne des
    mains (vérité du monde simulé), frottement en rotation de 3 mm de rayon
    effectif (presque un point), caméra aveugle : sa rotation au bout de `duree` s, la plus
    grande, et s'il est tombé."""
    b = Banc()
    b.r.objet_visible = False
    b.r.RAYON_CONTACT_M = 0.001
    b.r._centre_tenu = np.array([e, b.r._centre_tenu[1], z])
    b.r._pose_prise = (None, None, None)        # loin de la table : rien ne l'arrête
    b.r.rotation_objet_deg = depart_deg
    pire = abs(depart_deg)
    for _ in range(round(duree / b.r._dt)):
        b.pas()
        pire = max(pire, abs(b.r.rotation_objet_deg))
        if b.r.held is None:
            break
    return b.r.rotation_objet_deg, pire, b.r.held is None


fin, pire, tombe = rejeu(0.009, -0.005)
check(f"21 h 37 (centre 0,9 cm devant, ligne 0,5 cm au-dessus) : le carton tourne, "
      f"{pire:.0f}° (essai : 50°, puis il a glissé)", pire > 40.0, f"{fin} {pire} {tombe}")
# La même géométrie, caméra de tête qui voit le carton : l'OS l'arrête.
b = Banc()
b.r.RAYON_CONTACT_M = 0.001
b.r._centre_tenu = np.array([0.009, b.r._centre_tenu[1], -0.005])
b.r._pose_prise = (None, None, None)
vu = None
for _ in range(round(3.0 / b.r._dt)):
    b.pas()
    if b.m.a_reposer:
        vu = abs(b.r.rotation_objet_deg)
        break
check(f"… caméra de tête : transport interrompu, carton tourné de "
      f"{vu if vu is None else round(vu, 1)}° (au plus 10°), « il tourne entre les paumes »",
      vu is not None and vu < 10.0 and "il tourne entre les paumes" in (b.m.raison or "")
      and b.r.held is not None, f"{vu} {b.m.raison}")
fin, pire, tombe = rejeu(0.009, 0.02)
check("Ligne 2 cm SOUS le centre : il bascule et tombe", tombe, f"{fin} {pire} {tombe}")
fin, pire, tombe = rejeu(0.009, -VISEE, depart_deg=-20.0)
equilibre = math.degrees(math.atan2(0.009, VISEE))
check(f"Prise en pendule (ligne {100 * VISEE:g} cm au-dessus), écarté de 20° : il revient vers "
      f"son équilibre ({equilibre:.1f}° sans frottement), à {abs(fin):.1f}° (le frottement des "
      f"mains, ou le torse, le retient là)",
      not tombe and abs(fin) <= 12.0, f"{fin} {pire} {tombe}")
fin, pire, tombe = rejeu(0.009, -VISEE, depart_deg=20.0)
check(f"… écarté de 20° de l'autre côté : il revient aussi, à {abs(fin):.1f}°",
      not tombe and abs(fin) <= 12.0, f"{fin} {pire} {tombe}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
