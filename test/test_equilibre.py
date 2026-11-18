"""
Équilibre debout du G1 (axis_os/services/locomotion/equilibre.py) : son
centre de masse au-dessus de ses pieds, de l'URDF.

  1. Le robot debout : masse totale, centre de masse, appui des pieds
     (talons et pointes de l'URDF), marge.
  2. Penché (tangage de la taille), bras devant, carton aux mains : la
     marge diminue avec le tangage et la charge, jusqu'à sortir de l'appui.
  3. Bassin abaissé : pieds à plat, à la même place sous lui, et plus près
     de lui d'autant.
  4. Le bord d'une table n'est jamais plus près du bassin que la pointe des
     pieds (ReglagesPrise.bord_min_m) : pieds contre la table, pas plus.
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.locomotion.equilibre import TAILLE, jambes, modele_g1  # noqa: E402
from axis_os.services.manipulation.cinematique import ChaineBras  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise  # noqa: E402

FAIL = 0
BRAS_DEVANT = {"left_shoulder_pitch_joint": -0.9, "right_shoulder_pitch_joint": -0.9,
               "left_elbow_joint": 0.6, "right_elbow_joint": 0.6}


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


m = modele_g1()

section("1. Debout")
t0 = time.perf_counter()
e = m.equilibre(jambes())
dt = time.perf_counter() - t0
check(f"Masse totale de l'URDF : {e.masse:.1f} kg (G1 : ~35 kg)", 30.0 <= e.masse <= 37.0)
xs = e.polygone[:, 0]
check(f"Appui des pieds : de {100 * xs.min():.1f} cm (talons) à {100 * xs.max():+.1f} cm "
      f"(pointes), {100 * np.ptp(e.polygone[:, 1]):.0f} cm de large",
      -0.10 < xs.min() < -0.05 and 0.08 < xs.max() < 0.11 and np.ptp(e.polygone[:, 1]) > 0.25)
check(f"Centre de masse debout : {100 * e.centre[0]:+.1f} cm devant le bassin, marge "
      f"{100 * e.marge:.1f} cm", 0.05 <= e.marge and abs(e.centre[1]) < 0.005, e.texte())
check(f"Calcul en {1000 * dt:.1f} ms (à chaque demande de posture)", dt < 0.05)

section("2. Penché, bras devant, carton aux mains")
marges = []
for th in (0.0, 0.1, 0.2, 0.3, 0.4):
    q = dict(jambes(), **BRAS_DEVANT, **{TAILLE: th})
    x = 0.45 + 0.4 * np.sin(th)
    vide = m.equilibre(q).marge
    charge = m.equilibre(q, [(1.2, (x, 0.0, 0.15))]).marge
    marges.append((vide, charge))
    print(f"    tangage {th:.1f} rad : marge {100 * vide:+.1f} cm mains vides, "
          f"{100 * charge:+.1f} cm avec 1,2 kg à {100 * x:.0f} cm")
check("La marge diminue avec le tangage, mains vides et chargées",
      all(a[0] > b[0] and a[1] > b[1] for a, b in zip(marges, marges[1:])))
check("Le carton porté la diminue encore", all(c < v for v, c in marges))
loin = m.equilibre(dict(jambes(), **BRAS_DEVANT, **{TAILLE: 0.52}), [(5.0, (0.70, 0.0, 0.1))])
check(f"Penché au maximum de la taille (0,52 rad) avec 5 kg à 70 cm : hors de l'appui "
      f"({100 * loin.marge:+.1f} cm)", loin.marge < 0.0, loin.texte())

section("3. Bassin abaissé")
jambe = ChaineBras.depuis_urdf("left", fin="left_ankle_roll_link")
p0, r0 = jambe.pose({k: v for k, v in jambes().items() if k.startswith("left")})
for h in (-0.05, -0.10):
    q = jambes(h)
    p, r = jambe.pose({k: v for k, v in q.items() if k.startswith("left")})
    check(f"Bassin à {100 * h:+.0f} cm : pieds {100 * (p[2] - p0[2]):.1f} cm plus près de lui "
          f"(repère du bassin), à {1000 * abs(p[0] - p0[0]):.1f} mm de leur place, à plat",
          abs((p[2] - p0[2]) - abs(h)) < 1e-3 and abs(p[0] - p0[0]) < 1e-3
          and float(np.linalg.norm(r - r0)) < 1e-6, f"{p} {p0}")

section("4. Pieds contre la table, pas plus")
pointe = max(float(m.reperes(jambes())[lien][:3, :3] @ c @ np.array([1.0, 0.0, 0.0])
                   + m.reperes(jambes())[lien][0, 3]) + rayon
             for lien, spheres in m.contacts.items() for c, rayon in spheres)
check(f"Pointe des pieds à {100 * pointe:.1f} cm devant le bassin ; bord d'une table jamais "
      f"plus près que {100 * ReglagesPrise().bord_min_m:.0f} cm (bord_min_m)",
      ReglagesPrise().bord_min_m >= pointe, f"{pointe}")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
