"""
Prise mesurée dans la géométrie de la scène Isaac Sim T1
(axis_os/apps/tri_colis/scene_demo.py) : table de prise, carton au bord du
plateau, robot tourné d'un quart de tour (cap de prise +y), distance de
prise de la façade, hauteur du bassin mesurée sur GPU.

Constat GPU (RTX PRO 5000, 2026-10-28) : « prise refusée — hors de portée
des bras : objet à 0.32 m devant le bassin et +0.17 m en hauteur » à chaque
essai, alors que l'ancienne prise tenait ce carton. Cause : le lacet du
carton (repère monde) n'était pas replié ; devant un robot tourné d'un quart
de tour, il valait -90° et les mains visaient l'avant et l'arrière du
carton. Robot simulé léger d'axis_os.testing : cinématique de l'URDF et
contact simulé.
"""
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.apps.tri_colis import scene_demo as sd  # noqa: E402
from axis_os.apps.tri_colis.facade import ConfigFacade  # noqa: E402
from axis_os.services.manipulation.cinematique import ModeleBras  # noqa: E402
from axis_os.services.manipulation.prise import ReglagesPrise, distance_ideale  # noqa: E402
from axis_os.services.manipulation.service import Etat, Objet, posture_bras  # noqa: E402
from axis_os.testing import SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0
R = ReglagesPrise()
CFG = ConfigFacade()
BASSIN_GPU = 0.73          # « +0.17 m en hauteur » pour un carton centré à 0,90 m
X_CARTON = 4.0
Z_CARTON = sd.PLATEAU + sd.HAUTEUR_CARTON / 2


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def carton(largeur=CFG.largeur, lacet_deg=0.0, dy=0.0):
    """Le carton tel que la façade le transmet : centre perçu, dimensions de
    ConfigFacade, lacet dans le repère monde (0 : aligné sur la table)."""
    return Objet((X_CARTON, sd.Y_CARTON + dy, Z_CARTON), largeur, CFG.profondeur, CFG.hauteur,
                 lacet=math.radians(lacet_deg), bord_m=dy)


def robot_devant(d, largeur=CFG.largeur, dy=0.0, lacet_deg=0.0):
    """Robot simulé face à la table (cap +y), bassin à `d` du centre du carton."""
    r = SimulatedRobot(objects=[SimulatedObject("carton", (X_CARTON, sd.Y_CARTON + dy, Z_CARTON),
                                                size=(largeur, CFG.profondeur, CFG.hauteur))],
                       start=(X_CARTON, sd.Y_CARTON + dy - d, sd.CAP_PRISE), time_scale=2.0)
    r.PELVIS_HEIGHT_M = BASSIN_GPU
    return r


section("1. Le constat GPU : carton à 0,32 m devant le bassin, +0,17 m — atteignable")
with robot_devant(0.32) as robot:
    m = robot.manipulation
    c = m._vers_bassin(carton().centre)
    check(f"Géométrie du constat : carton à {c[0]:.2f} m devant, {c[2]:+.2f} m",
          abs(c[0] - 0.32) < 1e-6 and abs(c[2] - 0.17) < 1e-6)
    check("Lacet monde 0, robot tourné d'un quart de tour : faces latérales, écart 0°",
          abs(m._lacet_relatif(carton())) < 1e-9)
    ok, raison = m._planifier(carton())
    check("Portée : les mains atteignent le milieu des deux faces", ok, raison)
    if ok:
        cg, cd = m._plan["cibles"]
        check("… les mains visent les faces gauche et droite (pas l'avant et l'arrière)",
              abs(cg[0] - 0.32) < 1e-6 and abs(cd[0] - 0.32) < 1e-6 and cg[1] > 0 > cd[1])
for d in (0.36,):
    with robot_devant(d) as robot:
        ok, raison = robot.manipulation._planifier(carton())
        check(f"Géométrie de l'ancienne prise (carton à {d:.2f} m) : atteignable", ok, raison)
for lacet in (-12.0, 12.0):
    with robot_devant(0.32) as robot:
        m = robot.manipulation
        check(f"Carton tourné de {lacet:+.0f}° (bornes de l'évaluation) : écart {lacet:+.0f}°, "
              f"atteignable", abs(math.degrees(m._lacet_relatif(carton(lacet_deg=lacet))) - lacet)
              < 1e-6 and m._planifier(carton(lacet_deg=lacet))[0])

section("2. Distance de prise de la façade, puis prise complète")
with robot_devant(0.32) as robot:
    d, raison = robot.manipulation.distance_de_prise(carton(), d_min=CFG.distance_prise)
check(f"distance_de_prise : {d} m (à partir de {CFG.distance_prise} m)",
      d is not None and CFG.distance_prise <= d <= 0.40, raison)
with robot_devant(d) as robot:
    m = robot.manipulation
    res = m.prendre(carton())
    t0 = time.monotonic()
    while m.etat not in (Etat.TENU, Etat.ECHEC) and time.monotonic() - t0 < 20:
        time.sleep(0.02)
    check(f"Prise à {d:.3f} m : TENU", res.ok and m.etat is Etat.TENU, f"{res.reason} {m.raison}")
    check(f"… avant-bras à {100 * m._plan['garde']:.1f} cm du plateau (3 au moins)",
          m._plan["garde"] >= R.marge_plateau_m)
    check("… aucune consigne refusée par le garde-fou", robot.locomotion.stats["refused"] == 0)
for largeur in (0.30,):
    with robot_devant(0.32, largeur=largeur) as robot:
        ok, raison = robot.manipulation._planifier(carton(largeur=largeur))
        check(f"Carton de {largeur:.2f} m de large, à 0,32 m : atteignable", ok, raison)

section("3. Portée et « avant-bras à 3 cm du plateau » ne se contredisent pas (table T1)")
modele, port = ModeleBras(), posture_bras(0.20, -0.30, 0.0)
for bassin in (0.70, 0.72, 0.73, 0.75):
    for dy in (0.0, 0.05):           # carton au bord, ou reculé de 5 cm (évaluation)
        z = Z_CARTON - bassin
        d, raison = distance_ideale(modele, CFG.largeur, CFG.profondeur, CFG.hauteur, z, port,
                                    bord_m=dy, distances=np.arange(CFG.distance_prise, 0.45, 0.005))
        ok = d is not None
        if ok:
            with robot_devant(d, dy=dy) as robot:
                robot.PELVIS_HEIGHT_M = bassin
                ok, raison = robot.manipulation._planifier(carton(dy=dy))
        check(f"Bassin à {bassin:.2f} m, carton à {100 * dy:.0f} cm du bord : distance "
              f"{d if d is None else round(d, 3)} m, prise planifiée", ok, raison)

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
