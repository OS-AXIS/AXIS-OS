"""
Position d'un carton d'après la caméra de profondeur, sur des cartes de
profondeur synthétiques (lancer de rayons : sol, table, carton tourné ou
non), avec la géométrie de la caméra de tête du banc T1.

Constat GPU (2026-10-29) : carton perçu 2 à 5 cm trop près (x perçu 0.319
contre 0.349 réel, repère du bassin), et ~2 cm trop haut. Cause : la
profondeur du quart le plus proche de la boîte (le HAUT de la face avant,
pour une caméra qui regarde vers le bas) était portée le long du rayon du
CENTRE de la boîte ; une boîte qui déborde sur le bord de la table
aggravait l'écart. PerceptionService._point_de_face reporte chaque pixel le
long de son propre rayon et prend la face avant.
"""
import itertools
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.frames import Quaternion, Transform, Vector3  # noqa: E402
from axis_os.hal.core.types import CameraIntrinsics  # noqa: E402
from axis_os.services.perception.service import PerceptionService  # noqa: E402

FAIL = 0
K = CameraIntrinsics(fx=550.0, fy=550.0, cx=320.0, cy=240.0)
W, H = 640, 480
# Caméra de tête de T1 : 0,2 m devant et 0,4 m au-dessus du bassin (0,73 m),
# visant 1,5 m devant, 5 cm au-dessus du bassin. Repère du robot = monde.
CAM = np.array([0.2, 0.0, 1.13])
CIBLE = np.array([1.5, 0.0, 0.78])
PROF, LARG, HAUT = 0.20, 0.24, 0.20         # carton de la scène T1
Z_CENTRE = 0.90                              # sur un plateau à 0,80 m


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def rotation_camera():
    f = CIBLE - CAM
    f /= np.linalg.norm(f)
    r = np.cross(f, [0.0, 0.0, 1.0])
    r /= np.linalg.norm(r)
    return np.stack([r, np.cross(f, r), f], 1)       # colonnes : x droite, y bas, z avant


def quaternion(m):
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2.0, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2.0, m[1, 0] - m[0, 1])
    return Quaternion(w=w, x=x, y=y, z=z)


ROT = rotation_camera()
TF = Transform("world", "camera_optical", translation=Vector3(*CAM), rotation=quaternion(ROT))


def touche_boite(c, d, demi):
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (-demi - c) / d, (demi - c) / d
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    return np.where((tmax >= tmin) & (tmax > 0), np.where(tmin > 0, tmin, np.inf), np.inf)


def rendre(objets):
    """Carte de profondeur (distance au plan image) : objets (centre,
    demi-tailles, lacet), et le sol."""
    v, u = np.mgrid[0:H, 0:W]
    ray = np.stack([(u - K.cx) / K.fx, (v - K.cy) / K.fy, np.ones((H, W))], -1).reshape(-1, 3)
    d = ray @ ROT.T
    s = np.full(len(d), np.inf)
    for c, demi, lacet in objets:
        cz, sz = math.cos(lacet), math.sin(lacet)
        rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1.0]])
        s = np.minimum(s, touche_boite((CAM - np.asarray(c)) @ rz, d @ rz, np.asarray(demi)))
    with np.errstate(divide="ignore", invalid="ignore"):
        s = np.minimum(s, np.where(d[:, 2] < 0, -CAM[2] / d[:, 2], np.inf))
    return s.reshape(H, W)


def boite_detection(c, demi, lacet, marge, bas=0):
    cz, sz = math.cos(lacet), math.sin(lacet)
    us, vs = [], []
    for a, b, e in itertools.product((-1, 1), repeat=3):
        x, y = a * demi[0], b * demi[1]
        p = np.array([c[0] + cz * x - sz * y, c[1] + sz * x + cz * y, c[2] + e * demi[2]])
        q = (p - CAM) @ ROT
        us.append(K.fx * q[0] / q[2] + K.cx)
        vs.append(K.fy * q[1] / q[2] + K.cy)
    return (min(us) - marge, min(vs) - marge, max(us) - min(us) + 2 * marge,
            max(vs) - min(vs) + 2 * marge + bas)


def ancien(prof, bbox):
    """L'estimateur d'avant : profondeur proche, portée le long du rayon du centre."""
    dist = PerceptionService._depth_of_object(prof, bbox)
    ray = K.pixel_to_ray(bbox[0] + bbox[2] / 2.0, bbox[1] + bbox[3] / 2.0)
    p = TF.apply(Vector3(ray.x * dist, ray.y * dist, ray.z * dist))
    return np.array([p.x, p.y, p.z])


def nouveau(prof, bbox):
    p = PerceptionService._point_de_face(prof, bbox, K, TF)
    return None if p is None else np.array([p.x, p.y, p.z])


print("\n  Cartons sur la table de T1 : 0,75 à 1,4 m, de face ou décalés, tournés de ±12°")
pire = {"ancien": np.zeros(3), "nouveau": np.zeros(3)}
cas = 0
for dist, lat, lacet, recul, bas in itertools.product((0.75, 0.9, 1.1, 1.4), (0.0, 0.15, 0.3),
                                                      (-12.0, 0.0, 12.0), (0.0, 0.05), (0, 30)):
    centre = (dist, lat, Z_CENTRE)
    demi = (PROF / 2, LARG / 2, HAUT / 2)
    plateau = ((dist - PROF / 2 - recul + 0.3, 0.0, 0.78), (0.3, 1.0, 0.02), 0.0)
    prof = rendre([(centre, demi, math.radians(lacet)), plateau])
    bbox = boite_detection(centre, demi, math.radians(lacet), 8, bas)
    if bbox[0] < 0 or bbox[1] < 0 or bbox[0] + bbox[2] > W or bbox[1] + bbox[3] > H:
        continue
    cas += 1
    for nom, p in (("ancien", ancien(prof, bbox)), ("nouveau", nouveau(prof, bbox))):
        # La façade ajoute une demi-profondeur dans la direction du regard.
        e = np.abs(np.array([p[0] + PROF / 2 - dist, p[1] - lat, p[2] - Z_CENTRE]))
        pire[nom] = np.maximum(pire[nom], e)
a, n = 100 * pire["ancien"], 100 * pire["nouveau"]
print(f"  {cas} cas ; pire écart (cm) en profondeur / latéral / hauteur : "
      f"avant {a[0]:.1f} / {a[1]:.1f} / {a[2]:.1f}, maintenant {n[0]:.1f} / {n[1]:.1f} / {n[2]:.1f}")
check("Le constat est reproduit : l'estimateur d'avant voit le carton trop près (plus de 2 cm)",
      pire["ancien"][0] > 0.02, f"{a[0]:.1f} cm")
check("Profondeur : à moins de 1,5 cm dans tous les cas", pire["nouveau"][0] <= 0.015, f"{n[0]:.1f}")
check("Hauteur : à moins de 2,5 cm, et mieux qu'avant", pire["nouveau"][2] <= 0.025
      and pire["nouveau"][2] < pire["ancien"][2], f"{n[2]:.1f} contre {a[2]:.1f}")
check("Latéral : pas pire qu'avant", pire["nouveau"][1] <= pire["ancien"][1] + 1e-3,
      f"{n[1]:.1f} contre {a[1]:.1f}")

print("\n  Cas limites")
vide = np.full((H, W), np.nan)
check("Aucune profondeur valide : None, jamais une position par défaut",
      PerceptionService._point_de_face(vide, (100, 100, 50, 50), K, TF) is None)
check("Boîte hors image : None", PerceptionService._point_de_face(
    rendre([]), (900, 900, 20, 20), K, TF) is None)

print(f"\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}")
sys.exit(1 if FAIL else 0)
