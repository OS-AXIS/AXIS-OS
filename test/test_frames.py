"""
Repères et transformations — la brique qui relie perception et action.

Ce que ce test protège : la chaîne pixel -> direction dans le repère du
robot. Une erreur ici ne lève AUCUNE exception — elle produit un vecteur
parfaitement valide qui pointe dans la mauvaise direction, et c'est la
préhension qui tendra la main à côté.
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.frames import (
    FRAME_BASE, FRAME_CAMERA, FRAME_CAMERA_OPTICAL, FRAME_WORLD,
    FrameTree, Transform, camera_optical_to_link,
)
from axis_os.hal.core.types import CameraIntrinsics, Quaternion, Vector3

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def near(a, b, tol=1e-6):
    return abs(a - b) < tol


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


section("1 — Transformation rigide")

t = Transform(FRAME_BASE, FRAME_CAMERA, Vector3(0.1, 0.0, 0.5))
p = t.apply(Vector3(1.0, 0.0, 0.0))
check("apply() ajoute bien la translation",
      near(p.x, 1.1) and near(p.z, 0.5))

d = t.apply_direction(Vector3(1.0, 0.0, 0.0))
check("apply_direction() N'applique PAS la translation — une direction "
      "n'a pas de position",
      near(d.x, 1.0) and near(d.y, 0.0) and near(d.z, 0.0))

back = t.inverse().apply(p)
check("inverse() puis apply() redonne le point de départ",
      near(back.x, 1.0) and near(back.y, 0.0) and near(back.z, 0.0))

h = math.sqrt(2.0) / 2.0
r = Transform(FRAME_BASE, "turret", Vector3(), Quaternion(w=h, x=0, y=0, z=h))
rp = r.apply(Vector3(1.0, 0.0, 0.0))
check("Rotation de 90° autour de z : (1,0,0) -> (0,1,0)",
      near(rp.x, 0.0, 1e-9) and near(rp.y, 1.0, 1e-9))

section("2 — Arbre de repères : un seul parent par repère")

tree = FrameTree()
tree.set(Transform(FRAME_BASE, FRAME_CAMERA, Vector3(0.1, 0.0, 0.5)))
tree.set(camera_optical_to_link())
check("Les trois repères sont connus",
      set(tree.frames()) == {FRAME_BASE, FRAME_CAMERA, FRAME_CAMERA_OPTICAL})

try:
    tree.set(Transform(FRAME_WORLD, FRAME_CAMERA, Vector3()))
    ok = False
except ValueError:
    ok = True
check("Rattacher un repère à un second parent est refusé (arbre, pas graphe)",
      ok)

try:
    tree.set(Transform("a", "a", Vector3()))
    ok = False
except ValueError:
    ok = True
check("Un repère parent de lui-même est refusé", ok)

_tf, warn = tree.lookup(FRAME_BASE, "repere_inconnu")
check("Deux repères non reliés : réponse None + explication",
      _tf is None and warn is not None)

section("3 — Chaîne complète : pixel -> direction dans le repère du robot")

# C'est LE cas d'usage : la perception détecte un objet en pixels,
# la préhension a besoin d'une direction dans le repère du robot.
k = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
tf, warn = tree.lookup(FRAME_BASE, FRAME_CAMERA_OPTICAL)
check("La chaîne base_link <- camera_optical est résolue",
      tf is not None and warn is None)

centre = tf.apply_direction(k.pixel_to_ray(320.0, 240.0))
check("Centre de l'image -> droit devant le robot (x>0, y≈0, z≈0)",
      centre.x > 0.99 and near(centre.y, 0.0, 1e-9) and near(centre.z, 0.0, 1e-9))

droite = tf.apply_direction(k.pixel_to_ray(620.0, 240.0))
check("Pixel à DROITE dans l'image -> y NÉGATIF (y pointe à gauche en "
      "convention robotique)",
      droite.y < -0.1 and droite.x > 0)

haut = tf.apply_direction(k.pixel_to_ray(320.0, 40.0))
check("Pixel en HAUT dans l'image -> z POSITIF (y pointe en bas en "
      "convention optique : la conversion est bien appliquée)",
      haut.z > 0.1 and haut.x > 0)

check("camera_optical_to_link() n'est PAS l'identité — l'oublier ferait "
      "pointer la préhension à côté sans aucune erreur visible",
      not near(camera_optical_to_link().rotation.w, 1.0, 1e-3))

section("3b — Deux repères FRÈRES sous un ancêtre commun")

# Cas réel qui a cassé : robot et caméra tous deux enfants du monde,
# au lieu d'une caméra descendant du robot. La version précédente
# composait dans le mauvais ordre et levait « le repère intermédiaire
# ne correspond pas ».
FRAME_WORLD_ = FRAME_WORLD
fr = FrameTree()
# Robot à x=2, tourné de 90° autour de z (il regarde vers +y du monde).
h = math.sqrt(2.0) / 2.0
fr.set(Transform(FRAME_WORLD_, FRAME_BASE, Vector3(2.0, 0.0, 0.0),
                 Quaternion(w=h, x=0, y=0, z=h)))
# Caméra à x=3 dans le monde, sans rotation.
fr.set(Transform(FRAME_WORLD_, "cam", Vector3(3.0, 0.0, 0.5)))

tf, warn = fr.lookup(FRAME_BASE, "cam")
check("Deux frères sous le monde : la composition aboutit",
      tf is not None and warn is None)

# La caméra est 1 m devant le robot selon +x du monde. Le robot étant
# tourné de 90°, ce +x du monde est son -y à lui. Un point à l'origine
# de la caméra doit donc apparaître en (0, -1, 0.5) dans le repère du
# robot.
p_base = tf.apply(Vector3(0.0, 0.0, 0.0))
check(f"Position convertie correctement dans le repère du robot "
      f"({p_base.x:+.2f}, {p_base.y:+.2f}, {p_base.z:+.2f}) — attendu "
      f"(0, -1, +0.5)",
      near(p_base.x, 0.0, 1e-6) and near(p_base.y, -1.0, 1e-6)
      and near(p_base.z, 0.5, 1e-6))

# Et l'inverse doit ramener au point de départ.
back, _ = fr.lookup("cam", FRAME_BASE)
p_cam = back.apply(p_base)
check("La transformation inverse ramène au point de départ",
      near(p_cam.x, 0.0, 1e-6) and near(p_cam.y, 0.0, 1e-6)
      and near(p_cam.z, 0.0, 1e-6))

check("Un repère vers la racine fonctionne aussi",
      fr.lookup(FRAME_WORLD_, FRAME_BASE)[0] is not None)
check("Et depuis la racine",
      fr.lookup(FRAME_BASE, FRAME_WORLD_)[0] is not None)

section("4 — Fraîcheur : une pose périmée se signale")

now = time.time_ns()
fresh = FrameTree(max_age_s=0.5)
fresh.set(Transform(FRAME_BASE, "bras", Vector3(), timestamp_ns=now))
_t, w = fresh.lookup(FRAME_BASE, "bras", now_ns=now)
check("Transformation fraîche : aucun avertissement", w is None)

_t, w = fresh.lookup(FRAME_BASE, "bras", now_ns=now + int(2e9))
check("Transformation vieille de 2s : avertissement émis, réponse quand "
      "même rendue (l'appelant décide)",
      w is not None and _t is not None)

statique = FrameTree(max_age_s=0.5)
statique.set(Transform(FRAME_BASE, FRAME_CAMERA, Vector3()))   # timestamp 0
_t, w = statique.lookup(FRAME_BASE, FRAME_CAMERA, now_ns=now + int(9e9))
check("Un montage mécanique fixe (timestamp 0) ne périme JAMAIS", w is None)

# L'âge d'une chaîne est celui de son maillon le plus ancien.
vieux = Transform(FRAME_BASE, "b", Vector3(), timestamp_ns=now - int(3e9))
recent = Transform("b", "c", Vector3(), timestamp_ns=now)
check("Une pose composée prend l'âge du maillon le PLUS ANCIEN",
      vieux.compose(recent).timestamp_ns == now - int(3e9))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
if FAIL == 0:
    print("\n  ✓ pixel -> direction dans le repère robot, physiquement correct")
    print("  ✓ Arbre strict : un parent par repère, cycles refusés")
    print("  ✓ Poses périmées signalées, montages fixes jamais périmés")

sys.exit(0 if FAIL == 0 else 1)
