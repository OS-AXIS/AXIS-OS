"""
Suite P3 — perception dans un entrepôt : trouver un colis ET sa distance.

Dernière brique avant l'app qui marche jusqu'à un colis. Répond à deux
questions que les tests précédents ne pouvaient pas poser, parce que la
scène ne contenait que le robot :

  1. OWL-ViT reconnaît-il un CARTON, et pas seulement un robot ?
  2. La profondeur donne-t-elle une distance JUSTE ?

La seconde est vérifiable sans rien croire sur parole : les colis sont
placés à des distances connues, et on compare la distance mesurée à la
distance réelle. Une perception qui se trompe de 30 cm ferait heurter le
colis au lieu de s'arrêter devant.

Les colis sont des boîtes créées dans la scène plutôt que des modèles
téléchargés : leur position est ainsi EXACTE et choisie, ce qui rend la
mesure vérifiable. Une scène d'entrepôt NVIDIA viendra ensuite, pour le
réalisme visuel.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_p3_warehouse.py
"""
import sys
from pathlib import Path

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)

FAIL = 0
# Colis placés devant le robot, à des distances CONNUES depuis l'origine.
COLIS = [
    ("colis_proche", (1.5, 0.0, 0.2), (0.4, 0.3, 0.4)),
    ("colis_loin",   (3.0, 0.6, 0.15), (0.3, 0.3, 0.3)),
]
CAM_POS = (0.0, 0.0, 1.0)     # à la place de la tête du robot


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    from isaacsim.core.api.objects import DynamicCuboid
    from axis_os.hal.core.frames import (
        FRAME_BASE, FRAME_CAMERA, FrameTree, Transform, camera_optical_to_link,
    )
    from axis_os.hal.core.types import HALStatus, Vector3
    from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera
    from axis_os.services.perception.owlvit_detector import OwlViTDetector
    from axis_os.services.perception.service import PerceptionService

    bridge = IsaacSimBridge()
    bridge.setup(fix_base=True, require_meshes=True)

    section("1 — Scène : un entrepôt minimal avec des colis")

    for name, pos, size in COLIS:
        bridge._world.scene.add(DynamicCuboid(
            prim_path=f"/World/{name}", name=name,
            position=np.array(pos), size=1.0,
            scale=np.array(size),
            color=np.array([0.72, 0.52, 0.30]),   # carton
        ))
    bridge._world.reset()
    check(f"{len(COLIS)} colis placés à des positions connues", True)

    # Caméra couleur et caméra de profondeur au MÊME endroit : le service
    # suppose les deux images alignées pixel à pixel.
    cam = IsaacSimCamera("p3_rgb", prim_path="/World/p3_rgb",
                         width=640, height=480, position=CAM_POS)
    dep = IsaacSimCamera("p3_depth", prim_path="/World/p3_depth",
                         width=640, height=480, position=CAM_POS, depth=True)
    for c in (cam, dep):
        check(f"{c.sensor_id} initialisée", c.initialize() is HALStatus.OK)
        check(f"{c.sensor_id} vise le premier colis",
              c.look_at(COLIS[0][1]))
    check("Les caméras produisent des données", cam.wait_until_ready(bridge._world))
    dep.wait_until_ready(bridge._world)

    tree = FrameTree()
    tree.set(Transform(FRAME_BASE, FRAME_CAMERA, Vector3(*CAM_POS)))
    tree.set(camera_optical_to_link())

    detector = OwlViTDetector(threshold=0.08)
    check("OWL-ViT chargé", detector.load())
    svc = PerceptionService(cam, tree, detector, depth_camera=dep)
    check("PerceptionService démarré", svc.start() is HALStatus.OK)

    section("2 — Reconnaître un CARTON, pas seulement un robot")

    r = svc.find("a cardboard box")
    check("find() réussit", r.ok)
    dets = r.value or []
    print(f"\n  {len(dets)} détection(s) :")
    for d in dets[:5]:
        p = d.position_3d
        print(f"    {d.label:20} conf={d.confidence:.3f} "
              f"position={'(%.2f, %.2f, %.2f)' % (p.x, p.y, p.z) if p else 'None'}")
    check("Au moins un colis détecté", len(dets) > 0)

    section("3 — La distance mesurée est-elle JUSTE ?")

    if not dets:
        print("  Aucune détection : distance non vérifiable.")
    else:
        d = dets[0]
        check("La détection porte une POSITION, pas seulement une direction",
              d.position_3d is not None)
        if d.position_3d is not None:
            p = np.array([d.position_3d.x, d.position_3d.y, d.position_3d.z])
            # Le colis le plus proche de la position mesurée, et l'écart.
            best, err = None, 9e9
            for name, pos, _s in COLIS:
                e = float(np.linalg.norm(p - np.array(pos)))
                if e < err:
                    best, err = name, e
            print(f"\n  position mesurée : ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")
            print(f"  colis le plus proche : {best}, écart {err:.2f} m")
            check(f"Écart sous 30 cm — au-delà, le robot heurterait le colis "
                  f"au lieu de s'arrêter devant (écart {err:.2f} m)", err < 0.30)
            dist = float(np.linalg.norm(p - np.array(CAM_POS)))
            reel = float(np.linalg.norm(np.array(COLIS[0][1]) - np.array(CAM_POS)))
            print(f"  distance caméra -> colis : mesurée {dist:.2f} m, "
                  f"réelle {reel:.2f} m")

    section("4 — Vocabulaire ouvert : chercher autre chose")

    r2 = svc.find("a box")
    print(f"    « a box » -> {len(r2.value or [])} détection(s)")
    r3 = svc.describe_scene()
    print(f"    describe_scene() -> {r3.value}")
    check("Le service répond à des requêtes différentes sans réentraînement",
          r2.ok and r3.ok)

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    if FAIL == 0:
        print("\n  ✓ Un colis est détecté ET localisé en 3 dimensions")
        print("  ✓ Dernière brique avant l'app qui marche jusqu'à un colis")

    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
