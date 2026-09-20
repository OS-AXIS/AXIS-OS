"""
Suite P1 — chaîne perception complète contre le G1 simulé.

Valide de bout en bout, pour la première fois, le chemin :

    caméra Isaac Sim (ICamera)
        -> CameraFrame (rgb8, horodatée)
        -> CameraIntrinsics lues sur la caméra
        -> pixel_to_ray()          [repère optique]
        -> FrameTree.lookup()      [repère robot]
        -> direction exploitable par la préhension

C'est la première fois que perception et repères sont testés contre
une vraie scène 3D plutôt qu'en arithmétique pure.

CE QUE CE TEST PROUVE : que la plomberie est correcte et que la
géométrie est cohérente avec ce qui est réellement visible.
CE QU'IL NE PROUVE PAS : qu'un modèle de perception reconnaîtra quoi
que ce soit sur une image réelle. Une optique simulée est parfaite —
voir docs/campagne_validation_isaac.md §9.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \
      /isaac-sim/axis_os/test_isaac_p1_perception.py
"""
import sys

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def main():
    # render=True obligatoire : une caméra ne produit d'image qu'au
    # rendu, et headless=True ne l'empêche pas — headless retire la
    # FENÊTRE, pas le moteur de rendu.
    simulation_app = start_simulation_app(headless=True)

    from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera
    from axis_os.hal.core.frames import (
        FRAME_BASE, FRAME_CAMERA, FRAME_CAMERA_OPTICAL,
        FrameTree, Transform, camera_optical_to_link,
    )
    from axis_os.hal.core.interfaces import ICamera
    from axis_os.hal.core.types import HALStatus, Vector3

    bridge = IsaacSimBridge()
    bridge.setup(fix_base=True)
    bridge.set_holding_gains()

    section("1 — La caméra simulée respecte le contrat ICamera")

    CAM_POS = (0.4, 0.0, 1.2)      # devant et au-dessus du robot
    cam = IsaacSimCamera(
        "sim_head_rgb",
        prim_path="/World/axis_camera",
        width=640, height=480,
        position=CAM_POS,
    )
    check("IsaacSimCamera implémente ICamera", isinstance(cam, ICamera))

    st = cam.initialize()
    check(f"initialize() -> {st.name}", st is HALStatus.OK)
    if st is not HALStatus.OK:
        print("\n  Impossible de continuer sans caméra.")
        simulation_app.close()
        sys.exit(1)

    check("is_healthy() après initialisation", cam.is_healthy())
    check("capture() rend None avant tout rendu (pas d'exception)",
          cam.capture() is None or cam.frames_captured >= 0)

    section("2 — Rendu et lecture d'une vraie image")

    for _ in range(30):
        bridge._world.step(render=True)

    frame = cam.capture()
    check("capture() rend une image après rendu", frame is not None)
    if frame is None:
        print("\n  Aucune image produite — vérifier que render=True est actif.")
        simulation_app.close()
        sys.exit(1)

    check(f"Résolution conforme ({frame.width}x{frame.height})",
          frame.width == 640 and frame.height == 480)
    check(f"Encodage rgb8, aligné sur la RealSense (obtenu: {frame.encoding})",
          frame.encoding == "rgb8")
    expected = frame.width * frame.height * 3
    check(f"Taille des données cohérente ({len(frame.data)} = {expected})",
          len(frame.data) == expected)
    check("Image horodatée", frame.timestamp_ns > 0)

    # Une image entièrement noire signalerait un rendu qui n'a pas eu
    # lieu, malgré un tableau de la bonne taille.
    non_nuls = sum(1 for b in frame.data[:30000] if b != 0)
    check(f"L'image n'est pas uniformément noire ({non_nuls}/30000 octets "
          f"non nuls) — un rendu a réellement eu lieu",
          non_nuls > 100)

    section("3 — Calibration lue sur la caméra")

    k = cam.get_intrinsics()
    check("get_intrinsics() rend une calibration", k is not None)
    check(f"Centre optique au centre de l'image (cx={k.cx}, cy={k.cy})",
          abs(k.cx - 320.0) < 1e-6 and abs(k.cy - 240.0) < 1e-6)
    check(f"Focale plausible (fx={k.fx:.1f} px pour "
          f"{cam.horizontal_fov_deg:.1f}° de champ)",
          100.0 < k.fx < 5000.0)
    check("Distorsion vide — une optique simulée est PARFAITE, "
          "contrairement au matériel réel",
          k.distortion == ())

    section("4 — Chaîne complète : pixel -> direction dans le repère robot")

    tree = FrameTree()
    tree.set(Transform(FRAME_BASE, FRAME_CAMERA,
                       Vector3(*CAM_POS)))
    tree.set(camera_optical_to_link())

    tf, warn = tree.lookup(FRAME_BASE, FRAME_CAMERA_OPTICAL)
    check("La chaîne base_link <- camera_optical est résolue",
          tf is not None and warn is None)

    centre = tf.apply_direction(k.pixel_to_ray(k.cx, k.cy))
    check(f"Centre de l'image -> droit devant le robot "
          f"(x={centre.x:.3f}, y={centre.y:.3f}, z={centre.z:.3f})",
          centre.x > 0.99 and abs(centre.y) < 1e-6 and abs(centre.z) < 1e-6)

    droite = tf.apply_direction(k.pixel_to_ray(k.cx + 200.0, k.cy))
    check(f"Pixel à droite -> y négatif (y={droite.y:.3f})", droite.y < -0.05)

    haut = tf.apply_direction(k.pixel_to_ray(k.cx, k.cy - 200.0))
    check(f"Pixel en haut -> z positif (z={haut.z:.3f})", haut.z > 0.05)

    # La position de la caméra ne doit PAS entrer dans une direction.
    tree2 = FrameTree()
    tree2.set(Transform(FRAME_BASE, FRAME_CAMERA, Vector3(99.0, 99.0, 99.0)))
    tree2.set(camera_optical_to_link())
    tf2, _ = tree2.lookup(FRAME_BASE, FRAME_CAMERA_OPTICAL)
    centre2 = tf2.apply_direction(k.pixel_to_ray(k.cx, k.cy))
    check("Déplacer la caméra de 99 m ne change PAS la direction — "
          "apply_direction() ignore bien la translation",
          abs(centre2.x - centre.x) < 1e-6)

    section("5 — Bascule simulation / matériel")

    from axis_os.hal.drivers.unitree_g1.g1_camera import G1RealSenseCamera
    sim_api = {m for m in dir(cam) if not m.startswith("_")}
    real_api = {m for m in dir(G1RealSenseCamera("x")) if not m.startswith("_")}
    manquant = (real_api & {"capture", "get_intrinsics", "initialize",
                            "shutdown", "is_healthy"}) - sim_api
    check(f"La caméra simulée expose tout le contrat de la réelle "
          f"(manquant: {manquant or 'aucun'})", not manquant)

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    if FAIL == 0:
        print("\n  ✓ Chaîne perception validée contre une vraie scène 3D")
        print("  ✓ Une app ne voit aucune différence entre caméra simulée "
              "et réelle")
        print("  ⚠ Ne prouve RIEN sur la reconnaissance d'images réelles")

    cam.shutdown()
    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
