"""
Suite P2 — détection réelle par OWL-ViT contre le G1 simulé.

Première fois que la chaîne complète tourne avec un VRAI modèle :

    caméra Isaac Sim -> image du G1 dans la scène
        -> OWL-ViT ("a robot", "a humanoid robot")
        -> Detection avec boîte en pixels
        -> CameraIntrinsics + FrameTree
        -> direction vers le robot dans le repère de la caméra

La cible est le G1 lui-même : c'est le seul objet certain d'être dans
la scène, et sa position est connue, donc la direction calculée est
vérifiable au lieu d'être crue sur parole.

PRÉREQUIS DANS LE CONTENEUR :
    docker exec -u root isim-isaac-sim-1 /isaac-sim/python.sh \
        -m pip install transformers pillow

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \
      /isaac-sim/axis_os/test_isaac_p2_detection.py
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
    simulation_app = start_simulation_app(headless=True)

    from axis_os.hal.core.frames import (
        FRAME_BASE, FRAME_CAMERA, FrameTree, Transform,
        camera_optical_to_link,
    )
    from axis_os.hal.core.types import HALStatus, Vector3
    from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera
    from axis_os.services.perception.owlvit_detector import OwlViTDetector
    from axis_os.services.perception.service import PerceptionService

    bridge = IsaacSimBridge()
    bridge.setup(fix_base=True, require_meshes=True)
    bridge.set_holding_gains()

    section("1 — Chargement du modèle")

    detector = OwlViTDetector(threshold=0.08)
    loaded = detector.load()
    check("OWL-ViT chargé", loaded)
    if not loaded:
        print(f"\n  {detector.stats['load_error']}")
        print("  Installer : /isaac-sim/python.sh -m pip install "
              "transformers pillow")
        simulation_app.close()
        sys.exit(1)
    check(f"Exécution sur GPU (device={detector.stats['device']})",
          detector.stats["device"] == "cuda")

    section("2 — Caméra braquée sur le robot")

    # Viser le CENTRE RÉEL du robot, mesuré après placement. L'ancienne
    # version codait en dur une orientation (0,0,0,1) qui faisait
    # regarder la caméra à 90° de la cible : aucune image n'a jamais
    # contenu le robot, d'où zéro détection. Le problème n'était pas
    # OWL-ViT.
    import omni.usd
    from pxr import Usd, UsdGeom
    stage = omni.usd.get_context().get_stage()
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = bb.ComputeWorldBound(stage.GetPrimAtPath("/World/g1")).ComputeAlignedRange()
    cz = (rng.GetMin()[2] + rng.GetMax()[2]) / 2.0
    CAM_POS = (2.5, 0.0, cz + 0.2)
    cam = IsaacSimCamera("p2_cam", prim_path="/World/p2_camera",
                         width=640, height=480, position=CAM_POS)
    check("initialize() -> OK", cam.initialize() is HALStatus.OK)
    check("look_at() VÉRIFIÉ : la caméra regarde réellement le robot",
          cam.look_at((0.0, 0.0, cz)))
    check("La caméra produit des données non nulles",
          cam.wait_until_ready(bridge._world))

    tree = FrameTree()
    tree.set(Transform(FRAME_BASE, FRAME_CAMERA, Vector3(*CAM_POS)))
    tree.set(camera_optical_to_link())

    svc = PerceptionService(cam, tree, detector)
    check("PerceptionService démarré", svc.start() is HALStatus.OK)
    check("health() opérationnel", svc.health().healthy)

    section("3 — Détection réelle")

    r = svc.find("a robot")
    check("find() réussit", r.ok)
    if not r.ok:
        print(f"  raison : {r.reason}")

    dets = r.value or []
    print(f"\n  {len(dets)} détection(s) :")
    for d in dets[:5]:
        print(f"    {d.label:24} conf={d.confidence:.3f} "
              f"bbox={d.bbox} dir={d.direction}")

    check("Au moins une détection sur une scène contenant un robot",
          len(dets) > 0)

    if dets:
        d = dets[0]
        check("La détection porte une direction dans le repère robot",
              d.direction is not None)
        check("position_3d reste None — caméra RGB sans profondeur",
              d.position_3d is None)
        check("L'horodatage est celui de l'image",
              d.timestamp_ns > 0)

        x, y, w, h = d.bbox
        check(f"Boîte dans les limites de l'image ({d.bbox})",
              0 <= x < 640 and 0 <= y < 480 and w > 0 and h > 0)

        # Le G1 est à l'origine, la caméra à x=2.5 : le robot est donc
        # devant elle, donc proche du centre de l'image en horizontal.
        centre_x = x + w / 2.0
        check(f"Le robot est cadré près du centre horizontal "
              f"(centre_x={centre_x:.0f}, image 640 px)",
              200 < centre_x < 440)

    section("4 — Vocabulaire ouvert : une requête jamais entraînée")

    r2 = svc.find("a humanoid robot with two legs")
    check("Une description libre et inhabituelle est acceptée sans "
          "réentraînement — c'est la propriété qui permet à une app "
          "tierce de chercher un objet imprévu",
          r2.ok)
    print(f"    -> {len(r2.value or [])} détection(s)")

    r3 = svc.describe_scene()
    check("describe_scene() produit une description", r3.ok)
    print(f"    -> {r3.value}")

    section("5 — Coût de l'inférence")

    st = detector.stats
    print(f"    {st['inferences']} inférences, "
          f"moyenne {st['mean_ms']:.0f} ms")
    check(f"Latence mesurée et rapportée ({st['mean_ms']:.0f} ms)",
          st["inferences"] > 0)
    print(f"\n    À comparer au budget d'un cycle de commande : 20 ms.")
    print(f"    Une politique de perception NE TOURNE PAS dans la boucle")
    print(f"    de contrôle — c'est exactement pourquoi SetpointFollower")
    print(f"    interpole entre deux consignes lentes.")

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    if FAIL == 0:
        print("\n  ✓ Détection à vocabulaire ouvert sur une vraie scène 3D")
        print("  ✓ Chaîne complète image -> objet -> direction repère robot")
        print("  ⚠ Une scène simulée n'est PAS une image réelle : textures")
        print("    propres, éclairage parfait, aucun bruit de capteur")

    cam.shutdown()
    svc.stop()
    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
