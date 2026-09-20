"""
DIAGNOSTIC — la caméra regarde-t-elle le robot, et que voit OWL-ViT ?

Zéro détection sans erreur d'inférence laisse deux possibilités, et
on ne peut pas les départager sans regarder l'image elle-même :
  A. la caméra ne cadre pas le robot (orientation du prim)
  B. elle le cadre, mais le modèle n'y reconnaît rien

Ce script sauvegarde l'image, mesure son contenu, et interroge le
modèle avec un seuil très bas et plusieurs formulations.
"""
import sys
sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)


def main():
    simulation_app = start_simulation_app(headless=True)
    from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera
    from axis_os.services.perception.owlvit_detector import OwlViTDetector
    import numpy as np

    bridge = IsaacSimBridge()
    bridge.setup(fix_base=True)
    bridge.set_holding_gains()

    # Position du robot dans la scène, pour orienter la caméra vers lui.
    pos, _ = bridge._robot.get_world_pose()
    print(f"\n[diag] pose du robot dans le monde : {pos}")

    detector = OwlViTDetector(threshold=0.01)   # seuil très bas
    detector.load()

    # Plusieurs points de vue : on cherche lequel cadre le robot.
    vues = [
        ("face_2m5",  (2.5, 0.0, 0.9),  None),
        ("face_1m5",  (1.5, 0.0, 0.9),  None),
        ("cote",      (0.0, 2.0, 0.9),  None),
        ("haut",      (1.5, 0.0, 2.0),  None),
    ]

    for nom, p, orient in vues:
        cam = IsaacSimCamera(f"diag_{nom}", prim_path=f"/World/diag_{nom}",
                             width=640, height=480, position=p,
                             orientation=orient)
        if cam.initialize().name != "OK":
            print(f"  [{nom}] initialisation échouée"); continue
        # ORIENTER vers le robot : sans ça le prim garde son axe par
        # défaut et regarde ailleurs. C'est ce qui rendait les quatre
        # points de vue indiscernables.
        ok_aim = cam.look_at((float(pos[0]), float(pos[1]), 0.7))
        print(f"  [{nom}] look_at -> {ok_aim}")
        cam.wait_until_ready(bridge._world)
        f = cam.capture()
        if f is None:
            print(f"  [{nom}] aucune image"); continue

        arr = np.frombuffer(f.data, dtype=np.uint8).reshape(f.height, f.width, 3)
        # Un fond uniforme a un ecart-type quasi nul ; un robot visible
        # cree du contraste.
        std = float(arr.std())
        moy = float(arr.mean())
        # Combien de pixels s'ecartent nettement du fond ?
        # CRITÈRE CORRIGÉ. Le précédent comparait chaque pixel à la
        # MÉDIANE de l'image : sur un simple dégradé de fond, la moitié
        # des pixels s'en écartent mécaniquement, d'où les 45 % mesurés
        # sur quatre images pourtant vides. On cherche plutôt des
        # pixels SOMBRES, puisqu'un G1 non texturé se détache d'un fond
        # clair (moyenne ~192).
        lum = arr.mean(axis=2)
        pct_objet = float((lum < 120).mean() * 100)
        # Et on compare les moitiés gauche/droite : un fond dégradé est
        # symétrique, un robot décentré ne l'est pas.
        g, d_ = lum[:, :320].mean(), lum[:, 320:].mean()
        asym = abs(float(g - d_))

        dets = detector.detect(f, "a robot")
        top = max((d.confidence for d in dets), default=0.0)

        print(f"\n  [{nom}] pos={p}")
        print(f"     moyenne={moy:6.1f}  ecart-type={std:6.1f}  "
              f"pixels sombres={pct_objet:5.1f}%  asym.G/D={asym:5.1f}")
        print(f"     -> {len(dets)} detection(s), meilleure conf={top:.4f}")

        # Sauver l'image pour inspection visuelle.
        try:
            from PIL import Image
            out = f"/isaac-sim/diag_{nom}.png"
            Image.fromarray(arr).save(out)
            print(f"     image ecrite : {out}")
        except Exception as e:
            print(f"     sauvegarde impossible : {e}")

        cam.shutdown()

    print("\n[diag] INTERPRETATION")
    print("  Si les 4 vues donnent des chiffres IDENTIQUES : les cameras")
    print("  ne regardent pas la scene (look_at inoperant).")
    print("  Si elles different et que pixels sombres > 3% : le robot est")
    print("  cadre. Zero detection signifie alors que le modele ne")
    print("  reconnait pas un G1 non texture sur fond vide.")
    simulation_app.close()


if __name__ == "__main__":
    main()
