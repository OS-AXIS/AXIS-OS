"""
DIAGNOSTIC DIFFÉRENTIEL — le robot est-il dans l'image, oui ou non ?

Toutes les tentatives précédentes interprétaient UNE image avec des
seuils (pixels sombres, écart à la médiane…). Chacune a pu se tromper
selon la couleur du robot, l'éclairage ou le fond.

Ici on rend DEUX fois la même vue — robot visible, puis robot masqué —
et on compare. Un pixel qui change est un pixel du robot, par
construction. Aucun seuil de couleur, aucune hypothèse sur l'éclairage.

Et on lit l'orientation réelle de la caméra dans la scène : une
caméra USD regarde selon -Z avec +Y vers le haut de l'image. Si son
axe « haut » est horizontal dans le monde, elle est couchée de 90°.
"""
import sys
sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)


def ascii_mask(mask, step_y=20, step_x=10):
    rows = []
    for y in range(0, mask.shape[0], step_y):
        rows.append("".join("#" if mask[y, x] else "." for x in range(0, mask.shape[1], step_x)))
    return "\n".join(rows)


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    import omni.usd
    from pxr import Gf, Usd, UsdGeom
    from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera

    bridge = IsaacSimBridge()
    bridge.setup(fix_base=True, require_meshes=True)
    stage = omni.usd.get_context().get_stage()
    robot = stage.GetPrimAtPath("/World/g1")

    # Viser le CENTRE RÉEL de la géométrie du robot, mesuré après
    # placement — pas une hauteur supposée. C'est exactement ce genre de
    # supposition (viser z=0.7 au-dessus d'un robot enterré) qui a
    # faussé les essais précédents.
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = bb.ComputeWorldBound(robot).ComputeAlignedRange()
    mn, mx = rng.GetMin(), rng.GetMax()
    cz = (mn[2] + mx[2]) / 2.0
    print(f"\n[0] robot : z de {mn[2]:+.2f} à {mx[2]:+.2f} m "
          f"(pieds au sol si le minimum est proche de 0)")
    TARGET = (0.0, 0.0, cz)
    # 2.5 m : à 36° de champ vertical, on voit ~1.6 m de haut, assez pour
    # un G1 de 1.33 m en entier.
    cam = IsaacSimCamera("diff_cam", prim_path="/World/diff_cam",
                         width=640, height=480, position=(2.5, 0.0, cz + 0.2))
    cam.initialize()
    ok = cam.look_at(TARGET)
    print(f"[1] look_at -> {ok}  {getattr(cam, '_last_look_at', '')}")

    # --- Orientation réelle de la caméra ------------------------------
    cam_prim = stage.GetPrimAtPath("/World/diff_cam")
    m = UsdGeom.Xformable(cam_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())
    fwd = m.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
    up = m.TransformDir(Gf.Vec3d(0, 1, 0)).GetNormalized()
    print(f"[2] axe de visée dans le monde : ({fwd[0]:+.2f}, {fwd[1]:+.2f}, {fwd[2]:+.2f})")
    print(f"      attendu : vers la cible, soit environ (-1.00, +0.00, -0.08)")
    print(f"    axe « haut » de l'image      : ({up[0]:+.2f}, {up[1]:+.2f}, {up[2]:+.2f})")
    # Seuil RESSERRÉ : l'ancien (|haut.z| > 0.3) a déclaré « correcte »
    # une caméra dont le haut valait (+0.93, 0, +0.37), c'est-à-dire
    # presque horizontal. Une caméra droite a son haut franchement
    # vertical.
    if up[2] > 0.8:
        print(f"      -> orientation correcte : le haut de l'image pointe vers le ciel")
    else:
        print(f"      -> CAMÉRA MAL ORIENTÉE : son haut n'est pas vertical")

    # --- Image AVEC le robot ------------------------------------------
    cam.wait_until_ready(bridge._world)
    for _ in range(10):
        bridge._world.step(render=True)
    fa = cam.capture()
    a = np.frombuffer(fa.data, dtype=np.uint8).reshape(fa.height, fa.width, 3).astype(int)

    # --- Image SANS le robot ------------------------------------------
    UsdGeom.Imageable(robot).MakeInvisible()
    for _ in range(15):
        bridge._world.step(render=True)
    fb = cam.capture()
    b = np.frombuffer(fb.data, dtype=np.uint8).reshape(fb.height, fb.width, 3).astype(int)
    UsdGeom.Imageable(robot).MakeVisible()

    diff = np.abs(a - b).sum(axis=2)
    mask = diff > 30
    pct = float(mask.mean() * 100)
    print(f"\n[3] pixels qui CHANGENT quand on masque le robot : {pct:.2f} %")
    if pct < 0.1:
        print("      -> le robot N'APPARAÎT PAS dans l'image")
    else:
        ys, xs = np.where(mask)
        print(f"      -> le robot EST dans l'image : x de {xs.min()} à {xs.max()}, "
              f"y de {ys.min()} à {ys.max()}")
        print(f"\n[4] silhouette du robot (# = pixel du robot) :")
        print(ascii_mask(mask))

    simulation_app.close()


if __name__ == "__main__":
    main()
