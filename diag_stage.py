"""
DIAGNOSTIC — que contient RÉELLEMENT la scène USD sous /World/g1 ?

Les meshes sont présents sur le disque (vérifié par
find_missing_meshes), et pourtant les caméras rendent exactement la
même image qu'avant leur ajout, à la décimale près. Il faut donc
regarder la scène elle-même plutôt que de deviner une cinquième fois.

Ce script répond à quatre questions, chacune excluant une hypothèse :
  1. Y a-t-il des prims Mesh sous /World/g1 ?       -> import des visuels
  2. Sont-ils visibles, et avec quel « purpose » ?   -> masquage USD
  3. Où sont-ils dans l'espace (boîte englobante) ?  -> hors champ / sous le sol
  4. Quelles variantes sont sélectionnées ?          -> visuels dans une
                                                        variante non active
"""
import sys
sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app, find_missing_meshes, URDF_PATH,
)


def main():
    simulation_app = start_simulation_app(headless=True)

    # RALLUMER les diagnostics USD, masqués par défaut dans Isaac Sim.
    # C'est derrière « USD Diagnostics are currently muted » que se
    # cachaient les erreurs de fichiers introuvables — et c'est là que se
    # trouve, selon toute vraisemblance, la raison pour laquelle les STL
    # présents sur disque ne sont pas importés.
    try:
        import carb.settings
        s = carb.settings.get_settings()
        s.set("/persistent/app/usd/muteUsdDiagnostics", False)
        s.set("/app/usd/muteUsdDiagnostics", False)
        print("[0] diagnostics USD RÉACTIVÉS")
    except Exception as exc:        # noqa: BLE001
        print(f"[0] impossible de réactiver les diagnostics USD : {exc}")

    import omni.usd
    from pxr import Usd, UsdGeom
    from collections import Counter

    print(f"\n[1] meshes manquants sur disque : {len(find_missing_meshes(URDF_PATH))}")

    bridge = IsaacSimBridge()
    bridge.setup(fix_base=True)

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath("/World/g1")
    print(f"[2] /World/g1 valide : {root.IsValid()}")

    # Parcours COMPLET, y compris prims inactifs et payloads non
    # chargés — un visuel peut exister sans être chargé.
    all_types, active_types = Counter(), Counter()
    inactive = 0
    for p in Usd.PrimRange(root, Usd.PrimAllPrimsPredicate):
        all_types[p.GetTypeName() or "<sans type>"] += 1
        if not p.IsActive():
            inactive += 1
    for p in Usd.PrimRange(root):
        active_types[p.GetTypeName() or "<sans type>"] += 1

    print(f"\n[3] prims (TOUS, y compris inactifs/non chargés) :")
    for t, n in all_types.most_common(10):
        print(f"      {t:28} {n}")
    print(f"    prims inactifs : {inactive}")
    print(f"    prims Mesh ACTIFS : {active_types.get('Mesh', 0)}")

    # Visibilité et purpose des meshes actifs
    vis, purpose, points = Counter(), Counter(), []
    for p in Usd.PrimRange(root):
        if p.IsA(UsdGeom.Mesh):
            img = UsdGeom.Imageable(p)
            vis[str(img.ComputeVisibility())] += 1
            purpose[str(img.ComputePurpose())] += 1
            pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
            points.append(len(pts) if pts is not None else 0)
    print(f"\n[4] visibilité des meshes : {dict(vis)}")
    print(f"    purpose des meshes    : {dict(purpose)}")
    print(f"      (seuls 'default' et 'render' apparaissent à la caméra ;")
    print(f"       'guide' et 'proxy' sont masqués au rendu)")
    if points:
        vides = sum(1 for n in points if n == 0)
        print(f"    meshes VIDES (0 sommet) : {vides}/{len(points)}")

    # Boîte englobante dans le monde
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = cache.ComputeWorldBound(root).ComputeAlignedRange()
    if rng.IsEmpty():
        print(f"\n[5] boîte englobante : VIDE — aucune géométrie rendable")
    else:
        mn, mx = rng.GetMin(), rng.GetMax()
        print(f"\n[5] boîte englobante monde :")
        print(f"      min = ({mn[0]:+.2f}, {mn[1]:+.2f}, {mn[2]:+.2f})")
        print(f"      max = ({mx[0]:+.2f}, {mx[1]:+.2f}, {mx[2]:+.2f})")
        print(f"      hauteur = {mx[2]-mn[2]:.2f} m  (un G1 fait ~1.3 m)")
        if mx[2] < 0.05:
            print(f"      -> le robot est ENTIÈREMENT SOUS le plan de sol")

    # Variantes
    vsets = root.GetVariantSets()
    print(f"\n[6] variantes sur /World/g1 :")
    for name in vsets.GetNames():
        vs = vsets.GetVariantSet(name)
        print(f"      {name:12} options={vs.GetVariantNames()} "
              f"sélection='{vs.GetVariantSelection()}'")
    if not vsets.GetNames():
        print("      (aucune)")

    pos, _ = bridge._robot.get_world_pose()
    print(f"\n[7] pose de la base : z = {float(pos[2]):+.4f} m")
    print("      (en base fixe, le bassin d'un G1 debout est vers +0.79 m ;")
    print("       à z≈0, les jambes traversent le sol)")

    simulation_app.close()


if __name__ == "__main__":
    main()
