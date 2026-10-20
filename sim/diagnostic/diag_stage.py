"""
Diagnostic — que contient réellement la scène USD sous /World/g1 ?

Les meshes sont présents sur le disque (vérifié par
find_missing_meshes), et pourtant les caméras rendent exactement la
même image qu'avant leur ajout, à la décimale près. Il faut donc
regarder la scène elle-même plutôt que de deviner une cinquième fois.

Ce script a d'abord répondu à quatre questions, chacune excluant une
hypothèse (sections [3] à [6] de l'affichage) :
  1. Y a-t-il des prims Mesh sous /World/g1 ?       -> import des visuels
  2. Sont-ils visibles, et avec quel « purpose » ?   -> masquage USD
  3. Où sont-ils dans l'espace (boîte englobante) ?  -> hors champ / sous le sol
  4. Quelles variantes sont sélectionnées ?          -> visuels dans une
                                                        variante non active

Les sections [0] à [2] préparent ces mesures : diagnostics USD
réactivés, meshes manquants sur disque, présence de /World/g1. Les
sections [8] à [11], ajoutées ensuite, vérifient la conversion des STL
(geometries.usd), le contenu des instances USD, le chargement des
payloads, puis la visibilité et l'échelle des meshes vus en entrant dans
les instances. La section [7], affichée en dernier, donne la hauteur de
la base du robot, pour vérifier que ses jambes ne traversent pas le sol.
"""
import sys
sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app, find_missing_meshes, URDF_PATH,
)


def main():
    simulation_app = start_simulation_app(headless=True)

    # Rallumer les diagnostics USD, masqués par défaut dans Isaac Sim.
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

    # Parcours complet, y compris prims inactifs et payloads non
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

    # --- [8] La conversion a-t-elle produit de la géométrie ? --------
    # On ouvre geometries.usd comme son propre stage, hors de toute
    # composition : ça répond sans ambiguïté à « les STL ont-ils été
    # convertis en meshes ? », indépendamment de la façon dont la scène
    # les assemble.
    import glob
    geo_files = glob.glob("/isaac-sim/g1_axisos.usd/**/geometries.usd",
                          recursive=True)
    print(f"\n[8] fichier(s) geometries.usd : {geo_files}")
    for gf in geo_files:
        gstage = Usd.Stage.Open(gf)
        n_mesh, n_pts = 0, 0
        for p in gstage.Traverse():
            if p.IsA(UsdGeom.Mesh):
                n_mesh += 1
                pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
                n_pts += len(pts) if pts is not None else 0
        print(f"    meshes dans le fichier : {n_mesh}, sommets : {n_pts}")

    # --- [9] Dans la scène, en entrant dans les instances -----------
    # Correction du parcours [3] : il n'entrait pas dans les instances
    # USD. Or instances.usda indique que les meshes sont instanciés — ils
    # vivent dans des prototypes partagés, invisibles à un parcours
    # ordinaire. Le « 0 mesh » de [3] pouvait donc être un artefact de
    # méthode.
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    n_inst_mesh, n_instanceable = 0, 0
    for p in Usd.PrimRange(root, pred):
        if p.IsA(UsdGeom.Mesh):
            n_inst_mesh += 1
        if p.IsInstance():
            n_instanceable += 1
    print(f"\n[9] prims instanciés sous /World/g1 : {n_instanceable}")
    print(f"    meshes EN ENTRANT dans les instances : {n_inst_mesh}")

    # --- [10] Payloads chargés ? --------------------------------------
    payload_prims, unloaded = 0, []
    for p in Usd.PrimRange(root, Usd.PrimAllPrimsPredicate):
        if p.HasAuthoredPayloads():
            payload_prims += 1
            if not p.IsLoaded():
                unloaded.append(str(p.GetPath()))
    print(f"\n[10] prims portant un payload : {payload_prims}")
    print(f"     payloads NON chargés : {len(unloaded)}")
    for u in unloaded[:5]:
        print(f"       {u}")

    # --- [11] Les 47 meshes : rendus, et à la bonne échelle ? -------
    # 47, c'est le nombre de meshes que [9] a comptés sur le G1 à 23 DOF
    # lors de l'essai du 2026-09-22 : 27 visuels et 20 de collision,
    # exactement les références de l'URDF. Les messages ci-dessous
    # reprennent ce nombre tel quel.
    # [4] ne portait que sur le parcours sans instances, donc vide. On
    # refait la mesure en entrant dans les instances.
    vis11, pur11, sizes = Counter(), Counter(), []
    bb_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
         UsdGeom.Tokens.guide, UsdGeom.Tokens.proxy])
    for p in Usd.PrimRange(root, pred):
        if p.IsA(UsdGeom.Mesh):
            img = UsdGeom.Imageable(p)
            vis11[str(img.ComputeVisibility())] += 1
            pur = str(img.ComputePurpose())
            pur11[pur] += 1
            r = bb_cache.ComputeWorldBound(p).ComputeAlignedRange()
            if not r.IsEmpty():
                s = r.GetSize()
                sizes.append((max(s[0], s[1], s[2]), pur, p.GetName()))
    print(f"\n[11] visibilité des 47 meshes : {dict(vis11)}")
    print(f"     purpose des 47 meshes    : {dict(pur11)}")
    print(f"       -> seuls 'default' et 'render' sont vus par la caméra")
    if sizes:
        sizes.sort()
        print(f"     plus grande dimension d'un mesh : de {sizes[0][0]:.4f} m "
              f"à {sizes[-1][0]:.4f} m")
        print(f"       (attendu : de ~0.02 m pour une petite pièce à ~0.4 m "
              f"pour le torse ; des valeurs vers 0.0004 ou 400 trahiraient "
              f"une erreur d'unité mm/m à la conversion)")
        for sz, pur, name in sizes[-3:]:
            print(f"       {name:32} {sz:.4f} m  purpose={pur}")

    # Boîte englobante selon le purpose : si elle ne vient que des
    # primitives de collision (guide), les visuels sont hors du rendu.
    for label, toks in [("default+render", [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]),
                        ("guide seul", [UsdGeom.Tokens.guide])]:
        c = UsdGeom.BBoxCache(Usd.TimeCode.Default(), toks)
        r = c.ComputeWorldBound(root).ComputeAlignedRange()
        if r.IsEmpty():
            print(f"     bbox {label:15} : VIDE")
        else:
            print(f"     bbox {label:15} : z de {r.GetMin()[2]:+.2f} à "
                  f"{r.GetMax()[2]:+.2f}")

    print("\n[LECTURE]")
    print("  [8]=0 meshes        -> les STL n'ont jamais été convertis")
    print("  [8]>0 mais [9]=0    -> convertis mais pas assemblés dans la scène")
    print("  [9]>0               -> les meshes SONT dans la scène ; si l'image")
    print("                         reste vide, le problème est au RENDU")
    print("  [10] non chargés>0  -> la géométrie est dans un payload désactivé")

    pos, _ = bridge._robot.get_world_pose()
    print(f"\n[7] pose de la base : z = {float(pos[2]):+.4f} m")
    print("      (en base fixe, le bassin d'un G1 debout est vers +0.79 m ;")
    print("       à z≈0, les jambes traversent le sol)")

    simulation_app.close()


if __name__ == "__main__":
    main()
