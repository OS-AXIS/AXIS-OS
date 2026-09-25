"""
Suite W3 — prendre un carton SUR UNE TABLE, le soulever, l'emporter.

Scène réaliste, comme un carton sur un convoyeur ou une étagère : il est
posé sur une table, visible et stable DÈS LE DÉBUT. Dans W2, il était
placé entre les mains en cours de simulation — à l'écran, il apparaissait
de nulle part, et il tombait avant que les bras ne se referment.

  A. 0–3 s    debout, bras ouverts AUTOUR du carton posé sur la table
  B. 3–4 s    les bras SERRENT
  C. 4–5 s    les bras SOULÈVENT le carton de la table
  D. 5–8 s    recul à 0.2 m/s          -> s'écarter de la table
  E. 8–12 s   demi-tour sur place
  F. 12–16 s  marche à 0.3 m/s         -> il emporte le carton
  G. 16–18 s  arrêt

Le sol serait plus difficile, pas plus simple : les mains sont à 90 cm,
et ramasser au sol exige de s'accroupir ET de se pencher bras tendus —
le cas le plus exigeant pour l'équilibre. Étape suivante, pas celle-ci.

Positions de la table et du carton calculées pour que le carton tombe
ENTRE les mains ouvertes, d'après la position mesurée au test W2 :
mains centrées 0.29 m devant le bassin, à 0.90 m, écartées de 0.33 m.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_w3_gr00t_table.py
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)

POLICIES = Path("/isaac-sim/policies")
MASSE_KG = 1.0
PROFONDEUR, HAUTEUR = 0.20, 0.20
FAIL = 0

# Postures en ANGLES BRUTS, calculées par CINÉMATIQUE DIRECTE sur l'URDF
# 29 DOF — et non plus traduites par les étiquettes anatomiques de la
# table de conventions.
#
# DÉCOUVERTE du premier essai : la « flexion antérieure » de la table
# (tangage d'épaule +0.6) envoyait les mains DERRIÈRE le bassin — x=-0.16
# mesuré en simulation, x=-0.16 calculé par cinématique, deux sources
# indépendantes. En angles bruts, un tangage d'épaule POSITIF recule le
# bras, et un coude POSITIF le déplie (le coude du G1 est déjà plié à
# angle droit à zéro). Les étiquettes anatomiques de ces deux
# articulations sont inversées ; le roulis, lui, est conforme.
#
# Posture retenue, vérifiée par cinématique : mains 29 cm devant le
# bassin et 18 cm au-dessus, écartées de 34 cm.
EPAULE_TANGAGE = -0.30        # bras légèrement levés vers l'avant
COUDE = 0.0                   # coude à angle droit (zéro du G1)
ROULIS_OUVERT = 0.20          # bras écartés : 34 cm entre les mains
ROULIS_SERRE = -0.40          # vise AU-DELÀ du contact : la raideur serre
LARGEUR = 0.24                # carton plus étroit que l'écartement ouvert
EPAULE_LEVEE = -0.50          # soulever : les mains montent d'environ 6 cm
# Position des mains mesurée au test W2, bras ouverts : c'est là que doit
# se trouver le carton.
MAINS_X, MAINS_Z = 0.29, 0.90
TABLE_HAUT = MAINS_Z - HAUTEUR / 2          # plateau sous le carton
# PREMIER ESSAI : table dont le bord avant était à +0.14 m du bassin. La
# CUISSE du robot, en posture accroupie, touchait la table dès 0.3 s ; la
# politique d'équilibre a répondu à la poussée en RECULANT de 26 cm, et
# les mains ont fini devant le carton, à serrer le vide. Le calcul avait
# prévu les genoux, pas le volume de la cuisse.
#
# Table avancée : bord avant à +0.26 m, 16 cm de dégagement. Carton
# avancé à +0.33 m pour que son centre de masse repose sur la table ; il
# s'étend de +0.23 à +0.43, donc ses flancs restent ENTRE les mains
# (centrées à +0.29).
# Carton CENTRÉ sur les mains. Décalé de 4 cm vers l'avant au troisième
# essai, il basculait en se soulevant. Son centre de masse (+0.29) reste
# au-dessus de la table (bord à +0.26).
CARTON_X = 0.29
# QUATRIÈME ESSAI : carton centré à +0.29, mais bord de table à +0.26 —
# 3 cm d'appui seulement. Il a basculé par-dessus le bord dès la première
# seconde et s'est coincé entre la table et les jambes du robot.
#
# Contradiction de géométrie : le carton doit être à +0.29 (entre les
# mains), mais un bloc plein ne peut pas s'approcher à moins de +0.26 sans
# toucher les cuisses. Solution, celle de toute vraie table : un PLATEAU
# FIN, à 80 cm, qui passe AU-DESSUS des cuisses et avance jusqu'à +0.18 ;
# et un PIED placé en arrière, hors de portée des jambes. Le carton repose
# alors entièrement sur le plateau.
TABLE_AVANT, TABLE_ARRIERE = 0.18, 0.60
EPAISSEUR_PLATEAU = 0.04
PIED_AVANT = 0.42
# Gains des bras RAIDIS pour la prise. NVIDIA utilise une raideur de 4
# aux POIGNETS — réglage de téléopération, où l'on veut des poignets
# souples. Constaté au troisième essai de W3 : sous le serrage des épaules,
# les poignets pliaient, les mains PIVOTAIENT et s'écartaient du carton au
# lieu de le pincer ; il montait d'un centimètre, basculait, et glissait.
# La force de serrage se perdait dans les poignets.
#            épaule tangage, roulis, lacet, coude, poignet roulis, tangage, lacet
KP_BRAS = [90, 100, 40, 60, 40, 40, 40]
KD_BRAS = [2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0]


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    import omni.usd
    from pxr import Usd, UsdGeom
    from isaacsim.core.api.materials import PhysicsMaterial
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    from isaacsim.core.utils.types import ArticulationAction

    from axis_os.hal.core.types import ActuatorCommand, ControlMode
    from axis_os.hal.drivers.isaac_sim.video import SceneRecorder
    from axis_os.hal.drivers.unitree_g1.conventions import conventions_pour
    from axis_os.hal.drivers.unitree_g1.gr00t_wbc_policy import (
        GR00TWBCAdapter, GR00TWBCConfig, GR00TWBCPolicy,
    )
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        body_angular_velocity,
    )
    from axis_os.services.locomotion.service import (
        BaseState, LocomotionService, LocomotionState,
    )

    cfg = GR00TWBCConfig()
    pol = GR00TWBCPolicy(POLICIES / "gr00t_wbc_balance.onnx",
                         POLICIES / "gr00t_wbc_walk.onnx", cfg)
    pol.load()
    conv = conventions_pour("29dof")

    def pose(roulis):
        # Roulis symétrique : positif à gauche et négatif à droite écartent
        # les deux bras (vérifié par cinématique, conforme à la table).
        return {"left_shoulder_pitch_joint": EPAULE_TANGAGE,
                "right_shoulder_pitch_joint": EPAULE_TANGAGE,
                "left_shoulder_roll_joint": roulis,
                "right_shoulder_roll_joint": -roulis,
                "left_elbow_joint": COUDE, "right_elbow_joint": COUDE}

    bridge = IsaacSimBridge(variante="29dof")
    bridge.setup(fix_base=False, require_meshes=True, physics_dt=cfg.physics_dt)
    robot, world = bridge._robot, bridge._world
    idx = bridge._joint_index
    names = sorted(idx, key=lambda j: idx[j])

    # Carton créé AU DÉPART, loin ; téléporté plus tard.
    materiau = PhysicsMaterial(prim_path="/World/Materiaux/carton",
                               static_friction=1.0, dynamic_friction=0.8,
                               restitution=0.0)
    # Table FIXE, plateau juste sous les mains ; carton posé dessus, entre
    # les mains ouvertes. Visible et stable dès la première image.
    world.scene.add(FixedCuboid(
        prim_path="/World/table_plateau", name="table_plateau",
        position=np.array([(TABLE_AVANT + TABLE_ARRIERE) / 2, 0.0,
                           TABLE_HAUT - EPAISSEUR_PLATEAU / 2]),
        size=1.0,
        scale=np.array([TABLE_ARRIERE - TABLE_AVANT, 0.50, EPAISSEUR_PLATEAU]),
        color=np.array([0.45, 0.45, 0.50])))
    world.scene.add(FixedCuboid(
        prim_path="/World/table_pied", name="table_pied",
        position=np.array([(PIED_AVANT + TABLE_ARRIERE) / 2, 0.0,
                           (TABLE_HAUT - EPAISSEUR_PLATEAU) / 2]),
        size=1.0,
        scale=np.array([TABLE_ARRIERE - PIED_AVANT, 0.30,
                        TABLE_HAUT - EPAISSEUR_PLATEAU]),
        color=np.array([0.35, 0.35, 0.40])))
    carton = world.scene.add(DynamicCuboid(
        prim_path="/World/carton", name="carton",
        position=np.array([CARTON_X, 0.0, TABLE_HAUT + HAUTEUR / 2 + 0.002]),
        size=1.0,
        scale=np.array([PROFONDEUR, LARGEUR, HAUTEUR]), mass=MASSE_KG,
        color=np.array([0.72, 0.52, 0.30]), physics_material=materiau))

    g = pol.gains()
    for i, j in enumerate(cfg.body_joints[cfg.num_actions:]):
        g[j] = (KP_BRAS[i % 7], KD_BRAS[i % 7])
    # Posé AVANT world.reset() : modifier la physique d'une scène déjà
    # initialisée invalide sa vue physique (constaté ici au premier essai du
    # caoutchouc, comme pour la suppression d'objet en A3 et le changement
    # de taille du carton en W2). Règle : tout ce qui touche la physique se
    # pose avant l'initialisation.
    #
    # MAINS EN CAOUTCHOUC : leur donner le frottement du caoutchouc. Au
    # cinquième essai, le carton a été soulevé, tenu en reculant et pendant
    # tout le demi-tour, puis a glissé jusqu'à tomber en marche avant. Les
    # mains avaient le frottement PAR DÉFAUT du simulateur, celui d'une
    # surface quelconque — alors qu'Unitree les a faites en caoutchouc
    # précisément pour qu'elles adhèrent.
    import omni.usd
    from pxr import UsdShade
    caoutchouc = PhysicsMaterial(prim_path="/World/Materiaux/caoutchouc",
                                 static_friction=1.2, dynamic_friction=1.0,
                                 restitution=0.0)
    _stage = omni.usd.get_context().get_stage()
    _mat = UsdShade.Material(_stage.GetPrimAtPath("/World/Materiaux/caoutchouc"))
    appliques = []
    for _prim in _stage.Traverse():
        _n = _prim.GetName()
        if "/World/g1" in str(_prim.GetPath()) and (
                "rubber_hand" in _n or "wrist_yaw_link" in _n):
            UsdShade.MaterialBindingAPI.Apply(_prim).Bind(
                _mat, UsdShade.Tokens.weakerThanDescendants, "physics")
            appliques.append(_n)
    print(f"  caoutchouc appliqué à : {sorted(set(appliques)) or 'AUCUN LIEN TROUVÉ'}")

    world.reset()
    robot.get_articulation_controller().set_gains(
        kps=np.array([g[j][0] for j in names]),
        kds=np.array([g[j][1] for j in names]))
    q0 = {j: 0.0 for j in names}
    q0.update(dict(zip(cfg.controlled_joints, cfg.default_angles)))
    # Bras DÉJÀ ouverts autour du carton : le robot ne le percute pas en
    # prenant la posture.
    q0.update(pose(ROULIS_OUVERT))
    q0v = np.array([q0[j] for j in names])
    robot.set_joint_positions(q0v)
    robot.set_joint_velocities(np.zeros_like(q0v))
    robot.apply_action(ArticulationAction(joint_positions=q0v))

    hal = bridge.build_hal()
    hal.initialize_all()
    sim_t = [0.0]

    def read_state() -> BaseState:
        _p, quat = robot.get_world_pose()
        omega = body_angular_velocity(quat, robot.get_angular_velocity())
        jp, jv = robot.get_joint_positions(), robot.get_joint_velocities()
        return BaseState(tuple(float(v) for v in quat),
                         tuple(float(v) for v in omega),
                         {j: float(jp[idx[j]]) for j in names},
                         {j: float(jv[idx[j]]) for j in names})

    def send_targets(targets):
        return hal.send_joint_commands(
            {j: ActuatorCommand(mode=ControlMode.POSITION, position=v)
             for j, v in targets.items()})

    adapt = GR00TWBCAdapter(pol, pose_bras=pose(ROULIS_OUVERT))
    loco = LocomotionService(adapt, read_state, send_targets,
                             command_timeout_s=0.5, clock=lambda: sim_t[0])
    loco.start()

    stage = omni.usd.get_context().get_stage()
    chemins = {}

    def position_lien(nom):
        """Position MONDE d'un lien du robot, lue dans la scène. Le lien de
        la main peut avoir été fusionné à l'import : on retombe alors sur le
        lien de poignet qui le porte."""
        for candidat in (nom, nom.replace("rubber_hand", "wrist_yaw_link")):
            if candidat not in chemins:
                for prim in stage.Traverse():
                    if prim.GetName() == candidat and "/World/g1" in str(prim.GetPath()):
                        chemins[candidat] = prim
                        break
            prim = chemins.get(candidat)
            if prim is not None:
                m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())
                t = m.ExtractTranslation()
                return np.array([t[0], t[1], t[2]], dtype=float), candidat
        return None, None

    # CAMÉRA : suit le robot, trois quarts avant, PROCHE et visée à hauteur
    # de poitrine — c'est la prise qu'on veut voir, pas la silhouette.
    rec = SceneRecorder(world, "/isaac-sim/videos/w3_table.mp4",
                        position=(2.0, -2.2, 1.3), cible=(0.0, 0.0, 0.9),
                        largeur=1280, hauteur=720,
                        titre="AXIS-OS — le G1 prend un carton sur une table")
    rec.suivre(lambda: robot.get_world_pose()[0], decalage=(2.0, -2.2, 0.4),
               hauteur_visee=0.9, decalage_cadre=(-0.35, -0.30))
    video_ok = rec.start()
    TEXTES = {"B": ("Les bras serrent le carton", (40, 110, 200)),
              "C": ("Il soulève le carton de la table", (40, 110, 200)),
              "D": ("Il recule avec le carton", (40, 140, 70)),
              "E": ("Demi-tour", (40, 140, 70)),
              "F": ("Il emporte le carton", (40, 140, 70)),
              "G": ("Arrêt, carton en main", (40, 140, 70))}
    phase_vue = None
    lache_signale = False
    relache = False
    k = 0

    trace = []
    politiques = {}
    chute = None
    carton_place = False
    z_depart = None
    t_mur = time.monotonic()
    carton_place = True                 # il est sur la table dès le départ
    z_depart = TABLE_HAUT + HAUTEUR / 2
    while sim_t[0] < 18.0 and time.monotonic() - t_mur < 1200:
        t = sim_t[0]
        phase = ("A" if t < 3 else "B" if t < 4 else "C" if t < 5
                 else "D" if t < 8 else "E" if t < 12 else "F" if t < 16
                 else "G")
        if phase == "B":                         # serrer
            a = min(1.0, (t - 3.0) / 0.5)
            adapt.pose_bras.update(pose(ROULIS_OUVERT + a * (ROULIS_SERRE - ROULIS_OUVERT)))
        if phase == "C":                         # soulever
            a = min(1.0, (t - 4.0) / 0.8)
            p_ = pose(ROULIS_SERRE)
            for cote in ("left", "right"):
                p_[f"{cote}_shoulder_pitch_joint"] = (
                    EPAULE_TANGAGE + a * (EPAULE_LEVEE - EPAULE_TANGAGE))
            adapt.pose_bras.update(p_)
        if phase == "D":
            loco.set_velocity(-0.2, 0.0, 0.0)    # reculer, loin de la table
        if phase == "E":
            loco.set_velocity(0.0, 0.0, 0.6)     # demi-tour
        if phase == "F":
            loco.set_velocity(0.3, 0.0, 0.0)     # emporter
        if phase == "G":
            loco.halt()

        if carton_place and len(trace) < 1000:
            pc, _ = carton.get_world_pose()
            pr, _ = robot.get_world_pose()
            trace.append((t, phase, float(pc[2]),
                          float(np.linalg.norm(np.asarray(pc[:2]) - np.asarray(pr[:2]))),
                          float(pr[0])))

        if video_ok and phase != phase_vue:
            phase_vue = phase
            if phase in TEXTES:
                rec.evenement(TEXTES[phase][0], t, duree_s=2.0,
                              couleur=TEXTES[phase][1])
        if (video_ok and carton_place and not lache_signale and trace
                and trace[-1][2] < z_depart - 0.15):
            lache_signale = True
            rec.evenement("Carton lâché", t, duree_s=3.0, couleur=(200, 50, 45))

        loco.tick()
        if loco.state is LocomotionState.FALLEN:
            chute = (t, phase)
            break
        politiques.setdefault(phase, set()).add(pol.derniere_politique)
        capture = video_ok and (k % 2 == 0)        # 25 images/s
        if capture:
            rec.cadrer()                           # AVANT le rendu
        for i in range(cfg.decimation - 1):
            world.step(render=(capture and i == 0))
        sim_t[0] += cfg.control_dt
        k += 1
        if capture:
            zc = trace[-1][2] if trace else None
            rec.image([
                f"Phase : {phase}",
                f"Politique : {pol.derniere_politique}",
                "Carton : pas encore placé" if zc is None
                else f"Hauteur du carton : {zc:.2f} m",
                f"Garde-fou : {loco.stats['refused']} refus",
            ], t)

    if video_ok:
        rec.terminer()
    print(f"\n{'=' * 66}\n  RÉSULTATS\n{'=' * 66}")
    print("  t     phase  hauteur carton  distance au bassin  x robot")
    derniere = -1.0
    for t, ph, z, d, x in trace:
        pas = 0.25 if 6.3 <= t <= 8.0 else 1.0     # finesse au relâchement
        if t - derniere >= pas:
            print(f"  {t:5.1f}   {ph}     {z:6.3f} m        {d:5.2f} m        {x:+.2f}")
            derniere = t

    if chute:
        print(f"\n  CHUTE du robot à t = {chute[0]:.2f} s, phase {chute[1]}")
    check("Le robot ne tombe pas", chute is None)

    def dans(ph):
        return [(z, d) for t, p, z, d, x in trace if p == ph]

    # Le robot ne doit PAS reculer pendant qu'il attend debout : c'est le
    # signe d'un contact parasite (au premier essai, la table poussait sa
    # cuisse, et il a reculé de 26 cm).
    xa = [x for t, p, z, d, x in trace if p in ("A", "B")]
    if xa:
        # 8 cm : la mise en contact des bras raidis déplace le robot de
        # quelques centimètres (6 cm constatés) ; une collision parasite,
        # elle, l'avait repoussé de 26 cm.
        check(f"Aucun recul parasite avant le serrage : "
              f"{abs(xa[-1] - xa[0]) * 100:.1f} cm (au premier essai : 26 cm, "
              f"la table poussait la cuisse)", abs(xa[-1] - xa[0]) < 0.08)
    # Soulèvement : sur C et le DÉBUT de D — au cinquième essai, le carton
    # a quitté la table (+2.5 cm) juste au passage en phase D.
    pts_c = [(z, d) for t, p, z, d, x in trace
             if p == "C" or (p == "D" and t < 6.0)]
    if pts_c:
        zc = max(z for z, _ in pts_c)
        check(f"Le carton est SOULEVÉ de la table : {zc:.2f} m, départ "
              f"{z_depart:.2f}", zc > z_depart + 0.02)
    for ph, texte in (("D", "en reculant"), ("E", "pendant le demi-tour"),
                      ("F", "EN MARCHANT"), ("G", "à l'arrêt")):
        pts = dans(ph)
        if not pts:
            continue
        zmin = min(z for z, _ in pts)
        dmax = max(d for _, d in pts)
        check(f"Carton TENU {texte} : hauteur min {zmin:.2f} m, jamais à "
              f"plus de {dmax:.2f} m du bassin",
              zmin > TABLE_HAUT - 0.05 and dmax < 0.7)
    print(f"\n  politiques : { {p: sorted(v) for p, v in politiques.items()} }")
    print(f"  locomotion : {loco.stats}")
    check("Aucune consigne refusée par le garde-fou", loco.stats["refused"] == 0)

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    if FAIL == 0:
        print("\n  ✓ Le G1 prend un carton sur une table et l'emporte")
    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
