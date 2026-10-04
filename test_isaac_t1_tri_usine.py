"""
Suite T1 — la démonstration : tri de colis autonome dans un entrepôt, avec
des personnes qui circulent. G1 29 DOF, GR00T WBC, Isaac Sim 6.1.

Ce script branche, SANS LES MODIFIER, les services répétés sur CPU par
test/test_tri_pile_logicielle.py, sur la même disposition
(axis_os/apps/tri_colis/scene_demo.py) :

  TriColis (logique métier) + Superviseur (langage naturel)
    -> FacadeServices
       -> NavigationService -> LocomotionService (+ GardienProximite <- lidar)
       -> ManipulationService (bras, dépose sur plateau)
       -> PerceptionCouleur -> PerceptionService (OWL-ViT + profondeur)

Ce que la simulation remplace :
  - la physique du robot et des cartons : réelle (PhysX) ;
  - le lidar : rayons PhysX, modelés sur le Mid-360 inversé du G1 ;
  - les personnes : silhouettes cinématiques (0.4 × 0.4 × 1.7 m) qui
    circulent et s'écartent du robot — personnages animés à venir ;
  - la pose du robot : lue dans la simulation (odométrie/SLAM à venir) ;
  - la vérification de prise : position simulée du carton (sur le vrai
    robot : l'effort aux épaules) ;
  - un opérateur qui réapprovisionne la table de prise et débarrasse les
    postes de dépose.

Usage (dans le conteneur) :
  /isaac-sim/python.sh /isaac-sim/axis_os/test_isaac_t1_tri_usine.py \\
      [--duree 1800] [--acceleration 10] [--sans-video] [--graine 3]

  --duree        secondes SIMULÉES (défaut 240 : premier essai court)
  --acceleration la vidéo montre 1 image pour N×2 pas de 20 ms
                 (10 : 30 min -> 3 min de vidéo)
  --port-direct  suivi EN DIRECT dans un navigateur (défaut 8080) :
                   ssh -L 8080:localhost:8080 <machine>, puis
                   http://localhost:8080 — image, journal, bouton d'arrêt
"""
import argparse
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)

POLICIES = Path("/isaac-sim/policies")
VIDEOS = Path("/isaac-sim/videos")
FAIL = 0

# Réglages des bras validés par W3/W4 (poignets raidis).
KP_BRAS = [90, 100, 40, 60, 40, 40, 40]
KD_BRAS = [2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0]


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duree", type=float, default=240.0)
    # Collecte de démonstrations pour le VLA (axis_os/vla/collecte.py) :
    # prises répétées, cartons placés au hasard, sans personnes.
    ap.add_argument("--collecte-vla", default=None, metavar="DOSSIER")
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--acceleration", type=int, default=10)
    ap.add_argument("--sans-video", action="store_true")
    ap.add_argument("--graine", type=int, default=3)
    ap.add_argument("--port-direct", type=int, default=8080,
                    help="page de suivi en direct (0 : désactivée)")
    args = ap.parse_args()
    DUREE = args.duree
    rng = random.Random(args.graine)

    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
    from isaacsim.core.api.materials import PhysicsMaterial
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    from isaacsim.core.utils.types import ArticulationAction

    from axis_os.apps.tri_colis import scene_demo as sd
    from axis_os.apps.tri_colis.couleur import PerceptionCouleur
    from axis_os.apps.tri_colis.facade import ConfigFacade, FacadeServices
    from axis_os.apps.tri_colis.logique import ConfigTri, TriColis
    from axis_os.apps.tri_colis.superviseur import InterpreteurRegles, Superviseur
    from axis_os.hal.core.frames import (
        FRAME_BASE, FRAME_CAMERA_OPTICAL, FRAME_WORLD, FrameTree, Transform,
    )
    from axis_os.hal.core.types import ActuatorCommand, ControlMode, Quaternion, Vector3
    from axis_os.hal.drivers.isaac_sim.direct import SuiviDirect
    from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera
    from axis_os.hal.drivers.isaac_sim.sim_lidar import IsaacSimLidar
    from axis_os.hal.drivers.isaac_sim.video import SceneRecorder
    from axis_os.hal.drivers.unitree_g1.gr00t_wbc_policy import (
        GR00TWBCAdapter, GR00TWBCConfig, GR00TWBCPolicy,
    )
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        body_angular_velocity, quat_wxyz_to_matrix,
    )
    from axis_os.services.locomotion.proximite import GardienProximite
    from axis_os.services.locomotion.service import (
        BaseState, LocomotionService, LocomotionState,
    )
    from axis_os.services.manipulation.service import Etat, ManipulationService, Postures
    from axis_os.services.navigation.service import NavigationService, Pose2D
    from axis_os.services.perception.owlvit_detector import OwlViTDetector
    from axis_os.services.perception.service import PerceptionService

    cfg = GR00TWBCConfig()
    pol = GR00TWBCPolicy(POLICIES / "gr00t_wbc_balance.onnx",
                         POLICIES / "gr00t_wbc_walk.onnx", cfg)
    pol.load()
    P = Postures()

    bridge = IsaacSimBridge(variante="29dof")
    bridge.setup(fix_base=False, require_meshes=True, physics_dt=cfg.physics_dt)
    robot, world = bridge._robot, bridge._world
    idx = bridge._joint_index
    names = sorted(idx, key=lambda j: idx[j])
    stage = omni.usd.get_context().get_stage()

    # ================================================================
    # SCÈNE — tout ce qui touche la physique AVANT world.reset()
    # ================================================================
    GRIS, GRIS_F = np.array([0.45, 0.45, 0.50]), np.array([0.35, 0.35, 0.40])

    def table(nom, r, recul_pied=0.25):
        """Plateau fin + pied en retrait : les cuisses du robot passent
        dessous (leçon de W3 : la table poussait la cuisse)."""
        world.scene.add(FixedCuboid(
            prim_path=f"/World/{nom}_plateau", name=f"{nom}_plateau",
            position=np.array([(r.xmin + r.xmax) / 2, (r.ymin + r.ymax) / 2,
                               sd.PLATEAU - 0.02]),
            size=1.0, scale=np.array([r.xmax - r.xmin, r.ymax - r.ymin, 0.04]),
            color=GRIS))
        y0 = r.ymin + recul_pied
        world.scene.add(FixedCuboid(
            prim_path=f"/World/{nom}_pied", name=f"{nom}_pied",
            position=np.array([(r.xmin + r.xmax) / 2, (y0 + r.ymax) / 2,
                               (sd.PLATEAU - 0.04) / 2]),
            size=1.0, scale=np.array([(r.xmax - r.xmin) * 0.8, r.ymax - y0,
                                      sd.PLATEAU - 0.04]),
            color=GRIS_F))

    table("prise", sd.TABLE_PRISE)
    for n, r in sd.TABLES_DEPOSE.items():
        table(f"poste_{n}", r)
    for i, e in enumerate(sd.ETAGERES):
        world.scene.add(FixedCuboid(
            prim_path=f"/World/etagere_{i}", name=f"etagere_{i}",
            position=np.array([(e.xmin + e.xmax) / 2, (e.ymin + e.ymax) / 2, 0.9]),
            size=1.0, scale=np.array([e.xmax - e.xmin, e.ymax - e.ymin, 1.8]),
            color=np.array([0.30, 0.32, 0.38])))

    # Cartons : un STOCK créé d'avance (on n'ajoute rien après reset), rangé
    # hors du site, hors de portée du lidar. L'opérateur simulé les pose sur
    # la table de prise et débarrasse les postes.
    mat_carton = PhysicsMaterial(prim_path="/World/Materiaux/carton",
                                 static_friction=1.0, dynamic_friction=0.8)
    STOCK = {"rouge": 4, "bleu": 4, "vert": 2}
    cartons = []                               # dicts : prim, couleur, lieu
    for couleur, n in STOCK.items():
        for i in range(n):
            k = len(cartons)
            c = world.scene.add(DynamicCuboid(
                prim_path=f"/World/carton_{couleur}_{i}", name=f"carton_{couleur}_{i}",
                position=np.array([30.0 + 0.5 * k, 30.0, sd.HAUTEUR_CARTON / 2]),
                size=1.0, scale=np.array([sd.LARGEUR_CARTON, sd.PROFONDEUR_CARTON,
                                          sd.HAUTEUR_CARTON]),
                mass=1.0, color=np.array(sd.COULEURS_RGB[couleur]),
                physics_material=mat_carton))
            cartons.append({"obj": c, "couleur": couleur, "lieu": "stock", "t": 0.0,
                            "parc": np.array([30.0 + 0.5 * k, 30.0, sd.HAUTEUR_CARTON / 2])})

    # Personnes : silhouettes CINÉMATIQUES (déplacées, pas poussées).
    COLLECTE = args.collecte_vla is not None
    # Collecte VLA : pas de personnes (le geste de prise seul est enregistré ;
    # la sécurité reste active, elle n'a simplement personne à protéger).
    figurants = [] if COLLECTE else sd.figurants()
    silhouettes = []
    for i, f in enumerate(figurants):
        path = f"/World/personne_{i}"
        s = world.scene.add(DynamicCuboid(
            prim_path=path, name=f"personne_{i}",
            position=np.array([f.p[0], f.p[1], 0.85]), size=1.0,
            scale=np.array([0.4, 0.4, 1.7]), mass=70.0,
            color=np.array([0.95, 0.55, 0.10])))
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).CreateKinematicEnabledAttr(True)
        silhouettes.append(s)

    # Mains en caoutchouc (W3) : lié avant reset, sinon ignoré.
    PhysicsMaterial(prim_path="/World/Materiaux/caoutchouc",
                    static_friction=1.2, dynamic_friction=1.0)
    mat = UsdShade.Material(stage.GetPrimAtPath("/World/Materiaux/caoutchouc"))
    for prim in stage.Traverse():
        n = prim.GetName()
        if "/World/g1" in str(prim.GetPath()) and ("rubber_hand" in n or "wrist_yaw_link" in n):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    # Éclairage. Premier essai GPU : la seule lumière venait de derrière la
    # table, la face des cartons tournée vers le robot était NOIRE sur la
    # caméra de tête — couleur illisible, aucun colis trié. Un entrepôt est
    # éclairé de partout : lumière ambiante (dôme) + lumière venant du côté
    # du robot (vers +y, en plongée de 30°).
    from pxr import Gf, UsdLux
    dome = UsdLux.DomeLight.Define(stage, "/World/eclairage_ambiant")
    dome.CreateIntensityAttr(400.0)   # 900 délavait les couleurs
    # QUATRE lumières en diagonale (une par coin, plongée de 50°), comme
    # les rampes d'un entrepôt. Historique GPU, faces des cartons TOURNÉES
    # VERS LE ROBOT restées NOIRES (RGB 0,0,0) avec :
    #   - une lumière de face à 60°, puis presque verticale (15°) ;
    #   - deux lumières en diagonale du côté du robot (±40°) — les faces
    #     latérales des cartons s'allumaient, les faces avant restaient
    #     noires (/tete.jpg, /tete_ok.jpg). Le dôme ne suffit pas à les
    #     éclairer, et le sens réel des lumières n'avait jamais été vérifié.
    # Avec une lumière par coin, toute face verticale en reçoit au moins
    # deux, quel que soit le sens de rotation retenu par USD ; l'ombre du
    # robot n'en masque qu'une à la fois. Le sens réel est affiché au
    # démarrage (« éclairage : … vers »).
    for nom_l, lacet in (("eclairage_1", 45.0), ("eclairage_2", -45.0),
                         ("eclairage_3", 135.0), ("eclairage_4", -135.0)):
        lum = UsdLux.DistantLight.Define(stage, f"/World/{nom_l}")
        lum.CreateIntensityAttr(700.0)
        lum.CreateAngleAttr(1.0)
        xf_l = UsdGeom.Xformable(lum)
        xf_l.AddRotateZOp().Set(lacet)
        xf_l.AddRotateXOp().Set(50.0)
        m_l = xf_l.ComputeLocalToWorldTransform(0.0)
        d_l = m_l.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
        print(f"  éclairage : {nom_l} vers ({d_l[0]:+.2f}, {d_l[1]:+.2f}, {d_l[2]:+.2f})",
              flush=True)

    # Caméra de tête, grand angle (±30°) : les trois emplacements de la
    # table de prise doivent tenir dans l'image depuis le poste de prise.
    cam = IsaacSimCamera("tete_rgb", prim_path="/World/tete_rgb", width=640, height=480,
                         position=(0.2, 0.0, 1.2), focal_length_mm=18.0)
    dep = IsaacSimCamera("tete_prof", prim_path="/World/tete_prof", width=640, height=480,
                         position=(0.2, 0.0, 1.2), depth=True, focal_length_mm=18.0)

    g = pol.gains()
    for i, j in enumerate(cfg.body_joints[cfg.num_actions:]):
        g[j] = (KP_BRAS[i % 7], KD_BRAS[i % 7])

    world.reset()
    # ================================================================

    robot.get_articulation_controller().set_gains(
        kps=np.array([g[j][0] for j in names]), kds=np.array([g[j][1] for j in names]))
    pos0, _ = robot.get_world_pose()
    yaw0 = sd.DEPART.cap
    robot.set_world_pose(position=np.array([sd.DEPART.x, sd.DEPART.y, float(pos0[2])]),
                         orientation=np.array([math.cos(yaw0 / 2), 0.0, 0.0,
                                               math.sin(yaw0 / 2)]))
    bras_depart = {"left_shoulder_pitch_joint": P.tangage_port,
                   "right_shoulder_pitch_joint": P.tangage_port,
                   "left_shoulder_roll_joint": P.roulis_ouvert,
                   "right_shoulder_roll_joint": -P.roulis_ouvert,
                   "left_elbow_joint": P.coude, "right_elbow_joint": P.coude}
    q0 = {j: 0.0 for j in names}
    q0.update(dict(zip(cfg.controlled_joints, cfg.default_angles)))
    q0.update(bras_depart)
    q0v = np.array([q0[j] for j in names])
    robot.set_joint_positions(q0v)
    robot.set_joint_velocities(np.zeros_like(q0v))
    robot.apply_action(ArticulationAction(joint_positions=q0v))

    for c in (cam, dep):
        c.initialize()
    cam.wait_until_ready(world)
    dep.wait_until_ready(world)

    sim_t = [0.0]
    horloge = lambda: sim_t[0]

    # ---- capteurs et pose ------------------------------------------------
    def lacet(q):
        w, x, y, z = (float(v) for v in q)
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def pose2d() -> Pose2D:
        p, q = robot.get_world_pose()
        return Pose2D(float(p[0]), float(p[1]), lacet(q), horodatage=sim_t[0])

    def pose_tete():
        p, q = robot.get_world_pose()
        R = quat_wxyz_to_matrix(q)
        # 0.62 m au-dessus du bassin, soit ~1.42 m : AU-DESSUS de la tête du
        # G1 (1.32 m). Premier essai GPU à 0.45 m : le capteur était DANS le
        # volume de collision de la tête, chaque rayon s'y arrêtait — balayage
        # vide, robot arrêté pour de bon.
        return np.asarray(p, dtype=float) + R @ np.array([0.0, 0.0, 0.62]), R

    lidar = IsaacSimLidar("mid360", pose_tete, horloge)
    lidar.initialize()

    def vers_robot(p_monde):
        pos, quat = robot.get_world_pose()
        R = quat_wxyz_to_matrix(quat)
        return R.T @ (np.asarray(p_monde, dtype=float) - np.asarray(pos, dtype=float))

    prims_mains = {}

    def main_monde(nom):
        for cand in (nom, nom.replace("rubber_hand", "wrist_yaw_link")):
            if cand not in prims_mains:
                for prim in stage.Traverse():
                    if prim.GetName() == cand and "/World/g1" in str(prim.GetPath()):
                        prims_mains[cand] = prim
                        break
            prim = prims_mains.get(cand)
            if prim is not None:
                m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                t_ = m.ExtractTranslation()
                return np.array([t_[0], t_[1], t_[2]], dtype=float)
        return None

    def mains():
        return main_monde("left_rubber_hand"), main_monde("right_rubber_hand")

    def carton_aux_mains(rayon=0.25):
        g_, d_ = mains()
        m = (g_ + d_) / 2
        best, dmin = None, rayon
        for c in cartons:
            pc = np.asarray(c["obj"].get_world_pose()[0], dtype=float)
            d = float(np.linalg.norm(pc - m))
            if d < dmin:
                best, dmin = c, d
        return best

    diag_prise = [-1e9]

    def verifier_prise():
        # Simulation : le carton a-t-il quitté la table avec les mains ? Sur
        # le vrai robot : l'effort mesuré aux épaules.
        c = carton_aux_mains()
        ok_ = c is not None and float(c["obj"].get_world_pose()[0][2]) > \
            sd.PLATEAU + sd.HAUTEUR_CARTON / 2 + 0.015
        if not ok_ and sim_t[0] - diag_prise[0] > 5.0:
            diag_prise[0] = sim_t[0]          # une fois par tentative, pas à chaque pas
            # DIAGNOSTIC d'une prise ratée : où étaient les mains, le carton,
            # et dans quel sens le carton était tourné (poussé en avançant ?).
            g_, d_ = mains()
            pr_ = pose2d()
            print(f"  [prise ratée {sim_t[0]:5.1f} s] robot ({pr_.x:.2f}, {pr_.y:.2f}, "
                  f"cap {math.degrees(pr_.cap):.0f}°), main G {np.round(g_, 3)}, "
                  f"main D {np.round(d_, 3)}, écart des mains {np.linalg.norm(g_ - d_):.3f} m",
                  flush=True)
            for c_ in cartons:
                if c_["lieu"] != "prise":
                    continue
                pc_, q_ = c_["obj"].get_world_pose()
                lacet = math.degrees(2 * math.atan2(float(q_[3]), float(q_[0])))
                print(f"      carton {c_['couleur']} en ({pc_[0]:.3f}, {pc_[1]:.3f}, "
                      f"{pc_[2]:.3f}), tourné de {lacet:.0f}°"
                      f"{'  <- entre les mains' if c_ is c else ''}", flush=True)
        return ok_

    # ---- pile AXIS-OS -----------------------------------------------------
    hal = bridge.build_hal()
    hal.initialize_all()

    def read_state() -> BaseState:
        _p, quat = robot.get_world_pose()
        omega = body_angular_velocity(quat, robot.get_angular_velocity())
        jp, jv = robot.get_joint_positions(), robot.get_joint_velocities()
        return BaseState(tuple(float(v) for v in quat), tuple(float(v) for v in omega),
                         {j: float(jp[idx[j]]) for j in names},
                         {j: float(jv[idx[j]]) for j in names})

    def send_targets(targets):
        # Chaque cible traverse la HAL, donc le JointSafetyGuard.
        return hal.send_joint_commands(
            {j: ActuatorCommand(mode=ControlMode.POSITION, position=v)
             for j, v in targets.items()})

    from axis_os.hal.drivers.unitree_g1.robot import build_g1_joint_definitions as _defs
    adapt = GR00TWBCAdapter(pol, pose_bras=bras_depart,
                            limites={d.joint_id: (d.limits.pos_min, d.limits.pos_max)
                                     for d in _defs("29dof")})
    gardien = GardienProximite(lidar.source_gardien(),
                               lambda: (pose2d().x, pose2d().y, pose2d().cap),
                               obstacles=sd.OBSTACLES, horloge=horloge)
    loco = LocomotionService(adapt, read_state, send_targets, command_timeout_s=0.5,
                             clock=horloge, gardien=gardien)
    nav = NavigationService(sd.carte(), pose2d, loco, horloge=horloge)
    # Le ManipulationService écrit DANS le dictionnaire de bras de
    # l'adaptateur : il en est le seul écrivain.
    manip = ManipulationService(adapt.pose_bras, mains, vers_robot, verifier_prise,
                                gardien=gardien, horloge=horloge)

    tree = FrameTree()
    detecteur = OwlViTDetector(threshold=0.06)       # seuil de W4
    detecteur.load()
    perception = PerceptionService(cam, tree, detecteur, depth_camera=dep)
    percep_couleur = PerceptionCouleur(perception, cam, sd.COULEURS,
                                       requetes=("a box", "a cardboard box"))
    journal = []
    rec = None
    suivi = None
    if args.port_direct:
        suivi = SuiviDirect(str(VIDEOS / "direct"), port=args.port_direct)
        if not suivi.demarrer():
            suivi = None

    def journaliser(msg):
        journal.append(msg)
        print("  " + msg, flush=True)
        if "navigation vers" in msg and "échec" in msg:
            # DIAGNOSTIC (essai 30 min : 4 fois « navigation vers prise en
            # échec » sans arrêt de sécurité) : où est le robot, où va-t-il,
            # et ce que fait le suiveur.
            pr_ = pose2d()
            but_ = getattr(nav, "_but", None)
            but_ = nav.carte.postes.get(but_, but_) if isinstance(but_, str) else but_
            sv_ = nav.suiveur
            print(f"      robot ({pr_.x:.3f}, {pr_.y:.3f}, cap {math.degrees(pr_.cap):.1f}°), "
                  f"but {but_}, chemin {None if nav.chemin is None else nav.chemin[-3:]}, "
                  f"alignement {getattr(sv_, '_alignement', '?')}, "
                  f"stagnation {getattr(sv_, '_pas_sans_progres', '?')}, "
                  f"consigne {loco._twist}, nav {nav.stats}", flush=True)
        if suivi is not None:
            suivi.journal(msg)
        if rec is not None and any(k in msg for k in ("LIVRÉ", "ALERTE", "superviseur",
                                                       "SIGNALÉ", "repéré")):
            couleur = (200, 50, 45) if "ALERTE" in msg or "SIGNALÉ" in msg else (40, 140, 70)
            texte = msg.split("] ", 1)[-1][:70]
            rec.evenement(texte, sim_t[0], duree_s=3.0, couleur=couleur)

    facade = FacadeServices(nav, loco, manip, percep_couleur, pose2d, horloge, sd.CAP_PRISE,
                            journal=journaliser,
                            cfg=ConfigFacade(requete="{couleur}", confiance_min=0.06,
                                             z_min=0.70, z_max=1.20),
                            postes_depose=sd.postes_depose())
    tri = TriColis(facade, ConfigTri(regles=dict(sd.REGLES_INITIALES), duree_mission_s=DUREE))
    sup = Superviseur(tri, list(sd.POSTES), sd.COULEURS, InterpreteurRegles())
    for s in (loco, nav, manip):
        s.start()
    perception.start()

    # ---- l'opérateur simulé -------------------------------------------------
    def poser_carton(c, xyz, lacet_deg=0.0):
        a = math.radians(lacet_deg) / 2.0
        c["obj"].set_world_pose(position=np.array(xyz, dtype=float),
                                orientation=np.array([math.cos(a), 0.0, 0.0, math.sin(a)]))
        c["obj"].set_linear_velocity(np.zeros(3))
        c["obj"].set_angular_velocity(np.zeros(3))

    def sur(r, p, marge=0.0):
        return r.xmin - marge <= p[0] <= r.xmax + marge and r.ymin - marge <= p[1] <= r.ymax + marge

    def operateur(t, stats):
        # 1. État de chaque carton, d'après sa position.
        for c in cartons:
            if c["lieu"] == "stock":
                continue
            p = np.asarray(c["obj"].get_world_pose()[0], dtype=float)
            lieu = None
            if p[2] > sd.PLATEAU - 0.05:
                if sur(sd.TABLE_PRISE, p):
                    lieu = "prise"
                for n, r in sd.TABLES_DEPOSE.items():
                    if sur(r, p):
                        lieu = n
                if lieu is None:
                    lieu = "porte"              # dans les mains, en l'air
            elif p[2] < 0.3:
                lieu = "sol"
            if lieu != c["lieu"]:
                if lieu == "sol" and c["lieu"] != "sol":
                    stats["chutes_carton"] += 1
                    journaliser(f"[{t:7.1f}s] OPÉRATEUR : carton {c['couleur']} tombé au sol")
                if lieu in sd.TABLES_DEPOSE:
                    stats["sur_plateau"].append((c["couleur"], lieu))
                c["lieu"], c["t"] = lieu, t
            # 2. Débarrasser : postes de dépose après 20 s, sol après 30 s,
            #    verts (sans règle) sur la table de prise après 90 s.
            attente = {"sol": 30.0, "prise": 90.0 if c["couleur"] == "vert" else None}
            limite = 20.0 if c["lieu"] in sd.TABLES_DEPOSE else attente.get(c["lieu"])
            if limite is not None and t - c["t"] > limite:
                poser_carton(c, c["parc"])
                c["lieu"], c["t"] = "stock", t
        # 2b. Carton BOUSCULÉ sur la table de prise. Constaté sur GPU (T1,
        #     30 min) : un carton poussé à 5 cm au-delà du bord ; le gardien le
        #     voit comme un obstacle inconnu à 52 cm et arrête le robot —
        #     à raison, il ne peut pas savoir que ce n'est pas une main —,
        #     qui reste bloqué et le SIGNALE. Dans un vrai atelier, l'opérateur
        #     alerté vient remettre le carton en place : on fait de même
        #     après 20 s, et on le journalise (le déplacement aussi, pour
        #     comprendre quel geste l'a causé).
        pr_op = pose2d()
        for c in cartons:
            if c["lieu"] != "prise" or c is carton_aux_mains():
                continue
            pc, q = c["obj"].get_world_pose()
            pc = np.asarray(pc, dtype=float)
            lacet = abs(math.degrees(2 * math.atan2(float(q[3]), float(q[0]))))
            lacet = min(lacet, abs(180.0 - lacet))
            place = c.get("place")
            if place is None:
                c["place"] = place = pc.copy()
            depl = float(np.linalg.norm(pc[:2] - place[:2]))
            if depl > 0.03 and not c.get("deplace_signale"):
                c["deplace_signale"] = True
                journaliser(f"[{t:7.1f}s] OPÉRATEUR : carton {c['couleur']} DÉPLACÉ de "
                            f"{100 * depl:.0f} cm (tourné de {lacet:.0f}°) — robot en "
                            f"({pr_op.x:.2f}, {pr_op.y:.2f}), tâche {tri.etat}, "
                            f"manipulation {manip.etat.name}")
            depasse = pc[1] - sd.PROFONDEUR_CARTON / 2 < sd.TABLE_PRISE.ymin - 0.01
            if (depasse or lacet > 15.0 or depl > 0.08):
                c.setdefault("t_derange", t)
                if t - c["t_derange"] > 20.0:
                    x_ = min(sd.EMPLACEMENTS_X, key=lambda v: abs(v - pc[0]))
                    poser_carton(c, (x_, sd.Y_CARTON, sd.PLATEAU + sd.HAUTEUR_CARTON / 2 + 0.003))
                    c["place"] = np.array([x_, sd.Y_CARTON, pc[2]])
                    c.pop("t_derange", None)
                    c["deplace_signale"] = False
                    stats["remises_en_place"] = stats.get("remises_en_place", 0) + 1
                    journaliser(f"[{t:7.1f}s] OPÉRATEUR : carton {c['couleur']} remis en "
                                f"place (il dépassait ou était de travers)")
            else:
                c.pop("t_derange", None)
        # 3. Réapprovisionner : table remplie au départ, puis un emplacement
        #    toutes les ~20 s. Jamais celui devant lequel le robot travaille.
        if t >= stats["prochain_reappro"]:
            premier = stats["prochain_reappro"] == 0.0
            stats["prochain_reappro"] = t + 20.0
            occupes = [x for x in sd.EMPLACEMENTS_X for c in cartons
                       if c["lieu"] == "prise"
                       and abs(float(c["obj"].get_world_pose()[0][0]) - x) < 0.13]
            pr = pose2d()
            accoste = pr.y > sd.TABLE_PRISE.ymin - 0.7
            libres = [x for x in sd.EMPLACEMENTS_X if x not in occupes
                      and not (accoste and abs(x - pr.x) < 0.35)]
            for _ in range(len(libres) if premier else 1):
                dispo = [c for c in cartons if c["lieu"] == "stock"]
                if not (libres and dispo):
                    break
                poids = [3 if c["couleur"] != "vert" else 1 for c in dispo]
                c = rng.choices(dispo, weights=poids)[0]
                x = rng.choice(libres)
                libres.remove(x)
                poser_carton(c, (x, sd.Y_CARTON, sd.PLATEAU + sd.HAUTEUR_CARTON / 2 + 0.003))
                c["lieu"], c["t"] = "prise", t
                c["place"], c["deplace_signale"] = None, False
                c.pop("t_derange", None)

    # ---- vidéo ------------------------------------------------------------
    video_ok = False
    if not args.sans_video:
        rec = SceneRecorder(world, str(VIDEOS / "t1_tri_usine.mp4"),
                            position=(5.0, -2.5, 4.0), cible=(4.5, 3.0, 0.8),
                            largeur=1920, hauteur=1080,
                            titre="AXIS-OS — tri de colis autonome, avec des humains")
        rec.suivre(lambda: robot.get_world_pose()[0], decalage=(2.6, -2.6, 2.2),
                   hauteur_visee=0.9, decalage_cadre=(-0.3, 0.3))
        video_ok = rec.start()
        rec.direct = suivi
    if suivi is not None and not video_ok:
        print("  [direct] sans vidéo, la page ne montre que le journal")
    pas_video = 2 * max(1, args.acceleration)

    # ---- collecte VLA -------------------------------------------------------
    collecteur, enreg = None, None
    if COLLECTE:
        from PIL import Image as _Img
        from axis_os.vla.collecte import CollecteurPrises
        from axis_os.vla.enregistreur import EnregistreurEpisodes
        bras_j = list(cfg.body_joints[cfg.num_actions:])
        enreg = EnregistreurEpisodes(args.collecte_vla, fps=10, noms_etat=bras_j,
                                     noms_action=bras_j + ["base_vx", "base_vy", "base_wz"])

        def placer_vla(rng_):
            dispo = [c for c in cartons if c["lieu"] == "stock" and c["couleur"] != "vert"]
            c = rng_.choice(dispo)
            x = rng_.uniform(3.55, 4.45)
            y = sd.Y_CARTON + rng_.uniform(0.0, 0.05)
            lacet = rng_.uniform(-12.0, 12.0)
            poser_carton(c, (x, y, sd.PLATEAU + sd.HAUTEUR_CARTON / 2 + 0.003), lacet)
            c["lieu"], c["t"] = "prise", sim_t[0]
            return c["couleur"], {"x": round(x, 3), "y": round(y, 3), "lacet": round(lacet, 1)}

        def retirer_vla():
            for c in cartons:
                if c["lieu"] != "stock":
                    poser_carton(c, c["parc"])
                    c["lieu"] = "stock"

        def etat_action_vla():
            st = read_state()
            tw = loco._twist
            return ([st.joint_pos[j] for j in bras_j],
                    [adapt.pose_bras[j] for j in bras_j] + [tw.vx, tw.vy, tw.wz])

        def image_vla():
            f_ = cam.capture()
            if f_ is None:
                return None
            im = _Img.frombytes("RGB", (f_.width, f_.height), f_.data).reduce(2)
            return np.asarray(im)

        collecteur = CollecteurPrises(facade, enreg, placer_vla, retirer_vla, horloge,
                                      etat_action_vla, image_vla, n_episodes=args.episodes,
                                      graine=args.graine, journal=journaliser)
        DUREE = 1e9                          # la collecte s'arrête d'elle-même

    # ---- boucle à 50 Hz -----------------------------------------------------
    stats = {"chutes_carton": 0, "sur_plateau": [], "prochain_reappro": 0.0}
    # Table de prise remplie au départ. Pas de pas de physique « à vide » :
    # sans la politique d'équilibre, le robot tomberait avant de commencer.
    if not COLLECTE:
        operateur(0.0, stats)
    viol_humain, d_min_humain = 0, float("inf")
    # Rapprochement CUMULÉ vers chaque personne tant qu'elle est à moins de
    # 0.75 m. Un robot debout oscille de quelques millimètres en gardant
    # l'équilibre : ce n'est pas « aller vers » quelqu'un. Est en défaut un
    # rapprochement de plus de 1 cm pendant un même épisode de proximité.
    rapproche = {}                      # index personne -> cumul (m)
    pas_vers_humain, rapproche_max, lignes_rappr = 0, 0.0, 0
    ordre_donne = False
    chute = None
    k = 0
    etat_prec, diag_image, arret_signale = None, False, False
    t_mur = time.monotonic()
    print(f"\n  T1 — {DUREE:.0f} s simulées, vidéo "
          f"{'non' if not video_ok else f'accélérée ×{args.acceleration}'}\n")
    pilote = collecteur if COLLECTE else tri
    while not pilote.termine and sim_t[0] < DUREE + 300.0:
        if suivi is not None and suivi.arret_demande:
            print("  Arrêt demandé depuis la page de suivi — bilan et vidéo écrits.")
            break
        t = sim_t[0]
        pr = pose2d()
        # Personnes
        for f, s in zip(figurants, silhouettes):
            f.pas(cfg.control_dt, pr.x, pr.y)
            s.set_world_pose(position=np.array([f.p[0], f.p[1], 0.85]))
        if k % 10 == 0 and not COLLECTE:
            operateur(t, stats)
        # Pile AXIS-OS
        nav.tick()
        facade.tick()
        loco.tick()
        manip.tick()
        pilote.pas()
        if loco.state is LocomotionState.FALLEN:
            chute = (t, tri.etat)
            break
        # DIAGNOSTIC PERCEPTION : une recherche vient de ne rien donner.
        # On dit ce qu'OWL-ViT a vu, et on garde ce que voyait la tête.
        if etat_prec == "CHERCHER" and tri.colis is not None \
                and tri.etat not in ("CHERCHER", "ATTENDRE"):
            # DIAGNOSTIC : position PERÇUE du colis visé contre sa position
            # RÉELLE. Un écart latéral de quelques cm suffit à ce qu'une main
            # pousse le carton en approchant (constaté sur RTX 5090).
            pp = tri.colis.position
            reel = min((c_ for c_ in cartons if c_["lieu"] == "prise"),
                       key=lambda c_: math.hypot(
                           float(c_["obj"].get_world_pose()[0][0]) - pp[0],
                           float(c_["obj"].get_world_pose()[0][1]) - pp[1]),
                       default=None)
            if reel is not None:
                pr_ = np.asarray(reel["obj"].get_world_pose()[0], dtype=float)
                msg_ = (f"[{t:7.1f}s] PERCEPTION : colis {tri.colis.couleur} perçu en "
                        f"({pp[0]:.3f}, {pp[1]:.3f}, {pp[2]:.3f}), réel {reel['couleur']} en "
                        f"({pr_[0]:.3f}, {pr_[1]:.3f}, {pr_[2]:.3f}) — écart x "
                        f"{100 * (pp[0] - pr_[0]):+.1f} cm, y {100 * (pp[1] - pr_[1]):+.1f} cm")
                journaliser(msg_)
        if etat_prec == "CHERCHER" and tri.etat not in ("CHERCHER", "ATTENDRE") \
                and suivi is not None:
            # Recherche RÉUSSIE : on garde aussi ce que voyait la tête, pour
            # comparer avec une recherche vide (/tete_ok.jpg).
            f_ = cam.capture()
            if f_ is not None:
                from PIL import Image as _I, ImageDraw as _D
                im_ = _I.frombytes("RGB", (f_.width, f_.height), f_.data)
                dr_ = _D.Draw(im_)
                for d_ in percep_couleur.diagnostic.get("detections", [])[:8]:
                    x_, y_, w_, h_ = d_["bbox"]
                    dr_.rectangle([x_, y_, x_ + w_, y_ + h_], outline=(0, 255, 0), width=2)
                    dr_.text((x_ + 3, max(0, y_ - 12)), str(d_["couleur"]), fill=(0, 255, 0))
                dr_.text((5, 5), f"t = {t:.1f} s (recherche réussie)", fill=(255, 255, 255))
                im_.save(str(VIDEOS / "direct" / "tete_ok.jpg"))
        if etat_prec == "CHERCHER" and tri.etat == "ATTENDRE":
            dg = percep_couleur.diagnostic
            print(f"  [perception {t:5.1f} s] rien de triable — détections par "
                  f"requête {dg.get('par_requete')}, image={dg.get('image')}", flush=True)
            for r_ in dg.get("raisons", []):
                print(f"      raison : {r_}", flush=True)
            for d_ in dg.get("detections", [])[:8]:
                print(f"      vu : couleur={d_['couleur']} conf={d_['confiance']} "
                      f"pos={d_['position']} bbox={d_['bbox']}", flush=True)
                if d_.get("mesure"):
                    print(f"           {d_['mesure']}", flush=True)
            if suivi is not None:
                # Vue de la tête À CHAQUE recherche vide (écrasée), avec les
                # boîtes d'OWL-ViT et la couleur retenue : on VOIT pourquoi.
                f_ = cam.capture()
                if f_ is not None:
                    from PIL import Image as _I, ImageDraw as _D
                    im_ = _I.frombytes("RGB", (f_.width, f_.height), f_.data)
                    dr_ = _D.Draw(im_)
                    for d_ in dg.get("detections", [])[:8]:
                        x_, y_, w_, h_ = d_["bbox"]
                        dr_.rectangle([x_, y_, x_ + w_, y_ + h_], outline=(255, 255, 0), width=2)
                        dr_.text((x_ + 3, max(0, y_ - 12)), str(d_["couleur"]), fill=(255, 255, 0))
                    dr_.text((5, 5), f"t = {t:.1f} s", fill=(255, 255, 255))
                    im_.save(str(VIDEOS / "direct" / "tete.jpg"))
                    if not diag_image:
                        diag_image = True
                        print("      vue de la tête (mise à jour à chaque recherche vide) : "
                              "/tete.jpg sur la page de suivi", flush=True)
        etat_prec = tri.etat
        # DIAGNOSTIC SÉCURITÉ : à chaque arrêt, dire OÙ est l'obstacle et CE
        # QUE c'est — une personne, ou autre chose (le robot lui-même mal
        # filtré par le lidar, une table mal déclarée…). Premier essai GPU :
        # arrêt près de la table de prise, cause non identifiable sans ça.
        v_ = loco.dernier_verdict
        if v_ is not None and v_.facteur <= 0.0 and not arret_signale:
            arret_signale = True
            pt = v_.point
            pr_ = pose2d()
            quoi = "aucun point (capteur muet ?)"
            if pt is not None:
                proches = sorted((math.hypot(f.p[0] - pt[0], f.p[1] - pt[1]), i)
                                 for i, f in enumerate(figurants))
                # Collecte VLA : aucune personne dans la scène.
                d_pers, i_pers = proches[0] if proches else (math.inf, -1)
                quoi = (f"PERSONNE n°{i_pers} (à {d_pers:.2f} m du point)" if d_pers < 0.35
                        else f"PAS une personne (la plus proche à {d_pers:.2f} m du point)")
                sur_robot = math.hypot(pt[0] - pr_.x, pt[1] - pr_.y)
                quoi += f", point à {sur_robot:.2f} m du centre du robot"
            msg = (f"[{t:7.1f}s] SÉCURITÉ arrêt : {v_.raison} — obstacle en "
                   f"{None if pt is None else (round(pt[0], 2), round(pt[1], 2))}, "
                   f"robot en ({pr_.x:.2f}, {pr_.y:.2f}) — {quoi}")
            print("  " + msg, flush=True)
            if suivi is not None:
                suivi.journal(msg)
        elif v_ is not None and v_.facteur > 0.0:
            arret_signale = False
        # Pendant un arrêt qui DURE : toutes les 10 s simulées, l'état complet.
        # Premier essai GPU : arrêt de plus de 60 s alors que la personne
        # devait partir après 45 s — on veut voir où elle est vraiment, pour
        # la simulation (f.p) ET pour la physique (ce que voit le lidar).
        if v_ is not None and v_.facteur <= 0.0 and k % 500 == 0:
            pr_ = pose2d()
            print(f"  [sécurité {t:6.1f} s] toujours arrêté : {v_.raison} — point "
                  f"{None if v_.point is None else tuple(round(c, 2) for c in v_.point)}, "
                  f"robot ({pr_.x:.2f}, {pr_.y:.2f})", flush=True)
            print(f"      lidar : {lidar.stats}", flush=True)
            for i_, (f_, s_) in enumerate(zip(figurants, silhouettes)):
                ph = s_.get_world_pose()[0]
                print(f"      personne n°{i_} : prévue ({f_.p[0]:.2f}, {f_.p[1]:.2f}), "
                      f"physique ({float(ph[0]):.2f}, {float(ph[1]):.2f}), "
                      f"à {math.hypot(f_.p[0] - pr_.x, f_.p[1] - pr_.y):.2f} m du robot, "
                      f"pause {max(0.0, f_.attente):.0f} s", flush=True)
        if not ordre_donne and t > DUREE / 2 and not COLLECTE:
            sup.ordonner("les colis bleus vont maintenant au poste C")
            ordre_donne = True
        # Rendu : pour la perception (au prochain pas, la logique cherche)
        # et pour la vidéo.
        percevoir = tri.etat == "CHERCHER" or facade.veut_percevoir or (
            COLLECTE and collecteur.veut_rendu(t + cfg.control_dt))
        filmer = video_ok and k % pas_video == 0
        if percevoir or filmer:
            p_, q_ = robot.get_world_pose()
            R = quat_wxyz_to_matrix(q_)
            p_w = np.asarray(p_, dtype=float)
            for c in (cam, dep):
                c._position = tuple(float(v) for v in (p_w + R @ np.array([0.2, 0.0, 0.4])))
                c.look_at(tuple(float(v) for v in (p_w + R @ np.array([1.5, 0.0, 0.05]))))
            tree.set(Transform(FRAME_WORLD, FRAME_BASE,
                               translation=Vector3(*(float(v) for v in p_w)),
                               rotation=Quaternion(w=float(q_[0]), x=float(q_[1]),
                                                   y=float(q_[2]), z=float(q_[3]))))
            tf = cam.optical_transform(parent=FRAME_WORLD, child=FRAME_CAMERA_OPTICAL)
            if tf is not None:
                tree.set(tf)
            if filmer:
                rec.cadrer()
        # send_targets (dans loco.tick) a DÉJÀ fait un pas de physique : on
        # complète la décimation. En faire un de trop décalerait le temps
        # simulé de 25 % et ferait tourner GR00T hors de sa fréquence.
        for i in range(cfg.decimation - 1):
            world.step(render=((percevoir or filmer) and i == 0))
        sim_t[0] += cfg.control_dt
        k += 1
        # Invariant : le robot ne se rapproche jamais d'une personne proche.
        pa = pose2d()
        for f in figurants:
            d = math.hypot(f.p[0] - pa.x, f.p[1] - pa.y) - 0.28   # surface de la silhouette
            d_min_humain = min(d_min_humain, d)
            d_av = math.hypot(f.p[0] - pr.x, f.p[1] - pr.y) - 0.28
            i_f = figurants.index(f)
            if d >= 0.75:
                rapproche.pop(i_f, None)
                continue
            if d < d_av - 1e-3:
                pas_vers_humain += 1
                cumul = rapproche.get(i_f, 0.0) + (d_av - d)
                rapproche[i_f] = cumul
                rapproche_max = max(rapproche_max, cumul)
                if cumul > 0.01:
                    viol_humain += 1
                if lignes_rappr < 12:
                    lignes_rappr += 1
                    t_ = getattr(loco, "_twist", None)
                    tw_ = None if t_ is None else \
                        f"vx {t_.vx:+.2f} vy {t_.vy:+.2f} wz {t_.wz:+.2f}"
                    print(f"  [rapprochement {sim_t[0]:6.1f} s] PERSONNE n°{i_f} à {d:.2f} m, "
                          f"robot rapproché de {100 * (d_av - d):.1f} cm (cumul "
                          f"{100 * cumul:.1f} cm), tâche {tri.etat}, "
                          f"sécurité x{getattr(loco.dernier_verdict, 'facteur', float('nan')):.2f}"
                          f"{'' if tw_ is None else f', consigne {tw_}'}", flush=True)
        if filmer:
            b = tri.bilan
            v = loco.dernier_verdict
            rec.image([
                f"Temps : {int(t // 60):02d}:{int(t % 60):02d}",
                f"Colis triés : {b.colis_tries}  "
                + " ".join(f"{c} {n}" for c, n in sorted(b.par_couleur.items())),
                f"Taux de réussite : {100 * b.taux_reussite():.0f} %",
                f"Arrêts de sécurité : {gardien.stats['arrets']} "
                f"({b.temps_arret_securite_s:.0f} s)",
                f"Sécurité : {v.raison[:38] if v else '—'}",
                f"Tâche : {tri.etat}",
                "Règles : " + ", ".join(f"{c}→{p}" for c, p in sorted(tri.cfg.regles.items())),
                f"Garde-fou : {loco.stats['refused']} refus",
            ], t)
        if suivi is not None and k % 25 == 0:
            b_ = tri.bilan
            suivi.resume(f"{int(t // 60):02d}:{int(t % 60):02d} simulé · "
                         f"{b_.colis_tries} triés · tâche {tri.etat} · "
                         f"{time.monotonic() - t_mur:.0f} s réelles")
        if k % 2500 == 0:
            if COLLECTE:
                print(f"  … t={t:6.0f} s simulées, {time.monotonic() - t_mur:6.0f} s réelles, "
                      f"collecte {collecteur.stats}, état {collecteur.etat}", flush=True)
            else:
                print(f"  … t={t:6.0f} s simulées, {time.monotonic() - t_mur:6.0f} s réelles, "
                      f"{tri.bilan.colis_tries} triés, état {tri.etat}")

    if video_ok:
        rec.terminer()
    if suivi is not None:
        suivi.resume("test terminé — bilan dans le terminal")

    # ---- bilan ----------------------------------------------------------------
    if COLLECTE:
        if enreg.actif:
            enreg.fin(False, sim_t[0], raison="collecte interrompue")
        print(f"\n{'=' * 66}\n  COLLECTE VLA\n{'=' * 66}")
        print(f"  épisodes : {collecteur.stats}\n  images : {enreg.stats['images']}"
              f"\n  dossier : {args.collecte_vla}\n  garde-fou : {loco.stats['refused']} refus"
              f"\n  gardien : {gardien.stats}\n  temps réel : {time.monotonic() - t_mur:.0f} s")
        if chute:
            print(f"  CHUTE à t = {chute[0]:.1f} s")
        simulation_app.close()
        sys.exit(0)
    b = tri.bilan
    print(f"\n{'=' * 66}\n  RÉSULTATS T1\n{'=' * 66}")
    print("  " + sup.rapport())
    print(f"  gardien : {gardien.stats}\n  navigation : {nav.stats}\n  façade : {facade.stats}"
          f"\n  manipulation : {manip.stats}\n  lidar : {lidar.stats}"
          f"\n  couleur : {percep_couleur.stats}\n  cartons remis en place par l'opérateur : {stats.get('remises_en_place', 0)}"
          f"\n  écrêtages dans les butées (avant le garde-fou) : {adapt.stats}"
          f"\n  personne la plus proche : {d_min_humain:.2f} m"
          f"\n  temps réel : {time.monotonic() - t_mur:.0f} s")
    if chute:
        print(f"  CHUTE à t = {chute[0]:.1f} s, état {chute[1]}")
    check("Le robot ne tombe pas", chute is None)
    check(f"Colis triés : {b.colis_tries} (au moins un par 3 min)",
          b.colis_tries >= max(1, int(DUREE / 180)))
    bons = [c for c, lieu in stats["sur_plateau"]
            if lieu in ("A", "B", "C")]
    check(f"Cartons arrivés sur un plateau de dépose : {len(bons)}", len(bons) >= b.colis_tries)
    check(f"Aucun carton tombé au sol ({stats['chutes_carton']})", stats["chutes_carton"] == 0)
    check(f"Le robot ne s'est jamais rapproché de plus de 1 cm d'une personne à moins "
          f"de 0.75 m ({viol_humain} pas en défaut ; {pas_vers_humain} pas d'oscillation, "
          f"rapprochement cumulé max {100 * rapproche_max:.1f} cm)", viol_humain == 0)
    check("Aucune consigne refusée par le garde-fou", loco.stats["refused"] == 0)
    check("Fin propre, bras vides", tri.etat == "FIN" and manip.etat is Etat.LIBRE)
    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
