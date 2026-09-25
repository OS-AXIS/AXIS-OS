"""
Suite W4 — « Va prendre le carton. » Sans lui dire où il est.

Le robot ne reçoit AUCUNE position. Il doit :

  CHERCHER    tourner sur lui-même jusqu'à ce qu'OWL-ViT reconnaisse
              « a cardboard box » dans l'image de sa caméra ;
  APPROCHER   marcher vers le carton en recalculant sa position à chaque
              image, et s'ALIGNER pour que ses mains l'encadrent ;
  SAISIR      serrer et soulever — déclenché par la perception, quand il
              est en place, pas à une seconde fixée d'avance ;
  RECULER, SE RETOURNER, EMPORTER, DÉPOSER au sol.

CE QUI RELÈVE DE L'IA : la recherche et la localisation (OWL-ViT +
profondeur), et tout l'équilibre et la marche (GR00T WBC). CE QUI RESTE
UN GESTE PROGRAMMÉ : le mouvement des bras lui-même. OWL-ViT voit, il ne
commande pas de bras : l'app sait COMMENT on prend un carton, la
perception lui dit QUAND et OÙ.

Scène : carton sur une table SUR LE CÔTÉ GAUCHE du robot, hors de son
champ de vision au départ. Le robot doit le trouver.

Deux limites, assumées et signalées :
  - fin d'approche À L'AVEUGLE : sous 0.5 m environ, le carton sort du
    champ de la caméra ; le robot finit sur la dernière position vue,
    recalée par sa propre pose. En simulation la pose est exacte ; sur le
    vrai robot, il faudra une odométrie.
  - la vérification de la prise lit la position du carton dans la
    simulation. Sur le vrai robot, ce serait l'effort mesuré aux épaules.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_w4_gr00t_autonome.py
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
FAIL = 0

# ---- scène (INCONNUE du robot) ---------------------------------------
MASSE_KG, PROFONDEUR, LARGEUR, HAUTEUR = 1.0, 0.20, 0.24, 0.20
TABLE_HAUT = 0.80
# Carton à gauche du robot, sur une table orientée vers lui.
CARTON = (0.0, 1.49)
# Plateau fin qui s'avance au-dessus des cuisses, pied en arrière (W3).
PLATEAU_Y = (1.38, 1.80)
PIED_Y = (1.62, 1.80)

# ---- savoir-faire de l'app (postures validées par W3) -----------------
EPAULE_TANGAGE, EPAULE_LEVEE, COUDE = -0.30, -0.50, 0.0
ROULIS_OUVERT, ROULIS_SERRE = 0.20, -0.40
KP_BRAS = [90, 100, 40, 60, 40, 40, 40]
KD_BRAS = [2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0]
MAINS_DEVANT = 0.29          # le carton doit finir à 29 cm devant le bassin
TOL_AVANT, TOL_COTE = 0.03, 0.025


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    import omni.usd
    from pxr import UsdShade
    from isaacsim.core.api.materials import PhysicsMaterial
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    from isaacsim.core.utils.types import ArticulationAction

    from axis_os.hal.core.frames import (
        FRAME_BASE, FRAME_CAMERA_OPTICAL, FRAME_WORLD, FrameTree, Transform,
    )
    from axis_os.hal.core.types import (
        ActuatorCommand, ControlMode, Quaternion, Vector3,
    )
    from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera
    from axis_os.hal.drivers.isaac_sim.video import SceneRecorder
    from axis_os.hal.drivers.unitree_g1.gr00t_wbc_policy import (
        GR00TWBCAdapter, GR00TWBCConfig, GR00TWBCPolicy,
    )
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        body_angular_velocity, quat_wxyz_to_matrix,
    )
    from axis_os.services.locomotion.service import (
        BaseState, LocomotionService, LocomotionState,
    )
    from axis_os.services.perception.owlvit_detector import OwlViTDetector
    from axis_os.services.perception.service import PerceptionService

    cfg = GR00TWBCConfig()
    pol = GR00TWBCPolicy(POLICIES / "gr00t_wbc_balance.onnx",
                         POLICIES / "gr00t_wbc_walk.onnx", cfg)
    pol.load()

    def pose_bras(roulis, tangage=EPAULE_TANGAGE):
        return {"left_shoulder_pitch_joint": tangage,
                "right_shoulder_pitch_joint": tangage,
                "left_shoulder_roll_joint": roulis,
                "right_shoulder_roll_joint": -roulis,
                "left_elbow_joint": COUDE, "right_elbow_joint": COUDE}

    bridge = IsaacSimBridge(variante="29dof")
    bridge.setup(fix_base=False, require_meshes=True, physics_dt=cfg.physics_dt)
    robot, world = bridge._robot, bridge._world
    idx = bridge._joint_index
    names = sorted(idx, key=lambda j: idx[j])

    # ---- scène, TOUT posé avant world.reset() -----------------------
    world.scene.add(FixedCuboid(
        prim_path="/World/table_plateau", name="table_plateau",
        position=np.array([CARTON[0], sum(PLATEAU_Y) / 2, TABLE_HAUT - 0.02]),
        size=1.0, scale=np.array([0.50, PLATEAU_Y[1] - PLATEAU_Y[0], 0.04]),
        color=np.array([0.45, 0.45, 0.50])))
    world.scene.add(FixedCuboid(
        prim_path="/World/table_pied", name="table_pied",
        position=np.array([CARTON[0], sum(PIED_Y) / 2, (TABLE_HAUT - 0.04) / 2]),
        size=1.0, scale=np.array([0.30, PIED_Y[1] - PIED_Y[0], TABLE_HAUT - 0.04]),
        color=np.array([0.35, 0.35, 0.40])))
    carton = world.scene.add(DynamicCuboid(
        prim_path="/World/carton", name="carton",
        position=np.array([CARTON[0], CARTON[1], TABLE_HAUT + HAUTEUR / 2 + 0.002]),
        size=1.0,
        # tourné de 90° : le carton fait face au robot qui arrive par -y
        scale=np.array([LARGEUR, PROFONDEUR, HAUTEUR]), mass=MASSE_KG,
        color=np.array([0.72, 0.52, 0.30]),
        physics_material=PhysicsMaterial(prim_path="/World/Materiaux/carton",
                                         static_friction=1.0,
                                         dynamic_friction=0.8)))
    PhysicsMaterial(prim_path="/World/Materiaux/caoutchouc",
                    static_friction=1.2, dynamic_friction=1.0)
    stage = omni.usd.get_context().get_stage()
    mat = UsdShade.Material(stage.GetPrimAtPath("/World/Materiaux/caoutchouc"))
    for prim in stage.Traverse():
        n = prim.GetName()
        if "/World/g1" in str(prim.GetPath()) and (
                "rubber_hand" in n or "wrist_yaw_link" in n):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    cam = IsaacSimCamera("tete_rgb", prim_path="/World/tete_rgb",
                         width=640, height=480, position=(0.2, 0.0, 1.2))
    dep = IsaacSimCamera("tete_prof", prim_path="/World/tete_prof",
                         width=640, height=480, position=(0.2, 0.0, 1.2),
                         depth=True)

    g = pol.gains()
    for i, j in enumerate(cfg.body_joints[cfg.num_actions:]):
        g[j] = (KP_BRAS[i % 7], KD_BRAS[i % 7])
    world.reset()
    robot.get_articulation_controller().set_gains(
        kps=np.array([g[j][0] for j in names]),
        kds=np.array([g[j][1] for j in names]))
    q0 = {j: 0.0 for j in names}
    q0.update(dict(zip(cfg.controlled_joints, cfg.default_angles)))
    q0.update(pose_bras(ROULIS_OUVERT))
    q0v = np.array([q0[j] for j in names])
    robot.set_joint_positions(q0v)
    robot.set_joint_velocities(np.zeros_like(q0v))
    robot.apply_action(ArticulationAction(joint_positions=q0v))

    for c in (cam, dep):
        c.initialize()
    cam.wait_until_ready(world)
    dep.wait_until_ready(world)
    tree = FrameTree()
    detecteur = OwlViTDetector(threshold=0.09)
    detecteur.load()
    perception = PerceptionService(cam, tree, detecteur, depth_camera=dep)
    perception.start()

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

    adapt = GR00TWBCAdapter(pol, pose_bras=pose_bras(ROULIS_OUVERT))
    loco = LocomotionService(adapt, read_state, send_targets,
                             command_timeout_s=0.5, clock=lambda: sim_t[0])
    loco.start()

    rec = SceneRecorder(world, "/isaac-sim/videos/w4_autonome.mp4",
                        position=(2.2, -1.2, 1.6), cible=(0.0, 0.6, 0.8),
                        largeur=1280, hauteur=720,
                        titre="AXIS-OS — « Va prendre le carton »")
    rec.suivre(lambda: robot.get_world_pose()[0], decalage=(2.4, -1.4, 0.7),
               hauteur_visee=0.9, decalage_cadre=(-0.2, 0.4))
    video_ok = rec.start()

    def lacet(q):
        w, x, y, z = (float(v) for v in q)
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def vers_robot(p_monde):
        """Point du monde -> repère du robot (x devant, y à gauche)."""
        pos, quat = robot.get_world_pose()
        R = quat_wxyz_to_matrix(quat)
        return R.T @ (np.asarray(p_monde, dtype=float) - np.asarray(pos, dtype=float))

    # ---- l'APP : une machine à états ----------------------------------
    etat = "CHERCHER"
    estim = None                 # centre du carton estimé, repère MONDE
    vues = 0
    en_place = 0
    t_etat = 0.0
    lacet_depart = None
    x_depart = None
    journal = []
    chute = None
    z_table = TABLE_HAUT + HAUTEUR / 2

    def changer(nouvel, t, texte, couleur=(40, 110, 200)):
        nonlocal etat, t_etat
        etat, t_etat = nouvel, t
        journal.append((round(t, 1), nouvel))
        print(f"  [{t:5.1f} s] {nouvel:10} — {texte}")
        if video_ok:
            rec.evenement(texte, t, duree_s=2.5, couleur=couleur)

    changer("CHERCHER", 0.0, "« Va prendre le carton » — recherche")
    k = 0
    t_mur = time.monotonic()
    while sim_t[0] < 60.0 and time.monotonic() - t_mur < 1800:
        t = sim_t[0]
        rendre = (k % 2 == 0)
        percevoir = (k % 5 == 0) and etat in ("CHERCHER", "APPROCHER")

        # -- caméra de tête, montée sur le robot, et arbre des repères --
        if rendre or percevoir:
            pos, quat = robot.get_world_pose()
            R = quat_wxyz_to_matrix(quat)
            p_w = np.asarray(pos, dtype=float)
            for c in (cam, dep):
                c._position = tuple(float(v) for v in (p_w + R @ np.array([0.2, 0.0, 0.4])))
                c.look_at(tuple(float(v) for v in (p_w + R @ np.array([1.5, 0.0, -0.55]))))
            tree.set(Transform(FRAME_WORLD, FRAME_BASE,
                               translation=Vector3(*(float(v) for v in p_w)),
                               rotation=Quaternion(w=float(quat[0]), x=float(quat[1]),
                                                   y=float(quat[2]), z=float(quat[3]))))
            tf = cam.optical_transform(parent=FRAME_WORLD, child=FRAME_CAMERA_OPTICAL)
            if tf is not None:
                tree.set(tf)

        # -- perception : où est le carton ? --
        if percevoir:
            r = perception.find("a cardboard box", frame=FRAME_WORLD)
            for d in (r.value or []) if r.ok else []:
                p = d.position_3d
                if p is None or not (0.6 < p.z < 1.3):
                    continue           # ni le sol, ni le plafond
                # La profondeur mesure la FACE AVANT : le centre est 10 cm
                # plus loin, dans l'axe du regard.
                pr, _ = robot.get_world_pose()
                v = np.array([p.x - float(pr[0]), p.y - float(pr[1])])
                v = v / max(np.linalg.norm(v), 1e-6)
                c = np.array([p.x + v[0] * PROFONDEUR / 2,
                              p.y + v[1] * PROFONDEUR / 2])
                estim = c if estim is None else 0.7 * estim + 0.3 * c
                vues += 1
                break

        # -- décisions --
        if etat == "CHERCHER":
            loco.set_velocity(0.0, 0.0, 0.4)           # balayer
            if vues >= 3:
                changer("APPROCHER", t, "Carton reconnu par OWL-ViT — approche")

        elif etat == "APPROCHER" and estim is not None:
            cible = vers_robot([estim[0], estim[1], 0.0])
            ex, ey = cible[0] - MAINS_DEVANT, cible[1]
            cap = math.atan2(cible[1], cible[0])
            loin = cible[0] > 0.7
            vx = float(np.clip(0.8 * ex, -0.10, 0.25 if loin else 0.10))
            vy = 0.0 if loin else float(np.clip(1.0 * ey, -0.10, 0.10))
            wz = float(np.clip(1.5 * cap, -0.5, 0.5)) if loin else float(
                np.clip(1.0 * cap, -0.2, 0.2))
            if abs(cap) > 0.4:
                vx = 0.0                                # d'abord s'orienter
            loco.set_velocity(vx, vy, wz)
            if abs(ex) < TOL_AVANT and abs(ey) < TOL_COTE:
                en_place += 1
            else:
                en_place = 0
            if en_place >= 15:                          # 0.3 s stable
                loco.halt()
                changer("SAISIR", t, "En place — il serre le carton")

        elif etat == "SAISIR":
            loco.halt()
            a = min(1.0, (t - t_etat) / 0.5)
            adapt.pose_bras.update(pose_bras(ROULIS_OUVERT + a * (ROULIS_SERRE - ROULIS_OUVERT)))
            if t - t_etat > 0.8:
                changer("SOULEVER", t, "Il soulève le carton")

        elif etat == "SOULEVER":
            loco.halt()
            a = min(1.0, (t - t_etat) / 0.8)
            adapt.pose_bras.update(pose_bras(
                ROULIS_SERRE, EPAULE_TANGAGE + a * (EPAULE_LEVEE - EPAULE_TANGAGE)))
            if t - t_etat > 1.2:
                zc = float(carton.get_world_pose()[0][2])
                if zc > z_table + 0.015:
                    changer("RECULER", t, "Prise confirmée — il recule",
                            (40, 140, 70))
                else:
                    changer("ECHEC", t, "Prise NON confirmée — il s'arrête",
                            (200, 50, 45))

        elif etat == "RECULER":
            loco.set_velocity(-0.2, 0.0, 0.0)
            if t - t_etat > 2.5:
                lacet_depart = lacet(robot.get_world_pose()[1])
                changer("TOURNER", t, "Demi-tour avec le carton", (40, 140, 70))

        elif etat == "TOURNER":
            loco.set_velocity(0.0, 0.0, 0.6)
            dl = (lacet(robot.get_world_pose()[1]) - lacet_depart + math.pi) % (2 * math.pi) - math.pi
            if abs(dl) > math.pi * 0.9 or t - t_etat > 8.0:
                x_depart = np.asarray(robot.get_world_pose()[0], dtype=float)
                changer("EMPORTER", t, "Il emporte le carton", (40, 140, 70))

        elif etat == "EMPORTER":
            loco.set_velocity(0.3, 0.0, 0.0)
            d = np.linalg.norm(np.asarray(robot.get_world_pose()[0])[:2] - x_depart[:2])
            if d > 1.0:
                changer("DEPOSER", t, "Il dépose le carton au sol", (40, 140, 70))

        elif etat == "DEPOSER":
            loco.halt()
            a = min(1.0, (t - t_etat) / 1.0)
            adapt.pose_bras.update(pose_bras(
                ROULIS_SERRE + a * (ROULIS_OUVERT - ROULIS_SERRE),
                EPAULE_LEVEE + a * (0.3 - EPAULE_LEVEE)))
            if t - t_etat > 2.5:
                changer("FINI", t, "Mission accomplie", (40, 140, 70))

        elif etat in ("FINI", "ECHEC"):
            loco.halt()
            if t - t_etat > 2.0:
                break

        loco.tick()
        if loco.state is LocomotionState.FALLEN:
            chute = (t, etat)
            break
        if rendre and video_ok:
            rec.cadrer()
        for i in range(cfg.decimation - 1):
            world.step(render=((rendre or percevoir) and i == 0))
        sim_t[0] += cfg.control_dt
        k += 1
        if rendre and video_ok:
            zc = float(carton.get_world_pose()[0][2])
            rec.image([f"État : {etat}",
                       f"Carton vu : {vues} fois" if vues else "Carton : pas encore vu",
                       f"Politique : {pol.derniere_politique}",
                       f"Hauteur du carton : {zc:.2f} m",
                       f"Garde-fou : {loco.stats['refused']} refus"], t)

    if video_ok:
        rec.terminer()

    print(f"\n{'=' * 66}\n  RÉSULTATS\n{'=' * 66}")
    print(f"  parcours : {journal}")
    pc = np.asarray(carton.get_world_pose()[0], dtype=float)
    print(f"  carton final : ({pc[0]:+.2f}, {pc[1]:+.2f}, {pc[2]:.2f})")
    if chute:
        print(f"  CHUTE à t = {chute[0]:.1f} s, état {chute[1]}")
    check("Le robot ne tombe pas", chute is None)
    etats = [e for _, e in journal]
    check("Il a TROUVÉ le carton sans qu'on lui dise où", "APPROCHER" in etats)
    check("Il s'est placé tout seul pour le saisir", "SAISIR" in etats)
    check("Il a SOULEVÉ le carton (prise confirmée)", "RECULER" in etats)
    check("Il l'a emporté et déposé", "FINI" in etats)
    loin_table = math.hypot(pc[0] - CARTON[0], pc[1] - CARTON[1])
    check(f"Carton déposé AU SOL, loin de la table ({loin_table:.2f} m)",
          pc[2] < 0.3 and loin_table > 1.0)
    check("Aucune consigne refusée par le garde-fou", loco.stats["refused"] == 0)

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
