"""
Suite W5 — la posture de GR00T WBC : torse penché, bassin abaissé, robot
immobile, sans puis avec un carton aux mains (lancer_posture.sh).

La manipulation se penche pour atteindre plus loin par la consigne de
posture du service de marche (LocomotionService.posture : bornée, centre de
masse vérifié sur un modèle statique, marge de 2 cm). Décision de revue :
l'app de tri ne se penche pas tant que ce banc n'est pas passé.

  A. 0–3 s    debout
  B. 3–5 s    bras en posture de port
  C.          calibration des bras (couples et mains), celle du banc T1
  1. sans carton : trois séries de paliers (banc_posture.paliers) — torse
     de 0,05 à 0,35 rad, bassin de 2,5 à 10 cm, puis les deux ensemble ;
     chaque palier par LocomotionService.posture, tenu MAINTIEN_S une fois
     atteint, et droit entre deux séries
  2. le carton T1 (1 kg, dans les moyens des mains) posé sur un plateau
     devant le robot, là où T1 le prend, et pris par l'OS avec la même prise
     que T1 (ManipulationService.prendre : gains du banc T1, poignets, coude
     et lacet d'épaule raidis pendant la prise, mains caoutchouc, couples
     estimés comme ceux du G1) ; le plateau retiré, seule la prise le retient
  3. avec carton : les mêmes séries, le carton dans le bilan d'équilibre
     (masse et position, d'après les angles mesurés des bras)

Essai GPU du 19 novembre 2026, 21 h 51 : sans carton, tout passait ; le
carton de 1,2 kg, glissé entre des mains aux poignets mous (kp 4), sans
caoutchouc, était « LÂCHÉ par la seule prise » ; la caméra spectateur ne
rendait rien (le seul dôme du pont : l'éclairage est désormais celui de T1).

Par palier (videos/posture.csv, banc_posture.COLONNES) : accepté ou refusé
par l'OS, marge d'équilibre prévue ; tangage mesuré de la taille ;
inclinaison et vitesse angulaire du bassin (centrale inertielle) ;
glissement des pieds (vérité de la simulation, pour le banc seul). Au
premier signe de perte d'équilibre (banc_posture.signe_de_perte), la série
s'arrête et le robot revient droit.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_w5_gr00t_posture.py
"""
import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (  # noqa: E402
    IsaacSimBridge, start_simulation_app,
)
from banc_posture import (  # noqa: E402
    COLONNES, MAINTIEN_S, Palier, inclinaison_deg, paliers, verdict,
)
from banc_prise import (  # noqa: E402
    demarrer_services, gains_prise, objets_inconnus, plateau_site, poser_devant,
    preparer_scene, profil_robot_banc,
)

POLICIES = Path("/isaac-sim/policies")
SORTIE = Path("/isaac-sim/videos")
PROFONDEUR, LARGEUR, HAUTEUR = 0.20, 0.24, 0.20     # le carton de la scène T1
MASSE_CARTON = 1.0              # le carton T1 : au plus 1 kg, dans les moyens des mains
X_CARTON = 0.44                 # son centre devant le bassin, là où T1 le prend
# Posture de port en angles bruts, celle de W2 et de l'OS (Postures : mains
# 29 cm devant le bassin, 18 cm au-dessus, écartées de 34 cm).
EPAULE_TANGAGE, COUDE = -0.30, 0.0
ROULIS_OUVERT = 0.20
# La même prise que T1 : ses gains de bras (raidis pendant la prise par
# banc_prise.gains_prise) et ses couples estimés (bruit_couples, sa graine).
KP_BRAS = [90, 100, 40, 60, 40, 40, 40]
KD_BRAS = [2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0]
GRAINE_COUPLES = 3
RETOUR_MAX_S = 10.0             # le retour droit, au plus
CALIBRATION_MAX_S = 40.0        # la calibration des bras, au plus
PRISE_MAX_S = 40.0              # la prise de l'OS, au plus
FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    import omni.usd
    from pxr import Usd, UsdGeom, UsdPhysics
    from isaacsim.core.api.materials import PhysicsMaterial
    from isaacsim.core.api.objects import DynamicCuboid
    from isaacsim.core.utils.types import ArticulationAction

    from axis_os.apps.tri_colis import scene_demo as sd
    from axis_os.hal.core.types import ActuatorCommand, ControlMode
    from axis_os.hal.drivers.isaac_sim.bruit_couples import BruitCouples
    from axis_os.hal.drivers.isaac_sim.video import SceneRecorder
    from axis_os.hal.drivers.unitree_g1.gr00t_wbc_policy import (
        GR00TWBCAdapter, GR00TWBCConfig, GR00TWBCPolicy,
    )
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        body_angular_velocity, quat_wxyz_to_matrix,
    )
    from axis_os.hal.drivers.unitree_g1.robot import build_g1_joint_definitions
    from axis_os.services.locomotion.equilibre import TAILLE
    from axis_os.services.locomotion.service import (
        BaseState, LocomotionService, LocomotionState,
    )
    from axis_os.services.manipulation.cinematique import ModeleBras
    from axis_os.services.manipulation.estimation import HauteurBassin
    from axis_os.services.manipulation.prise import CalibrationMains, MesuresBras, ReglagesPrise
    from axis_os.services.profil_robot import ProfilRobotErreur, lire_profil
    from axis_os.services.manipulation.service import (
        Etat, ManipulationService, Objet, Postures, posture_bras,
    )

    cfg = GR00TWBCConfig()
    pol = GR00TWBCPolicy(POLICIES / "gr00t_wbc_balance.onnx",
                         POLICIES / "gr00t_wbc_walk.onnx", cfg)
    pol.load()

    def pose(roulis):
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
    # Couples ESTIMÉS, comme ceux du vrai G1 et du banc T1 (même graine).
    bridge.bruit_couples = BruitCouples(len(names), graine=GRAINE_COUPLES)
    stage = omni.usd.get_context().get_stage()

    # Scène : posée avant world.reset(). Le carton T1 et son plateau, créés AU DÉPART, loin ; posés devant le
    # robot au moment de la prise (le plateau, cinématique, se déplace).
    materiau = PhysicsMaterial(prim_path="/World/Materiaux/carton",
                               static_friction=1.0, dynamic_friction=0.8, restitution=0.0)
    carton = world.scene.add(DynamicCuboid(
        prim_path="/World/carton", name="carton",
        position=np.array([10.0, 0.0, HAUTEUR / 2]), size=1.0,
        scale=np.array([PROFONDEUR, LARGEUR, HAUTEUR]), mass=MASSE_CARTON,
        color=np.array([0.72, 0.52, 0.30]), physics_material=materiau))
    plateau = world.scene.add(DynamicCuboid(
        prim_path="/World/plateau", name="plateau",
        position=np.array([10.0, 3.0, sd.PLATEAU / 2]), size=1.0,
        scale=np.array([0.40, 0.60, sd.PLATEAU]), mass=50.0,
        color=np.array([0.55, 0.55, 0.58])))
    UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath("/World/plateau")).CreateKinematicEnabledAttr(True)
    # Préparation commune des bancs (banc_prise.preparer_scene, celle de T1),
    # APRÈS les corps rigides : mains caoutchouc et éclairage de T1, puis
    # world.reset(). Posés avant les corps, ils invalidaient la vue physique
    # et DynamicCuboid plantait (« Simulation view object is invalidated »,
    # essai GPU du 20/11).
    preparer_scene(world, stage, journal=lambda m_: print(f"  {m_}", flush=True))

    g = pol.gains()
    for i, j in enumerate(cfg.body_joints[cfg.num_actions:]):
        g[j] = (KP_BRAS[i % 7], KD_BRAS[i % 7])
    robot.get_articulation_controller().set_gains(
        kps=np.array([g[j][0] for j in names]), kds=np.array([g[j][1] for j in names]))
    # Gains de prise : poignets, coude et lacet d'épaule raidis de la
    # fermeture au dépôt, à la demande de l'OS (MesuresBras.raideur), comme T1.
    g_prise = gains_prise(g)

    def regler_raideur(active):
        gg = g_prise if active else g
        robot.get_articulation_controller().set_gains(
            kps=np.array([gg[j][0] for j in names]), kds=np.array([gg[j][1] for j in names]))
    q0 = {j: 0.0 for j in names}
    q0.update(dict(zip(cfg.controlled_joints, cfg.default_angles)))
    q0v = np.array([q0[j] for j in names])
    robot.set_joint_positions(q0v)
    robot.set_joint_velocities(np.zeros_like(q0v))
    robot.apply_action(ArticulationAction(joint_positions=q0v))
    # Pose d'apparition : celle où le robot est remis debout avant la boucle.
    pos0, quat0 = robot.get_world_pose()
    lacet0 = 2.0 * math.atan2(float(quat0[3]), float(quat0[0]))

    hal = bridge.build_hal()
    hal.initialize_all()
    sim_t = [0.0]
    t_physique = [0.0]                   # temps simulé du dernier pas de physique

    def read_state() -> BaseState:
        _p, quat = robot.get_world_pose()
        omega = body_angular_velocity(quat, robot.get_angular_velocity())
        jp, jv = robot.get_joint_positions(), robot.get_joint_velocities()
        return BaseState(tuple(float(v) for v in quat), tuple(float(v) for v in omega),
                         {j: float(jp[idx[j]]) for j in names},
                         {j: float(jv[idx[j]]) for j in names})

    def send_targets(targets):
        return hal.send_joint_commands(
            {j: ActuatorCommand(mode=ControlMode.POSITION, position=v)
             for j, v in targets.items()})

    adapt = GR00TWBCAdapter(pol)
    loco = LocomotionService(adapt, read_state, send_targets,
                             command_timeout_s=0.5, clock=lambda: sim_t[0])
    modele = ModeleBras()

    # ---- la prise de l'OS, branchée comme dans le banc T1 -----------------
    chaine_bras = [j for j in names if j.startswith(("waist_", "left_shoulder", "right_shoulder",
                                                     "left_elbow", "right_elbow", "left_wrist",
                                                     "right_wrist"))]
    bras_tous = [j for j in chaine_bras if not j.startswith("waist_")]

    def efforts_bras():
        # Couple ESTIMÉ (effort mesuré + erreurs d'estimation du G1), celui
        # que lit aussi le JointSafetyGuard.
        e = bridge.couples()
        if e is None:
            return None
        return {j: (float(e[idx[j]]), sim_t[0] - t_physique[0]) for j in bras_tous}

    def vers_robot(p_monde):
        pos, quat = robot.get_world_pose()
        return quat_wxyz_to_matrix(quat).T @ (np.asarray(p_monde, dtype=float)
                                              - np.asarray(pos, dtype=float))

    def mains():
        """Mains vues par l'OS : cinématique sur les angles mesurés, posée
        dans le monde par la pose du bassin."""
        p_, q_ = robot.get_world_pose()
        r_ = quat_wxyz_to_matrix(q_)
        g_, d_ = mesures_bras.modele.mains({j: read_state().joint_pos[j] for j in chaine_bras})
        p_ = np.asarray(p_, dtype=float)
        return p_ + r_ @ g_, p_ + r_ @ d_

    hauteur_bassin = HauteurBassin()
    mesures_bras = MesuresBras(
        ModeleBras(), lambda: {j: read_state().joint_pos[j] for j in chaine_bras}, efforts_bras,
        lambda: hauteur_bassin(read_state().joint_pos,
                               quat_wxyz_to_matrix(robot.get_world_pose()[1])),
        couple_max={d.joint_id: d.limits.torque_max for d in build_g1_joint_definitions("29dof")
                    if d.joint_id in bras_tous},
        raideur=regler_raideur,
        arret_urgence=lambda: hal.estop_active,
        commandables=tuple(adapt.pose_bras),
        vers_bassin=vers_robot)
    manip = ManipulationService(adapt.pose_bras, mains, vers_robot,
                                lambda: manip.objet_entre_les_mains(),
                                horloge=lambda: sim_t[0], mesures=mesures_bras,
                                journal=lambda m: print(f"  [{sim_t[0]:6.1f}s] {m}", flush=True))
    # Services démarrés comme dans T1 (banc_prise.demarrer_services) : sans
    # elle, prendre() refusait « service non démarré » (essai GPU du 20/11,
    # série avec carton jamais faite).
    demarrer_services([("locomotion", loco), ("manipulation", manip)],
                      journal=lambda m_: print(f"  {m_}", flush=True))

    chemins = {}

    def position_lien(nom):
        """Position MONDE d'un lien du robot, lue dans la scène (voir W2)."""
        for candidat in (nom, nom.replace("rubber_hand", "wrist_yaw_link")):
            if candidat not in chemins:
                for prim in stage.Traverse():
                    if prim.GetName() == candidat and "/World/g1" in str(prim.GetPath()):
                        chemins[candidat] = prim
                        break
            prim = chemins.get(candidat)
            if prim is not None:
                m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                t = m.ExtractTranslation()
                return np.array([t[0], t[1], t[2]], dtype=float)
        return None

    PIEDS = ("left_ankle_roll_link", "right_ankle_roll_link")

    rec = SceneRecorder(world, str(SORTIE / "posture.mp4"),
                        position=(1.6, -2.4, 1.2), cible=(0.0, 0.0, 0.8),
                        largeur=1280, hauteur=720,
                        titre="AXIS-OS — posture GR00T WBC (banc)")
    rec.suivre(lambda: robot.get_world_pose()[0], decalage=(1.6, -2.4, 0.4),
               hauteur_visee=0.8, decalage_cadre=(-0.30, -0.30))
    video_ok = rec.start()
    etat = {"k": 0, "texte": "", "chute": None}

    def pas():
        """Un pas de commande : la marche (un pas de physique), la
        manipulation (qui n'écrit les bras que pendant un geste), puis le
        reste de la physique."""
        loco.tick()
        manip.tick()
        # Aucune décision de chute avant le premier pas de physique de la boucle.
        if loco.state is LocomotionState.FALLEN and etat["chute"] is None and etat["k"] > 0:
            etat["chute"] = f"à {sim_t[0]:.2f} s ({etat['texte']})"
        capture = video_ok and etat["k"] % 2 == 0
        if capture:
            rec.cadrer()
        for i in range(cfg.decimation - 1):
            world.step(render=(capture and i == 0))
        sim_t[0] += cfg.control_dt
        t_physique[0] = sim_t[0]
        etat["k"] += 1
        if capture:
            t_, a_ = loco.posture_actuelle
            rec.image([etat["texte"],
                       f"Torse : {math.degrees(t_):.0f}°, bassin : -{100 * a_:.1f} cm",
                       f"Garde-fou : {loco.stats['refused']} refus"], sim_t[0])

    def pendant(duree):
        t0 = sim_t[0]
        while sim_t[0] - t0 < duree and etat["chute"] is None:
            pas()

    # Calibration des bras, celle du banc T1 : quelques postures, bras posés
    # et immobiles ; couples estimés contre pesanteur du modèle (biais des
    # couples), mains calculées contre mains de la scène.
    P = Postures()
    calib = CalibrationMains(mesures_bras.modele, ReglagesPrise().ecart_modele_max_m)
    postures_calib = [("port", posture_bras(P.roulis_ouvert, P.tangage_port, P.coude)),
                      ("ouverte", posture_bras(P.roulis_presaisie, P.tangage_port, P.coude)),
                      ("basse", posture_bras(0.25, -0.15, 0.50)),
                      ("haute", posture_bras(0.25, -0.70, 0.20)),
                      ("port", posture_bras(P.roulis_ouvert, P.tangage_port, P.coude))]
    etat_calib = {"i": 0, "arrive": None}

    def pas_calibration():
        """Un pas de la calibration ; vrai une fois finie."""
        nom, cible = postures_calib[etat_calib["i"]]
        reste = 0.0
        for j, v in cible.items():           # 0,02 rad par pas au plus (1 rad/s)
            e = v - adapt.pose_bras[j]
            adapt.pose_bras[j] += max(-0.02, min(0.02, e))
            reste = max(reste, abs(e))
        if reste > 1e-6:
            return False
        if etat_calib["arrive"] is None:
            etat_calib["arrive"] = sim_t[0]
        if sim_t[0] - etat_calib["arrive"] < 0.6:  # le bras se pose
            return False
        if sim_t[0] - etat_calib["arrive"] < 1.0:
            manip.echantillon_calibration_couples(nom)
            return False
        st_ = read_state()
        g_, d_ = mains()
        calib.ajouter(nom, {j: st_.joint_pos[j] for j in chaine_bras},
                      (vers_robot(g_), vers_robot(d_)))
        etat_calib["i"] += 1
        etat_calib["arrive"] = None
        if etat_calib["i"] < len(postures_calib):
            return False
        ok_, texte_ = calib.bilan()
        for ligne_ in texte_.splitlines() + manip.terminer_calibration_couples().splitlines():
            print(f"  {ligne_}", flush=True)
        if not ok_:
            manip.calibration_refus = (
                f"cinématique des bras non calibrée : main réelle à {100 * calib.ecart_max:.1f} "
                f"cm du modèle (limite {100 * calib.ecart_max_m:.1f})")
        manip.resynchroniser()               # bras revenus en posture de port
        # Calibration de charge du robot du banc (faite par T1, gardée à son
        # profil) : la pesée du carton s'en sert ; sans elle, l'OS le dit.
        chemin_, robot_id_ = profil_robot_banc(GRAINE_COUPLES)
        try:
            cal_ = lire_profil(chemin_, robot_id_).get("calibration_charge")
        except ProfilRobotErreur as e_:
            print(f"  profil du robot illisible, ignoré : {e_}", flush=True)
            cal_ = None
        print(f"  {manip.appliquer_calibration_charge(cal_)}", flush=True)
        return True

    def poser_carton():
        """Le carton T1 sur son plateau, devant le robot, là où T1 le prend
        (banc_prise.poser_devant) ; rend ce que la perception en dirait
        (Objet, repère monde)."""
        centre, lacet_r = poser_devant(robot, carton, plateau, X_CARTON, sd.PLATEAU, HAUTEUR)
        # Le plateau, déclaré comme une table du site (sa carte : la même API
        # que les tables de T1) ; le carton doit reposer dessus. Ce banc n'a
        # pas de lidar, donc pas de gardien de proximité.
        pr_, _ = robot.get_world_pose()
        site = [plateau_site((float(pr_[0]), float(pr_[1]), lacet_r), X_CARTON, sd.PLATEAU)]
        inconnus_ = objets_inconnus([("carton", (centre[0], centre[1]))], site)
        if inconnus_:
            raise RuntimeError(f"objets posés devant le robot hors des tables du site : {inconnus_}")
        return Objet(centre, LARGEUR, PROFONDEUR, HAUTEUR, lacet=lacet_r, bord_m=0.0)

    def mesure():
        """Centrale inertielle (inclinaison, norme de la vitesse angulaire)
        et pieds (positions monde, vérité de la simulation)."""
        st = read_state()
        return (inclinaison_deg(st.quat_wxyz), float(np.linalg.norm(st.ang_vel_body)),
                [position_lien(p) for p in PIEDS])

    def glissement(pieds, pieds0):
        if any(p is None for p in pieds + pieds0):
            return None
        return max(float(np.linalg.norm((p - p0)[:2])) for p, p0 in zip(pieds, pieds0))

    def carton_tenu(z0):
        """VÉRITÉ SIMULATION (banc seulement) : le carton est-il entre les
        mains ? Son centre à moins de 25 cm du milieu des mains de la scène
        (~13 cm devant elles quand il est tenu) : penché ou bassin abaissé, le
        carton tenu descend avec les mains. Sans mains lisibles : pas plus de
        5 cm sous sa hauteur de départ, près du robot."""
        pc, _ = carton.get_world_pose()
        pg, pd = position_lien("left_rubber_hand"), position_lien("right_rubber_hand")
        if pg is not None and pd is not None:
            return float(np.linalg.norm(np.asarray(pc, dtype=float) - (pg + pd) / 2.0)) < 0.25
        pr, _ = robot.get_world_pose()
        return (float(pc[2]) > z0 - 0.05
                and float(np.linalg.norm(np.asarray(pc[:2]) - np.asarray(pr[:2]))) < 0.7)

    def charge_aux_mains(tangage):
        """Le carton dans le bilan d'équilibre : sa masse, au milieu des
        mains, d'après les angles MESURÉS des bras, la taille à `tangage`."""
        q = dict(read_state().joint_pos)
        q[TAILLE] = tangage
        pg, pd = modele.mains(q)
        return [(MASSE_CARTON, tuple(float(v) for v in (pg + pd) / 2.0))]

    def retour_droit():
        r = loco.posture(0.0, 0.0)
        t0 = sim_t[0]
        while not loco.posture_atteinte and sim_t[0] - t0 < RETOUR_MAX_S \
                and etat["chute"] is None:
            pas()
        pendant(1.0)
        return r.ok and loco.posture_atteinte and not loco.penche

    def serie(nom, valeurs, charge_kg, z_carton):
        """Une série : chaque palier par la consigne de l'OS, mesuré ; au
        premier signe de perte d'équilibre, arrêt et retour droit."""
        releve = []
        incl0, _, pieds0 = mesure()
        for k, (tangage, abaissement) in enumerate(valeurs, 1):
            etat["texte"] = f"{nom}, {charge_kg:g} kg : palier {k}"
            p = Palier(nom, charge_kg, k, tangage, abaissement)
            r = loco.posture(tangage, abaissement,
                             charges=charge_aux_mains(tangage) if charge_kg else ())
            p.accepte, p.raison = r.ok, str(r.reason or "")
            p.marge_prevue_m = getattr(r.value, "marge", None)
            releve.append(p)
            print(f"  {nom:9s} {charge_kg:3g} kg  palier {k} ({tangage:.2f} rad, "
                  f"{100 * abaissement:4.1f} cm) : {'accepté' if r.ok else 'REFUSÉ'} "
                  f"— {p.raison}")
            if not r.ok:
                break                 # plus loin, l'OS refuserait aussi
            t_atteint = None
            while etat["chute"] is None:
                pas()
                incl, vit, pieds = mesure()
                if p.suivre(incl - incl0, vit, glissement(pieds, pieds0)):
                    break
                if loco.posture_atteinte and t_atteint is None:
                    t_atteint = sim_t[0]
                if t_atteint is not None and sim_t[0] - t_atteint >= MAINTIEN_S:
                    break
            if etat["chute"] and not p.signe:
                p.signe = f"chute {etat['chute']}"
            p.tangage_mesure = read_state().joint_pos.get(TAILLE)
            if charge_kg:
                p.carton_tenu = carton_tenu(z_carton)
            v = p.ligne()
            print(f"      mesuré : taille {v['tangage_mesure_rad']} rad, inclinaison "
                  f"{v['inclinaison_max_deg']}°, vitesse {v['vitesse_max_rad_s']} rad/s, "
                  f"glissement {v['glissement_max_cm'] or '?'} cm"
                  + (f", carton {'tenu' if p.carton_tenu else 'LÂCHÉ'}" if charge_kg else ""))
            if p.signe or p.carton_tenu is False or etat["chute"]:
                print(f"      ARRÊT : {p.signe or 'carton lâché'} — retour droit")
                break
        if etat["chute"] is None:
            etat["texte"] = f"{nom}, {charge_kg:g} kg : retour droit"
            retour_droit()
        return releve

    releve = []
    prise = {"tenu": False}
    # Constaté sur GPU (2026-11-19) : « chute à 0.00 s (Debout) », 0 tick, aucun
    # palier essayé. Avant la boucle, l'enregistreur vidéo et l'initialisation
    # font avancer la physique sans la politique d'équilibre : le robot y
    # penche. On le remet debout ici, en dernier, comme le banc T1 (code commun :
    # IsaacSimBridge.remettre_debout), et on dit dans quel état on l'a trouvé.
    print(f"  robot avant la boucle : {bridge.remettre_debout(q0v, pos0, lacet0)}", flush=True)
    try:
        etat["texte"] = "Debout"
        pendant(3.0)
        etat["texte"] = "Posture de port"
        t0 = sim_t[0]
        while sim_t[0] - t0 < 2.0 and etat["chute"] is None:
            a = min(1.0, (sim_t[0] - t0) / 1.0)
            adapt.pose_bras.update({j: a * v for j, v in pose(ROULIS_OUVERT).items()})
            pas()
        etat["texte"] = "Calibration des bras (comme T1)"
        t0 = sim_t[0]
        calibre = False
        while not calibre and sim_t[0] - t0 < CALIBRATION_MAX_S and etat["chute"] is None:
            pas()
            calibre = pas_calibration()
        if not calibre:
            print("  [✗] calibration des bras inachevée")

        print(f"\n{'=' * 66}\n  1. SANS CARTON\n{'=' * 66}")
        for nom, valeurs in paliers():
            if etat["chute"] is None:
                releve += serie(nom, valeurs, 0.0, None)

        if etat["chute"] is None and calibre:
            print(f"\n{'=' * 66}\n  2. LE CARTON T1 ({MASSE_CARTON:g} kg), PRIS PAR L'OS COMME EN "
                  f"T1\n{'=' * 66}")
            etat["texte"] = "La prise de l'OS, comme en T1"
            objet = poser_carton()
            pendant(0.5)                      # le carton se pose sur son plateau
            manip.resynchroniser()
            r_ = manip.prendre(objet)
            if not r_.ok:
                print(f"  [✗] prise refusée par l'OS : {r_.reason}")
            else:
                t0 = sim_t[0]
                while manip.etat not in (Etat.TENU, Etat.ECHEC) \
                        and not (manip.etat is Etat.LIBRE and sim_t[0] - t0 > 1.0) \
                        and sim_t[0] - t0 < PRISE_MAX_S and etat["chute"] is None:
                    pas()
                print(f"  prise de l'OS : {manip.etat.name}"
                      + (f" — {manip.raison}" if manip.raison else ""))
                if manip.etat is Etat.TENU:
                    # Plateau retiré : seul l'équilibre compte désormais, et rien
                    # ne doit porter le carton à la place des mains.
                    plateau.set_world_pose(position=np.array([10.0, 3.0, sd.PLATEAU / 2]))
                    z_carton = float(carton.get_world_pose()[0][2])
                    etat["texte"] = "Plateau retiré : seule la prise retient le carton"
                    pendant(2.0)
                    prise["tenu"] = carton_tenu(z_carton)
                    print(f"  carton {'tenu' if prise['tenu'] else 'LÂCHÉ'} par la seule prise, "
                          f"debout")
                    if prise["tenu"]:
                        print(f"\n{'=' * 66}\n  3. AVEC CARTON\n{'=' * 66}")
                        for nom, valeurs in paliers():
                            if etat["chute"] is None:
                                releve += serie(nom, valeurs, MASSE_CARTON, z_carton)
    finally:
        droit = etat["chute"] is None and not loco.penche
        if video_ok:
            rec.terminer()
        SORTIE.mkdir(parents=True, exist_ok=True)
        with open(SORTIE / "posture.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=COLONNES)
            w.writeheader()
            for p in releve:
                w.writerow(p.ligne())

    print(f"\n{'=' * 66}\n  RÉSULTATS POSTURE\n{'=' * 66}")
    passe, raisons = verdict(releve, etat["chute"], loco.stats["refused"], droit)
    for p in releve:
        print("  " + " ; ".join(f"{c}={v}" for c, v in p.ligne().items() if v))
    check("Aucun signe de perte d'équilibre sur un palier accepté par l'OS",
          not any(p.accepte and p.signe for p in releve))
    check("Le robot ne tombe pas", etat["chute"] is None)
    check("Aucune consigne refusée par le garde-fou", loco.stats["refused"] == 0)
    check(f"Carton T1 ({MASSE_CARTON:g} kg) pris par l'OS comme en T1, tenu par la seule prise",
          prise["tenu"])
    check("Carton toujours tenu", not any(p.carton_tenu is False for p in releve))
    check("Robot droit à la fin", droit)
    check("Verdict du banc (banc_posture.verdict) : chaque série essayée, sans et avec carton",
          passe)
    print(f"\n  BANC DE POSTURE : {'PASSÉ' if passe else 'ÉCHOUÉ'}")
    for r in raisons:
        print(f"    - {r}")
    print(f"  relevé : {SORTIE / 'posture.csv'} ; locomotion : {loco.stats} ; "
          f"manipulation : {manip.stats}")
    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
