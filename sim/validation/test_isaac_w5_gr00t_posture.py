"""
Suite W5 — la posture de GR00T WBC : torse penché, bassin abaissé, robot
immobile, sans puis avec un carton aux mains (lancer_posture.sh).

La manipulation se penche pour atteindre plus loin par la consigne de
posture du service de marche (LocomotionService.posture : bornée, centre de
masse vérifié sur un modèle statique, marge de 2 cm). Décision de revue :
l'app de tri ne se penche pas tant que ce banc n'est pas passé.

  A. 0–3 s    debout
  B. 3–5 s    bras en posture de port
  1. sans carton : trois séries de paliers (banc_posture.paliers) — torse
     de 0,05 à 0,35 rad, bassin de 2,5 à 10 cm, puis les deux ensemble ;
     chaque palier par LocomotionService.posture, tenu MAINTIEN_S une fois
     atteint, et droit entre deux séries
  2. le carton (masse_max_kg, carton T1) placé entre les mains, comme en
     W2, serré puis relâché : seule la prise le retient
  3. avec carton : les mêmes séries, le carton dans le bilan d'équilibre
     (masse et position, d'après les angles mesurés des bras)

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

POLICIES = Path("/isaac-sim/policies")
SORTIE = Path("/isaac-sim/videos")
PROFONDEUR, LARGEUR, HAUTEUR = 0.20, 0.24, 0.20     # le carton de la scène T1
# Posture de port en angles bruts, celle de W2 (vérifiée par cinématique :
# mains 29 cm devant le bassin, 18 cm au-dessus, écartées de 34 cm).
EPAULE_TANGAGE, COUDE = -0.30, 0.0
ROULIS_OUVERT, ROULIS_SERRE = 0.20, -0.10
KP_BRAS = [90, 60, 20, 60, 4, 4, 4]
KD_BRAS = [2.0, 1.0, 0.4, 1.0, 0.2, 0.2, 0.2]
RETOUR_MAX_S = 10.0             # le retour droit, au plus
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
    from pxr import Usd, UsdGeom
    from isaacsim.core.api.materials import PhysicsMaterial
    from isaacsim.core.api.objects import DynamicCuboid
    from isaacsim.core.utils.types import ArticulationAction

    from axis_os.hal.core.types import ActuatorCommand, ControlMode
    from axis_os.hal.drivers.isaac_sim.video import SceneRecorder
    from axis_os.hal.drivers.unitree_g1.gr00t_wbc_policy import (
        GR00TWBCAdapter, GR00TWBCConfig, GR00TWBCPolicy,
    )
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import body_angular_velocity
    from axis_os.services.locomotion.equilibre import TAILLE
    from axis_os.services.locomotion.service import (
        BaseState, LocomotionService, LocomotionState,
    )
    from axis_os.services.manipulation.cinematique import ModeleBras
    from axis_os.services.manipulation.prise import ReglagesPrise

    masse = ReglagesPrise().masse_max_kg       # la plus lourde que l'OS porte
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

    # Carton créé AU DÉPART, loin ; téléporté plus tard (voir W2).
    materiau = PhysicsMaterial(prim_path="/World/Materiaux/carton",
                               static_friction=1.0, dynamic_friction=0.8, restitution=0.0)
    carton = world.scene.add(DynamicCuboid(
        prim_path="/World/carton", name="carton",
        position=np.array([10.0, 0.0, HAUTEUR / 2]), size=1.0,
        scale=np.array([PROFONDEUR, LARGEUR, HAUTEUR]), mass=masse,
        color=np.array([0.72, 0.52, 0.30]), physics_material=materiau))

    g = pol.gains()
    for i, j in enumerate(cfg.body_joints[cfg.num_actions:]):
        g[j] = (KP_BRAS[i % 7], KD_BRAS[i % 7])
    world.reset()
    robot.get_articulation_controller().set_gains(
        kps=np.array([g[j][0] for j in names]), kds=np.array([g[j][1] for j in names]))
    q0 = {j: 0.0 for j in names}
    q0.update(dict(zip(cfg.controlled_joints, cfg.default_angles)))
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
    loco.start()
    modele = ModeleBras()

    stage = omni.usd.get_context().get_stage()
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

    def pas(serrer=False):
        """Un pas de commande (W2) : la marche, puis la physique."""
        if serrer:
            # Carton MAINTENU entre les mains pendant qu'elles se referment
            # (W2 : sans cela, il tombait avant).
            pg, pd = position_lien("left_rubber_hand"), position_lien("right_rubber_hand")
            if pg is not None and pd is not None:
                carton.set_world_pose(position=(pg + pd) / 2.0)
                carton.set_linear_velocity(np.zeros(3))
                carton.set_angular_velocity(np.zeros(3))
        loco.tick()
        if loco.state is LocomotionState.FALLEN and etat["chute"] is None:
            etat["chute"] = f"à {sim_t[0]:.2f} s ({etat['texte']})"
        capture = video_ok and etat["k"] % 2 == 0
        if capture:
            rec.cadrer()
        for i in range(cfg.decimation - 1):
            world.step(render=(capture and i == 0))
        sim_t[0] += cfg.control_dt
        etat["k"] += 1
        if capture:
            t_, a_ = loco.posture_actuelle
            rec.image([etat["texte"],
                       f"Torse : {math.degrees(t_):.0f}°, bassin : -{100 * a_:.1f} cm",
                       f"Garde-fou : {loco.stats['refused']} refus"], sim_t[0])

    def pendant(duree, serrer=False):
        t0 = sim_t[0]
        while sim_t[0] - t0 < duree and etat["chute"] is None:
            pas(serrer)

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
        pc, _ = carton.get_world_pose()
        pr, _ = robot.get_world_pose()
        return (float(pc[2]) > z0 - 0.05
                and float(np.linalg.norm(np.asarray(pc[:2]) - np.asarray(pr[:2]))) < 0.7)

    def charge_aux_mains(tangage):
        """Le carton dans le bilan d'équilibre : sa masse, au milieu des
        mains, d'après les angles MESURÉS des bras, la taille à `tangage`."""
        q = dict(read_state().joint_pos)
        q[TAILLE] = tangage
        pg, pd = modele.mains(q)
        return [(masse, tuple(float(v) for v in (pg + pd) / 2.0))]

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
    try:
        etat["texte"] = "Debout"
        pendant(3.0)
        etat["texte"] = "Posture de port"
        t0 = sim_t[0]
        while sim_t[0] - t0 < 2.0 and etat["chute"] is None:
            a = min(1.0, (sim_t[0] - t0) / 1.0)
            adapt.pose_bras.update({j: a * v for j, v in pose(ROULIS_OUVERT).items()})
            pas()

        print(f"\n{'=' * 66}\n  1. SANS CARTON\n{'=' * 66}")
        for nom, valeurs in paliers():
            if etat["chute"] is None:
                releve += serie(nom, valeurs, 0.0, None)

        z_carton = None
        if etat["chute"] is None:
            print(f"\n{'=' * 66}\n  2. LE CARTON ({masse:g} kg) ENTRE LES MAINS\n{'=' * 66}")
            etat["texte"] = "Les bras serrent le carton"
            pg, pd = position_lien("left_rubber_hand"), position_lien("right_rubber_hand")
            if pg is None or pd is None:
                print("  [✗] mains introuvables dans la scène — pas de série avec carton")
            else:
                carton.set_world_pose(position=(pg + pd) / 2.0)
                carton.set_linear_velocity(np.zeros(3))
                carton.set_angular_velocity(np.zeros(3))
                z_carton = float(((pg + pd) / 2.0)[2])
                t0 = sim_t[0]
                while sim_t[0] - t0 < 1.5 and etat["chute"] is None:
                    a = min(1.0, (sim_t[0] - t0) / 0.5)
                    adapt.pose_bras.update(pose(ROULIS_OUVERT + a * (ROULIS_SERRE - ROULIS_OUVERT)))
                    pas(serrer=True)
                etat["texte"] = "Carton relâché : seule la prise le retient"
                pendant(2.0)
                tenu = carton_tenu(z_carton)
                print(f"  carton {'tenu' if tenu else 'LÂCHÉ'} par la seule prise, debout")
                if tenu:
                    print(f"\n{'=' * 66}\n  3. AVEC CARTON\n{'=' * 66}")
                    for nom, valeurs in paliers():
                        if etat["chute"] is None:
                            releve += serie(nom, valeurs, masse, z_carton)
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
    check("Carton toujours tenu", not any(p.carton_tenu is False for p in releve))
    check("Robot droit à la fin", droit)
    check("Verdict du banc (banc_posture.verdict) : chaque série essayée, sans et avec carton",
          passe)
    print(f"\n  BANC DE POSTURE : {'PASSÉ' if passe else 'ÉCHOUÉ'}")
    for r in raisons:
        print(f"    - {r}")
    print(f"  relevé : {SORTIE / 'posture.csv'} ; locomotion : {loco.stats}")
    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
