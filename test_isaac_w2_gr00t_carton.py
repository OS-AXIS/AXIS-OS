"""
Suite W2 — porter un VRAI carton, serré entre les bras.

W1 a montré que le G1 29 DOF marche en posture de port. W2 ajoute la
CHARGE : un carton d'un kilo, tenu par friction entre les mains en
caoutchouc, pendant la marche.

  A. 0–3 s    debout
  B. 3–5 s    bras ouverts en posture de port
  à 5 s       le carton est placé entre les mains (écartement MESURÉ)
  C. 5–6.5 s  les bras SERRENT
  D. 6.5–9 s  debout, carton tenu        -> la prise tient-elle ?
  E. 9–15 s   marche à 0.3 m/s           -> marche-t-il en portant ?
  F. 15–18 s  arrêt                      -> et à l'arrêt ?

SERRER : la consigne des épaules vise AU-DELÀ de la surface du carton.
Les bras s'arrêtent au contact, et c'est la raideur des articulations —
l'écart entre consigne et position — qui produit la force de serrage.
Comme un humain qui presse un colis contre lui, sans doigts.

Le carton est créé AU DÉPART, loin du robot, puis TÉLÉPORTÉ entre les
mains : ajouter un objet en cours de simulation peut invalider la vue
physique d'Isaac Sim (constaté en A3 avec une suppression).

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_w2_gr00t_carton.py
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

# Convention ANATOMIQUE, traduite par la table validée.
PORT_OUVERT = {"shoulder_pitch": 0.6, "shoulder_roll": 0.10, "elbow": 1.2}
SERRE_ROULIS = -0.40                 # adduction : vise AU-DELÀ du contact
KP_BRAS = [90, 60, 20, 60, 4, 4, 4]
KD_BRAS = [2.0, 1.0, 0.4, 1.0, 0.2, 0.2, 0.2]


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

    def brut(j, v):
        c = conv[j]
        return c.sign * v + c.offset_rad

    def pose(roulis):
        out = {}
        for cote in ("left", "right"):
            for art, v in PORT_OUVERT.items():
                j = f"{cote}_{art}_joint"
                out[j] = brut(j, roulis if art == "shoulder_roll" else v)
        return out

    bridge = IsaacSimBridge(variante="29dof")
    bridge.setup(fix_base=False, require_meshes=True, physics_dt=cfg.physics_dt)
    robot, world = bridge._robot, bridge._world
    idx = bridge._joint_index
    names = sorted(idx, key=lambda j: idx[j])

    # Carton créé AU DÉPART, loin ; téléporté plus tard.
    materiau = PhysicsMaterial(prim_path="/World/Materiaux/carton",
                               static_friction=1.0, dynamic_friction=0.8,
                               restitution=0.0)
    carton = world.scene.add(DynamicCuboid(
        prim_path="/World/carton", name="carton",
        position=np.array([10.0, 0.0, HAUTEUR / 2]), size=1.0,
        scale=np.array([PROFONDEUR, 0.30, HAUTEUR]), mass=MASSE_KG,
        color=np.array([0.72, 0.52, 0.30]), physics_material=materiau))

    g = pol.gains()
    for i, j in enumerate(cfg.body_joints[cfg.num_actions:]):
        g[j] = (KP_BRAS[i % 7], KD_BRAS[i % 7])
    world.reset()
    robot.get_articulation_controller().set_gains(
        kps=np.array([g[j][0] for j in names]),
        kds=np.array([g[j][1] for j in names]))
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
        return BaseState(tuple(float(v) for v in quat),
                         tuple(float(v) for v in omega),
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

    trace = []
    politiques = {}
    chute = None
    carton_place = False
    z_depart = None
    t_mur = time.monotonic()
    while sim_t[0] < 18.0 and time.monotonic() - t_mur < 1200:
        t = sim_t[0]
        phase = ("A" if t < 3 else "B" if t < 5 else "C" if t < 6.5
                 else "D" if t < 9 else "E" if t < 15 else "F")

        if phase == "B":
            a = min(1.0, (t - 3.0) / 1.0)
            adapt.pose_bras.update({j: a * v for j, v in pose(PORT_OUVERT["shoulder_roll"]).items()})

        if not carton_place and t >= 5.0:
            pg, ng = position_lien("left_rubber_hand")
            pd, nd = position_lien("right_rubber_hand")
            if pg is None or pd is None:
                print("  [✗] mains introuvables dans la scène — test interrompu")
                break
            ecart = float(np.linalg.norm(pg - pd))
            largeur = float(np.clip(ecart - 0.04, 0.12, 0.45))
            centre = (pg + pd) / 2.0
            print(f"\n  mains ({ng}) : écartement mesuré {ecart:.3f} m -> "
                  f"carton de {largeur:.3f} m de large, centré en "
                  f"({centre[0]:.2f}, {centre[1]:.2f}, {centre[2]:.2f})")
            carton.set_local_scale(np.array([PROFONDEUR, largeur, HAUTEUR]))
            carton.set_world_pose(position=centre)
            carton.set_linear_velocity(np.zeros(3))
            carton.set_angular_velocity(np.zeros(3))
            carton_place = True
            z_depart = float(centre[2])

        if phase == "C":
            a = min(1.0, (t - 5.0) / 0.5)
            r = PORT_OUVERT["shoulder_roll"] + a * (SERRE_ROULIS - PORT_OUVERT["shoulder_roll"])
            adapt.pose_bras.update(pose(r))
        if phase == "E":
            loco.set_velocity(0.3, 0.0, 0.0)
        if phase == "F":
            loco.halt()

        if carton_place and len(trace) < 1000:
            pc, _ = carton.get_world_pose()
            pr, _ = robot.get_world_pose()
            trace.append((t, phase, float(pc[2]),
                          float(np.linalg.norm(np.asarray(pc[:2]) - np.asarray(pr[:2]))),
                          float(pr[0])))

        loco.tick()
        if loco.state is LocomotionState.FALLEN:
            chute = (t, phase)
            break
        politiques.setdefault(phase, set()).add(pol.derniere_politique)
        for _ in range(cfg.decimation - 1):
            world.step(render=False)
        sim_t[0] += cfg.control_dt

    print(f"\n{'=' * 66}\n  RÉSULTATS\n{'=' * 66}")
    print("  t     phase  hauteur carton  distance au bassin  x robot")
    derniere = -1.0
    for t, ph, z, d, x in trace:
        if t - derniere >= 1.0:
            print(f"  {t:5.1f}   {ph}     {z:6.3f} m        {d:5.2f} m        {x:+.2f}")
            derniere = t

    if chute:
        print(f"\n  CHUTE du robot à t = {chute[0]:.2f} s, phase {chute[1]}")
    check("Le robot ne tombe pas", chute is None)

    def dans(ph):
        return [(z, d) for t, p, z, d, x in trace if p == ph]

    for ph, texte in (("D", "debout"), ("E", "EN MARCHANT"), ("F", "à l'arrêt")):
        pts = dans(ph)
        if not pts or z_depart is None:
            continue
        zmin = min(z for z, _ in pts)
        dmax = max(d for _, d in pts)
        check(f"Carton TENU {texte} : hauteur min {zmin:.2f} m (départ "
              f"{z_depart:.2f}), jamais à plus de {dmax:.2f} m du bassin",
              zmin > z_depart - 0.15 and dmax < 0.7)

    xs = [x for t, p, z, d, x in trace if p == "E"]
    if xs:
        check(f"Il avance en portant : {xs[-1] - xs[0]:.2f} m en marche",
              xs[-1] - xs[0] > 0.8)
    print(f"\n  politiques : { {p: sorted(v) for p, v in politiques.items()} }")
    print(f"  locomotion : {loco.stats}")
    check("Aucune consigne refusée par le garde-fou", loco.stats["refused"] == 0)

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    if FAIL == 0:
        print("\n  ✓ Le G1 porte un carton d'un kilo en marchant")
    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
