"""
Suite A1 — première app AXIS-OS qui voit ET se déplace, bout en bout.

L'app tourne dans le SANDBOX, via le Play Store, et doit :
  chercher un colis -> marcher vers lui -> s'arrêter devant -> rapporter

Toutes les briques validées séparément sont ici assemblées pour la
première fois : perception (OWL-ViT + profondeur + repères), locomotion
(politique Unitree + service + échéance de fraîcheur), sandbox, Play
Store, permissions, garde-fou.

ARCHITECTURE DES FILS D'EXÉCUTION — la raison d'être de ce montage :

  thread principal : fait tourner la SIMULATION et la boucle de
                     locomotion à 50 Hz (MainThreadSimPump)
  thread mission   : PlayStoreRuntime, qui lance l'app
  sous-process     : l'app elle-même, sandboxée

L'app ne commande jamais la marche pas à pas : elle déclare une
vitesse, et l'OS tient la cadence. C'est le point : une app sandboxée
ne peut pas garantir 50 Hz, et n'a pas à le faire.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_a1_inspection.py
"""
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)

POLICY = Path("/isaac-sim/policies/g1_motion.pt")
# Colis de 60 cm : taille réaliste pour un carton de manutention,
# et assez grand pour rester détectable à 2.5 m — la confiance
# mesurée tournait autour de 0.10 avec un cube de 50 cm.
COLIS = ("colis", (2.5, 0.0, 0.3), (0.6, 0.5, 0.6))
FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    from isaacsim.core.api.objects import DynamicCuboid
    from isaacsim.core.utils.types import ArticulationAction

    from axis_os.apps.warehouse_inspector.app import WarehouseInspectorApp
    from axis_os.hal.core.frames import (
        FRAME_BASE, FRAME_CAMERA_OPTICAL, FRAME_WORLD, FrameTree, Transform,
    )
    from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
    from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
    from axis_os.hal.core.types import (
        ActuatorCommand, ControlMode, Quaternion, Vector3,
    )
    from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        G1LocomotionAdapter, G1LocomotionConfig, G1LocomotionPolicy,
        body_angular_velocity, quat_wxyz_to_matrix,
    )
    from axis_os.hal.drivers.unitree_g1.robot import _build_g1_joint_definitions
    from axis_os.hal.watchdog.system import WatchdogSystem
    from axis_os.services.locomotion.service import BaseState, LocomotionService
    from axis_os.services.perception.owlvit_detector import OwlViTDetector
    from axis_os.services.perception.service import PerceptionService
    from axis_os.store.runtime.play_store import PlayStoreRuntime

    cfg = G1LocomotionConfig()
    limits = {d.joint_id: (d.limits.pos_min, d.limits.pos_max)
              for d in _build_g1_joint_definitions()}
    pol = G1LocomotionPolicy(POLICY, cfg, joint_limits=limits)
    pol.load()

    bridge = IsaacSimBridge()
    bridge.setup(fix_base=False, require_meshes=True, physics_dt=cfg.physics_dt)
    robot, world = bridge._robot, bridge._world
    idx = bridge._joint_index
    names = sorted(idx, key=lambda j: idx[j])

    bridge._world.scene.add(DynamicCuboid(
        prim_path=f"/World/{COLIS[0]}", name=COLIS[0],
        position=np.array(COLIS[1]), size=1.0, scale=np.array(COLIS[2]),
        color=np.array([0.72, 0.52, 0.30])))
    world.reset()

    g = pol.gains()
    kps = np.array([g[j][0] for j in names])
    kds = np.array([g[j][1] for j in names])
    robot.get_articulation_controller().set_gains(kps=kps, kds=kds)
    q0 = np.array([pol.default_pose()[j] for j in names])
    robot.set_joint_positions(q0)
    robot.set_joint_velocities(np.zeros_like(q0))
    robot.apply_action(ArticulationAction(joint_positions=q0))

    # --- Caméras embarquées, suivant le robot ------------------------
    cam = IsaacSimCamera("head_rgb", prim_path="/World/head_rgb",
                         width=640, height=480, position=(0.2, 0.0, 1.2))
    dep = IsaacSimCamera("head_depth", prim_path="/World/head_depth",
                         width=640, height=480, position=(0.2, 0.0, 1.2),
                         depth=True)
    for c in (cam, dep):
        c.initialize()
    check("Caméras prêtes", cam.wait_until_ready(world))
    dep.wait_until_ready(world)

    tree = FrameTree()
    # Seuil bas : les scores d'OWL-ViT ne sont pas calibrés comme ceux
    # d'un détecteur à classes fermées, et un carton lointain tourne
    # autour de 0.10. Mieux vaut quelques détections faibles que le
    # robot perde sa cible une image sur deux.
    detector = OwlViTDetector(threshold=0.05)
    check("OWL-ViT chargé", detector.load())
    perception = PerceptionService(cam, tree, detector, depth_camera=dep)
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

    premier_refus = {}

    def send_targets_diag(targets):
        st = send_targets(targets)
        if not premier_refus:
            for j, s_ in st.items():
                if s_.name != "OK":
                    fb = hal.get_full_state().get(j)
                    premier_refus.update({
                        "joint": j, "cible": round(targets[j], 3),
                        "position_vue_par_le_garde": round(fb.position, 3) if fb else None,
                        "instant_sim_s": round(sim_t[0], 2)})
        return st

    locomotion = LocomotionService(G1LocomotionAdapter(pol), read_state,
                                   send_targets_diag, command_timeout_s=0.5,
                                   clock=lambda: sim_t[0])
    locomotion.start()

    # --- Play Store, avec les deux services --------------------------
    nrobot = NormalizedRobotHAL(hal, build_g1_normalizer())
    nrobot.initialize_all()
    runtime = PlayStoreRuntime(nrobot, WatchdogSystem(nrobot),
                               perception_service=perception,
                               locomotion_service=locomotion)
    app = WarehouseInspectorApp()
    man = app.manifest()
    runtime.install(app)
    runtime.start_app(man.app_id)

    holder = {"result": None, "error": None, "done": False}

    def mission():
        try:
            holder["result"] = runtime.run_mission(
                man.app_id, params={"timeout_s": 90.0})
        except BaseException as exc:            # noqa: BLE001
            holder["error"] = exc
        finally:
            holder["done"] = True

    th = threading.Thread(target=mission, daemon=True)
    th.start()

    # --- Thread principal : simulation + locomotion à 50 Hz ----------
    # L'app ne tient PAS cette cadence : elle déclare une vitesse, l'OS
    # l'exécute. C'est tout l'intérêt de l'architecture.
    # EXACTEMENT decimation pas de physique par pas de commande. La
    # première version en faisait UN DE PLUS (un world.step(render=True)
    # ajouté en fin de boucle pour la caméra) : la politique croyait
    # avancer de 20 ms quand la simulation avançait de 22 ms. 10 %
    # d'erreur, accumulée — or cette politique compte le temps pour sa
    # phase de marche (période 0.8 s). Son horloge interne dérivait de
    # la physique, elle a fini par commander un pied au mauvais moment
    # (action de 4.6 sur une cheville, contre moins de 1 en marche
    # normale), le garde-fou a refusé, et le robot est tombé à 18.8 s.
    #
    # Le rendu se fait DANS la décimation, jamais en plus.
    RENDER_TOUS_LES = 5          # la perception lit ~5 images/s, inutile
                                 # de rendre à 50 Hz — et le rendu est ce
                                 # qui ralentit le plus la boucle.
    t_start = time.monotonic()
    k = 0
    while not holder["done"] and time.monotonic() - t_start < 300.0:
        locomotion.tick()                    # envoie + 1 pas de physique
        rendre = (k % RENDER_TOUS_LES == 0)
        for i in range(cfg.decimation - 1):
            # Un seul rendu, sur l'un des pas de la décimation.
            world.step(render=(rendre and i == 0))
        sim_t[0] += cfg.control_dt
        k += 1

        if rendre:
            # CAMÉRA RÉELLEMENT MONTÉE SUR LE ROBOT : sa position ET son
            # axe de visée suivent l'orientation du robot. La première
            # version la plaçait en coordonnées du MONDE (regard toujours
            # vers +x) tout en la déclarant dans le repère du ROBOT. Tant
            # qu'il regardait droit devant, les deux coïncidaient ; dès
            # qu'il tournait, la direction reçue par l'app était fausse,
            # elle corrigeait à l'envers et le robot s'éloignait — il a
            # fini à x=-1.25 en cherchant un colis à x=+2.5.
            pos, quat = robot.get_world_pose()
            R = quat_wxyz_to_matrix(quat)
            p_w = np.asarray(pos, dtype=float)
            mount = R @ np.array([0.2, 0.0, 0.4])      # devant, au-dessus
            # Viser à 2 m et plus bas : à 3 m, la caméra regardait au-delà
            # d'un colis proche, et la profondeur portait sur le sol
            # derrière lui.
            aim = p_w + R @ np.array([2.0, 0.0, -0.9])
            for c in (cam, dep):
                c._position = tuple(float(v) for v in (p_w + mount))
                c.look_at(tuple(float(v) for v in aim))

            # L'ARBRE DES REPÈRES fait la conversion, au lieu de la
            # supposer : monde <- robot (odométrie) et monde <- caméra
            # (montage). PerceptionService demande robot <- caméra, et
            # FrameTree compose les deux. C'est exactement son rôle.
            tree.set(Transform(
                FRAME_WORLD, FRAME_BASE,
                translation=Vector3(*(float(v) for v in p_w)),
                rotation=Quaternion(w=float(quat[0]), x=float(quat[1]),
                                    y=float(quat[2]), z=float(quat[3]))))
            tf = cam.optical_transform(parent=FRAME_WORLD,
                                       child=FRAME_CAMERA_OPTICAL)
            if tf is not None:
                tree.set(tf)

    th.join(timeout=10.0)

    print(f"\n{'=' * 66}\n  RÉSULTAT\n{'=' * 66}")
    if holder["error"] is not None:
        import traceback
        traceback.print_exception(type(holder["error"]), holder["error"],
                                  holder["error"].__traceback__)
        check("Mission sans exception", False)
    r = holder["result"]
    check("La mission a rendu un résultat", r is not None)
    if r is not None:
        print(f"  succès={r.success}  durée={r.duration_s:.1f}s")
        print(f"  métriques : {r.metrics}")
        if r.error_message:
            print(f"  message : {r.error_message}")
        check("L'app a VU le colis", bool(r.metrics.get("colis_vu")))
        check("L'app a ATTEINT le colis", r.success)
        d = r.metrics.get("distance_finale_m")
        if d is not None:
            check(f"Arrêt à moins d'1 m du colis ({d} m) — devant, pas dessus",
                  d < 1.0)

    pos, _ = robot.get_world_pose()
    print(f"\n  position finale du robot : x={float(pos[0]):.2f} "
          f"(colis à x={COLIS[1][0]})")
    print(f"  locomotion : {locomotion.stats}")
    print(f"  temps simulé écoulé : {sim_t[0]:.1f} s")
    if premier_refus:
        print(f"  PREMIER refus : {premier_refus}")
    check("Aucune consigne refusée par le garde-fou",
          locomotion.stats["refused"] == 0)
    check("Le robot n'est pas tombé", locomotion.state.name != "FALLEN")

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    if FAIL == 0:
        print("\n  ✓ Une app sandboxée voit, marche et s'arrête devant un colis")
        print("  ✓ Perception, locomotion, permissions et garde-fou assemblés")

    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
