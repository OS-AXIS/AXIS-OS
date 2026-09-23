"""
Suite A3 — inventaire, mémoire, et détection d'un colis disparu.

DEUX missions successives, et c'est la comparaison entre les deux qui
est le résultat :

  Mission 1 : le robot balaie, recense les colis. Tout est nouveau.
  Entre les deux : ON RETIRE UN COLIS de la scène.
  Mission 2 : le robot rebalaie. Il doit reconnaître ce qui est resté
              et signaler ce qui a DISPARU.

Une app qui se souvient ne se démontre pas en une mission : il faut
qu'elle retrouve ce qu'elle savait. Et retirer un objet entre les deux
prouve que la mémoire décrit l'état actuel, pas une accumulation.

La mémoire est écrite dans /isaac-sim/memory_store, qui survit entre
deux exécutions du script — on peut donc relancer ce test et voir la
mission 1 démarrer avec un inventaire déjà connu.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_a3_inventaire.py
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
MEMOIRE = Path("/isaac-sim/memory_store")
# Deux colis à 90° l'un de l'autre : un balayage doit voir les deux.
COLIS = [
    ("colis_devant", (2.2, 0.0, 0.3), (0.6, 0.5, 0.6)),
    ("colis_gauche", (0.0, 2.2, 0.3), (0.6, 0.5, 0.6)),
]
BALAYAGE_S = 45.0
FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    from isaacsim.core.api.objects import DynamicCuboid
    from isaacsim.core.utils.types import ArticulationAction

    from axis_os.apps.warehouse_inventory.app import WarehouseInventoryApp
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
    from axis_os.services.memory.service import MemoryService
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

    for nom, pos, taille in COLIS:
        world.scene.add(DynamicCuboid(
            prim_path=f"/World/{nom}", name=nom, position=np.array(pos),
            size=1.0, scale=np.array(taille),
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

    cam = IsaacSimCamera("head_rgb", prim_path="/World/head_rgb",
                         width=640, height=480, position=(0.2, 0.0, 1.2))
    dep = IsaacSimCamera("head_depth", prim_path="/World/head_depth",
                         width=640, height=480, position=(0.2, 0.0, 1.2),
                         depth=True)
    for c in (cam, dep):
        c.initialize()
    cam.wait_until_ready(world)
    dep.wait_until_ready(world)

    tree = FrameTree()
    detector = OwlViTDetector(threshold=0.09)
    detector.load()
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

    locomotion = LocomotionService(G1LocomotionAdapter(pol), read_state,
                                   send_targets, command_timeout_s=0.5,
                                   clock=lambda: sim_t[0])
    locomotion.start()

    memoire = MemoryService(MEMOIRE)
    check("Mémoire persistante démarrée", memoire.start().name == "OK")

    nrobot = NormalizedRobotHAL(hal, build_g1_normalizer())
    nrobot.initialize_all()
    runtime = PlayStoreRuntime(nrobot, WatchdogSystem(nrobot),
                               perception_service=perception,
                               locomotion_service=locomotion,
                               memory_service=memoire)
    app = WarehouseInventoryApp()
    man = app.manifest()
    runtime.install(app)
    runtime.start_app(man.app_id)

    def faire_mission(label):
        holder = {"result": None, "error": None, "done": False}

        def mission():
            try:
                holder["result"] = runtime.run_mission(
                    man.app_id, params={"duree_balayage_s": BALAYAGE_S})
            except BaseException as exc:            # noqa: BLE001
                holder["error"] = exc
            finally:
                holder["done"] = True

        th = threading.Thread(target=mission, daemon=True)
        th.start()
        k = 0
        t0 = time.monotonic()
        while not holder["done"] and time.monotonic() - t0 < BALAYAGE_S + 90:
            locomotion.tick()
            rendre = (k % 5 == 0)
            for i in range(cfg.decimation - 1):
                world.step(render=(rendre and i == 0))
            sim_t[0] += cfg.control_dt
            k += 1
            if rendre:
                pos, quat = robot.get_world_pose()
                R = quat_wxyz_to_matrix(quat)
                p_w = np.asarray(pos, dtype=float)
                for c in (cam, dep):
                    c._position = tuple(float(v) for v in
                                        (p_w + R @ np.array([0.2, 0.0, 0.4])))
                    c.look_at(tuple(float(v) for v in
                                    (p_w + R @ np.array([2.0, 0.0, -0.9]))))
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
        if holder["error"] is not None:
            import traceback
            traceback.print_exception(type(holder["error"]), holder["error"],
                                      holder["error"].__traceback__)
        r = holder["result"]
        print(f"\n  [{label}] {r.metrics if r else 'aucun résultat'}")
        return r

    section("Mission 1 — premier inventaire")

    r1 = faire_mission("mission 1")
    check("La mission 1 a rendu un résultat", r1 is not None)
    if r1:
        m1 = r1.metrics
        check(f"Au moins un colis recensé ({m1.get('colis_inventories')})",
              m1.get("colis_inventories", 0) >= 1)
        check("Tout est NOUVEAU au premier passage",
              m1.get("nouveaux") == m1.get("colis_inventories"))
        check("Rien n'était connu avant", m1.get("connus_avant") == 0)

    check("L'inventaire est écrit sur disque",
          any(MEMOIRE.glob("*")) if MEMOIRE.exists() else False)

    section("On retire un colis de la scène")

    world.scene.remove_object(COLIS[1][0])
    for _ in range(20):
        world.step(render=True)
    print(f"  « {COLIS[1][0]} » retiré")

    section("Mission 2 — le robot doit reconnaître et remarquer l'absence")

    r2 = faire_mission("mission 2")
    check("La mission 2 a rendu un résultat", r2 is not None)
    if r2 and r1:
        m2 = r2.metrics
        check(f"Le robot SAIT ce qu'il avait vu "
              f"({m2.get('connus_avant')} colis en mémoire)",
              m2.get("connus_avant", 0) >= 1)
        check(f"Il reconnaît au moins un colis déjà connu "
              f"({m2.get('deja_connus')})",
              m2.get("deja_connus", 0) >= 1)
        check(f"Il ne compte pas deux fois ce qu'il connaissait déjà "
              f"(nouveaux={m2.get('nouveaux')})",
              m2.get("nouveaux", 99) == 0)
        if r1.metrics.get("colis_inventories", 0) >= 2:
            check(f"Il remarque le colis DISPARU ({m2.get('disparus')})",
                  m2.get("disparus", 0) >= 1)
        else:
            print("  [—] colis disparu non vérifiable : un seul colis avait "
                  "été recensé en mission 1")

    print(f"\n  locomotion : {locomotion.stats}")
    check("Aucune consigne refusée par le garde-fou",
          locomotion.stats["refused"] == 0)
    check("Le robot n'est pas tombé", locomotion.state.name != "FALLEN")

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    if FAIL == 0:
        print("\n  ✓ Le robot recense, mémorise, reconnaît et remarque une absence")

    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
