"""
Suite D1 — démonstration : ce qu'un OS fait, et qu'un robot seul ne fait pas.

Cinq scènes, sur un vrai robot debout, avec le vrai Play Store, le vrai
sandbox et le vrai garde-fou. Chacune montre un refus — c'est le sujet.

  1. CATALOGUE     : trois apps installées, chacune avec ses permissions
  2. MATÉRIEL      : une app refusée parce que le robot n'a pas l'organe
  3. PERMISSION    : une app qui tente ce qu'elle n'a pas déclaré
  4. CONCURRENCE   : deux apps qui veulent le robot en même temps
  5. COMMANDE      : une app qui envoie une consigne dangereuse

Aucune de ces scènes n'est spectaculaire prise isolément. Ensemble,
elles montrent la seule chose qu'un robot livré avec son logiciel ne
peut pas offrir : plusieurs applications tierces sur une même machine,
sans qu'aucune puisse dépasser ce qu'on lui a accordé.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_d1_multiapp.py
"""
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)

POLICY = Path("/isaac-sim/policies/g1_motion.pt")
MEMOIRE = Path("/isaac-sim/memory_store_demo")
FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def scene(n, t):
    print(f"\n{'=' * 66}\n  SCÈNE {n} — {t}\n{'=' * 66}")


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    from isaacsim.core.utils.types import ArticulationAction

    from axis_os.apps.voice_inspector.app import VoiceInspectorApp
    from axis_os.apps.warehouse_inspector.app import WarehouseInspectorApp
    from axis_os.apps.warehouse_inventory.app import WarehouseInventoryApp
    from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
    from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
    from axis_os.hal.core.types import ActuatorCommand, ControlMode
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        G1LocomotionAdapter, G1LocomotionConfig, G1LocomotionPolicy,
        body_angular_velocity,
    )
    from axis_os.hal.drivers.unitree_g1.robot import _build_g1_joint_definitions
    from axis_os.hal.watchdog.system import WatchdogSystem
    from axis_os.services.locomotion.service import BaseState, LocomotionService
    from axis_os.services.memory.service import MemoryService
    from axis_os.store.sdk.app_base import (
        MissionResult, RobotApp, RobotContext,
        PermissionError as SandboxPermissionError,
    )
    from axis_os.store.sdk.manifest import (
        AppCategory, AppManifest, Capability, Permission, ProtectionLevel,
        ResourceQuota, get_protection_level,
    )
    from axis_os.store.runtime.play_store import PlayStoreRuntime

    # ---- robot debout, politique de marche active ---------------------
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
    world.reset()
    g = pol.gains()
    robot.get_articulation_controller().set_gains(
        kps=np.array([g[j][0] for j in names]),
        kds=np.array([g[j][1] for j in names]))
    q0 = np.array([pol.default_pose()[j] for j in names])
    robot.set_joint_positions(q0)
    robot.set_joint_velocities(np.zeros_like(q0))
    robot.apply_action(ArticulationAction(joint_positions=q0))

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
    memoire.start()

    nrobot = NormalizedRobotHAL(hal, build_g1_normalizer())
    nrobot.initialize_all()
    runtime = PlayStoreRuntime(nrobot, WatchdogSystem(nrobot),
                               locomotion_service=locomotion,
                               memory_service=memoire)

    def faire_tourner(secondes):
        """Fait vivre le robot pendant que les scènes se jouent."""
        n = int(secondes / cfg.control_dt)
        for _ in range(n):
            locomotion.tick()
            for _ in range(cfg.decimation - 1):
                world.step(render=False)
            sim_t[0] += cfg.control_dt

    faire_tourner(1.0)

    # ================================================================
    scene(1, "Catalogue : trois apps, trois jeux de permissions")

    apps = [WarehouseInspectorApp(), VoiceInspectorApp(),
            WarehouseInventoryApp()]
    installees = []
    for a in apps:
        m = a.manifest()
        ok = runtime.install(a)
        dangereuses = [p.name for p in m.permissions
                       if get_protection_level(p) is ProtectionLevel.DANGEROUS]
        print(f"\n  {m.name}")
        print(f"    permissions : {[p.name for p in m.permissions]}")
        print(f"    dont DANGEREUSES (accord explicite requis) : "
              f"{dangereuses or 'aucune'}")
        print(f"    installée : {bool(ok)}")
        if ok:
            installees.append(m.app_id)

    check("Chaque app déclare ses permissions AVANT d'être installée",
          all(a.manifest().permissions for a in apps))
    check("Les apps qui déplacent le robot ont une permission DANGEREUSE",
          any(get_protection_level(p) is ProtectionLevel.DANGEROUS
              for p in apps[0].manifest().permissions))

    # ================================================================
    scene(2, "Matériel : une app refusée faute d'organe")

    class AppDeuxMains(RobotApp):
        """Exige une préhension à DEUX mains — le G1 23 DOF ne l'a pas."""

        def manifest(self):
            return AppManifest(
                app_id="com.demo.deux_mains", name="Manipulation bimanuelle",
                version="1.0.0", author="Demo", author_email="d@d.d",
                description="Saisit un objet à deux mains.",
                category=AppCategory.LOGISTICS,
                permissions=[Permission.ACTUATORS_WRITE_ARMS],
                capabilities_required=[Capability.GRASP_DUAL],
                quota=ResourceQuota())

        def on_start(self, ctx): return True
        def on_stop(self, ctx): pass
        def run_mission(self, ctx, params):
            return MissionResult(success=True, mission_id="m",
                                 app_id="com.demo.deux_mains", duration_s=0.0)

    refusee = runtime.install(AppDeuxMains())
    check("App exigeant une capacité absente : REFUSÉE à l'installation",
          not refusee)
    check("Elle n'apparaît pas dans le catalogue",
          runtime._apps.get("com.demo.deux_mains") is None)

    # ================================================================
    scene(3, "Permission : une app tente ce qu'elle n'a pas déclaré")

    class AppCurieuse(RobotApp):
        """Ne déclare que la caméra, mais essaie de faire marcher le robot."""

        def manifest(self):
            return AppManifest(
                app_id="com.demo.curieuse", name="App curieuse",
                version="1.0.0", author="Demo", author_email="d@d.d",
                description="Regarde… et tente de marcher.",
                category=AppCategory.RESEARCH,
                permissions=[Permission.SENSORS_CAMERA],
                quota=ResourceQuota(max_runtime_s=20.0))

        def on_start(self, ctx): return True
        def on_stop(self, ctx): pass

        def run_mission(self, ctx, params):
            try:
                ctx.walk(0.5)
                marche = True
            except SandboxPermissionError as exc:
                ctx.log(f"refus attendu : {exc}")
                marche = False
            return MissionResult(
                success=not marche, mission_id="m",
                app_id="com.demo.curieuse", duration_s=0.0,
                metrics={"a_pu_marcher": marche})

    x_avant = float(robot.get_world_pose()[0][0])
    runtime.install(AppCurieuse())
    runtime.start_app("com.demo.curieuse")
    r3 = runtime.run_mission("com.demo.curieuse", params={})
    faire_tourner(1.0)
    x_apres = float(robot.get_world_pose()[0][0])

    check("L'app N'A PAS PU faire marcher le robot",
          r3 is not None and r3.metrics.get("a_pu_marcher") is False)
    check(f"Et le robot n'a effectivement pas bougé "
          f"({abs(x_apres - x_avant):.3f} m)", abs(x_apres - x_avant) < 0.15)

    # ================================================================
    scene(4, "Concurrence : deux apps veulent le robot en même temps")

    class AppLente(RobotApp):
        def manifest(self):
            return AppManifest(
                app_id="com.demo.lente", name="App lente", version="1.0.0",
                author="Demo", author_email="d@d.d", description="Occupe.",
                category=AppCategory.RESEARCH, permissions=[],
                quota=ResourceQuota(max_runtime_s=30.0))

        def on_start(self, ctx): return True
        def on_stop(self, ctx): pass

        def run_mission(self, ctx, params):
            time.sleep(6.0)
            return MissionResult(success=True, mission_id="m",
                                 app_id="com.demo.lente", duration_s=6.0)

    runtime.install(AppLente())
    runtime.start_app("com.demo.lente")
    concurrent = {"resultat": "pas encore tenté"}

    def occuper():
        runtime.run_mission("com.demo.lente", params={})

    th = threading.Thread(target=occuper, daemon=True)
    th.start()
    time.sleep(1.5)                     # laisser la première prendre le verrou

    r4 = runtime.run_mission(installees[0], params={})
    concurrent["resultat"] = r4
    check("La SECONDE mission est refusée pendant que la première tourne",
          r4 is None or not r4.success)
    print(f"    (le robot ne peut pas servir deux maîtres à la fois)")
    th.join(timeout=15.0)
    faire_tourner(1.0)

    # ================================================================
    scene(5, "Commande : une consigne dangereuse est refusée")

    from axis_os.hal.core.types import HALStatus
    limite = limits["left_knee_joint"]
    dangereuse = limite[1] + 2.0        # bien au-delà de la butée
    st = hal.send_joint_commands({
        "left_knee_joint": ActuatorCommand(mode=ControlMode.POSITION,
                                           position=dangereuse)})
    refus = st.get("left_knee_joint")
    print(f"\n  consigne envoyée : {dangereuse:.2f} rad "
          f"(butée : {limite[1]:.2f} rad)")
    print(f"  réponse du garde-fou : {refus.name if refus else '?'}")
    check("Consigne hors butée REFUSÉE", refus is not None and refus != HALStatus.OK)

    faire_tourner(2.0)
    check("Le robot est toujours debout après toutes ces tentatives",
          locomotion.state.name != "FALLEN")

    print(f"\n  locomotion : {locomotion.stats}")

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    if FAIL == 0:
        print("\n  ✓ Trois apps tierces cohabitent sur un même robot")
        print("  ✓ Chacune est limitée à ce qu'elle a déclaré")
        print("  ✓ Matériel absent, permission manquante, robot occupé,")
        print("    consigne dangereuse : quatre refus, aucun effet de bord")

    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
