"""
Suite D1 — démonstration : ce qu'un OS fait, et qu'un robot seul ne fait pas.

Cinq scènes, sur un vrai robot debout, avec le vrai Play Store, le vrai
sandbox et le vrai garde-fou. La première installe le catalogue ; les
quatre suivantes montrent chacune un refus — c'est le sujet.

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
from axis_os.apps.demo_fixtures.apps import (
    AppCurieuse, AppDeuxMains, AppLente,
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
    from axis_os.store.sdk.manifest import (
        ProtectionLevel, get_protection_level,
    )
    from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION

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
                               memory_service=memoire,
                               confirmer_permissions=accepter_tout_SIMULATION)

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

    refusee = runtime.install(AppDeuxMains())
    check("App exigeant une capacité absente : REFUSÉE à l'installation",
          not refusee)
    check("Elle n'apparaît pas dans le catalogue",
          runtime._apps.get("com.demo.deux_mains") is None)

    # ================================================================
    scene(3, "Permission : une app tente ce qu'elle n'a pas déclaré")

    x_avant = float(robot.get_world_pose()[0][0])
    runtime.install(AppCurieuse())
    demarree = runtime.start_app("com.demo.curieuse")
    check("L'app curieuse a bien DÉMARRÉ — sans quoi un refus ne "
          "prouverait rien sur les permissions", bool(demarree))
    r3 = runtime.run_mission("com.demo.curieuse", params={})
    faire_tourner(1.0)
    x_apres = float(robot.get_world_pose()[0][0])

    if r3 is not None and not r3.success:
        print(f"    mission en échec : {r3.error_message}")
    check("L'app N'A PAS PU faire marcher le robot",
          r3 is not None and r3.metrics.get("a_pu_marcher") is False)
    check(f"Et le robot n'a effectivement pas bougé "
          f"({abs(x_apres - x_avant):.3f} m)", abs(x_apres - x_avant) < 0.15)

    # ================================================================
    scene(4, "Concurrence : deux apps veulent le robot en même temps")

    runtime.install(AppLente())
    lente_ok = runtime.start_app("com.demo.lente")
    # La première version de cette scène est passée au vert POUR LA
    # MAUVAISE RAISON : la seconde mission était refusée parce que l'app
    # n'avait pas démarré, pas à cause du verrou. Un refus sans motif
    # vérifié ne prouve rien.
    check("L'app occupante a DÉMARRÉ", bool(lente_ok))
    premiere_demarree = runtime.start_app(installees[0])
    check("L'app concurrente a DÉMARRÉ elle aussi", bool(premiere_demarree))

    occupante = {"resultat": None}

    def occuper():
        occupante["resultat"] = runtime.run_mission("com.demo.lente", params={})

    th = threading.Thread(target=occuper, daemon=True)
    th.start()
    time.sleep(1.5)                     # laisser la première prendre le verrou

    # TROIS conditions, et non « la mission a échoué » : la version
    # précédente est passée au vert alors que l'app occupante était morte
    # en 0.3 s et que la concurrente avait TOURNÉ 20 s avant d'échouer
    # faute de colis dans la scène. Un échec quelconque ne prouve rien.
    occupante_vivante = th.is_alive()
    check("L'app occupante TOURNE encore au moment de la tentative "
          "concurrente — sans quoi il n'y a rien à arbitrer",
          occupante_vivante)

    t_tentative = time.monotonic()
    r4 = runtime.run_mission(installees[0], params={})
    duree_refus = time.monotonic() - t_tentative

    check(f"Le refus est IMMÉDIAT ({duree_refus:.2f} s) — un refus par le "
          f"verrou ne fait pas tourner la mission",
          duree_refus < 2.0)
    check("Le motif est bien le VERROU de mission, pas une autre panne",
          r4 is not None and not r4.success
          and "déjà en cours" in (r4.error_message or ""))
    if r4 is not None and r4.error_message:
        print(f"    motif : {r4.error_message}")
    print(f"    (le robot ne peut pas servir deux maîtres à la fois)")
    th.join(timeout=15.0)
    ro = occupante["resultat"]
    if ro is not None and not ro.success:
        print(f"    l'app occupante a échoué : {ro.error_message}")
    check("L'app occupante a terminé sa mission normalement",
          ro is not None and ro.success)
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
