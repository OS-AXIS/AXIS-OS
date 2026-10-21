"""
Vidéo V1 — « L'app ne répond plus : AXIS-OS arrête le robot. »

Une VRAIE app, installée par le Play Store et isolée dans le sandbox,
fait marcher le robot puis se fige. Rien ne signale la panne : elle est
toujours là, elle ne dit juste plus rien.

Ce que la vidéo montre, dans l'ordre :
  1. le robot debout, aucune consigne
  2. l'app demande « avance à 0.5 m/s » — le robot marche
  3. l'app se fige — AXIS-OS constate la consigne expirée et arrête le
     robot EN ÉQUILIBRE, en une demi-seconde
  4. le sandbox finit par tuer l'app figée
  5. le robot est toujours debout

Incrusté à l'écran : consigne demandée, consigne effective, état du
service de marche, refus du garde-fou.

  docker exec -u root isim-isaac-sim-1 /isaac-sim/python.sh -m pip install imageio imageio-ffmpeg
  docker exec isim-isaac-sim-1 /isaac-sim/python.sh /isaac-sim/axis_os/video_v1_app_figee.py
  docker cp isim-isaac-sim-1:/isaac-sim/videos/v1_app_figee.mp4 ~/videos/
"""
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import (
    IsaacSimBridge, start_simulation_app,
)
from axis_os.apps.demo_fixtures.apps import AppQuiSeFige

POLICY = Path("/isaac-sim/policies/g1_motion.pt")
SORTIE = "/isaac-sim/videos/v1_app_figee.mp4"
AVANT_S = 2.0           # robot debout avant le lancement de l'app
APRES_S = 3.0           # robot debout après la fin de l'app


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    from isaacsim.core.utils.types import ArticulationAction

    from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
    from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
    from axis_os.hal.core.types import ActuatorCommand, ControlMode
    from axis_os.hal.drivers.isaac_sim.video import SceneRecorder
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        G1LocomotionAdapter, G1LocomotionConfig, G1LocomotionPolicy,
        body_angular_velocity,
    )
    from axis_os.hal.drivers.unitree_g1.robot import _build_g1_joint_definitions
    from axis_os.hal.watchdog.system import WatchdogSystem
    from axis_os.services.locomotion.service import BaseState, LocomotionService
    from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
    from axis_os.store.runtime.profil import Profil

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

    adaptateur = G1LocomotionAdapter(pol)
    loco = LocomotionService(adaptateur, read_state,
                             send_targets, command_timeout_s=0.5,
                             clock=lambda: sim_t[0])
    loco.start()

    nrobot = NormalizedRobotHAL(hal, build_g1_normalizer())
    nrobot.initialize_all()
    runtime = PlayStoreRuntime(nrobot, WatchdogSystem(nrobot),
                               locomotion_service=loco,
                               confirmer_permissions=accepter_tout_SIMULATION,
                               profil=Profil.SIMULATION)
    app = AppQuiSeFige()
    runtime.install(app)
    runtime.start_app(app.manifest().app_id)

    # Caméra qui SUIT le robot, en trois quarts avant, à ~3.7 m. Version
    # précédente : caméra fixe reculée à 6.5 m pour garder tout le trajet
    # dans le champ — le robot y paraissait minuscule. Une caméra proche
    # et fixe le perdrait ; une caméra qui l'accompagne le garde en gros
    # plan du début à la fin.
    rec = SceneRecorder(world, SORTIE, position=(2.4, -2.8, 1.25),
                        cible=(0.0, 0.0, 0.75),
                        titre="AXIS-OS — une app se fige en pleine marche")
    rec.suivre(lambda: robot.get_world_pose()[0],
               decalage=(2.4, -2.8, 0.5), hauteur_visee=0.75)
    if not rec.start():
        simulation_app.close()
        sys.exit(1)

    holder = {"result": None, "done": False, "lancee": False}

    def mission():
        holder["lancee"] = True
        try:
            holder["result"] = runtime.run_mission(app.manifest().app_id, {})
        finally:
            holder["done"] = True

    derniere = {"vx": None, "t": None}

    def etat_affiche():
        # On raconte l'HISTOIRE, pas l'état interne du service : celui-ci
        # efface une consigne expirée, ce qui affichait « consigne
        # demandée : aucune » juste après que l'app eut demandé 0.5 m/s.
        # Trompeur. On retient donc nous-mêmes la dernière consigne reçue.
        if loco._twist_t is not None and loco._twist.vx != 0.0:
            derniere["vx"], derniere["t"] = loco._twist.vx, loco._twist_t
        fraiche = (loco._twist_t is not None
                   and sim_t[0] - loco._twist_t <= 0.5)
        if not holder["lancee"]:
            app_txt = "App : pas encore lancée"
        elif holder["done"]:
            app_txt = "App : ARRÊTÉE par le sandbox"
        else:
            app_txt = "App : en cours d'exécution"
        pos, _ = robot.get_world_pose()
        if derniere["t"] is None:
            recue = "Dernière consigne de l'app : aucune"
        else:
            recue = (f"Dernière consigne de l'app : {derniere['vx']:.2f} m/s "
                     f"(il y a {sim_t[0] - derniere['t']:.1f} s)")
        if fraiche:
            appliquee = f"Consigne appliquée : {loco._twist.vx:.2f} m/s"
        elif derniere["t"] is None:
            # Rien n'a encore été demandé : « expirée » serait faux.
            appliquee = "Consigne appliquée : aucune — robot debout"
        else:
            appliquee = "Consigne appliquée : 0 — expirée, robot à l'arrêt"
        return [
            app_txt,
            recue,
            appliquee,
            f"Service de marche : {loco.state.name}",
            f"Garde-fou : {loco.stats['refused']} refus",
            f"Position du robot : x = {float(pos[0]):+.2f} m",
            # Le cap est ce que la politique de marche ne perçoit pas :
            # sans maintien, le robot pivotait de plus de 90° une fois
            # arrêté. On l'affiche pour que la correction soit VISIBLE.
            f"Cap du robot : {math.degrees(adaptateur._lacet(robot.get_world_pose()[1])):+.0f}°"
            + (f"  (AXIS-OS corrige : {adaptateur.correction_cap:+.2f} rad/s)"
               if abs(adaptateur.correction_cap) > 0.02 else "  (tenu)"),
        ]

    stale_vu = 0
    a_marche = False
    fin_app_t = None
    k = 0
    th = None
    t_mur = time.monotonic()
    while time.monotonic() - t_mur < 240.0:
        t = sim_t[0]

        if th is None and t >= AVANT_S:
            th = threading.Thread(target=mission, daemon=True)
            th.start()
            rec.evenement("Une app est lancée par le Play Store", t,
                          duree_s=2.0, couleur=(40, 110, 200))

        if not a_marche and loco._twist_t is not None and loco._twist.vx > 0:
            a_marche = True
            rec.evenement("L'app demande : avance à 0.5 m/s", t,
                          duree_s=2.5, couleur=(40, 110, 200))

        if loco.stats["stale"] > stale_vu:
            stale_vu = loco.stats["stale"]
            rec.evenement("L'app ne répond plus — AXIS-OS arrête le robot", t,
                          duree_s=4.0, couleur=(210, 120, 30))

        if holder["done"] and fin_app_t is None:
            fin_app_t = t
            rec.evenement("Le sandbox a arrêté l'app figée", t,
                          duree_s=3.0, couleur=(200, 50, 45))

        if fin_app_t is not None and t - fin_app_t > APRES_S:
            break

        loco.tick()
        capture = (k % 2 == 0)             # 25 images/s pour 50 Hz
        if capture:
            rec.cadrer()                   # AVANT le rendu, pas après
        for i in range(cfg.decimation - 1):
            world.step(render=(capture and i == 0))
        sim_t[0] += cfg.control_dt
        k += 1
        if capture:
            rec.image(etat_affiche(), t)

    chemin = rec.terminer()
    r = holder["result"]
    print(f"\n  mission : {'—' if r is None else r.error_message or 'terminée'}")
    print(f"  locomotion : {loco.stats}")
    print(f"  robot final : {loco.state.name}")
    print(f"\n  VIDÉO : {chemin}")
    simulation_app.close()


if __name__ == "__main__":
    main()
