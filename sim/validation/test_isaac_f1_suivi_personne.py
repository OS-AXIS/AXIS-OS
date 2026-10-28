"""
Suite F1 — « suis-moi » : le G1 suit une personne qui marche, s'arrête,
passe derrière un poteau, et qu'une deuxième personne croise. G1 29 DOF,
GR00T WBC, Isaac Sim 6.1. L'app est celle de examples/follow_me, installée
par le Play Store et lancée dans son bac à sable, comme sur un robot.

CE SCRIPT N'A PAS ENCORE ÉTÉ EXÉCUTÉ : il a été écrit sans GPU. Le premier
essai dira ce qui ne tient pas ; corriger ici, pas dans l'OS, ce qui relève
de la scène.

Chaîne exercée, sans modification :

  FollowMeApp (bac à sable, pont JSON)
    -> RobotContext.person_in_front / follow / follow_status / stop_follow
       -> FollowService -> LocomotionService (+ GardienProximite <- lidar)
          -> GR00T WBC -> HAL (JointSafetyGuard)
       -> PeopleService (détecteur, suivi des pistes, ré-identification)

Ce que la simulation remplace :
  - les personnes : silhouettes cinématiques (0.4 × 0.4 × 1.7 m, comme
    celles du gardien dans la démo T1), déplacées par les trajets scriptés
    de axis_os.testing.SimulatedPerson ;
  - le détecteur : par défaut, la « vérité terrain » de la scène (le
    détecteur simulé de axis_os.testing : champ de la caméra de tête,
    portée, et obstacles qui cachent). Pourquoi : RT-DETR ne reconnaît pas
    une boîte orange comme une personne. --detecteur rtdetr le branche sur
    la caméra de tête et sa profondeur ; il ne verra rien tant que la scène
    n'a pas de personnages humains (omni.anim.people), non fournis ici ;
  - la voix : les phrases « suis-moi » puis « arrête-toi » sont « entendues »
    à des instants fixés (transcription scriptée, pas de Whisper) ;
  - la pose du robot : lue dans la simulation.

Usage (dans le conteneur) :
  /isaac-sim/python.sh /isaac-sim/axis_os/test_isaac_f1_suivi_personne.py \\
      [--duree 70] [--acceleration 2] [--sans-video] [--detecteur verite|rtdetr]

Vérifié à la fin : le robot a suivi, ne s'est jamais approché de la personne
suivie à moins de 1.4 m, n'a jamais reculé ni dépassé 0.25 m/s, ne l'a jamais
quittée pour la deuxième personne, s'est arrêté à la voix, n'est pas tombé ;
et rien de la personne ne reste en mémoire après le suivi.
"""
import argparse
import importlib.util
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.drivers.isaac_sim.bridge import IsaacSimBridge, start_simulation_app  # noqa: E402

POLICIES = Path("/isaac-sim/policies")
VIDEOS = Path("/isaac-sim/videos")
FAIL = 0


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def charger_app(dossiers):
    """FollowMeApp, depuis examples/follow_me/app.py (dépôt ou conteneur)."""
    for d in dossiers:
        chemin = Path(d) / "app.py"
        if chemin.exists():
            spec = importlib.util.spec_from_file_location("follow_me_app", chemin)
            mod = importlib.util.module_from_spec(spec)
            sys.modules["follow_me_app"] = mod
            spec.loader.exec_module(mod)
            return mod.FollowMeApp()
    raise SystemExit(f"examples/follow_me/app.py introuvable dans {dossiers}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duree", type=float, default=70.0, help="secondes SIMULÉES")
    ap.add_argument("--acceleration", type=int, default=2)
    ap.add_argument("--sans-video", action="store_true")
    ap.add_argument("--detecteur", choices=("verite", "rtdetr"), default="verite")
    ap.add_argument("--exemples", default=None, help="dossier examples/follow_me")
    args = ap.parse_args()
    DUREE = args.duree

    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    import omni.usd
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    from isaacsim.core.utils.types import ArticulationAction
    from pxr import UsdLux, UsdPhysics

    from axis_os.hal.core.frames import (
        FRAME_BASE, FRAME_CAMERA_OPTICAL, FRAME_WORLD, FrameTree, Transform,
    )
    from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
    from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
    from axis_os.hal.core.types import ActuatorCommand, AudioChunk, ControlMode, Quaternion, Vector3
    from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera
    from axis_os.hal.drivers.isaac_sim.sim_lidar import IsaacSimLidar
    from axis_os.hal.drivers.isaac_sim.video import SceneRecorder
    from axis_os.hal.drivers.unitree_g1.gr00t_wbc_policy import (
        GR00TWBCAdapter, GR00TWBCConfig, GR00TWBCPolicy,
    )
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        body_angular_velocity, quat_wxyz_to_matrix,
    )
    from axis_os.hal.drivers.unitree_g1.robot import build_g1_joint_definitions
    from axis_os.hal.watchdog.system import WatchdogSystem
    from axis_os.services.locomotion.proximite import GardienProximite
    from axis_os.services.locomotion.service import BaseState, LocomotionService, LocomotionState
    from axis_os.services.navigation.service import Carte, NavigationService, Pose2D, Rectangle
    from axis_os.services.people.follow import FollowService
    from axis_os.services.people.rtdetr_detector import RTDETRPersonDetector
    from axis_os.services.people.service import PeopleService
    from axis_os.services.speech.service import ISpeechRecognizer, SpeechService
    from axis_os.services.speech.synthesis import FakeSpeechSynthesizer
    from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
    from axis_os.store.runtime.profil import Profil
    from axis_os.testing.robot_simule import SimulatedPerson, SimulatedPersonDetector

    cfg = GR00TWBCConfig()
    pol = GR00TWBCPolicy(POLICIES / "gr00t_wbc_balance.onnx",
                         POLICIES / "gr00t_wbc_walk.onnx", cfg)
    pol.load()
    bridge = IsaacSimBridge(variante="29dof")
    bridge.setup(fix_base=False, require_meshes=True, physics_dt=cfg.physics_dt)
    robot, world = bridge._robot, bridge._world
    idx = bridge._joint_index
    names = sorted(idx, key=lambda j: idx[j])
    stage = omni.usd.get_context().get_stage()

    # ---- scène : un couloir, un poteau, un mur -------------------------------
    carte = Carte(-2.0, -5.0, 14.0, 5.0, obstacles=[
        Rectangle(6.8, -1.6, 7.2, -1.2),          # poteau, derrière lequel on passe
        Rectangle(11.0, -4.0, 11.3, 4.0),         # mur du fond
    ])
    for i, o in enumerate(carte.obstacles):
        world.scene.add(FixedCuboid(
            prim_path=f"/World/obstacle_{i}", name=f"obstacle_{i}",
            position=np.array([(o.xmin + o.xmax) / 2, (o.ymin + o.ymax) / 2, 1.0]),
            size=1.0, scale=np.array([o.xmax - o.xmin, o.ymax - o.ymin, 2.0]),
            color=np.array([0.35, 0.35, 0.40])))

    # Personnes : la personne suivie (haut rouge) et une autre (haut bleu).
    rouge = SimulatedPerson(2.5, 0.0, torso=(200, 30, 30), legs=(40, 40, 40))
    bleue = SimulatedPerson(5.6, -3.0, torso=(30, 50, 200), legs=(40, 40, 40))
    T_SUIS_MOI, T_ARRETE = 3.0, DUREE - 8.0
    rouge.wait(T_SUIS_MOI + 2.0).walk_to(6.0, 0.0, 0.5).wait(6.0) \
        .walk_to(6.0, -2.4, 0.4).walk_to(9.0, -2.4, 0.4).wait(DUREE)
    bleue.wait(17.0).walk_to(5.6, 3.0, 0.9).wait(DUREE)
    personnes = [rouge, bleue]
    silhouettes = []
    for i, (p, c) in enumerate(((rouge, (0.80, 0.12, 0.12)), (bleue, (0.12, 0.20, 0.80)))):
        path = f"/World/personne_{i}"
        s = world.scene.add(DynamicCuboid(
            prim_path=path, name=f"personne_{i}", position=np.array([p.x, p.y, 0.85]),
            size=1.0, scale=np.array([0.4, 0.4, 1.7]), mass=70.0, color=np.array(c)))
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).CreateKinematicEnabledAttr(True)
        silhouettes.append(s)

    dome = UsdLux.DomeLight.Define(stage, "/World/eclairage_ambiant")
    dome.CreateIntensityAttr(600.0)
    cam = IsaacSimCamera("tete_rgb", prim_path="/World/tete_rgb", width=640, height=480,
                         position=(0.2, 0.0, 1.2), focal_length_mm=18.0)
    dep = IsaacSimCamera("tete_prof", prim_path="/World/tete_prof", width=640, height=480,
                         position=(0.2, 0.0, 1.2), depth=True, focal_length_mm=18.0)
    world.reset()

    g = pol.gains()
    robot.get_articulation_controller().set_gains(
        kps=np.array([g[j][0] for j in names]), kds=np.array([g[j][1] for j in names]))
    q0 = {j: 0.0 for j in names}
    q0.update(dict(zip(cfg.controlled_joints, cfg.default_angles)))
    q0v = np.array([q0[j] for j in names])
    robot.set_joint_positions(q0v)
    robot.set_joint_velocities(np.zeros_like(q0v))
    robot.apply_action(ArticulationAction(joint_positions=q0v))
    for c in (cam, dep):
        c.initialize()
    cam.wait_until_ready(world)
    dep.wait_until_ready(world)

    sim_t = [0.0]
    horloge = lambda: sim_t[0]          # noqa: E731

    def lacet(q):
        w, x, y, z = (float(v) for v in q)
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def pose2d() -> Pose2D:
        p, q = robot.get_world_pose()
        return Pose2D(float(p[0]), float(p[1]), lacet(q), horodatage=sim_t[0])

    def pose_xyc():
        p = pose2d()
        return p.x, p.y, p.cap

    def pose_tete():
        p, q = robot.get_world_pose()
        R = quat_wxyz_to_matrix(q)
        return np.asarray(p, dtype=float) + R @ np.array([0.0, 0.0, 0.62]), R

    lidar = IsaacSimLidar("mid360", pose_tete, horloge)
    lidar.initialize()

    # ---- pile AXIS-OS ---------------------------------------------------------
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
        return hal.send_joint_commands(
            {j: ActuatorCommand(mode=ControlMode.POSITION, position=v) for j, v in targets.items()})

    adapt = GR00TWBCAdapter(pol, limites={d.joint_id: (d.limits.pos_min, d.limits.pos_max)
                                          for d in build_g1_joint_definitions("29dof")})
    gardien = GardienProximite(lidar.source_gardien(), pose_xyc, obstacles=carte.obstacles,
                               horloge=horloge)
    loco = LocomotionService(adapt, read_state, send_targets, command_timeout_s=0.5,
                             clock=horloge, gardien=gardien)
    nav = NavigationService(carte, pose2d, loco, horloge=horloge)
    consignes = []
    envoyer = loco.set_velocity

    def set_velocity(vx=0.0, vy=0.0, wz=0.0):
        consignes.append((sim_t[0], vx, vy, wz))
        return envoyer(vx, vy, wz)
    loco.set_velocity = set_velocity

    tree = FrameTree()

    class Monde:
        """Ce que lit le détecteur simulé : la pose du robot, la carte, les
        personnes."""
        def __init__(self):
            self.carte, self.people = carte, personnes
            self.x = self.y = self.heading = 0.0

        def _vers_robot(self, p):
            dx, dy = p[0] - self.x, p[1] - self.y
            c, s = math.cos(self.heading), math.sin(self.heading)
            return np.array([c * dx + s * dy, -s * dx + c * dy, p[2]])

    monde = Monde()
    if args.detecteur == "rtdetr":
        det = RTDETRPersonDetector()
        if not det.load():
            raise SystemExit(f"RT-DETR non chargé : {det.stats['load_error']}")
        people = PeopleService(det, camera=cam, frame_tree=tree, depth_camera=dep,
                               source_pose=pose_xyc, horloge=horloge)
    else:
        people = PeopleService(SimulatedPersonDetector(monde), source_pose=pose_xyc,
                               horloge=horloge)
    follow = FollowService(people, loco, gardien, source_pose=pose_xyc, horloge=horloge,
                           navigation=nav)
    for s in (loco, nav, people, follow):
        s.start()

    # Voix : les deux phrases, « entendues » à leur heure de simulation.
    phrases = [(T_SUIS_MOI, "suis-moi"), (T_ARRETE, "arrête-toi")]

    class Micro:
        def initialize(self):
            return "OK"

        def is_healthy(self):
            return True

        def capture(self, duree_s):
            fin = sim_t[0] + duree_s
            while sim_t[0] < fin and not (phrases and phrases[0][0] <= sim_t[0]):
                time.sleep(0.002)
            return AudioChunk(sample_rate=16000, channels=1, sample_width=2, data=b"")

    class Transcription(ISpeechRecognizer):
        def transcribe(self, audio):
            if phrases and phrases[0][0] <= sim_t[0]:
                return phrases.pop(0)[1], 1.0
            return "", 0.0

        @property
        def language(self):
            return "fr"

    speech = SpeechService(Micro(), Transcription())
    speech.start()
    voix = FakeSpeechSynthesizer()
    nrobot = NormalizedRobotHAL(hal, build_g1_normalizer())
    nrobot.initialize_all()
    runtime = PlayStoreRuntime(nrobot, WatchdogSystem(nrobot), locomotion_service=loco,
                               speech_service=speech, clock=horloge, profil=Profil.SIMULATION,
                               confirmer_permissions=accepter_tout_SIMULATION,
                               navigation_service=nav, speech_synthesizer=voix,
                               people_service=people, follow_service=follow)
    ici = Path(__file__).resolve().parent
    app = charger_app([d for d in (args.exemples, ici.parents[1] / "examples" / "follow_me",
                                   "/isaac-sim/examples/follow_me") if d])
    app_id = app.manifest().app_id
    check("App follow_me installée et démarrée", bool(runtime.install(app))
          and runtime.start_app(app_id))

    tenue = {"resultat": None, "fini": False}

    def mission():
        try:
            tenue["resultat"] = runtime.run_mission(app_id, params={"max_s": DUREE})
        finally:
            tenue["fini"] = True
    threading.Thread(target=mission, daemon=True).start()

    rec = None
    if not args.sans_video:
        rec = SceneRecorder(world, str(VIDEOS / "f1_suivi_personne.mp4"),
                            position=(1.0, -6.0, 4.0), cible=(5.0, 0.0, 0.8),
                            titre="AXIS-OS — « suis-moi », sans reconnaissance faciale")
        rec.suivre(lambda: robot.get_world_pose()[0], decalage=(-2.4, -3.2, 2.2),
                   hauteur_visee=0.9)
        if not rec.start():
            rec = None
    pas_video = 2 * max(1, args.acceleration)

    d_min, bleue_suivie, chute, k = math.inf, False, False, 0
    t_mur = time.monotonic()
    while sim_t[0] < DUREE + 10.0 and not tenue["fini"]:
        pr = pose2d()
        monde.x, monde.y, monde.heading = pr.x, pr.y, pr.cap
        for p, s in zip(personnes, silhouettes):
            p._avancer(cfg.control_dt)
            s.set_world_pose(position=np.array([p.x, p.y, 0.85]))
        if k % 5 == 0:
            people.tick()
        follow.tick()
        nav.tick()
        loco.tick()
        if loco.state is LocomotionState.FALLEN:
            chute = True
            break
        rendre = (args.detecteur == "rtdetr" and k % 5 == 0) or (rec is not None
                                                                 and k % pas_video == 0)
        if rendre:
            p_, q_ = robot.get_world_pose()
            R = quat_wxyz_to_matrix(q_)
            p_w = np.asarray(p_, dtype=float)
            for c in (cam, dep):
                c._position = tuple(float(v) for v in (p_w + R @ np.array([0.2, 0.0, 0.4])))
                c.look_at(tuple(float(v) for v in (p_w + R @ np.array([2.0, 0.0, -0.2]))))
            tree.set(Transform(FRAME_WORLD, FRAME_BASE,
                               translation=Vector3(*(float(v) for v in p_w)),
                               rotation=Quaternion(w=float(q_[0]), x=float(q_[1]),
                                                   y=float(q_[2]), z=float(q_[3]))))
            tf = cam.optical_transform(parent=FRAME_WORLD, child=FRAME_CAMERA_OPTICAL)
            if tf is not None:
                tree.set(tf)
            if rec is not None:
                rec.cadrer()
        for i in range(cfg.decimation - 1):
            world.step(render=(rendre and i == 0))
        sim_t[0] += cfg.control_dt
        k += 1
        if follow.active:
            pa = pose2d()
            d_min = min(d_min, math.hypot(rouge.x - pa.x, rouge.y - pa.y))
            cible = people.track(follow._cible)
            if cible is not None and math.hypot(cible.x - bleue.x, cible.y - bleue.y) < 0.3 \
                    and math.hypot(cible.x - rouge.x, cible.y - rouge.y) > 1.0:
                bleue_suivie = True
        if rec is not None and k % pas_video == 0:
            v = loco.dernier_verdict
            st = follow.status(follow.owner()) if follow.active else {"status": "—"}
            rec.image([
                f"Suivi : {st.get('status')} {st.get('phase', '')}",
                f"Distance : {st.get('distance_m') or '—'} m (cible 1.5 m)",
                f"Vitesse : {loco._twist.vx:.2f} m/s (max {FollowService.MAX_SPEED})",
                f"Gardien : {v.raison[:40] if v else '—'}",
                f"Reprises : {follow.stats['reacquisitions']}  doutes : "
                f"{follow.stats['identity_doubts']}",
                f"Détection : {people.metrics['detection_hz'] or 0:.0f} Hz",
                "Vie privée : aucun visage, aucune image gardée",
            ], sim_t[0])
            for texte, _ in voix.spoken[-1:]:
                rec.evenement(texte, sim_t[0], duree_s=2.0, couleur=(40, 140, 70))
    if rec is not None:
        rec.terminer()

    r = tenue["resultat"]
    print(f"\n{'=' * 66}\n  RÉSULTATS F1 ({time.monotonic() - t_mur:.0f} s réelles)\n{'=' * 66}")
    print(f"  mission : {None if r is None else r.metrics}\n  voix : {voix.spoken}"
          f"\n  suivi : {follow.stats}\n  personnes : {people.metrics}\n  gardien : {gardien.stats}")
    check("Pas de chute", not chute)
    check("La mission a fini à la voix (STOPPED_BY_APP)",
          r is not None and r.metrics.get("outcome") == "STOPPED_BY_APP", str(r and r.metrics))
    check("Le robot a suivi (plus de 2 m parcourus)", pose2d().x > 2.0, f"x = {pose2d().x:.2f}")
    check(f"Jamais à moins de 1.4 m de la personne suivie (minimum {d_min:.2f} m)", d_min >= 1.4)
    check("Jamais en arrière ni de côté, jamais plus de 0.25 m/s",
          all(0.0 <= c[1] <= FollowService.MAX_SPEED and c[2] == 0.0 for c in consignes))
    check("Jamais la personne bleue à la place de la rouge", not bleue_suivie)
    check("Rien de la personne en mémoire après le suivi",
          follow._reference is None and len(people._tracker) == 0)
    simulation_app.close()
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
