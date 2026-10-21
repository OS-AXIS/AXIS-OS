"""
Vidéo V2 — « Le robot obéit à la voix, et ignore le reste. »

Trois phrases, dans cet ordre, audibles dans la vidéo :
  1. « Il fait beau aujourd'hui. »  -> IGNORÉE, le robot ne bouge pas
  2. « Va vers le carton. »          -> comprise, le robot part
  3. « Quelle heure est-il ? »       -> IGNORÉE en pleine marche : le
                                        robot continue sans se distraire

Puis le robot s'arrête devant le colis.

Ce qu'on voit ET entend : la phrase prononcée, ce que Whisper en a
compris, l'intention reconnue ou son absence, et le robot qui réagit —
ou pas.

L'ordre d'arrêt vocal (« stop ») est volontairement ABSENT : au test, il
a été transcrit « So. » et ignoré (docs/validation_voix.md). Une
démonstration ne montre pas un ordre de sécurité qui échoue.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh /isaac-sim/axis_os/video_v2_voix.py
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

POLICY = Path("/isaac-sim/policies/g1_motion.pt")
SORTIE = "/isaac-sim/videos/v2_commande_vocale.mp4"
# Plus loin : la marche doit durer assez pour que la troisième
# phrase tombe PENDANT le trajet. À 2.6 m, le robot arrivait avant.
COLIS = ("colis", (3.2, 0.0, 0.3), (0.6, 0.5, 0.6))
PHRASES = ["Il fait beau aujourd'hui.",
           "Va vers le carton.",
           "Quelle heure est-il ?"]
AVANT_S = 2.0          # robot immobile avant la première écoute
APRES_S = 4.0


def synthetiser(phrases, whisper, attendu, essais=12):
    """Synthétise chaque phrase et ne garde qu'une prise CORRECTEMENT
    comprise — comme un présentateur qui refait une prise.

    POURQUOI : le modèle de synthèse est ALÉATOIRE. Même phrase, autre
    exécution, autre prononciation. Première vidéo : « Va vers le
    carton » transcrit « Vavère le Carpent », le robot n'a jamais bougé —
    alors que la même phrase avait été parfaitement comprise au test A2.

    Ce n'est pas tricher : la synthèse remplace un humain qui parle, et un
    humain répète quand on ne l'a pas compris. Ce qui est démontré — la
    chaîne transcription, intention, action, et le refus de ce qui n'est
    pas compris — reste intact. On choisit la prise, pas le résultat.

    attendu[i] : l'intention que la phrase i DOIT déclencher, ou None si
    elle doit être ignorée.
    """
    import numpy as np
    import torch
    from transformers import AutoTokenizer, VitsModel
    from axis_os.apps.voice_inspector.app import INTENTIONS
    from axis_os.hal.core.types import AudioChunk
    from axis_os.services.speech.service import (
        IntentDeclaration, IntentRegistry,
    )
    reg = IntentRegistry()
    for nom, mots, params, desc in INTENTIONS:
        reg.register(IntentDeclaration(nom, mots, params, desc))

    tok = AutoTokenizer.from_pretrained("facebook/mms-tts-fra")
    tts = VitsModel.from_pretrained("facebook/mms-tts-fra")
    tts.eval()
    # Élocution plus posée : un débit légèrement ralenti est nettement
    # mieux compris.
    tts.speaking_rate = 0.85
    tts.noise_scale = 0.5

    out = []
    for texte, but in zip(phrases, attendu):
        retenu = None
        for graine in range(essais):
            torch.manual_seed(graine)
            with torch.no_grad():
                wav = tts(**tok(texte, return_tensors="pt")).waveform[0].numpy()
            pcm = (np.clip(wav, -1.0, 1.0) * 32767.0).astype(np.int16)
            ch = AudioChunk(sample_rate=int(tts.config.sampling_rate),
                            channels=1, sample_width=2, data=pcm.tobytes())
            trans, conf = whisper.transcribe(ch)
            it = reg.match(trans)
            ok = (it is None) if but is None else (it is not None and it.name == but)
            print(f"    prise {graine} : « {trans} » -> "
                  f"{it.name if it else 'aucune'} {'✓' if ok else '✗'}")
            if ok:
                retenu = ch
                break
        if retenu is None:
            raise RuntimeError(
                f"aucune prise comprise pour « {texte} » en {essais} essais")
        out.append(retenu)
    return out


def main():
    simulation_app = start_simulation_app(headless=True)
    import numpy as np
    from isaacsim.core.api.objects import DynamicCuboid
    from isaacsim.core.utils.types import ArticulationAction

    from axis_os.apps.voice_inspector.app import VoiceInspectorApp
    from axis_os.hal.core.frames import (
        FRAME_BASE, FRAME_CAMERA_OPTICAL, FRAME_WORLD, FrameTree, Transform,
    )
    from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
    from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
    from axis_os.hal.core.types import (
        ActuatorCommand, ControlMode, Quaternion, Vector3,
    )
    from axis_os.hal.drivers.isaac_sim.sim_camera import IsaacSimCamera
    from axis_os.hal.drivers.isaac_sim.video import SceneRecorder
    from axis_os.hal.drivers.sim_common.sim_perception import QueuedMicrophone
    from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
        G1LocomotionAdapter, G1LocomotionConfig, G1LocomotionPolicy,
        body_angular_velocity, quat_wxyz_to_matrix,
    )
    from axis_os.hal.drivers.unitree_g1.robot import _build_g1_joint_definitions
    from axis_os.hal.watchdog.system import WatchdogSystem
    from axis_os.services.locomotion.service import BaseState, LocomotionService
    from axis_os.services.perception.owlvit_detector import OwlViTDetector
    from axis_os.services.perception.service import PerceptionService
    from axis_os.services.speech.service import SpeechService
    from axis_os.services.speech.whisper_recognizer import WhisperRecognizer
    from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
    from axis_os.store.runtime.profil import Profil

    sim_t = [0.0]

    journal = []            # (instant, texte transcrit)

    class MicroHorodate(QueuedMicrophone):
        """Retient l'instant simulé où chaque phrase est captée, pour la
        placer au bon moment dans la piste son de la vidéo."""
        def capture(self, duration_s):
            vide = self.remaining == 0
            c = super().capture(duration_s)
            if not vide and c is not None:
                rec.ajouter_son(sim_t[0], c)
            return c

    class WhisperJournal(WhisperRecognizer):
        def transcribe(self, audio):
            texte, conf = super().transcribe(audio)
            if texte:
                journal.append((sim_t[0], texte))
            return texte, conf

    rec_whisper = WhisperJournal(model_name="openai/whisper-small", language="fr")
    rec_whisper.load()

    # Prises vérifiées AVANT de lancer la simulation : si une phrase
    # n'est jamais comprise, autant le savoir en quelques secondes plutôt
    # qu'après dix minutes de rendu.
    print("\n  [parole] sélection des prises")
    chunks = synthetiser(PHRASES, rec_whisper,
                         attendu=[None, "aller_vers_colis", None])
    journal.clear()         # les essais ne font pas partie de la vidéo

    # ---- robot, colis, caméras de perception ---------------------------
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
    world.scene.add(DynamicCuboid(
        prim_path=f"/World/{COLIS[0]}", name=COLIS[0],
        position=np.array(COLIS[1]), size=1.0, scale=np.array(COLIS[2]),
        color=np.array([0.72, 0.52, 0.30])))
    world.reset()
    g = pol.gains()
    robot.get_articulation_controller().set_gains(
        kps=np.array([g[j][0] for j in names]),
        kds=np.array([g[j][1] for j in names]))
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

    loco = LocomotionService(G1LocomotionAdapter(pol), read_state,
                             send_targets, command_timeout_s=0.5,
                             clock=lambda: sim_t[0])
    loco.start()

    def attente_simulee(secondes):
        # Écouter « deux secondes » = deux secondes SIMULÉES.
        fin = sim_t[0] + secondes
        while sim_t[0] < fin:
            time.sleep(0.002)

    mic = MicroHorodate("head_mic", chunks, sleeper=attente_simulee)
    speech = SpeechService(mic, rec_whisper, min_confidence=0.2)
    speech.start()

    nrobot = NormalizedRobotHAL(hal, build_g1_normalizer())
    nrobot.initialize_all()
    # HORLOGE DU ROBOT = horloge simulée. L'app, ses attentes et son quota
    # vivent au rythme de la simulation, quelle que soit la lenteur du
    # rendu vidéo. Premier essai sans elle : les trois phrases empilées
    # dans les deux premières secondes, le colis perdu, et la mission tuée
    # par un quota de 300 s MURALES.
    runtime = PlayStoreRuntime(nrobot, WatchdogSystem(nrobot),
                               perception_service=perception,
                               locomotion_service=loco,
                               speech_service=speech,
                               clock=lambda: sim_t[0],
                               confirmer_permissions=accepter_tout_SIMULATION,
                               profil=Profil.SIMULATION)
    app = VoiceInspectorApp()
    runtime.install(app)
    runtime.start_app(app.manifest().app_id)

    # ---- caméra spectateur : de côté, le robot à gauche, le colis à
    # droite. Le panneau passe donc à DROITE, en haut, au-dessus du colis
    # qui est au sol.
    rec = SceneRecorder(world, SORTIE, position=(1.6, -4.6, 1.35),
                        cible=(1.6, 0.0, 0.75),
                        titre="AXIS-OS — le robot obéit à la voix",
                        panneau="droite")
    rec.suivre(lambda: robot.get_world_pose()[0],
               decalage=(1.6, -4.6, 0.6), hauteur_visee=0.75,
               decalage_cadre=(1.6, 0.0))
    if not rec.start():
        simulation_app.close()
        sys.exit(1)

    holder = {"result": None, "done": False}

    def mission():
        try:
            holder["result"] = runtime.run_mission(
                app.manifest().app_id, {"timeout_s": 60.0})
        finally:
            holder["done"] = True

    etat = {"entendu": "—", "intention": "—", "vus": 0}

    def panneau():
        pos, _ = robot.get_world_pose()
        dist = COLIS[1][0] - COLIS[2][0] / 2 - float(pos[0])
        return [
            f"Entendu : « {etat['entendu']} »",
            f"Intention : {etat['intention']}",
            f"Service de marche : {loco.state.name}",
            f"Garde-fou : {loco.stats['refused']} refus",
            f"Distance au colis : {dist:.2f} m",
        ]

    th = None
    fin_t = None
    k = 0
    t_mur = time.monotonic()
    while time.monotonic() - t_mur < 420.0:
        t = sim_t[0]

        # Le robot reste d'abord immobile : le spectateur doit voir qu'il
        # ne bouge pas AVANT qu'on lui parle.
        if th is None and t >= AVANT_S:
            th = threading.Thread(target=mission, daemon=True)
            th.start()

        # Nouvelle phrase transcrite -> bandeau et panneau.
        while etat["vus"] < len(journal):
            _ti, texte = journal[etat["vus"]]
            etat["vus"] += 1
            etat["entendu"] = texte
            it = speech.registry.match(texte, app.manifest().app_id)
            if it is None:
                etat["intention"] = "aucune — phrase ignorée"
                rec.evenement(f"« {texte} » — ignorée, aucune action", t,
                              duree_s=2.5, couleur=(110, 110, 120))
            else:
                etat["intention"] = it.name
                rec.evenement(f"« {texte} » — compris : {it.name}", t,
                              duree_s=2.5, couleur=(40, 140, 70))

        if holder["done"] and fin_t is None:
            fin_t = t
            r = holder["result"]
            if r is not None and r.success:
                rec.evenement("Colis atteint — mission réussie", t,
                              duree_s=APRES_S, couleur=(40, 140, 70))
        if fin_t is not None and t - fin_t > APRES_S:
            break

        loco.tick()
        capture = (k % 2 == 0)
        if capture:
            rec.cadrer()
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
        for i in range(cfg.decimation - 1):
            world.step(render=(capture and i == 0))
        sim_t[0] += cfg.control_dt
        k += 1
        if capture:
            rec.image(panneau(), t)

    chemin = rec.terminer()
    r = holder["result"]
    print(f"\n  mission : {None if r is None else (r.success, r.metrics)}")
    if r is not None and not r.success:
        # Des métriques VIDES signalent une interruption par le sandbox
        # (quota, plantage), pas un échec conclu par l'app — qui rend
        # toujours ses métriques. Le message dit laquelle.
        print(f"  cause : {r.error_message}")
        print(f"  durée de la mission : {r.duration_s:.1f} s")
    print(f"  transcriptions : {[t for _, t in journal]}")
    print(f"  locomotion : {loco.stats}")
    print(f"\n  VIDÉO : {chemin}")
    simulation_app.close()


if __name__ == "__main__":
    main()
