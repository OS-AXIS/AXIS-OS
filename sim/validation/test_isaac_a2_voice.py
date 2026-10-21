"""
Suite A2 — le robot obéit à la voix, bout en bout.

Trois phrases sont « prononcées » : une commande, une phrase hors sujet,
puis un ordre d'arrêt. Le robot doit se mettre en marche sur la
première, IGNORER la deuxième, et s'arrêter sur la troisième.

BOUCLE ACOUSTIQUE FERMÉE, si possible : les phrases sont SYNTHÉTISÉES
(modèle de synthèse vocale), puis transcrites par Whisper. Ce n'est pas
un raccourci — l'audio traverse réellement le modèle de transcription,
avec ses conversions de fréquence et de format.

Si le modèle de synthèse n'est pas disponible, le test bascule sur une
transcription scriptée et le DIT clairement : la chaîne
intention -> action reste validée, l'étage acoustique non. Mieux vaut
un test qui annonce ce qu'il ne prouve pas qu'un test qui laisse croire.

  docker exec isim-isaac-sim-1 /isaac-sim/python.sh \\
      /isaac-sim/axis_os/test_isaac_a2_voice.py
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
COLIS = ("colis", (2.5, 0.0, 0.3), (0.6, 0.5, 0.6))
PHRASES = [
    ("va vers le carton", "aller_vers_colis"),
    ("il fait beau aujourd'hui", None),      # doit être IGNORÉE
    ("stop", "arreter"),
]
FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def synthetiser(phrases):
    """Phrases -> AudioChunk, par synthèse vocale. None si indisponible."""
    try:
        import numpy as np
        import torch
        from transformers import AutoTokenizer, VitsModel
        from axis_os.hal.core.types import AudioChunk
    except ImportError as exc:
        print(f"  synthèse vocale indisponible ({exc})")
        return None
    try:
        tok = AutoTokenizer.from_pretrained("facebook/mms-tts-fra")
        tts = VitsModel.from_pretrained("facebook/mms-tts-fra")
        tts.eval()
    except Exception as exc:            # noqa: BLE001
        print(f"  modèle de synthèse non chargé : {exc}")
        return None

    out = []
    for texte, _ in phrases:
        with torch.no_grad():
            wav = tts(**tok(texte, return_tensors="pt")).waveform[0].numpy()
        pcm = np.clip(wav, -1.0, 1.0)
        pcm = (pcm * 32767.0).astype(np.int16)
        out.append(AudioChunk(sample_rate=int(tts.config.sampling_rate),
                              channels=1, sample_width=2, data=pcm.tobytes()))
    print(f"  {len(out)} phrase(s) synthétisée(s) à "
          f"{tts.config.sampling_rate} Hz")
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
    from axis_os.services.speech.service import ISpeechRecognizer, SpeechService
    from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
    from axis_os.store.runtime.profil import Profil

    section("1 — Audio : synthèse puis transcription réelle ?")

    chunks = synthetiser(PHRASES)
    acoustique_reelle = chunks is not None

    if acoustique_reelle:
        from axis_os.services.speech.whisper_recognizer import WhisperRecognizer
        rec = WhisperRecognizer(model_name="openai/whisper-small", language="fr")
        acoustique_reelle = rec.load()
    if not acoustique_reelle:
        print("\n  ⚠ ÉTAGE ACOUSTIQUE NON VALIDÉ : transcription scriptée.")
        print("    La chaîne intention -> action reste testée ; la")
        print("    reconnaissance vocale elle-même, non.")

        from axis_os.hal.core.types import AudioChunk

        class ScriptedRecognizer(ISpeechRecognizer):
            def __init__(self):
                self.i = 0

            def transcribe(self, audio):
                if self.i < len(PHRASES):
                    t = PHRASES[self.i][0]
                    self.i += 1
                    return t, 1.0
                return "", 0.0

            @property
            def language(self):
                return "fr"

        rec = ScriptedRecognizer()
        chunks = [AudioChunk(16000, 1, 2, b"\x01\x00" * 16000)
                  for _ in PHRASES]
    else:
        # Vérifier la transcription AVANT de lancer le robot : si
        # Whisper ne comprend pas, autant le savoir tout de suite.
        for (texte, _), ch in zip(PHRASES, chunks):
            trans, conf = rec.transcribe(ch)
            print(f"    « {texte} » -> « {trans} » (confiance {conf:.2f})")

    section("2 — Scène et services")

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

    mic = QueuedMicrophone("head_mic", chunks)
    speech = SpeechService(mic, rec, min_confidence=0.2)
    check("Service de parole démarré", speech.start().name == "OK")

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

    nrobot = NormalizedRobotHAL(hal, build_g1_normalizer())
    nrobot.initialize_all()
    runtime = PlayStoreRuntime(nrobot, WatchdogSystem(nrobot),
                               perception_service=perception,
                               locomotion_service=locomotion,
                               speech_service=speech,
                               confirmer_permissions=accepter_tout_SIMULATION,
                               profil=Profil.SIMULATION)
    app = VoiceInspectorApp()
    man = app.manifest()
    runtime.install(app)
    runtime.start_app(man.app_id)
    check("Vocabulaire déclaré par l'app",
          len(speech.registry.declarations) == 3)

    section("3 — Mission vocale")

    holder = {"result": None, "error": None, "done": False}

    def mission():
        try:
            holder["result"] = runtime.run_mission(
                man.app_id, params={"timeout_s": 60.0})
        except BaseException as exc:            # noqa: BLE001
            holder["error"] = exc
        finally:
            holder["done"] = True

    th = threading.Thread(target=mission, daemon=True)
    th.start()

    RENDER_TOUS_LES = 5
    t_start = time.monotonic()
    k = 0
    x_apres_commande = None
    while not holder["done"] and time.monotonic() - t_start < 240.0:
        locomotion.tick()
        rendre = (k % RENDER_TOUS_LES == 0)
        for i in range(cfg.decimation - 1):
            world.step(render=(rendre and i == 0))
        sim_t[0] += cfg.control_dt
        k += 1
        if rendre:
            pos, quat = robot.get_world_pose()
            R = quat_wxyz_to_matrix(quat)
            p_w = np.asarray(pos, dtype=float)
            for c in (cam, dep):
                c._position = tuple(float(v) for v in (p_w + R @ np.array([0.2, 0.0, 0.4])))
                c.look_at(tuple(float(v) for v in (p_w + R @ np.array([2.0, 0.0, -0.9]))))
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

    section("4 — Résultat")

    if holder["error"] is not None:
        import traceback
        traceback.print_exception(type(holder["error"]), holder["error"],
                                  holder["error"].__traceback__)
        check("Mission sans exception", False)

    r = holder["result"]
    check("La mission a rendu un résultat", r is not None)
    pos, _ = robot.get_world_pose()
    avance = float(pos[0])

    if r is not None:
        print(f"  succès={r.success}  métriques : {r.metrics}")
        comprises = r.metrics.get("commandes_comprises", 0)
        ignorees = r.metrics.get("phrases_ignorees", 0)
        check(f"Au moins une commande COMPRISE ({comprises})", comprises >= 1)
        check(f"Des phrases ont été IGNORÉES ({ignorees}) — une phrase hors "
              f"vocabulaire ne doit rien déclencher", ignorees >= 1)

    check(f"Le robot s'est déplacé sur commande vocale (x={avance:.2f})",
          avance > 0.5)
    print(f"\n  parole : {speech.stats}")
    print(f"  locomotion : {locomotion.stats}")
    check("Aucune consigne refusée par le garde-fou",
          locomotion.stats["refused"] == 0)
    check("Le robot n'est pas tombé", locomotion.state.name != "FALLEN")
    check("Des phrases sans intention ont été comptées comme telles",
          speech.stats["sans_intention"] >= 1
          or speech.stats["transcriptions_vides"] >= 1)

    print(f"\n{'=' * 66}")
    print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
    print("=" * 66)
    if FAIL == 0:
        print("\n  ✓ Le robot obéit à la voix et ignore ce qu'il ne comprend pas")
        if acoustique_reelle:
            print("  ✓ Boucle acoustique FERMÉE : synthèse -> Whisper -> intention")
        else:
            print("  ⚠ Étage acoustique NON validé (transcription scriptée)")

    simulation_app.close()
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
