"""
SDK, partie 5 : animations.

Vérifié ici :
  1. le format (load_animation) : champs, types, ordre des images ;
  2. la validation avant de jouer (validate_animation) : robot
     compatible, articulations connues et animables, butées, vitesses,
     accélérations, jambes refusées sans la mention « robot à l'arrêt,
     jambes immobiles », et immobiles avec elle ;
  3. les animations fournies avec le G1 (wave, point) sont valides et se
     jouent dans le simulateur léger : les bras suivent, sans dépasser la
     vitesse permise, les jambes ne sont jamais commandées ;
  4. play_animation / stop_animation : cas normal, attente ou non, arrêt,
     délai, pause devant une personne, BUSY, chemin de fichier refusé ;
     permission manquante, arguments invalides, arrêt d'urgence (avant et
     pendant), capacité absente ; en direct et dans le processus de l'app ;
  5. `axis animation validate <fichier>`.
"""
import contextlib
import copy
import io
import json
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.types import JointGroup  # noqa: E402
from axis_os.store.runtime.hal_bridge import HALBridgeServer  # noqa: E402
from axis_os.store.sdk.animation import (  # noqa: E402
    ANIM_MAX_VEL, Animation, library_names, library_path, load_animation,
    load_library_animation, validate_animation,
)
from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext  # noqa: E402
from axis_os.store.sdk.manifest import (  # noqa: E402
    PERMISSION_LEVELS, AppCategory, AppManifest, Permission, ProtectionLevel,
)
from axis_os.store.sdk.results import AxisRefusal, InvalidArgumentError, RefusalCode  # noqa: E402
from axis_os.testing import AppTestHarness, SimulatedRobot  # noqa: E402
from axis_os.testing.robot_simule import _HALSimulee  # noqa: E402

FAIL = 0
RACINE = Path(__file__).resolve().parents[1]
EPAULE = "right_shoulder_pitch_joint"


def check(label, cond, detail=""):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}" + (f"  ({detail})" if detail and not cond else ""))
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


def silencieux(f, *a, **kw):
    with contextlib.redirect_stdout(io.StringIO()):
        return f(*a, **kw)


def leve(code, f, *a, **kw):
    try:
        silencieux(f, *a, **kw)
        return False
    except AxisRefusal as exc:
        return exc.code is code


def manifeste(perms):
    return AppManifest(app_id="com.test.anim", name="Anim", version="1.0.0", author="T",
                       author_email="t@t.com", description="d",
                       category=AppCategory.UTILITIES, permissions=list(perms))


TOUT = [Permission.ANIMATION_PLAY, Permission.ACTUATORS_READ, Permission.ACTUATORS_WRITE_ARMS]


def contexte(robot, perms=None):
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog,
                                   manifeste(TOUT if perms is None else perms))
    ctx._set_clock(robot.clock)
    ctx._attach_services(locomotion=robot.locomotion, manipulation=robot.manipulation)
    return ctx


def base(**modif):
    """Une animation valide minimale (bras droit), modifiable."""
    d = {"format": "axis-animation", "format_version": 1, "name": "essai", "version": "1.0.0",
         "robots": ["G1"], "keyframes": [
             {"t": 0.0, "positions": {EPAULE: 0.0}},
             {"t": 1.0, "positions": {EPAULE: -0.5}},
             {"t": 2.0, "positions": {EPAULE: 0.0}}]}
    d.update(modif)
    return d


def avec_images(*images, **modif):
    return base(keyframes=[{"t": t, "positions": p} for t, p in images], **modif)


def refus_format(d):
    try:
        load_animation(d)
        return False
    except InvalidArgumentError:
        return True


ROBOT_DEFS = None


def defs_g1():
    global ROBOT_DEFS
    if ROBOT_DEFS is None:
        with SimulatedRobot() as r:
            ROBOT_DEFS = r.hal.get_joint_definitions()
    return ROBOT_DEFS


def problemes(d, modele="G1"):
    return validate_animation(load_animation(d), defs_g1(), robot_model=modele)


def jambes(robot):
    etat = robot.hal.get_full_state()
    return {d.joint_id: etat[d.joint_id].position for d in robot.hal.get_joint_definitions()
            if d.group is JointGroup.LOCOMOTION}


section("0. Permission déclarée")
check("ANIMATION_PLAY est DANGEROUS (elle fait bouger le robot)",
      PERMISSION_LEVELS[Permission.ANIMATION_PLAY] is ProtectionLevel.DANGEROUS)

section("1. Format")
check("Animation de base : chargée", isinstance(load_animation(base()), Animation))
check("… depuis un texte JSON aussi", load_animation(json.dumps(base())).name == "essai")
for label, d in [
        ("format inconnu", base(format="autre")),
        ("version de format 2", base(format_version=2)),
        ("nom avec espace", base(name="mon geste")),
        ("version non semver", base(version="v1")),
        ("robots vide", base(robots=[])),
        ("champ inconnu", base(vitesse=10)),
        ("une seule image", avec_images((0.0, {EPAULE: 0.0}))),
        ("première image à t = 0.5", avec_images((0.5, {EPAULE: 0.0}), (1.0, {EPAULE: 0.1}))),
        ("instants décroissants", avec_images((0.0, {EPAULE: 0.0}), (1.0, {EPAULE: 0.1}),
                                              (0.5, {EPAULE: 0.0}))),
        ("position NaN", avec_images((0.0, {EPAULE: 0.0}), (1.0, {EPAULE: float("nan")}))),
        ("position texte", avec_images((0.0, {EPAULE: 0.0}), (1.0, {EPAULE: "0.1"}))),
        ("durée de 2 min", avec_images((0.0, {EPAULE: 0.0}), (120.0, {EPAULE: 0.1}))),
        ("articulation dans une seule image", avec_images((0.0, {EPAULE: 0.0}),
                                                          (1.0, {"right_elbow_joint": -0.2})))]:
    check(f"Refusée au format : {label}", refus_format(d))
check("Refusée : texte JSON avec NaN", refus_format(
    '{"format": "axis-animation", "format_version": 1, "name": "x", "version": "1.0.0", '
    '"robots": ["G1"], "keyframes": [{"t": 0, "positions": {"a": NaN}}, '
    '{"t": 1, "positions": {"a": 0}}]}'))
check("Refusée : plus de 256 Kio", refus_format(base(description="x" * 400_000)))

section("2. Validation pour le G1, avant de jouer")
check("Animation de base : aucun problème", problemes(base()) == [], str(problemes(base())))
for label, d, attendu in [
        ("articulation inconnue", avec_images((0.0, {"tentacule": 0.0}), (1.0, {"tentacule": 0.1})),
         "inconnue"),
        ("taille (ni bras ni jambe)", avec_images((0.0, {"waist_yaw_joint": 0.0}),
                                                  (1.0, {"waist_yaw_joint": 0.2})), "non animable"),
        ("coude au-delà de sa butée", avec_images((0.0, {"right_elbow_joint": 0.0}),
                                                  (1.0, {"right_elbow_joint": -1.2})), "hors de"),
        ("épaule à 5 rad/s", avec_images((0.0, {EPAULE: 0.0}), (0.5, {EPAULE: -2.5})), "rad/s"),
        ("accélération de 36 rad/s²", avec_images(*[(i / 10, {EPAULE: 0.0 if i % 2 == 0 else -0.18})
                                                    for i in range(8)]), "accélération"),
        ("genou sans la mention", avec_images((0.0, {"left_knee_joint": 0.0}),
                                              (1.0, {"left_knee_joint": 0.0})), "jambe"),
        ("genou qui bouge malgré la mention",
         avec_images((0.0, {"left_knee_joint": 0.0}), (1.0, {"left_knee_joint": 0.3}),
                     stationary_legs_still=True), "bouge")]:
    p = problemes(d)
    check(f"Refusée : {label}", any(attendu in x for x in p), str(p))
check("Refusée : robot incompatible (H1)", any("pas pour H1" in x for x in problemes(base(), "H1")))
check("Genou immobile avec la mention « robot à l'arrêt, jambes immobiles » : acceptée",
      problemes(avec_images((0.0, {"left_knee_joint": 0.0, EPAULE: 0.0}),
                            (1.0, {"left_knee_joint": 0.0, EPAULE: -0.3}),
                            stationary_legs_still=True)) == [])
check("Animations fournies avec le G1 : wave et point (pas de cou : pas de hochement)",
      library_names("G1") == ["point", "wave"], str(library_names("G1")))
for nom in library_names("G1"):
    p = validate_animation(load_library_animation(nom, "G1"), defs_g1(), robot_model="G1")
    check(f"« {nom} » : valide pour le G1", p == [], str(p))

section("3. Jouer en simulation (en direct)")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    j0 = jambes(robot)
    vus = []
    arret = threading.Event()

    def echantillonner():
        while not arret.is_set():
            vus.append((time.monotonic(), robot.hal.get_full_state()[EPAULE].position))
            time.sleep(0.02)
    fil = threading.Thread(target=echantillonner)
    fil.start()
    r = silencieux(ctx.play_animation, "wave")
    arret.set()
    fil.join()
    plus_haut = min(p for _, p in vus)
    check("wave : PLAYED", r.ok and r.status == "PLAYED", str(r))
    check("… le bras est monté jusqu'à environ -1.5 rad", abs(plus_haut + 1.5) < 0.1,
          f"{plus_haut:.3f}")
    check("… et revenu au repos", abs(robot.hal.get_full_state()[EPAULE].position) < 0.05)
    vmax = max(abs(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(vus, vus[5:]))
    check(f"… sans dépasser {ANIM_MAX_VEL} rad/s (marge 25 %)", vmax <= ANIM_MAX_VEL * 1.25,
          f"{vmax:.2f} rad/s")
    check("… jambes jamais commandées (positions inchangées)", jambes(robot) == j0)

    r = silencieux(ctx.play_animation, "point", wait=False)
    check("point, wait=False : PLAYING aussitôt", r.ok and r.status == "PLAYING", str(r))
    r2 = silencieux(ctx.play_animation, "wave")
    check("Deuxième animation pendant la première : REFUSED/BUSY", r2.code == "BUSY", str(r2))
    time.sleep(0.6)
    r = silencieux(ctx.stop_animation)
    p1 = robot.hal.get_full_state()[EPAULE].position
    time.sleep(0.4)
    check("stop_animation : STOPPED, le bras s'arrête là où il est",
          r.status == "STOPPED" and abs(robot.hal.get_full_state()[EPAULE].position - p1) < 0.02
          and p1 < -0.1, f"{r} {p1:.3f}")
    check("stop_animation sans animation : IDLE", silencieux(ctx.stop_animation).status == "IDLE")
    r = silencieux(ctx.play_animation, base())
    check("Animation passée en dict par l'app : PLAYED", r.ok, str(r))
    r = silencieux(ctx.play_animation, "wave", timeout_s=1.0)
    p2 = robot.hal.get_full_state()[EPAULE].position
    time.sleep(0.3)
    check("Délai plus court que le geste : TIMEOUT, et le geste est arrêté",
          r.status == "TIMEOUT" and abs(robot.hal.get_full_state()[EPAULE].position - p2) < 0.02,
          str(r))

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    robot.add_person(0.6, 0.0)
    time.sleep(0.3)
    threading.Timer(1.5, robot.clear_people).start()
    t0 = time.monotonic()
    r = silencieux(ctx.play_animation, "point")
    duree = time.monotonic() - t0
    check("Personne trop près : le geste se met en pause, puis reprend et se termine",
          r.ok and duree > 3.5 + 1.0, f"{r} en {duree:.1f} s")

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    j = jambes(robot)
    genou = j["left_knee_joint"]
    d = avec_images((0.0, {"left_knee_joint": genou, EPAULE: 0.0}),
                    (1.0, {"left_knee_joint": genou, EPAULE: -0.3}), stationary_legs_still=True)
    r = silencieux(ctx.play_animation, d)
    check("Genou décrit immobile, posture identique, robot à l'arrêt : PLAYED, genou jamais "
          "commandé", r.ok and jambes(robot) == j, str(r))
    d2 = copy.deepcopy(d)
    for k in d2["keyframes"]:
        k["positions"]["left_knee_joint"] = genou + 0.3
    r = silencieux(ctx.play_animation, d2)
    check("Posture des jambes différente de celle décrite : REFUSED/SAFETY_LIMIT",
          r.code == "SAFETY_LIMIT", str(r))
    robot.locomotion.set_velocity(0.2, 0.0, 0.0)
    r = silencieux(ctx.play_animation, d)
    check("Robot en marche : REFUSED/SAFETY_LIMIT", r.code == "SAFETY_LIMIT", str(r))
    robot.locomotion.halt()

section("4. Permission, arguments, arrêt d'urgence, capacité")
with SimulatedRobot() as robot:
    check("Sans ANIMATION_PLAY : PERMISSION_DENIED",
          leve(RefusalCode.PERMISSION_DENIED, contexte(robot, [Permission.ACTUATORS_READ,
                                                               Permission.ACTUATORS_WRITE_ARMS]).play_animation, "wave"))
    check("Sans ACTUATORS_WRITE_ARMS (droits de move_arms) : PERMISSION_DENIED",
          leve(RefusalCode.PERMISSION_DENIED, contexte(robot, [Permission.ANIMATION_PLAY,
                                                               Permission.ACTUATORS_READ]).play_animation, "wave"))
    check("stop_animation sans ANIMATION_PLAY : PERMISSION_DENIED",
          leve(RefusalCode.PERMISSION_DENIED, contexte(robot, [Permission.ACTUATORS_READ]).stop_animation))
    ctx = contexte(robot)
    depart = robot.hal.get_full_state()[EPAULE].position
    secret = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
    secret.write(json.dumps(base()))
    secret.close()
    for label, args, kw in [("nom inconnu", ("danse",), {}),
                            ("chemin de fichier (texte)", (secret.name,), {}),
                            ("chemin de fichier (Path)", (Path(secret.name),), {}),
                            ("nombre", (42,), {}),
                            ("wait texte", ("wave",), {"wait": "oui"}),
                            ("délai nul", ("wave",), {"timeout_s": 0}),
                            ("délai de 1000 s", ("wave",), {"timeout_s": 1000}),
                            ("dict invalide", (base(format="x"),), {}),
                            ("vitesse excessive", (avec_images((0.0, {EPAULE: 0.0}),
                                                               (0.5, {EPAULE: -2.5})),), {})]:
        check(f"{label} : INVALID_ARGUMENT",
              leve(RefusalCode.INVALID_ARGUMENT, ctx.play_animation, *args, **kw))
    check("… et rien n'a bougé", abs(robot.hal.get_full_state()[EPAULE].position - depart) < 0.01)
    silencieux(robot.hal.emergency_stop_all)
    check("Sous arrêt d'urgence : ESTOP_ACTIVE", leve(RefusalCode.ESTOP_ACTIVE, ctx.play_animation,
                                                     "wave"))

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    threading.Timer(0.8, lambda: silencieux(robot.hal.emergency_stop_all)).start()
    r = silencieux(ctx.play_animation, "wave")
    p = robot.hal.get_full_state()[EPAULE].position
    time.sleep(0.3)
    check("Arrêt d'urgence pendant le geste : REFUSED/ESTOP_ACTIVE, le bras ne bouge plus",
          r.code == "ESTOP_ACTIVE" and robot.hal.get_full_state()[EPAULE].position == p, str(r))


class _HALSansBras(_HALSimulee):
    """Un G1 amputé de ses bras (pour « capacité absente »)."""
    def get_joint_definitions(self):
        return [d for d in super().get_joint_definitions()
                if d.group is not JointGroup.MANIPULATION]


with SimulatedRobot() as robot:
    from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
    from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
    from axis_os.hal.watchdog.system import WatchdogSystem
    with contextlib.redirect_stdout(io.StringIO()):
        hal = NormalizedRobotHAL(_HALSansBras("g1_sans_bras"), build_g1_normalizer())
        hal.initialize_all()
    ctx = RobotContext.for_testing(hal, WatchdogSystem(hal), manifeste(TOUT))
    r = silencieux(ctx.play_animation, "wave")
    check("Robot sans bras : REFUSED/CAPABILITY_MISSING", r.code == "CAPABILITY_MISSING", str(r))


# ─── Depuis le processus de l'app ────────────────────────────────────────

class AppSalut(RobotApp):
    """Joue une animation fournie, puis une à elle, puis une invalide."""
    def manifest(self):
        return AppManifest(app_id="com.test.salut", name="Salut", version="1.0.0", author="T",
                           author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.ANIMATION_PLAY, Permission.ACTUATORS_READ,
                                        Permission.ACTUATORS_WRITE_ARMS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        issues = {}
        if params.get("geste") == "long":
            r = ctx.play_animation("wave")
            return MissionResult(success=r.ok, mission_id="", app_id="", duration_s=0.0,
                                 metrics={"status": r.status, "code": r.code})
        issues["wave"] = ctx.play_animation("wave").status
        mien = {"format": "axis-animation", "format_version": 1, "name": "mien",
                "version": "1.0.0", "robots": ["G1"], "keyframes": [
                    {"t": 0.0, "positions": {"left_shoulder_pitch_joint": 0.0}},
                    {"t": 1.0, "positions": {"left_shoulder_pitch_joint": -0.4}},
                    {"t": 2.0, "positions": {"left_shoulder_pitch_joint": 0.0}}]}
        issues["mien"] = ctx.play_animation(mien).status
        trop_vite = dict(mien, keyframes=[{"t": 0.0, "positions": {"left_shoulder_pitch_joint": 0.0}},
                                          {"t": 0.2, "positions": {"left_shoulder_pitch_joint": -2.0}}])
        try:
            ctx.play_animation(trop_vite)
            issues["trop_vite"] = "accepté"
        except ValueError as exc:
            issues["trop_vite"] = type(exc).__name__
        issues["arret"] = ctx.stop_animation().status
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             metrics=issues)


class AppSalutSansDroit(RobotApp):
    def manifest(self):
        return AppManifest(app_id="com.test.salut_sans", name="Sans", version="1.0.0", author="T",
                           author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.ACTUATORS_READ, Permission.ACTUATORS_WRITE_ARMS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        ctx.play_animation("wave")
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


section("5. Depuis le processus de l'app")
with AppTestHarness(AppSalut()) as h:
    res = h.run()
    m = res.metrics if res is not None else {}
    check("wave puis une animation de l'app : PLAYED, PLAYED", m.get("wave") == "PLAYED"
          and m.get("mien") == "PLAYED", f"{m} {getattr(res, 'error_message', None)}")
    check("Animation trop rapide : InvalidArgumentError dans le processus de l'app",
          m.get("trop_vite") == "InvalidArgumentError", str(m))
    check("stop_animation sans animation : IDLE", m.get("arret") == "IDLE", str(m))
    threading.Timer(1.0, lambda: silencieux(h.robot.hal.emergency_stop_all)).start()
    res = h.run({"geste": "long"})
    p = h.robot.hal.get_full_state()[EPAULE].position
    time.sleep(0.3)
    check("Arrêt d'urgence pendant le geste (processus de l'app) : mission interrompue, bras figé",
          res is not None and not res.success and res.interrompue_par_arret_urgence
          and h.robot.hal.get_full_state()[EPAULE].position == p, getattr(res, "error_message", None))

with AppTestHarness(AppSalutSansDroit()) as h:
    res = h.run()
    check("play_animation sans ANIMATION_PLAY, depuis le processus : mission refusée",
          res is not None and not res.success and "ANIMATION_PLAY" in (res.error_message or ""),
          getattr(res, "error_message", None))

section("6. Pont : schémas")
with SimulatedRobot() as robot:
    serveur = HALBridgeServer(contexte(robot))

    def appel(payload):
        return silencieux(serveur.traiter, {"type": "appel", "action": "play_animation",
                                            "payload": payload, "request_id": 1})
    rep = appel({"wait": True, "name": "wave", "animation": base()})
    check("Nom ET animation : INVALID_ARGUMENT", rep["error"].startswith("INVALID_ARGUMENT"), str(rep))
    rep = appel({"wait": True})
    check("Ni nom ni animation : INVALID_ARGUMENT", rep["error"].startswith("INVALID_ARGUMENT"), str(rep))
    rep = appel({"wait": True, "name": "wave", "vitesse": 3})
    check("Champ inconnu : MALFORMED_COMMAND", "MALFORMED" in rep["error"], str(rep))
    rep = appel({"wait": True, "name": "wave", "timeout_s": 1e6})
    check("Délai d'un million de secondes : MALFORMED_COMMAND", "MALFORMED" in rep["error"], str(rep))
    rep = appel({"wait": True, "animation": "/etc/passwd"})
    check("Animation passée comme chemin : INVALID_ARGUMENT (aucun fichier lu)",
          rep["error"].startswith("INVALID_ARGUMENT"), str(rep))

section("7. axis animation validate")
for nom in ("wave", "point"):
    r = subprocess.run([sys.executable, "-m", "axis_os.cli.main", "animation", "validate",
                        str(library_path("G1") / f"{nom}.json")], cwd=RACINE,
                       capture_output=True, text=True, timeout=120)
    check(f"{nom}.json : code 0, « valide »", r.returncode == 0 and "valide" in r.stdout,
          r.stdout + r.stderr)
with tempfile.TemporaryDirectory() as dossier:
    mauvais = Path(dossier) / "trop_vite.json"
    mauvais.write_text(json.dumps(avec_images((0.0, {EPAULE: 0.0}), (0.5, {EPAULE: -2.5}))))
    r = subprocess.run([sys.executable, "-m", "axis_os.cli.main", "animation", "validate",
                        str(mauvais)], cwd=RACINE, capture_output=True, text=True, timeout=120)
    check("Animation trop rapide : code 1, le problème est dit", r.returncode == 1
          and "rad/s" in r.stdout, r.stdout + r.stderr)
Path(secret.name).unlink()

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
