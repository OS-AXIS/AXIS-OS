"""
SDK v1 : constats des revues adversariales, chacun avec son test.

R1 (pick ne demande l'exemption du gardien qu'autour d'un objet vu par la
perception de l'OS) est vérifié dans test/test_sdk_services_haut_niveau.py.

R2 — Un seul écrivain des bras par contexte. play_animation refusait de
jouer pendant un geste de manipulation ou un flux des bras, mais
stream_targets("arms"), pick() et rest_pose() ne vérifiaient pas qu'une
animation ou un flux tenait déjà les bras : deux écrivains se
disputaient alors les mêmes articulations (sauts de 0.57 à 0.62 rad
mesurés).

R3 — Animation : l'accélération n'était vérifiée qu'en moyenne entre deux
images ; l'interpolation en ligne droite inversait la vitesse d'un coup
(162 rad/s² mesurés). Un retard du fil était rattrapé d'un seul pas.

R4 — rest_pose(duration_s=0.5) depuis des bras levés : 6.2 rad/s. La
vitesse est désormais bornée (durée allongée, jamais raccourcie), et la
posture ne contient jamais d'articulation de jambe.

R5 — Flux de marche : un homme mort jusqu'à 1 s faisait rouler le robot
plus longtemps sans consigne que walk() (0.5 s).

R6 — go_to et un flux de marche commandaient la marche en même temps.

R7 — Une action longue (go_to…) continuait après la mort du processus de
l'app (plantage, quota) : 13 s de plus, jusqu'à 3.9 m.

R8 — pick() annulé (arrêt demandé, fin du hook) : la prise allait à son
terme, exemption du gardien active. place() annulé lâchait l'objet en
disant le geste refusé : une pose commencée va désormais à son terme.

R9 — Deux apps à la fois (on_start de l'une pendant la mission de l'autre) :
la seconde détournait la navigation de la première. Et request_stop se
perdait quand un autre hook de l'app commençait entre-temps.

R10 — Le flux de consignes ne s'arrêtait pas sur demande d'arrêt.

R11 — L'exemption de pick() n'était pas liée à la hauteur des mains.

Aussi : poll_events sans permission d'événement ; une consigne de marche
nulle comptait comme mouvement réel pour la facturation.
"""
import contextlib
import io
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.frames import FRAME_WORLD  # noqa: E402
from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext  # noqa: E402
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402
from axis_os.store.sdk.results import AxisRefusal, ObjectSpec, RefusalCode  # noqa: E402
from axis_os.testing import AppTestHarness, SimulatedObject, SimulatedRobot  # noqa: E402

FAIL = 0


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


TOUT = [Permission.ANIMATION_PLAY, Permission.ACTUATORS_READ, Permission.ACTUATORS_WRITE_ARMS,
        Permission.TELEOP_STREAM, Permission.MANIPULATION, Permission.SENSORS_CAMERA]


def contexte(robot):
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog, AppManifest(
        app_id="com.test.revue", name="Revue", version="1.0.0", author="T",
        author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
        permissions=list(TOUT)))
    ctx._set_clock(robot.clock)
    ctx._attach_services(locomotion=robot.locomotion, manipulation=robot.manipulation,
                         perception=robot.perception)
    return ctx


section("R2. Un seul écrivain des bras par contexte")
with SimulatedRobot(objects=[SimulatedObject("red box", (0.30, 0.0, 0.90))]) as robot:
    ctx = contexte(robot)
    silencieux(ctx.look_for, None, FRAME_WORLD)
    spec = ObjectSpec((0.30, 0.0, 0.90), (0.24, 0.20, 0.20))
    r = silencieux(ctx.play_animation, "wave", wait=False)
    check("Animation lancée", r.status == "PLAYING", str(r))
    check("… flux des bras refusé pendant l'animation : BUSY",
          leve(RefusalCode.BUSY, ctx.stream_targets, "arms"))
    r = silencieux(ctx.pick, spec, timeout_s=5.0)
    check("… pick refusé pendant l'animation : BUSY, aucun geste",
          r.code == "BUSY" and robot.manipulation.etat.name == "LIBRE", str(r))
    r = silencieux(ctx.rest_pose)
    check("… rest_pose refusé pendant l'animation : BUSY", r.code == "BUSY", str(r))
    silencieux(ctx.stop_animation)
    flux = silencieux(ctx.stream_targets, "arms")
    r = silencieux(ctx.pick, spec, timeout_s=5.0)
    check("Flux des bras ouvert : pick refusé (BUSY)", r.code == "BUSY", str(r))
    r = silencieux(ctx.play_animation, "wave")
    check("… play_animation refusée (BUSY)", r.code == "BUSY", str(r))
    r = silencieux(ctx.rest_pose)
    check("… rest_pose refusé (BUSY)", r.code == "BUSY", str(r))
    flux.close()
    time.sleep(0.1)
    r = silencieux(ctx.pick, spec, timeout_s=15.0)
    check("Flux fermé : pick accepté (HELD)", r.status == "HELD", str(r))
    check("Objet tenu : flux des bras refusé (BUSY)",
          leve(RefusalCode.BUSY, ctx.stream_targets, "arms"))


def espionner_bras(ctx):
    """Note chaque consigne de position envoyée aux bras : (instant, {articulation: position})."""
    vues = []
    reel = ctx.move_arms

    def move_arms(commands):
        vues.append((time.monotonic(), {j: c.position for j, c in commands.items()}))
        return reel(commands)
    ctx.move_arms = move_arms
    return vues


def vitesses_accelerations(vues, jid):
    pts = [(t, c[jid]) for t, c in vues if jid in c]
    v = [((b[1] - a[1]) / (b[0] - a[0]), (a[0] + b[0]) / 2) for a, b in zip(pts, pts[1:])
         if b[0] > a[0]]
    acc = [abs(y[0] - x[0]) / (y[1] - x[1]) for x, y in zip(v, v[1:]) if y[1] > x[1]]
    return max((abs(x[0]) for x in v), default=0.0), max(acc, default=0.0)


EPAULE = "right_shoulder_pitch_joint"

section("R3. Animation : accélération réelle bornée, pas de rattrapage brutal")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    # Valide en moyenne (±2 rad/s, segments de 1 s), mais l'interpolation en
    # ligne droite inverse la vitesse d'un coup à chaque image (162 rad/s²
    # mesurés avant la correction).
    aller_retour = {"format": "axis-animation", "format_version": 1, "name": "aller_retour",
                    "version": "1.0.0", "robots": ["G1"], "keyframes": [
                        {"t": 0.0, "positions": {EPAULE: 0.0}},
                        {"t": 0.6, "positions": {EPAULE: -1.2}},
                        {"t": 1.2, "positions": {EPAULE: 0.0}},
                        {"t": 1.8, "positions": {EPAULE: -1.2}}]}
    from axis_os.store.sdk.animation import ANIM_MAX_ACC, ANIM_MAX_VEL, AnimationPlayer
    silencieux(ctx.move_to_pose, {EPAULE: 0.0}, 1.0)
    vues = espionner_bras(ctx)
    r = silencieux(ctx.play_animation, aller_retour)
    vmax, amax = vitesses_accelerations(vues, EPAULE)
    check("Animation aller-retour jouée (PLAYED)", r.ok, str(r))
    check(f"… vitesse commandée au plus {ANIM_MAX_VEL} rad/s (marge 15 %)",
          vmax <= ANIM_MAX_VEL * 1.15, f"{vmax:.2f} rad/s")
    check(f"… accélération commandée au plus {ANIM_MAX_ACC} rad/s² (marge 50 %, gigue du fil)",
          amax <= ANIM_MAX_ACC * 1.5, f"{amax:.1f} rad/s²")
    pas = max((abs(b[1][EPAULE] - a[1][EPAULE]) for a, b in zip(vues, vues[1:])), default=0.0)
    check("… aucun pas plus grand que deux pas à vitesse maximale (pas de rattrapage)",
          pas <= ANIM_MAX_VEL * 2 * AnimationPlayer.PERIOD_S + 1e-9, f"{pas:.3f} rad")
    p, v = AnimationPlayer._pas_limite(0.0, 2.0, -1.0, 0.5)
    check("Limiteur, après un retard de 0.5 s : vitesse changée d'au plus a·dt",
          abs(v - 2.0) <= ANIM_MAX_ACC * 0.5 + 1e-9, f"{v}")

section("R4. rest_pose : vitesse bornée quelle que soit la durée demandée")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    silencieux(ctx.move_to_pose, {EPAULE: -2.0, "left_shoulder_pitch_joint": -2.0}, 2.0)
    vues = espionner_bras(ctx)
    t0 = time.monotonic()
    r = silencieux(ctx.rest_pose, duration_s=0.5)
    duree = time.monotonic() - t0
    vmax, _ = vitesses_accelerations(vues, EPAULE)
    check(f"rest_pose(0.5 s) depuis -2 rad : AT_REST, au plus {RobotContext.REST_POSE_MAX_SPEED} "
          f"rad/s (6.2 rad/s avant la correction)", r.ok and vmax <= RobotContext.REST_POSE_MAX_SPEED
          * 1.25 and duree >= 1.8, f"{r} {vmax:.2f} rad/s en {duree:.2f} s")
    from axis_os.store.runtime.hal_bridge import HALBridgeServer
    silencieux(ctx.move_to_pose, {EPAULE: -2.0}, 2.0)
    rep = silencieux(HALBridgeServer(ctx).traiter, {"type": "appel", "action": "rest_pose",
                                                    "payload": {"duration_s": 0.5},
                                                    "request_id": 1}, reste_s=1.5)
    check("Dans le processus de l'app, 1.5 s restantes pour un geste de 2 s : TIMEOUT, rien de "
          "raccourci", rep["data"]["status"] == "TIMEOUT"
          and abs(robot.hal.get_full_state()[EPAULE].position + 2.0) < 0.05, str(rep))

section("R5. Flux de marche : homme mort au plus 500 ms")
with SimulatedRobot() as robot:
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog, AppManifest(
        app_id="com.test.revue_marche", name="Revue", version="1.0.0", author="T",
        author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
        permissions=[Permission.TELEOP_STREAM, Permission.LOCOMOTION_WALK,
                     Permission.ACTUATORS_WRITE_LEGS, Permission.NAVIGATION_GOTO]))
    ctx._set_clock(robot.clock)
    ctx._attach_services(locomotion=robot.locomotion, navigation=robot.navigation)
    check("deadman_ms=800 pour la marche : INVALID_ARGUMENT (au-delà du délai de la locomotion)",
          leve(RefusalCode.INVALID_ARGUMENT, ctx.stream_targets, "walk", deadman_ms=800))
    flux = silencieux(ctx.stream_targets, "walk", deadman_ms=500)
    check("deadman_ms=500 : accepté", flux is not None)

    section("R6. Un seul écrivain de la marche")
    r = silencieux(ctx.go_to, 1.0, 0.0, timeout_s=5.0)
    check("Flux de marche ouvert : go_to refusé (BUSY), rien ne part",
          r.code == "BUSY" and robot.navigation.etat.name == "INACTIF", str(r))
    flux.close()
    import threading
    issue = {}
    # Pas de redirect_stdout dans le fil : il vaut pour tout le processus.
    fil = threading.Thread(target=lambda: issue.setdefault(
        "r", ctx.go_to(2.0, 0.0, timeout_s=3.0)))
    with contextlib.redirect_stdout(io.StringIO()):
        fil.start()
        time.sleep(0.5)
        try:
            ctx.stream_targets("walk")
            refuse = False
        except AxisRefusal as exc:
            refuse = exc.code is RefusalCode.BUSY
        fil.join()
    check("go_to en cours : ouvrir un flux de marche est refusé (BUSY)", refuse)



class AppMeurtEnRoute(RobotApp):
    """Lance un long go_to, et son processus meurt une seconde plus tard."""
    def manifest(self):
        return AppManifest(app_id="com.test.meurt_en_route", name="Meurt", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.NAVIGATION_GOTO, Permission.LOCOMOTION_WALK,
                                        Permission.ACTUATORS_WRITE_LEGS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        import os
        threading.Timer(1.0, lambda: os._exit(3)).start()
        ctx.go_to(4.0, 0.0, timeout_s=60.0)
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


class AppPatiente(RobotApp):
    """Marche à petits pas tant que l'arrêt n'est pas demandé."""
    def manifest(self):
        return AppManifest(app_id="com.test.patiente_revue", name="Patiente", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.MISSION_LIFECYCLE])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): ctx.sleep(1.0)

    def run_mission(self, ctx, params):
        fin = ctx.now() + 6.0
        while ctx.now() < fin:
            if ctx.should_stop():
                return MissionResult(success=False, mission_id="", app_id="", duration_s=0.0,
                                     error_message="arrêtée à la demande")
            ctx.sleep(0.05)
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


section("R7. Le processus de l'app meurt pendant go_to : le robot s'arrête")
with AppTestHarness(AppMeurtEnRoute()) as h:
    t0 = time.monotonic()
    res = h.run()
    duree = time.monotonic() - t0
    x = h.robot.pose[0]
    time.sleep(0.5)
    check("Mission rendue vite, en échec, robot arrêté là où l'app est morte "
          "(13 s et 3.9 m avant la correction)",
          res is not None and not res.success and duree < 4.0 and x < 1.0
          and abs(h.robot.pose[0] - x) < 0.02, f"{getattr(res, 'error_message', None)} "
          f"en {duree:.1f} s, x = {x:.2f}")
    check("… navigation annulée", h.robot.navigation.etat.name == "INACTIF")

section("R8. pick annulé : bras rouverts, exemption levée ; place va à son terme")
with SimulatedRobot(objects=[SimulatedObject("red box", (0.30, 0.0, 0.90))]) as robot:
    ctx = contexte(robot)
    silencieux(ctx.look_for, None, FRAME_WORLD)
    threading.Timer(0.3, lambda: ctx._demander_arret("human")).start()
    r = silencieux(ctx.pick, ObjectSpec((0.30, 0.0, 0.90), (0.24, 0.20, 0.20)), timeout_s=15.0)
    check("pick annulé : REFUSED/CANCELLED, exemption du gardien levée aussitôt",
          r.code == "CANCELLED" and robot.guard._exemption is None, str(r))
    time.sleep(2.5)
    check("… les bras se sont rouverts : l'objet n'est pas pris",
          robot.manipulation.etat.name in ("LIBRE", "LACHER") and robot.held is None,
          robot.manipulation.etat.name)

with SimulatedRobot(objects=[SimulatedObject("red box", (0.30, 0.0, 0.90))]) as robot:
    ctx = contexte(robot)
    silencieux(ctx.look_for, None, FRAME_WORLD)
    r = silencieux(ctx.pick, ObjectSpec((0.30, 0.0, 0.90), (0.24, 0.20, 0.20)), timeout_s=15.0)
    threading.Timer(0.2, lambda: ctx._demander_arret("human")).start()
    r2 = silencieux(ctx.place, None, timeout_s=15.0)
    check("place pendant une demande d'arrêt : la pose va à son terme (PLACED), rien n'est "
          "lâché en disant le geste refusé", r.ok and r2.ok and r2.status == "PLACED"
          and robot.manipulation.etat.name == "LIBRE", f"{r} {r2}")

section("R9. Deux apps à la fois : une seule conduit le robot")
with SimulatedRobot() as robot:
    def ctx_marche(app_id):
        c = RobotContext.for_testing(robot.hal, robot.watchdog, AppManifest(
            app_id=app_id, name="Marche", version="1.0.0", author="T",
            author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
            permissions=[Permission.NAVIGATION_GOTO, Permission.LOCOMOTION_WALK,
                         Permission.ACTUATORS_WRITE_LEGS]))
        c._set_clock(robot.clock)
        c._attach_services(locomotion=robot.locomotion, navigation=robot.navigation)
        return c
    a, b = ctx_marche("com.test.app_a"), ctx_marche("com.test.app_b")
    issue = {}
    fil = threading.Thread(target=lambda: issue.setdefault("a", a.go_to(1.5, 0.0, timeout_s=20)))
    with contextlib.redirect_stdout(io.StringIO()):
        fil.start()
        time.sleep(0.5)
        rb = b.go_to(0.3, -1.5, timeout_s=5.0)
        fil.join()
    x, y, _ = robot.pose
    check("L'app B veut go_to pendant celui de A : REFUSED/BUSY",
          rb.code == "BUSY", str(rb))
    check("… A arrive bien à SA destination", issue["a"].ok and abs(x - 1.5) < 0.2 and abs(y) < 0.2,
          f"{issue['a']} {robot.pose}")
    rb = silencieux(b.go_to, 1.5, 0.5, timeout_s=20.0)
    check("Une fois A arrivée, B peut conduire le robot", rb.ok, str(rb))

with AppTestHarness(AppPatiente()) as h:
    vu = {}
    threading.Timer(0.8, lambda: vu.setdefault(
        "stop_app", silencieux(h.runtime.stop_app, h.app_id))).start()
    threading.Timer(1.2, lambda: vu.setdefault(
        "request", h.runtime.request_stop(h.app_id))).start()
    t0 = time.monotonic()
    res = h.run()
    duree = time.monotonic() - t0
    check("request_stop pendant un on_stop concurrent : la mission l'entend quand même et "
          "s'arrête (avant : perdu, 6 s de mission)", vu.get("request") is True and res is not None
          and res.error_message == "arrêtée à la demande" and duree < 4.0,
          f"{vu} {getattr(res, 'error_message', None)} en {duree:.1f} s")

section("R10. Le flux s'arrête sur demande d'arrêt")
with SimulatedRobot() as robot:
    c = RobotContext.for_testing(robot.hal, robot.watchdog, AppManifest(
        app_id="com.test.flux_stop", name="Flux", version="1.0.0", author="T",
        author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
        permissions=[Permission.TELEOP_STREAM, Permission.LOCOMOTION_WALK,
                     Permission.ACTUATORS_WRITE_LEGS]))
    c._set_clock(robot.clock)
    c._attach_services(locomotion=robot.locomotion)
    flux = silencieux(c.stream_targets, "walk", max_speed=0.5)
    for _ in range(25):
        flux.push((0.2, 0.0, 0.0), deadman=True)
        time.sleep(0.02)
    c._demander_arret("human")
    acceptees = 0
    for _ in range(25):
        acceptees += flux.push((0.2, 0.0, 0.0), deadman=True).accepted
        time.sleep(0.02)
    x = robot.pose[0]
    time.sleep(0.4)
    check("Après la demande d'arrêt : plus aucune consigne acceptée, arrêt sûr, base immobile",
          acceptees == 0 and flux.metrics().state == "STOPPED" and abs(robot.pose[0] - x) < 0.01,
          f"{acceptees} {flux.metrics().state}")
    r = flux.resume()
    check("… resume() refusé tant que l'arrêt est demandé (CANCELLED)", r.code == "CANCELLED", str(r))
    flux.close()

section("R11. L'exemption de pick est liée à la hauteur des mains")
with SimulatedRobot(objects=[SimulatedObject("low box", (0.30, 0.0, 0.30))]) as robot:
    ctx = contexte(robot)
    silencieux(ctx.look_for, None, FRAME_WORLD)
    r = silencieux(ctx.pick, ObjectSpec((0.30, 0.0, 0.30), (0.24, 0.20, 0.20)), timeout_s=5.0)
    check("Objet vu à 30 cm du sol, mains à 90 cm : REFUSED/OUT_OF_REACH, aucune exemption",
          r.code == "OUT_OF_REACH" and robot.guard._exemption is None
          and robot.guard.journal_exemptions == [], str(r))

section("Pont : poll_events et facturation")
with SimulatedRobot() as robot:
    from axis_os.store.runtime.hal_bridge import HALBridgeServer
    nu = RobotContext.for_testing(robot.hal, robot.watchdog, AppManifest(
        app_id="com.test.nu", name="Nu", version="1.0.0", author="T", author_email="t@t.com",
        description="d", category=AppCategory.UTILITIES, permissions=[]))
    rep = silencieux(HALBridgeServer(nu).traiter, {"type": "appel", "action": "poll_events",
                                                   "payload": {}, "request_id": 1})
    check("poll_events sans permission d'événement : PERMISSION_DENIED",
          rep["error"].startswith("PERMISSION_DENIED"), str(rep))
    c = RobotContext.for_testing(robot.hal, robot.watchdog, AppManifest(
        app_id="com.test.facture", name="Fact", version="1.0.0", author="T",
        author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
        permissions=[Permission.TELEOP_STREAM, Permission.LOCOMOTION_WALK,
                     Permission.ACTUATORS_WRITE_LEGS]))
    c._set_clock(robot.clock)
    c._attach_services(locomotion=robot.locomotion)
    serveur = HALBridgeServer(c)
    rep = silencieux(serveur.traiter, {"type": "appel", "action": "stream_open", "payload": {
        "kind": "walk", "deadman_ms": 200, "max_rate_hz": 50, "max_age_ms": 150,
        "max_speed": 0.3}, "request_id": 2})
    sid = rep["data"]["stream_id"]
    rep = silencieux(serveur.traiter, {"type": "appel", "action": "stream_push", "payload": {
        "stream_id": sid, "deadman": True, "t": c.now(), "velocity": [0, 0, 0]},
        "request_id": 3})
    check("Consigne de marche nulle acceptée, mais pas comptée comme mouvement réel",
          rep["data"]["accepted"] and not serveur.had_any_motor_success, str(rep))
    c._flux_close(sid)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
