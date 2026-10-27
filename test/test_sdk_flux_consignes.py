"""
SDK, partie 4 : flux de consignes de téléopération locale (TargetStream).

Une manette factice pousse des consignes ; on vérifie, en direct puis
depuis le processus de l'app :
  1. cas normal : les bras suivent, sans dépasser la vitesse propre au
     flux ; la marche avance ; mesures (latence, gigue, rejets) ;
  2. l'OS jette les consignes trop vieilles, venues du futur, trop
     fréquentes, hors bornes (y compris une articulation de jambe) ;
  3. homme mort relâché, perte du flux, retard excessif : arrêt sûr ; la
     marche ralentit puis s'arrête (jamais de coupure nette) ; plus rien
     ne passe avant resume() ;
  4. permission manquante (TELEOP_STREAM, et les droits des commandes
     remplacées), arguments invalides, arrêt d'urgence (avant et pendant),
     capacité absente, flux déjà ouvert, fin du hook.
"""
import contextlib
import io
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.store.sdk.app_base import MissionResult, RobotApp, RobotContext  # noqa: E402
from axis_os.store.sdk.manifest import (  # noqa: E402
    PERMISSION_LEVELS, AppCategory, AppManifest, Permission, ProtectionLevel,
)
from axis_os.store.sdk.results import AxisRefusal, RefusalCode  # noqa: E402
from axis_os.store.sdk.stream import TargetStream  # noqa: E402
from axis_os.testing import AppTestHarness, SimulatedRobot  # noqa: E402

FAIL = 0
EPAULE = "left_shoulder_pitch_joint"


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
    return AppManifest(app_id="com.test.flux", name="Flux", version="1.0.0", author="T",
                       author_email="t@t.com", description="d",
                       category=AppCategory.UTILITIES, permissions=list(perms))


BRAS = [Permission.TELEOP_STREAM, Permission.ACTUATORS_READ, Permission.ACTUATORS_WRITE_ARMS]
MARCHE = [Permission.TELEOP_STREAM, Permission.LOCOMOTION_WALK, Permission.ACTUATORS_WRITE_LEGS]


def contexte(robot, perms=None, locomotion=True):
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog,
                                   manifeste(BRAS + MARCHE if perms is None else perms))
    ctx._set_clock(robot.clock)
    if locomotion:
        ctx._attach_services(locomotion=robot.locomotion)
    return ctx


def position(robot, jid=EPAULE):
    return robot.hal.get_full_state()[jid].position


class Manette:
    """Manette factice : rend, à chaque lecture, la consigne d'un scénario
    (liste de (durée en s, cible, homme mort))."""
    def __init__(self, scenario):
        self.scenario = list(scenario)
        self.t0 = time.monotonic()

    def lire(self):
        t = time.monotonic() - self.t0
        for duree, cible, homme_mort in self.scenario:
            if t < duree:
                return cible, homme_mort
            t -= duree
        return None


def piloter(flux, manette, periode=0.02):
    acks = []
    while True:
        lu = manette.lire()
        if lu is None:
            return acks
        acks.append(flux.push(lu[0], deadman=lu[1]))
        time.sleep(periode)


section("0. Permission déclarée")
check("TELEOP_STREAM est DANGEROUS (le flux fait bouger le robot)",
      PERMISSION_LEVELS[Permission.TELEOP_STREAM] is ProtectionLevel.DANGEROUS)

section("1. Bras : cas normal, vitesse propre au flux, mesures")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    flux = silencieux(ctx.stream_targets, "arms", max_speed=0.5, deadman_ms=200)
    check("Un TargetStream, limité aux bras", isinstance(flux, TargetStream)
          and EPAULE in flux.joints and not any("knee" in j or "hip" in j for j in flux.joints),
          str(flux.joints))
    p0 = position(robot)
    t0 = time.monotonic()
    echantillons = []
    for _ in range(50):
        ack = flux.push({EPAULE: p0 + 1.0}, deadman=True)
        echantillons.append((time.monotonic(), position(robot)))
        time.sleep(0.02)
    duree = time.monotonic() - t0
    deplacement = position(robot) - p0
    check("Les consignes sont acceptées", ack.accepted and ack.state == "ACTIVE", str(ack))
    check("Le bras avance vers la cible", deplacement > 0.2, f"{deplacement:.3f} rad")
    vitesse = max((b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(echantillons, echantillons[10:]))
    check("… sans dépasser 0.5 rad/s, la vitesse du flux (marge 20 %)",
          vitesse <= 0.5 * 1.2 and deplacement <= 0.5 * duree * 1.2,
          f"{vitesse:.3f} rad/s, {deplacement:.3f} rad en {duree:.2f} s")
    m = flux.metrics()
    check("Mesures : reçues, acceptées, latence, gigue",
          m.received == 50 and m.accepted == 50 and m.latency_ms_mean >= 0
          and m.jitter_ms >= 0 and m.jitter_ms < 50, str(m))

    section("2. Consignes jetées par l'OS")
    maintenant = ctx.now()
    for label, cible, t, code in [
            ("vieille de 1 s", {EPAULE: p0}, maintenant - 1.0, RefusalCode.STALE_TARGET),
            ("venue du futur", {EPAULE: p0}, maintenant + 5.0, RefusalCode.STALE_TARGET),
            ("épaule à 10 rad (hors butées)", {EPAULE: 10.0}, None, RefusalCode.OUT_OF_BOUNDS),
            ("genou (une jambe)", {"left_knee_joint": 0.3}, None, RefusalCode.OUT_OF_BOUNDS),
            ("articulation inconnue", {"tentacule": 0.0}, None, RefusalCode.OUT_OF_BOUNDS)]:
        ack = flux.push(cible, deadman=True, t=t)
        check(f"Consigne {label} : rejetée ({code.value})", not ack.accepted
              and ack.code == code.value, str(ack))
    m = flux.metrics()
    check("… et comptée par motif", m.rejected.get("STALE_TARGET") == 2
          and m.rejected.get("OUT_OF_BOUNDS") == 3, str(m.rejected))
    flux.close()

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    flux = silencieux(ctx.stream_targets, "arms", max_rate_hz=20)
    p0 = position(robot)
    acks = [flux.push({EPAULE: p0}, deadman=True) for _ in range(60)]
    ok = sum(a.accepted for a in acks)
    check("Rafale de 60 consignes, flux à 20 Hz : 20 acceptées, le reste RATE_LIMITED",
          ok == 20 and all(a.code == "RATE_LIMITED" for a in acks if not a.accepted), f"{ok}")
    flux.close()

section("3. Homme mort, perte du flux, retard : arrêt sûr, reprise explicite")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    flux = silencieux(ctx.stream_targets, "arms", max_speed=0.5, deadman_ms=200)
    p0 = position(robot)
    piloter(flux, Manette([(0.5, {EPAULE: p0 + 1.0}, True)]))
    ack = flux.push({EPAULE: p0 + 1.0}, deadman=False)
    p1 = position(robot)
    time.sleep(0.5)
    check("Homme mort relâché : arrêt sûr (STOPPED), le bras s'arrête sur place",
          ack.state == "STOPPED" and abs(position(robot) - p1) < 0.03,
          f"{ack} {p1:.3f} -> {position(robot):.3f}")
    ack = flux.push({EPAULE: p0 + 1.0}, deadman=True)
    check("… plus rien ne passe avant resume() (STREAM_STOPPED)", not ack.accepted
          and ack.code == "STREAM_STOPPED", str(ack))
    r = flux.resume()
    check("resume() : RESUMED", r.ok and r.status == "RESUMED", str(r))
    ack = flux.push({EPAULE: p0 + 1.0}, deadman=True)
    check("… puis la consigne passe à nouveau", ack.accepted, str(ack))
    piloter(flux, Manette([(0.3, {EPAULE: p0 + 1.0}, True)]))
    p2 = position(robot)
    time.sleep(0.6)
    m = flux.metrics()
    p3 = position(robot)
    time.sleep(0.3)
    # Pendant les 200 ms de l'homme mort, le bras peut encore avancer à la
    # vitesse du flux (0.5 rad/s), soit 0.1 rad ; ensuite, plus rien.
    check("Perte du flux (plus de consigne depuis 200 ms) : arrêt sûr, bras immobile",
          m.state == "STOPPED" and "plus de consigne" in m.stop_reason
          and abs(p3 - p2) < 0.1 + 0.05 and abs(position(robot) - p3) < 0.01,
          f"{m.state} {m.stop_reason} {p2:.3f} {p3:.3f} {position(robot):.3f}")
    flux.resume()
    retard = [flux.push({EPAULE: p0}, deadman=True, t=ctx.now() - 0.4) for _ in range(10)]
    check("Retard excessif (consignes de 400 ms) : toutes jetées (STALE_TARGET)",
          not any(a.accepted for a in retard), str(retard[-1]))
    check("Deux arrêts sûrs comptés", flux.metrics().safe_stops == 2, str(flux.metrics()))
    flux.close()

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    appels = []
    reel = robot.locomotion.set_velocity

    def espion(vx=0.0, vy=0.0, wz=0.0):
        appels.append((time.monotonic(), vx))
        return reel(vx, vy, wz)
    robot.locomotion.set_velocity = espion
    flux = silencieux(ctx.stream_targets, "walk", max_speed=0.5, deadman_ms=200)
    check("Flux de marche : vitesse maximale du flux = 0.5 × celle du robot",
          abs(flux.limits["max_vx_mps"] - 0.2) < 1e-9, str(flux.limits))
    ack = flux.push((0.25, 0.0, 0.0), deadman=True)
    check("0.25 m/s, au-delà de la vitesse du flux : rejetée (OUT_OF_BOUNDS), pas rognée",
          not ack.accepted and ack.code == "OUT_OF_BOUNDS", str(ack))
    piloter(flux, Manette([(1.5, (0.2, 0.0, 0.0), True)]))
    x1 = robot.pose[0]
    check("Marche : la base avance", x1 > 0.15, str(robot.pose))
    t_lache = time.monotonic()
    ack = flux.push((0.2, 0.0, 0.0), deadman=False)
    time.sleep(0.8)
    apres = [vx for t, vx in appels if t > t_lache]
    check("Homme mort relâché : la marche RALENTIT (plusieurs consignes décroissantes)",
          len([v for v in apres if 0 < v < 0.2]) >= 3
          and all(a >= b for a, b in zip(apres, apres[1:])), str(apres[:12]))
    check("… puis s'arrête (STOPPED), base immobile", flux.metrics().state == "STOPPED"
          and robot.pose[0] - x1 < 0.1, f"{flux.metrics().state} {robot.pose[0] - x1:.3f} m")
    x2 = robot.pose[0]
    time.sleep(0.5)
    check("… et le reste", abs(robot.pose[0] - x2) < 0.01, str(robot.pose))
    flux.resume()
    robot.add_person(robot.pose[0] + 0.6, 0.0)
    xp = robot.pose[0]
    piloter(flux, Manette([(0.8, (0.2, 0.0, 0.0), True)]))
    check("Personne devant : le gardien de proximité arrête quand même la marche du flux",
          robot.pose[0] - xp < 0.05, f"{robot.pose[0] - xp:.3f} m")
    flux.close()

section("4. Permission, arguments, arrêt d'urgence, capacité, flux déjà ouvert")
with SimulatedRobot() as robot:
    for label, perms, kind in [
            ("Sans TELEOP_STREAM", [Permission.ACTUATORS_READ, Permission.ACTUATORS_WRITE_ARMS],
             "arms"),
            ("Bras sans ACTUATORS_WRITE_ARMS", [Permission.TELEOP_STREAM,
                                                Permission.ACTUATORS_READ], "arms"),
            ("Marche sans LOCOMOTION_WALK", [Permission.TELEOP_STREAM,
                                             Permission.ACTUATORS_WRITE_LEGS], "walk")]:
        check(f"{label} : PERMISSION_DENIED",
              leve(RefusalCode.PERMISSION_DENIED, contexte(robot, perms).stream_targets, kind))
    ctx = contexte(robot)
    for label, kind, kw in [("kind « legs »", "legs", {}), ("homme mort de 10 ms", "arms",
                                                            {"deadman_ms": 10}),
                            ("homme mort de 5 s", "arms", {"deadman_ms": 5000}),
                            ("homme mort NaN", "arms", {"deadman_ms": float("nan")}),
                            ("1000 Hz", "arms", {"max_rate_hz": 1000}),
                            ("bras à 5 rad/s", "arms", {"max_speed": 5.0}),
                            ("marche à 90 %", "walk", {"max_speed": 0.9}),
                            ("vitesse nulle", "walk", {"max_speed": 0.0}),
                            ("âge maximal texte", "arms", {"max_age_ms": "100"})]:
        check(f"{label} : INVALID_ARGUMENT",
              leve(RefusalCode.INVALID_ARGUMENT, ctx.stream_targets, kind, **kw))
    flux = silencieux(ctx.stream_targets, "arms")
    check("Second flux ouvert par le même contexte : BUSY",
          leve(RefusalCode.BUSY, ctx.stream_targets, "walk"))
    for label, cible, kw in [("cible NaN", {EPAULE: float("nan")}, {"deadman": True}),
                             ("cible vide", {}, {"deadman": True}),
                             ("cible liste", [0.1], {"deadman": True}),
                             ("homme mort texte", {EPAULE: 0.0}, {"deadman": "oui"}),
                             ("t infini", {EPAULE: 0.0}, {"deadman": True, "t": float("inf")})]:
        check(f"push {label} : INVALID_ARGUMENT",
              leve(RefusalCode.INVALID_ARGUMENT, flux.push, cible, **kw))
    flux.close()
    marche = silencieux(ctx.stream_targets, "walk")
    for label, cible in [("deux composantes", (0.1, 0.0)), ("vitesse NaN", (float("nan"), 0, 0))]:
        check(f"push marche {label} : INVALID_ARGUMENT",
              leve(RefusalCode.INVALID_ARGUMENT, marche.push, cible, deadman=True))
    marche.close()
    check("Après close(), un nouveau flux s'ouvre", silencieux(ctx.stream_targets, "arms") is not None)
    ctx._fermer()
    time.sleep(0.2)
    check("Fin du hook : le flux est fermé (CLOSED)", ctx._flux.state == "CLOSED", ctx._flux.state)

    sans_loco = contexte(robot, locomotion=False)
    check("Flux de marche sans service de locomotion : CAPABILITY_MISSING",
          leve(RefusalCode.CAPABILITY_MISSING, sans_loco.stream_targets, "walk"))

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    flux = silencieux(ctx.stream_targets, "walk", max_speed=0.5)
    piloter(flux, Manette([(0.6, (0.2, 0.0, 0.0), True)]))
    silencieux(robot.hal.emergency_stop_all)
    time.sleep(0.2)
    x = robot.pose[0]
    check("Arrêt d'urgence pendant le flux : push lève EstopActiveError (ESTOP_ACTIVE)",
          leve(RefusalCode.ESTOP_ACTIVE, flux.push, (0.2, 0.0, 0.0), deadman=True))
    time.sleep(0.3)
    check("… le flux est arrêté, la base immobile", flux.metrics().state == "STOPPED"
          and robot.pose[0] == x, f"{flux.metrics().state} {robot.pose}")
    r = flux.resume()
    check("… resume() refusé tant que l'arrêt d'urgence dure (ESTOP_ACTIVE)",
          not r.ok and r.code == "ESTOP_ACTIVE", str(r))
    flux.close()
    check("Ouvrir un flux sous arrêt d'urgence : ESTOP_ACTIVE",
          leve(RefusalCode.ESTOP_ACTIVE, ctx.stream_targets, "arms"))


# ─── Depuis le processus de l'app (pont JSON) ────────────────────────────

class AppTeleop(RobotApp):
    """Téléopération locale de la marche, pilotée par une manette factice :
    1 s en avant homme mort tenu, puis homme mort relâché."""
    def manifest(self):
        return AppManifest(app_id="com.test.teleop_pont", name="Teleop", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.TELEOP_STREAM, Permission.LOCOMOTION_WALK,
                                        Permission.ACTUATORS_WRITE_LEGS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        scenario = [(params.get("avance_s", 1.0), (0.15, 0.0, 0.0), True),
                    (0.3, (0.15, 0.0, 0.0), False)]
        issues = {}
        try:
            ctx.stream_targets("walk", deadman_ms=float("nan"))
        except ValueError as exc:
            issues["invalide"] = type(exc).__name__
        with ctx.stream_targets("walk", deadman_ms=200, max_speed=0.5) as flux:
            t0 = time.monotonic()
            dernier = None
            while True:
                t = time.monotonic() - t0
                lu = None
                for duree, cible, homme_mort in scenario:
                    if t < duree:
                        lu = (cible, homme_mort)
                        break
                    t -= duree
                if lu is None:
                    break
                dernier = flux.push(lu[0], deadman=lu[1])
                time.sleep(0.02)
            m = flux.metrics()
        issues.update(state=dernier.state, accepted=m.accepted, safe_stops=m.safe_stops,
                      latency=m.latency_ms_mean, jitter=m.jitter_ms,
                      stale=m.rejected.get("STALE_TARGET", 0))
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0,
                             metrics=issues)


class AppTeleopSansDroit(RobotApp):
    """Ouvre un flux sans avoir déclaré TELEOP_STREAM : doit être refusée."""
    def manifest(self):
        return AppManifest(app_id="com.test.teleop_sans", name="Sans", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.LOCOMOTION_WALK, Permission.ACTUATORS_WRITE_LEGS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        flux = ctx.stream_targets("walk")
        flux.push((0.1, 0.0, 0.0), deadman=True)
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


section("5. Depuis le processus de l'app")
with AppTestHarness(AppTeleop()) as h:
    res = h.run()
    m = res.metrics if res is not None else {}
    check("Téléopération dans le processus de l'app : la base avance d'environ 0.15 m",
          res is not None and res.success and 0.08 < h.robot.pose[0] < 0.3,
          f"{getattr(res, 'error_message', None)} {h.robot.pose}")
    check("Homme mort relâché par la manette : arrêt sûr", m.get("state") in ("STOPPING", "STOPPED")
          and m.get("safe_stops") == 1, str(m))
    check("Mesures traversent le pont : consignes acceptées, latence et gigue faibles",
          m.get("accepted", 0) > 30 and m.get("latency", 1e9) < 50 and m.get("jitter", 1e9) < 50
          and m.get("stale") == 0, str(m))
    check("Argument invalide refusé dans le processus (InvalidArgumentError)",
          m.get("invalide") == "InvalidArgumentError", str(m))
    x = h.robot.pose[0]
    time.sleep(0.5)
    check("… la base reste arrêtée après la mission", abs(h.robot.pose[0] - x) < 0.01,
          str(h.robot.pose))

with AppTestHarness(AppTeleop()) as h:
    threading.Timer(0.6, lambda: silencieux(h.robot.hal.emergency_stop_all)).start()
    res = h.run({"avance_s": 5.0})
    x = h.robot.pose[0]
    time.sleep(0.4)
    check("Arrêt d'urgence pendant la téléopération : mission interrompue, base immobile",
          res is not None and not res.success and res.interrompue_par_arret_urgence
          and h.robot.pose[0] == x, f"{getattr(res, 'error_message', None)} {h.robot.pose}")

with AppTestHarness(AppTeleopSansDroit()) as h:
    res = h.run()
    check("Flux sans TELEOP_STREAM, depuis le processus : mission refusée, rien n'a bougé",
          res is not None and not res.success and "TELEOP_STREAM" in (res.error_message or "")
          and h.robot.pose[0] == 0.0, getattr(res, "error_message", None))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
