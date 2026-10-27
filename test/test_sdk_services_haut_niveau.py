"""
SDK, partie 1 : go_to(), pick(), place(), rest_pose().

Sur le robot simulé léger (axis_os.testing), avec les vrais services de
navigation, de locomotion et de manipulation, et le vrai gardien de
proximité. Vérifié ici, en direct (RobotContext) puis depuis le
processus de l'app (pont JSON) :
  1. cas normal : go_to arrive et s'oriente ; pick tient l'objet (décrit
     ou vu par look_for) ; place le pose ; rest_pose ramène les bras ;
  2. issues claires : chemin bloqué, délai dépassé, personne devant (le
     gardien arrête le robot), objet hors d'atteinte, prise ratée (bras
     rouverts), plateau trop haut ;
  3. permission manquante : PermissionError (PERMISSION_DENIED) ;
  4. arguments invalides (NaN, infini, hors bornes, mauvais type,
     détection dans le repère du robot, taille absente) :
     InvalidArgumentError, sans le moindre mouvement ;
  5. arrêt d'urgence : refus avant de partir ; pendant le trajet, arrêt
     et issue REFUSED/ESTOP_ACTIVE ;
  6. capacité absente : pas de navigation, pas de manipulation, pas de
     posture de repos -> CAPABILITY_MISSING ;
  7. pont : schémas stricts, délai borné par le temps qui reste au hook,
     action longue arrêtée à la fin du hook.
"""
import contextlib
import io
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.frames import FRAME_WORLD  # noqa: E402
from axis_os.services.manipulation.service import Etat  # noqa: E402
from axis_os.services.navigation.service import Carte, EtatNavigation, Rectangle  # noqa: E402
from axis_os.store.runtime.hal_bridge import HALBridgeServer  # noqa: E402
from axis_os.store.sdk.app_base import (  # noqa: E402
    EstopActiveError, MissionResult, PermissionError as SdkPermissionError, RobotApp,
    RobotContext,
)
from axis_os.store.sdk.manifest import (  # noqa: E402
    PERMISSION_LEVELS, AppCategory, AppManifest, Permission, ProtectionLevel,
)
from axis_os.store.sdk.results import (  # noqa: E402
    ActionResult, AxisRefusal, InvalidArgumentError, ObjectSpec, RefusalCode, object_spec,
)
from axis_os.testing import AppTestHarness, SimulatedObject, SimulatedRobot  # noqa: E402
from axis_os.testing.robot_simule import _HALSimulee  # noqa: E402

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
    """Vrai si l'appel lève un AxisRefusal de ce code."""
    try:
        silencieux(f, *a, **kw)
        return False
    except AxisRefusal as exc:
        return exc.code is code


TOUT = [Permission.NAVIGATION_GOTO, Permission.MANIPULATION, Permission.ACTUATORS_READ,
        Permission.ACTUATORS_WRITE_ARMS, Permission.SENSORS_CAMERA]


def manifeste(perms, app_id="com.test.haut_niveau"):
    return AppManifest(app_id=app_id, name="Haut niveau", version="1.0.0", author="T",
                       author_email="t@t.com", description="d",
                       category=AppCategory.UTILITIES, permissions=list(perms))


def contexte(robot, perms=None, **services):
    """RobotContext en direct, branché sur les services du robot simulé.
    (perms=None plutôt que TOUT : le processus de l'app recharge ce module
    sans ses affectations calculées, voir axis_os/store/runtime/hote_app.py.)"""
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog,
                                   manifeste(TOUT if perms is None else perms))
    ctx._set_clock(robot.clock)
    reglages = dict(locomotion=robot.locomotion, navigation=robot.navigation,
                    manipulation=robot.manipulation, perception=robot.perception)
    reglages.update(services)
    ctx._attach_services(**reglages)
    return ctx


def boite_devant(robot, label="red box"):
    """Une boîte de 24 cm entre les mains du robot (30 cm devant lui, à
    hauteur de port)."""
    o = SimulatedObject(label, (robot.x + 0.30, robot.y, 0.90))
    robot.objects.append(o)
    return o


section("0. Permissions déclarées")
check("NAVIGATION_GOTO et MANIPULATION sont DANGEROUS (elles font bouger le robot)",
      PERMISSION_LEVELS[Permission.NAVIGATION_GOTO] is ProtectionLevel.DANGEROUS
      and PERMISSION_LEVELS[Permission.MANIPULATION] is ProtectionLevel.DANGEROUS)

section("1. go_to : cas normal et issues")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    r = silencieux(ctx.go_to, 1.0, 0.5, math.pi / 2, timeout_s=30.0)
    x, y, cap = robot.pose
    check("Arrivé : status ARRIVED, ok, code None", isinstance(r, ActionResult) and r.ok
          and r.status == "ARRIVED" and r.code is None, str(r))
    check("… à moins de 15 cm du but, orienté vers +y (à 0.2 rad près)",
          math.hypot(x - 1.0, y - 0.5) < 0.15 and abs(cap - math.pi / 2) < 0.2,
          f"{robot.pose}")
    check("Aucune erreur dans la boucle des services", robot.errors == [], str(robot.errors))

    # Délai trop court : TIMEOUT, et le robot est arrêté.
    r = silencieux(ctx.go_to, -3.0, 0.5, timeout_s=0.5)
    time.sleep(0.3)
    p1 = robot.pose
    time.sleep(0.6)
    check("Délai dépassé : TIMEOUT (code TIMEOUT)", r.status == "TIMEOUT"
          and r.code == RefusalCode.TIMEOUT.value, str(r))
    check("… navigation annulée, robot immobile", robot.navigation.etat is EtatNavigation.INACTIF
          and math.dist(p1[:2], robot.pose[:2]) < 0.02, f"{p1} -> {robot.pose}")

carte = Carte(-5.0, -5.0, 5.0, 5.0, obstacles=[Rectangle(1.5, -0.5, 2.5, 0.5)])
with SimulatedRobot(carte=carte) as robot:
    ctx = contexte(robot)
    r = silencieux(ctx.go_to, 2.0, 0.0, timeout_s=10.0)
    check("But dans un obstacle de la carte : BLOCKED (code BLOCKED)",
          r.status == "BLOCKED" and r.code == RefusalCode.BLOCKED.value, str(r))
    r = silencieux(ctx.go_to, 9.0, 0.0, timeout_s=10.0)
    check("But hors de la carte : BLOCKED", r.status == "BLOCKED", str(r))

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    robot.add_person(0.7, 0.0)
    r = silencieux(ctx.go_to, 2.0, 0.0, timeout_s=1.5)
    check("Personne sur le chemin : le gardien tient le robot, TIMEOUT/PROXIMITY_GUARD",
          r.status == "TIMEOUT" and r.code == RefusalCode.PROXIMITY_GUARD.value, str(r))
    check("… et le robot ne s'est pas approché de la personne", robot.pose[0] < 0.2,
          str(robot.pose))

section("2. go_to : arguments invalides, permission, arrêt d'urgence, capacité absente")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    for label, args, kw in [
            ("x NaN", (float("nan"), 0.0), {"timeout_s": 5.0}),
            ("y infini", (0.0, float("inf")), {"timeout_s": 5.0}),
            ("x hors bornes", (1e9, 0.0), {"timeout_s": 5.0}),
            ("x texte", ("1", 0.0), {"timeout_s": 5.0}),
            ("x booléen", (True, 0.0), {"timeout_s": 5.0}),
            ("theta hors bornes", (0.5, 0.0, 10.0), {"timeout_s": 5.0}),
            ("timeout nul", (0.5, 0.0), {"timeout_s": 0.0}),
            ("timeout négatif", (0.5, 0.0), {"timeout_s": -1.0}),
            ("timeout au-delà de 600 s", (0.5, 0.0), {"timeout_s": 601.0}),
            ("timeout NaN", (0.5, 0.0), {"timeout_s": float("nan")})]:
        check(f"{label} : INVALID_ARGUMENT", leve(RefusalCode.INVALID_ARGUMENT, ctx.go_to,
                                                  *args, **kw))
    try:
        ctx.go_to(0.5, 0.0)
        manque = False
    except TypeError:
        manque = True
    check("timeout_s obligatoire (pas de trajet sans limite de durée)", manque)
    check("… aucun départ après ces refus", robot.navigation.etat is EtatNavigation.INACTIF
          and robot.pose[:2] == (0.0, 0.0), str(robot.pose))

    sans = contexte(robot, perms=[Permission.MANIPULATION])
    check("Sans NAVIGATION_GOTO : PermissionError, code PERMISSION_DENIED",
          leve(RefusalCode.PERMISSION_DENIED, sans.go_to, 0.5, 0.0, timeout_s=5.0))
    try:
        silencieux(sans.go_to, 0.5, 0.0, timeout_s=5.0)
    except SdkPermissionError:
        check("… c'est bien la PermissionError du SDK", True)

    nul = contexte(robot, navigation=None)
    r = silencieux(nul.go_to, 0.5, 0.0, timeout_s=5.0)
    check("Pas de service de navigation : REFUSED/CAPABILITY_MISSING",
          r.status == "REFUSED" and r.code == RefusalCode.CAPABILITY_MISSING.value, str(r))

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    silencieux(robot.hal.emergency_stop_all)
    check("Arrêt d'urgence déjà actif : EstopActiveError (ESTOP_ACTIVE)",
          leve(RefusalCode.ESTOP_ACTIVE, ctx.go_to, 1.0, 0.0, timeout_s=5.0))
    check("… aucun départ", robot.navigation.etat is EtatNavigation.INACTIF)

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    threading.Timer(1.0, lambda: silencieux(robot.hal.emergency_stop_all)).start()
    t0 = time.monotonic()
    r = silencieux(ctx.go_to, 3.0, 0.0, timeout_s=20.0)
    duree = time.monotonic() - t0
    p1 = robot.pose
    time.sleep(0.5)
    check("Arrêt d'urgence pendant le trajet : REFUSED/ESTOP_ACTIVE, aussitôt",
          r.status == "REFUSED" and r.code == RefusalCode.ESTOP_ACTIVE.value and duree < 1.5,
          f"{r} en {duree:.2f} s")
    check("… navigation annulée, la base ne bouge plus",
          robot.navigation.etat is EtatNavigation.INACTIF and robot.pose == p1,
          f"{p1} -> {robot.pose}")

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    threading.Timer(0.8, ctx._fermer).start()
    r = silencieux(ctx.go_to, 3.0, 0.0, timeout_s=20.0)
    check("Fin du hook pendant le trajet : REFUSED/CANCELLED, robot arrêté",
          r.code == RefusalCode.CANCELLED.value
          and robot.navigation.etat is EtatNavigation.INACTIF, str(r))

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    threading.Timer(0.8, robot.fall).start()
    r = silencieux(ctx.go_to, 3.0, 0.0, timeout_s=20.0)
    check("Chute pendant le trajet : REFUSED/SAFETY_LIMIT", r.code == RefusalCode.SAFETY_LIMIT.value,
          str(r))

section("3. pick / place : cas normal et issues")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    boite = boite_devant(robot)
    r = silencieux(ctx.pick, ObjectSpec(boite.position, (0.24, 0.20, 0.20)), timeout_s=15.0)
    check("pick(ObjectSpec) : HELD, l'objet a suivi les mains", r.ok and r.status == "HELD"
          and robot.held is boite and robot.manipulation.etat is Etat.TENU, str(r))
    check("… l'objet est soulevé", boite.position[2] > 0.90, str(boite.position))
    r2 = silencieux(ctx.pick, ObjectSpec(boite.position, (0.24, 0.20, 0.20)), timeout_s=5.0)
    check("Deuxième pick pendant qu'il tient : REFUSED/BUSY", r2.code == RefusalCode.BUSY.value,
          str(r2))
    r3 = silencieux(ctx.rest_pose)
    check("rest_pose pendant qu'il tient : REFUSED/BUSY (l'objet n'est pas lâché)",
          r3.code == RefusalCode.BUSY.value and robot.manipulation.etat is Etat.TENU, str(r3))
    r = silencieux(ctx.place, None, timeout_s=15.0)
    check("place() : PLACED, mains vides", r.ok and r.status == "PLACED"
          and robot.manipulation.etat is Etat.LIBRE and robot.held is None, str(r))
    r = silencieux(ctx.place, None, timeout_s=5.0)
    check("place() sans rien en main : REFUSED (FAILED)", r.status == "REFUSED"
          and r.code == RefusalCode.FAILED.value, str(r))

with SimulatedRobot(start=(1.0, 0.0, 0.0)) as robot:
    ctx = contexte(robot)
    boite = boite_devant(robot)
    vues = silencieux(ctx.look_for, "red", FRAME_WORLD)
    base = silencieux(ctx.look_for, "red")
    check("look_for(..., world) : la détection porte son repère", len(vues) == 1
          and vues[0].frame == "world", str(vues))
    r = silencieux(ctx.pick, vues[0], size_m=(0.24, 0.20, 0.20), timeout_s=15.0)
    check("pick(Detection monde, taille) : HELD", r.ok and r.status == "HELD", str(r))
    r = silencieux(ctx.place, 2.4, timeout_s=5.0)
    check("Plateau trop haut : REFUSED/OUT_OF_REACH, objet toujours tenu",
          r.code == RefusalCode.OUT_OF_REACH.value and robot.manipulation.etat is Etat.TENU, str(r))
    r = silencieux(ctx.place, 0.40, timeout_s=20.0)
    check("Plateau à 40 cm, plus bas que les bras : FAILED/OUT_OF_REACH, objet toujours tenu",
          r.status == "FAILED" and r.code == RefusalCode.OUT_OF_REACH.value
          and robot.manipulation.etat is Etat.TENU, str(r))
    time.sleep(1.0)          # après l'échec, les bras remontent l'objet
    r = silencieux(ctx.place, 0.70, timeout_s=20.0)
    check("Plateau à 70 cm : PLACED, l'objet est descendu jusqu'à l'effleurer",
          r.ok and boite.position[2] < 0.90 and robot.manipulation.etat is Etat.LIBRE,
          f"{r} {boite.position}")
    check("Détection dans le repère du robot : INVALID_ARGUMENT",
          leve(RefusalCode.INVALID_ARGUMENT, ctx.pick, base[0], size_m=(0.24, 0.2, 0.2),
               timeout_s=5.0))
    check("Détection sans taille : INVALID_ARGUMENT",
          leve(RefusalCode.INVALID_ARGUMENT, ctx.pick, vues[0], timeout_s=5.0))
    for label, centre, taille in [("centre NaN", (float("nan"), 0, 0.9), (0.2, 0.2, 0.2)),
                                  ("centre à 2 km", (2000.0, 0, 0.9), (0.2, 0.2, 0.2)),
                                  ("taille de 1 mm", (1.3, 0, 0.9), (0.001, 0.2, 0.2)),
                                  ("taille de 2 m", (1.3, 0, 0.9), (2.0, 0.2, 0.2)),
                                  ("taille texte", (1.3, 0, 0.9), ("0.2", 0.2, 0.2)),
                                  ("centre de 2 valeurs", (1.3, 0), (0.2, 0.2, 0.2))]:
        check(f"ObjectSpec {label} : INVALID_ARGUMENT",
              leve(RefusalCode.INVALID_ARGUMENT, ctx.pick, ObjectSpec(centre, taille),
                   timeout_s=5.0))
    check("Cible d'un autre type : INVALID_ARGUMENT",
          leve(RefusalCode.INVALID_ARGUMENT, ctx.pick, "red box", timeout_s=5.0))
    check("place(hauteur NaN) : INVALID_ARGUMENT",
          leve(RefusalCode.INVALID_ARGUMENT, ctx.place, float("nan"), timeout_s=5.0))
    check("place(hauteur négative) : INVALID_ARGUMENT",
          leve(RefusalCode.INVALID_ARGUMENT, ctx.place, -0.5, timeout_s=5.0))
    check("pick(timeout infini) : INVALID_ARGUMENT",
          leve(RefusalCode.INVALID_ARGUMENT, ctx.pick, object_spec((1.3, 0, 0.9), (0.2, 0.2, 0.2)),
               timeout_s=float("inf")))
    check("… aucun geste lancé par ces refus", robot.manipulation.etat is Etat.LIBRE)

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    r = silencieux(ctx.pick, object_spec((1.5, 0.0, 0.9), (0.24, 0.20, 0.20)), timeout_s=5.0)
    check("Objet à 1.5 m (pas entre les mains) : REFUSED/OUT_OF_REACH, bras immobiles",
          r.code == RefusalCode.OUT_OF_REACH.value and robot.manipulation.etat is Etat.LIBRE, str(r))

with SimulatedRobot(grasp_succeeds=False) as robot:
    ctx = contexte(robot)
    boite = boite_devant(robot)
    r = silencieux(ctx.pick, ObjectSpec(boite.position, (0.24, 0.20, 0.20)), timeout_s=15.0)
    check("Prise qui ne tient pas : FAILED (code FAILED)", r.status == "FAILED"
          and r.code == RefusalCode.FAILED.value, str(r))
    check("… les bras se rouvrent progressivement (pas restés serrés)",
          robot.manipulation.etat in (Etat.LACHER, Etat.LIBRE), robot.manipulation.etat.name)

section("4. pick / place : permission, arrêt d'urgence, capacité absente")
with SimulatedRobot() as robot:
    boite = boite_devant(robot)
    spec = ObjectSpec(boite.position, (0.24, 0.20, 0.20))
    sans = contexte(robot, perms=[Permission.NAVIGATION_GOTO])
    check("pick sans MANIPULATION : PERMISSION_DENIED",
          leve(RefusalCode.PERMISSION_DENIED, sans.pick, spec, timeout_s=5.0))
    check("place sans MANIPULATION : PERMISSION_DENIED",
          leve(RefusalCode.PERMISSION_DENIED, sans.place, None, timeout_s=5.0))
    ctx = contexte(robot)
    silencieux(robot.hal.emergency_stop_all)
    check("pick sous arrêt d'urgence : ESTOP_ACTIVE",
          leve(RefusalCode.ESTOP_ACTIVE, ctx.pick, spec, timeout_s=5.0))
    check("place sous arrêt d'urgence : ESTOP_ACTIVE",
          leve(RefusalCode.ESTOP_ACTIVE, ctx.place, None, timeout_s=5.0))
    check("… aucun geste lancé", robot.manipulation.etat is Etat.LIBRE)

with SimulatedRobot() as robot:
    ctx = contexte(robot)
    boite = boite_devant(robot)
    threading.Timer(0.3, lambda: silencieux(robot.hal.emergency_stop_all)).start()
    r = silencieux(ctx.pick, ObjectSpec(boite.position, (0.24, 0.20, 0.20)), timeout_s=15.0)
    check("Arrêt d'urgence pendant la prise : REFUSED/ESTOP_ACTIVE",
          r.code == RefusalCode.ESTOP_ACTIVE.value, str(r))

with SimulatedRobot(manipulation=False) as robot:
    ctx = contexte(robot)
    r = silencieux(ctx.pick, object_spec((0.3, 0.0, 0.9), (0.24, 0.2, 0.2)), timeout_s=5.0)
    check("Robot sans manipulation : pick REFUSED/CAPABILITY_MISSING",
          r.code == RefusalCode.CAPABILITY_MISSING.value, str(r))
    r = silencieux(ctx.place, None, timeout_s=5.0)
    check("… place REFUSED/CAPABILITY_MISSING", r.code == RefusalCode.CAPABILITY_MISSING.value,
          str(r))

section("5. rest_pose")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    bras = {"left_shoulder_pitch_joint": 0.6, "right_shoulder_pitch_joint": 0.6,
            "left_elbow_joint": 0.8, "right_elbow_joint": 0.8}
    check("Bras écartés de la posture de repos", silencieux(ctx.move_to_pose, bras, 1.0))
    r = silencieux(ctx.rest_pose, duration_s=1.5)
    etat = robot.hal.get_full_state()
    repos = robot.hal.get_rest_pose()
    ecart = max(abs(etat[j].position - v) for j, v in repos.items())
    check("rest_pose : AT_REST, bras à la posture déclarée par le driver",
          r.ok and r.status == "AT_REST" and ecart < 0.05, f"{r} écart {ecart:.3f}")
    check("La posture de repos du G1 ne touche que les bras",
          all("leg" not in j and "hip" not in j and "knee" not in j and "ankle" not in j
              for j in repos), str(sorted(repos)))
    for label, d in [("NaN", float("nan")), ("0.1 s", 0.1), ("60 s", 60.0), ("texte", "2")]:
        check(f"duration_s {label} : INVALID_ARGUMENT",
              leve(RefusalCode.INVALID_ARGUMENT, ctx.rest_pose, duration_s=d))
    sans = contexte(robot, perms=[Permission.ACTUATORS_READ])
    check("Sans ACTUATORS_WRITE_ARMS : PERMISSION_DENIED (mêmes droits que move_to_pose)",
          leve(RefusalCode.PERMISSION_DENIED, sans.rest_pose))
    silencieux(robot.hal.emergency_stop_all)
    check("Sous arrêt d'urgence : ESTOP_ACTIVE", leve(RefusalCode.ESTOP_ACTIVE, ctx.rest_pose))


class _HALSansRepos(_HALSimulee):
    """Un driver qui ne déclare pas de posture de repos."""
    def get_rest_pose(self):
        return None


with SimulatedRobot() as robot:
    from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
    from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
    from axis_os.hal.watchdog.system import WatchdogSystem
    with contextlib.redirect_stdout(io.StringIO()):
        hal = NormalizedRobotHAL(_HALSansRepos("g1_sans_repos"), build_g1_normalizer())
        hal.initialize_all()
    ctx = RobotContext.for_testing(hal, WatchdogSystem(hal), manifeste(TOUT))
    r = silencieux(ctx.rest_pose)
    check("Driver sans posture de repos : REFUSED/CAPABILITY_MISSING",
          r.code == RefusalCode.CAPABILITY_MISSING.value, str(r))
    sans = RobotContext.for_testing(hal, WatchdogSystem(hal), manifeste([]))
    check("… mais sans permission, c'est d'abord PERMISSION_DENIED",
          leve(RefusalCode.PERMISSION_DENIED, sans.rest_pose))


# ─── Depuis le processus de l'app (pont JSON) ────────────────────────────

class AppFetch(RobotApp):
    """Va devant la boîte, la prend, revient la poser. Rend l'issue de
    chaque étape dans metrics."""
    def manifest(self):
        return AppManifest(app_id="com.test.fetch_pont", name="Fetch", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.NAVIGATION_GOTO, Permission.MANIPULATION,
                                        Permission.SENSORS_CAMERA, Permission.ACTUATORS_READ,
                                        Permission.ACTUATORS_WRITE_ARMS])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        issues = {}
        vues = ctx.look_for("red", "world")
        issues["vues"] = [d.frame for d in vues]
        if params.get("invalide"):
            try:
                ctx.go_to(float("nan"), 0.0, timeout_s=5.0)
                issues["nan"] = "accepté"
            except ValueError as exc:
                issues["nan"] = type(exc).__name__
            try:
                ctx.pick(vues[0], timeout_s=5.0)
                issues["sans_taille"] = "accepté"
            except ValueError as exc:
                issues["sans_taille"] = type(exc).__name__
        boite = vues[0].position_3d
        r = ctx.go_to(boite.x - 0.30, boite.y, 0.0, timeout_s=30.0)
        issues["go_to"] = r.status
        r = ctx.pick(vues[0], size_m=(0.24, 0.20, 0.20), timeout_s=15.0)
        issues["pick"] = r.status
        r = ctx.go_to(0.0, 0.0, timeout_s=30.0)
        issues["retour"] = r.status
        r = ctx.place(None, timeout_s=15.0)
        issues["place"] = r.status
        r = ctx.rest_pose(duration_s=1.0)
        issues["rest_pose"] = r.status
        ok = all(issues[k] in ("ARRIVED", "HELD", "PLACED", "AT_REST")
                 for k in ("go_to", "pick", "retour", "place", "rest_pose"))
        return MissionResult(success=ok, mission_id="", app_id="", duration_s=0.0,
                             metrics=issues)


class AppSansDroitDeMarcher(RobotApp):
    """Appelle go_to sans avoir déclaré NAVIGATION_GOTO : doit être refusée."""
    def manifest(self):
        return AppManifest(app_id="com.test.sans_goto", name="Sans", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.SENSORS_CAMERA])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        if params.get("geste") == "pick":
            ctx.pick(ObjectSpec((0.3, 0.0, 0.9), (0.2, 0.2, 0.2)), timeout_s=5.0)
        else:
            ctx.go_to(1.0, 0.0, timeout_s=5.0)
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


class AppLongTrajet(RobotApp):
    """Un long trajet, pendant lequel le test déclenche l'arrêt d'urgence."""
    def manifest(self):
        return AppManifest(app_id="com.test.long_trajet", name="Long", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.NAVIGATION_GOTO])

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        r = ctx.go_to(4.0, 0.0, timeout_s=params.get("timeout", 60.0))
        return MissionResult(success=r.ok, mission_id="", app_id="", duration_s=0.0,
                             metrics={"status": r.status, "code": r.code})


section("6. Depuis le processus de l'app (pont JSON)")
with AppTestHarness(AppFetch(), objects=[SimulatedObject("red box", (1.0, 0.0, 0.90))]) as h:
    res = h.run({"invalide": True})
    m = res.metrics if res is not None else {}
    check("Mission fetch dans son processus : look_for -> go_to -> pick -> go_to -> place "
          "-> rest_pose", res is not None and res.success, f"{m} {getattr(res, 'error_message', '')}")
    check("La détection traverse le pont avec son repère", m.get("vues") == ["world"], str(m))
    check("Arguments invalides refusés dans le processus de l'app (InvalidArgumentError)",
          m.get("nan") == "InvalidArgumentError" and m.get("sans_taille") == "InvalidArgumentError",
          str(m))
    check("La boîte a été rapportée près du point de départ", abs(h.robot.objects[0].position[0]) < 0.6,
          str(h.robot.objects[0].position))
    check("Aucune erreur dans la boucle des services", h.robot.errors == [], str(h.robot.errors))

with AppTestHarness(AppSansDroitDeMarcher()) as h:
    res = h.run()
    check("go_to sans NAVIGATION_GOTO, depuis le processus : mission refusée",
          res is not None and not res.success and "NAVIGATION_GOTO" in (res.error_message or ""),
          getattr(res, "error_message", None))
    res = h.run({"geste": "pick"})
    check("pick sans MANIPULATION : mission refusée",
          res is not None and not res.success and "MANIPULATION" in (res.error_message or ""),
          getattr(res, "error_message", None))
    check("… et le robot n'a pas bougé", h.robot.pose[:2] == (0.0, 0.0), str(h.robot.pose))

with AppTestHarness(AppLongTrajet()) as h:
    threading.Timer(1.5, lambda: silencieux(h.robot.hal.emergency_stop_all)).start()
    t0 = time.monotonic()
    res = h.run()
    duree = time.monotonic() - t0
    p1 = h.robot.pose
    time.sleep(0.5)
    check("Arrêt d'urgence pendant go_to (processus de l'app) : mission interrompue, vite",
          res is not None and not res.success and duree < 4.0,
          f"{getattr(res, 'error_message', None)} en {duree:.1f} s")
    check("… navigation annulée, base immobile",
          h.robot.navigation.etat is EtatNavigation.INACTIF and h.robot.pose == p1,
          f"{p1} -> {h.robot.pose}")

section("7. Pont : schémas et délais")
with SimulatedRobot() as robot:
    ctx = contexte(robot)
    serveur = HALBridgeServer(ctx)

    def appel(action, payload, reste_s=None):
        return silencieux(serveur.traiter, {"type": "appel", "action": action,
                                            "payload": payload, "request_id": 1},
                          reste_s=reste_s)

    for label, action, payload in [
            ("go_to : x NaN", "go_to", {"x": float("nan"), "y": 0, "timeout_s": 5}),
            ("go_to : x texte", "go_to", {"x": "1", "y": 0, "timeout_s": 5}),
            ("go_to : timeout_s manquant", "go_to", {"x": 1, "y": 0}),
            ("go_to : champ inconnu", "go_to", {"x": 1, "y": 0, "timeout_s": 5, "vitesse": 9}),
            ("go_to : timeout au-delà de 600 s", "go_to", {"x": 1, "y": 0, "timeout_s": 1e6}),
            ("pick : centre de 4 valeurs", "pick", {"center": [1, 0, 0.9, 1],
                                                    "size": [0.2, 0.2, 0.2], "timeout_s": 5}),
            ("pick : taille non numérique", "pick", {"center": [1, 0, 0.9],
                                                     "size": ["a", 0.2, 0.2], "timeout_s": 5}),
            ("place : hauteur de 10 m", "place", {"surface_height_m": 10, "timeout_s": 5}),
            ("rest_pose : durée de 1 h", "rest_pose", {"duration_s": 3600})]:
        rep = appel(action, payload)
        check(f"{label} : refusé par le schéma (MALFORMED_COMMAND)",
              rep is not None and not rep["success"] and "MALFORMED" in rep["error"], str(rep))
    rep = appel("pick", {"center": [1, 0, 0.9], "size": [0.001, 0.2, 0.2], "timeout_s": 5})
    check("pick : taille de 1 mm (passe le schéma) refusée par RobotContext : INVALID_ARGUMENT",
          rep is not None and rep["error"].startswith("INVALID_ARGUMENT:"), str(rep))
    rep = appel("go_to", {"x": 1, "y": 0, "timeout_s": 0})
    check("go_to : délai nul refusé (jamais rallongé par l'OS)",
          rep is not None and rep["error"].startswith("INVALID_ARGUMENT:"), str(rep))
    t0 = time.monotonic()
    rep = appel("go_to", {"x": -4.0, "y": 0, "timeout_s": 60}, reste_s=1.5)
    duree = time.monotonic() - t0
    check("Délai borné par le temps qui reste au hook (1.5 s) : rend la main à temps",
          rep is not None and rep["data"]["status"] == "TIMEOUT" and duree < 1.6,
          f"{rep} en {duree:.2f} s")

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
