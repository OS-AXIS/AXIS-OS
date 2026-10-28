"""
Position du robot (where_am_i), lieux nommés et zones (carte du sens,
versionnée, écrite par l'administrateur seulement) : c'est l'OS qui fait
respecter les zones INTERDITE et LENTE, pour toute commande de marche (walk,
go_to, flux « walk », follow). Lieux requis à l'installation.
"""
import contextlib
import io
import json
import math
import os
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.services.navigation.site_map import (  # noqa: E402
    FORBIDDEN, NAMED, SLOW, STATION, Place, SiteMap, SiteMapError, Zone,
)
from axis_os.services.navigation.zones import GardienZones  # noqa: E402
from axis_os.store.runtime import hal_bridge  # noqa: E402
from axis_os.store.sdk.app_base import (  # noqa: E402
    EstopActiveError, MissionResult, PermissionError, RobotApp, RobotContext,
)
from axis_os.store.sdk.manifest import (  # noqa: E402
    AppCategory, AppManifest, Permission, ResourceQuota, TriggerSpec,
)
from axis_os.store.sdk.results import InvalidArgumentError, Location  # noqa: E402
from axis_os.testing import AppTestHarness, SimulatedRobot  # noqa: E402

FAIL = 0
ECHELLE = 2.0


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


def leve(exc, f, *a, **kw):
    try:
        silencieux(f, *a, **kw)
        return False
    except exc:
        return True


def carre(x0, y0, x1, y1):
    return ((x0, y0), (x1, y0), (x1, y1), (x0, y1))


INTERDITE = Zone("local électrique", FORBIDDEN, carre(1.5, -1.0, 2.5, 1.0))
LENTE = Zone("allée", SLOW, carre(-1.0, 2.0, 4.0, 3.0), max_speed_mps=0.1)
HALL = Zone("hall", NAMED, carre(-0.5, -3.5, 0.5, -2.5))
LIEUX = [Place("quai 3", 4.0, 0.0, 0.0), Place("station", -2.0, 0.0, 3.14, kind=STATION),
         Place("bureau", 0.0, 2.5)]
TOUT = [Permission.LOCATION, Permission.NAVIGATION_GOTO, Permission.LOCOMOTION_WALK,
        Permission.ACTUATORS_WRITE_LEGS, Permission.TELEOP_STREAM, Permission.PEOPLE_DETECT,
        Permission.PEOPLE_FOLLOW]


def manifeste(perms=None, app_id="com.test.lieux", **kw):
    return AppManifest(app_id=app_id, name="Lieux", version="1.0.0", author="T",
                       author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
                       permissions=list(TOUT if perms is None else perms), **kw)


def contexte(robot, perms=None):
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog, manifeste(perms))
    ctx._set_clock(robot.clock)
    ctx._attach_services(locomotion=robot.locomotion, navigation=robot.navigation,
                         site_map=robot.site_map, people=robot.people_service,
                         follow=robot.follow_service)
    return ctx


def distance_zone(robot, zone=None):
    # (zone=None plutôt que INTERDITE : le processus de l'app recharge ce
    # module sans ses affectations calculées.)
    return (zone or INTERDITE).distance(robot.x, robot.y)


def surveiller(robot, duree, zone=None, pendant=None):
    """Distance minimale du robot à la zone pendant `duree` s du robot."""
    fin, d_min = robot.clock() + duree, math.inf
    while robot.clock() < fin:
        if pendant is not None:
            pendant()
        d_min = min(d_min, distance_zone(robot, zone))
        time.sleep(0.01)
    return d_min


section("1. Carte du sens : versions, vérification, historique, fichier")
carte = SiteMap(LIEUX, [INTERDITE, LENTE, HALL])
check("Version 1 à la création", carte.version == 1)
v = carte.publish(LIEUX + [Place("quai 4", 5.0, 1.0)], [INTERDITE], author="admin")
check("Publication : version 2, l'ancienne gardée dans l'historique",
      v == 2 and carte.history()[-1].version == 1 and carte.current.place("quai 4") is not None)
for nom, lieux, zones in (
        ("nom en double", LIEUX + [Place("quai 3", 1.0, 1.0)], []),
        ("coordonnée NaN", [Place("x", float("nan"), 0.0)], []),
        ("polygone plat", [], [Zone("plat", FORBIDDEN, ((0, 0), (1, 1), (2, 2)))]),
        ("deux sommets", [], [Zone("ligne", FORBIDDEN, ((0, 0), (1, 1)))]),
        ("lieu dans une zone interdite", [Place("piège", 2.0, 0.0)], [INTERDITE]),
        ("deux stations", [Place("s1", 0, 0, kind=STATION), Place("s2", 1, 1, kind=STATION)], []),
        ("zone lente sans vitesse", [], [Zone("l", SLOW, carre(0, 0, 1, 1))]),
        ("type inconnu", [], [Zone("z", "magique", carre(0, 0, 1, 1))])):
    try:
        carte.publish(lieux, zones, author="admin")
        refuse = False
    except SiteMapError:
        refuse = True
    check(f"Refusée : {nom} (et la version courante ne change pas)",
          refuse and carte.version == 2)
check("Retour à la version 1 : nouvelle version 3, contenu de la 1",
      carte.rollback(1, author="admin") == 3 and len(carte.current.zones) == 3)
with tempfile.TemporaryDirectory() as d:
    chemin = os.path.join(d, "carte.json")
    c1 = SiteMap(LIEUX, [INTERDITE], path=chemin)
    c1.publish(LIEUX, [INTERDITE, LENTE], author="admin")
    c2 = SiteMap(path=chemin)
    check("Enregistrée puis relue : même version, même contenu, historique",
          c2.version == 2 and len(c2.current.zones) == 2 and c2.history()[0].version == 1)
    doc = json.load(open(chemin))
    check("Fichier lisible, au format axis-site-map", doc["format"] == "axis-site-map")
check("Une app ne peut pas écrire la carte : aucune action d'écriture sur le pont",
      not any(k in a for a in hal_bridge.SCHEMAS_ACTIONS for k in ("publish", "rollback",
                                                                   "site_map", "set_place")))
check("… ni aucune méthode de RobotContext",
      not any(k in m for m in dir(RobotContext) if not m.startswith("_")
              for k in ("publish", "rollback", "set_zone", "set_place")))

section("2. Gardien de zones (unitaire)")
pose = [0.0, 0.0, 0.0]
g = GardienZones(SiteMap([], [INTERDITE, LENTE]), lambda: tuple(pose))
pose[:] = [1.2, 0.0, 0.0]
v = g.limiter(0.3, 0.0, 0.2)
check("Vers la zone interdite : translation annulée, rotation gardée",
      v.vx == 0.0 and v.vy == 0.0 and v.wz == 0.2 and v.zone == "local électrique", str(v))
v = g.limiter(0.0, 0.3, 0.0)
check("Le long de la zone (de côté), trop près : refusé aussi", v.vy == 0.0, str(v))
pose[:] = [1.3, 0.0, math.pi]
v = g.limiter(0.3, 0.0, 0.0)
check("Déjà trop près : s'éloigner reste permis", v.vx == 0.3, str(v))
pose[:] = [0.0, 0.0, 0.0]
check("Loin de toute zone : consigne inchangée", g.limiter(0.3, 0.0, 0.0).vx == 0.3)
pose[:] = [0.0, 2.5, 0.0]
v = g.limiter(0.3, 0.0, 0.0)
check("Zone lente : vitesse plafonnée à 0.1 m/s", abs(math.hypot(v.vx, v.vy) - 0.1) < 1e-9, str(v))
pose[:] = [float("nan"), 0.0, 0.0]
v = g.limiter(0.3, 0.0, 0.3)
check("Pose NaN : arrêt complet", (v.vx, v.vy, v.wz) == (0.0, 0.0, 0.0), str(v))
g2 = GardienZones(SiteMap(), lambda: (float("nan"), 0, 0))
check("Carte sans zone : rien à faire respecter", g2.limiter(0.3, 0, 0).vx == 0.3)

section("3. walk(), flux « walk », go_to, follow : jamais dans la zone interdite")
carte = SiteMap(LIEUX, [INTERDITE, LENTE, HALL])
with SimulatedRobot(time_scale=ECHELLE, site_map=carte) as robot:
    ctx = contexte(robot)
    d_min = surveiller(robot, 8.0, pendant=lambda: ctx.walk(0.3))
    check(f"walk(0.3) droit sur la zone : le robot s'arrête devant (au plus près "
          f"{d_min:.2f} m du bord, rayon {robot.zone_guard.rayon:.2f} m)",
          d_min >= robot.zone_guard.rayon - 0.05 and robot.x < 1.5, f"x = {robot.x:.2f}")
    silencieux(ctx.stop_walking)
with SimulatedRobot(time_scale=ECHELLE, site_map=carte) as robot:
    ctx = contexte(robot)
    flux = ctx.stream_targets("walk", deadman_ms=300, max_speed=0.5)
    d_min = math.inf
    fin = robot.clock() + 8.0
    while robot.clock() < fin:
        flux.push((0.2, 0.0, 0.0), deadman=True, t=ctx.now())
        d_min = min(d_min, distance_zone(robot))
        ctx.sleep(0.05)
    flux.close()
    check(f"Flux « walk » droit sur la zone : arrêt devant (au plus près {d_min:.2f} m)",
          d_min >= robot.zone_guard.rayon - 0.05)
with SimulatedRobot(time_scale=ECHELLE, site_map=carte) as robot:
    ctx = contexte(robot)
    r = ctx.go_to(2.0, 0.0, timeout_s=5.0)
    check("go_to vers un point de la zone interdite : REFUSED / FORBIDDEN_ZONE",
          r.status == "REFUSED" and r.code == "FORBIDDEN_ZONE", str(r))
    r = ctx.go_to(1.2, 0.0, timeout_s=5.0)
    check("… vers un point trop près pour que le robot y tienne : refusé aussi",
          r.code == "FORBIDDEN_ZONE", str(r))
    res = {}
    import threading
    th = threading.Thread(target=lambda: res.update(r=ctx.go_to("quai 3", timeout_s=60.0)))
    th.start()
    d_min = math.inf
    while th.is_alive():
        d_min = min(d_min, distance_zone(robot))
        time.sleep(0.01)
    check("go_to(\"quai 3\") de l'autre côté : ARRIVED, en contournant la zone",
          res["r"].status == "ARRIVED" and math.hypot(robot.x - 4.0, robot.y) < 0.2, str(res["r"]))
    check(f"… sans jamais l'approcher à moins du rayon du robot ({d_min:.2f} m)",
          d_min >= robot.zone_guard.rayon - 0.05)
with SimulatedRobot(time_scale=ECHELLE, site_map=carte) as robot:
    p = robot.add_person(1.0, 0.0, torso=(200, 30, 30))
    ctx = contexte(robot)
    r = ctx.follow(ctx.person_in_front(5.0).person_id, distance_m=1.5)
    p.walk_to(3.5, 0.0, speed_mps=0.4)
    d_min = surveiller(robot, 9.0, pendant=ctx.follow_status)
    check(f"follow : la personne traverse la zone, le robot s'arrête devant "
          f"({d_min:.2f} m)", d_min >= robot.zone_guard.rayon - 0.05)
    silencieux(ctx.stop_follow)

section("4. Zone lente, et carte qui change en route")
with SimulatedRobot(time_scale=ECHELLE, site_map=carte, start=(-0.8, 2.5, 0.0)) as robot:
    ctx = contexte(robot)
    x0, t0 = robot.x, robot.clock()
    surveiller(robot, 3.0, pendant=lambda: ctx.walk(0.3))
    vitesse = (robot.x - x0) / (robot.clock() - t0)
    check(f"Dans l'allée lente : {vitesse:.3f} m/s pour 0.3 demandés (plafond 0.1)",
          vitesse <= 0.1 + 0.02)
    silencieux(ctx.stop_walking)
c = SiteMap(LIEUX, [])
with SimulatedRobot(time_scale=ECHELLE, site_map=c) as robot:
    ctx = contexte(robot)
    res = {}
    th = threading.Thread(target=lambda: res.update(r=ctx.go_to("quai 3", timeout_s=40.0)))
    th.start()
    time.sleep(0.5)
    c.publish(LIEUX, [INTERDITE], author="admin")
    d_min = math.inf
    while th.is_alive():
        d_min = min(d_min, distance_zone(robot))
        time.sleep(0.01)
    check("Zone publiée par l'administrateur en route : jamais entrée",
          d_min >= robot.zone_guard.rayon - 0.05 and not INTERDITE.contains(robot.x, robot.y),
          f"{d_min:.2f} m, {res.get('r')}")
    check("… et le robot arrive en la contournant (replanification)",
          res["r"].status == "ARRIVED", str(res["r"]))

section("5. where_am_i, places, zones, événements d'entrée et de sortie")
with SimulatedRobot(time_scale=ECHELLE, site_map=carte, start=(0.0, -2.0, -math.pi / 2)) as robot:
    ctx = contexte(robot)
    loc = ctx.where_am_i()
    check("where_am_i : position, cap, heure, lieu le plus proche",
          isinstance(loc, Location) and loc.known and abs(loc.x) < 0.01 and abs(loc.y + 2.0) < 0.01
          and abs(loc.heading + math.pi / 2) < 0.01 and loc.timestamp is not None
          and loc.nearest_place == "station" and loc.map_version == carte.version, str(loc))
    check("… incertitude inconnue avec l'odométrie : None, jamais inventée",
          loc.uncertainty_m is None)
    check("places() et zones() : la carte, en lecture",
          {p.name for p in ctx.places()} == {"quai 3", "station", "bureau"}
          and {z.name: z.kind for z in ctx.zones()} == {"local électrique": "forbidden",
                                                        "allée": "slow", "hall": "named"})
    vus = []
    ctx.on_enter_zone("hall", lambda n: vus.append(("entrée", n)))
    ctx.on_exit_zone("hall", lambda n: vus.append(("sortie", n)))
    fin = robot.clock() + 9.0
    while robot.clock() < fin:
        ctx.walk(0.3)
        ctx.sleep(0.1)
    silencieux(ctx.stop_walking)
    ctx.sleep(0.3)
    check("on_enter_zone puis on_exit_zone, chacun une fois", vus == [("entrée", "hall"),
                                                                     ("sortie", "hall")], str(vus))
    check("where_am_i dit les zones où est le robot", "hall" not in ctx.where_am_i().zones)
    ctx._fermer()

section("6. Permissions, arguments invalides, arrêt d'urgence, capacité absente")
with SimulatedRobot(time_scale=ECHELLE, site_map=carte) as robot:
    sans = contexte(robot, [p for p in TOUT if p is not Permission.LOCATION])
    for nom, f in (("where_am_i", sans.where_am_i), ("places", sans.places),
                   ("zones", sans.zones),
                   ("on_enter_zone", lambda: sans.on_enter_zone("hall", print)),
                   ("on_exit_zone", lambda: sans.on_exit_zone("hall", print))):
        check(f"{nom} sans LOCATION : PermissionError", leve(PermissionError, f))
    ctx = contexte(robot)
    for nom, f in (("go_to vers un lieu inconnu", lambda: ctx.go_to("quai 99", timeout_s=5)),
                   ("go_to(nom, y)", lambda: ctx.go_to("quai 3", 1.0, timeout_s=5)),
                   ("go_to nom vide", lambda: ctx.go_to("  ", timeout_s=5)),
                   ("on_enter_zone d'une zone inconnue", lambda: ctx.on_enter_zone("x", print)),
                   ("on_enter_zone sans fonction", lambda: ctx.on_enter_zone("hall", 3)),
                   ("on_exit_zone nom non texte", lambda: ctx.on_exit_zone(42, print))):
        check(f"{nom} : InvalidArgumentError", leve(InvalidArgumentError, f))
    silencieux(robot.hal.emergency_stop_all)
    check("go_to(\"quai 3\") pendant un arrêt d'urgence : EstopActiveError",
          leve(EstopActiveError, ctx.go_to, "quai 3", timeout_s=5))
    check("where_am_i reste servi (lecture seule)", ctx.where_am_i().known)
with SimulatedRobot(time_scale=ECHELLE) as robot:
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog, manifeste())
    ctx._attach_services(locomotion=robot.locomotion)
    check("Sans navigation : where_am_i inconnu (known=False), jamais inventé",
          ctx.where_am_i() == Location(known=False))
    check("Sans carte : places() et zones() vides, go_to(nom) refusé",
          ctx.places() == [] and ctx.zones() == []
          and leve(InvalidArgumentError, ctx.go_to, "quai 3", timeout_s=5))


section("7. Lieux requis à l'installation")


class AppQuai(RobotApp):
    def __init__(self, lieux=None, triggers=None):
        self.lieux, self.triggers = lieux or [], triggers or []

    def manifest(self):
        perms = [Permission.LOCATION] + ([Permission.SCHEDULED_MISSIONS] if self.triggers else [])
        return manifeste(perms, app_id="com.test.quai", required_places=list(self.lieux),
                         triggers=list(self.triggers))

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


with SimulatedRobot(site_map=carte) as robot:
    rt = robot.make_runtime()
    sortie = io.StringIO()
    with contextlib.redirect_stdout(sortie):
        ok = rt.install(AppQuai(["quai 3", "quai 7"]))
    check("Lieu requis absent : installation refusée, message clair",
          not ok and "quai 7" in sortie.getvalue() and "carte du site" in sortie.getvalue())
    check("Lieux requis présents : installée", silencieux(rt.install, AppQuai(["quai 3", "station"])))
    sortie = io.StringIO()
    with contextlib.redirect_stdout(sortie):
        ok = rt.install(AppQuai([], [TriggerSpec("arrivee", "zone_enter", zone="zone 51")]))
    check("Déclencheur sur une zone absente de la carte : refusée", not ok
          and "zone 51" in sortie.getvalue())
    sortie = io.StringIO()
    with contextlib.redirect_stdout(sortie):
        rt.install(AppQuai(["quai 3"]))
    check("L'installation montre « cette app connaît la position du robot »",
          "connaît la position du robot" in sortie.getvalue())


section("8. Depuis le processus de l'app")


class AppTournee(RobotApp):
    def manifest(self):
        return manifeste([Permission.LOCATION, Permission.NAVIGATION_GOTO,
                          Permission.LOCOMOTION_WALK, Permission.ACTUATORS_WRITE_LEGS],
                         app_id="com.test.tournee", required_places=["quai 3"],
                         quota=ResourceQuota(max_runtime_s=120.0))

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        m = {"lieux": sorted(p.name for p in ctx.places()),
             "zones": sorted(z.name for z in ctx.zones())}
        m["interdit"] = ctx.go_to(2.0, 0.0, timeout_s=5).code
        try:
            ctx.go_to("nulle part", timeout_s=5)
            m["inconnu"] = "accepté"
        except ValueError as exc:
            m["inconnu"] = type(exc).__name__
        m["quai"] = ctx.go_to("quai 3", timeout_s=60).status
        loc = ctx.where_am_i()
        m["ici"] = [loc.nearest_place, round(loc.x, 1)]
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0, metrics=m)


with AppTestHarness(AppTournee(), site_map=carte, time_scale=ECHELLE) as h:
    m = h.run().metrics
    check("places(), zones() à travers le pont", m.get("lieux") == ["bureau", "quai 3", "station"]
          and m.get("zones") == ["allée", "hall", "local électrique"], str(m))
    check("go_to interdit : FORBIDDEN_ZONE ; lieu inconnu : InvalidArgumentError",
          m.get("interdit") == "FORBIDDEN_ZONE" and m.get("inconnu") == "InvalidArgumentError",
          str(m))
    check("go_to(\"quai 3\") : ARRIVED ; where_am_i : au quai 3", m.get("quai") == "ARRIVED"
          and m.get("ici") == ["quai 3", 4.0], str(m))

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
