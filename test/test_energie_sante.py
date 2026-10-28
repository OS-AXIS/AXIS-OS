"""
Santé matérielle et énergie : protection thermique (JointSafetyGuard, plus
stricte : temperature_max, bridage, NaN = danger), service de santé
(statuts, mesures périmées), API battery / motors_health / thermal_status /
on_thermal_change, arrêt sûr sous température critique ; retour à la station
de charge sous un seuil de batterie (arrêt coopératif puis forcé de la
mission), arrêt sur place sous le seuil critique.
"""
import contextlib
import io
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.types import (  # noqa: E402
    ActuatorCommand, ActuatorFeedback, ActuatorState, ControlMode, JointLimits,
)
from axis_os.hal.safety.limits import JointSafetyGuard, SafetyErrorKind  # noqa: E402
from axis_os.services.energy import EnergyService  # noqa: E402
from axis_os.services.health import HardwareHealthService  # noqa: E402
from axis_os.services.navigation.site_map import FORBIDDEN, STATION, Place, SiteMap, Zone  # noqa: E402
from axis_os.store.sdk.app_base import (  # noqa: E402
    MissionResult, PermissionError, RobotApp, RobotContext,
)
from axis_os.store.sdk.manifest import (  # noqa: E402
    AppCategory, AppManifest, Permission, ResourceQuota,
)
from axis_os.store.sdk.results import BatteryInfo, InvalidArgumentError  # noqa: E402
from axis_os.testing import SimulatedRobot  # noqa: E402

FAIL = 0
ECHELLE = 2.0
GENOU = "left_knee_joint"


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


def attendre(cond, delai):
    fin = time.monotonic() + delai
    while time.monotonic() < fin:
        if cond():
            return True
        time.sleep(0.05)
    return cond()


def retour(temp, ts=None):
    return ActuatorFeedback(joint_id="j", state=ActuatorState.ACTIVE, position=0.0, velocity=0.0,
                            torque=0.0, current=0.0, temperature=temp, voltage=48.0,
                            **({} if ts is None else {"timestamp_ns": ts}))


def limites(tmax=80.0):
    return JointLimits(pos_min=-1.0, pos_max=1.0, vel_max=10.0, torque_max=50.0,
                       current_max=15.0, temperature_max=tmax)


def commande(v=None, couple=None):
    return ActuatorCommand(mode=ControlMode.POSITION, position=0.0, velocity=v, torque=couple)


section("1. JointSafetyGuard : protection thermique, seulement plus stricte")
with contextlib.redirect_stdout(io.StringIO()):
    g = JointSafetyGuard("j", limites(70.0))
    ok70, v70 = g.validate_command(commande(), retour(71.0))
    g95 = JointSafetyGuard("j", limites(95.0))
    ok85, _ = g95.validate_command(commande(), retour(85.0))
    g80 = JointSafetyGuard("j", limites())
    ok_nan, v_nan = g80.validate_command(commande(), retour(math.nan))
    ok_froid, _ = g80.validate_command(commande(v=10.0, couple=50.0), retour(64.0))
    ok_vite, v_vite = g80.validate_command(commande(v=8.0), retour(72.5))
    ok_lent, _ = g80.validate_command(commande(v=7.0), retour(72.5))
    ok_couple, v_couple = g80.validate_command(commande(couple=40.0), retour(72.5))
    sante = g.check_feedback_health(retour(75.0))
check("temperature_max = 70 °C : refusée à 71 °C (avant : seuil fixe 80 °C)",
      not ok70 and v70.rule == "TEMPERATURE_CRITICAL" and v70.limit == 70.0)
check("temperature_max = 95 °C : le seuil reste 80 °C (jamais plus permissif)", not ok85)
check("Température NaN : refusée (avant : ignorée)", not ok_nan
      and v_nan.rule == "TEMPERATURE_UNKNOWN" and v_nan.kind is SafetyErrorKind.INVALID_VALUE)
check("64 °C : pleine vitesse et plein couple permis", ok_froid)
check("72,5 °C (bridage à 75 %) : 8 rad/s refusés, 7 permis",
      not ok_vite and v_vite.rule == "THERMAL_DERATING" and abs(v_vite.limit - 7.5) < 1e-9
      and ok_lent)
check("72,5 °C : 40 N·m refusés (37,5 permis au plus)", not ok_couple
      and v_couple.kind is SafetyErrorKind.THERMAL_DERATING)
check("check_feedback_health suit temperature_max", any(
    v.rule == "TEMPERATURE_CRITICAL" and v.limit == 70.0 for v in sante))


section("2. Service de santé : statuts, mesure absente ou périmée")


class FauxHAL:
    def __init__(self, mesure=True):
        self.mesure, self.temp, self.ts = mesure, 30.0, None

    def measures_motor_temperature(self):
        return self.mesure

    def get_joint_definitions(self):
        from types import SimpleNamespace
        return [SimpleNamespace(joint_id="j", limits=limites())]

    def get_full_state(self):
        return {"j": retour(self.temp, self.ts)}

    def get_battery(self):
        return None


horloge = [0.0]
hal = FauxHAL()
s = HardwareHealthService(hal, clock=lambda: horloge[0])
vus = []
s.add_listener(lambda a, n: vus.append(n))


def pas(temp=None, dt=0.5, ts=None):
    if temp is not None:
        hal.temp = temp
    hal.ts = ts
    horloge[0] += dt
    return s.tick()


check("Jamais tické : critique, marche à 0 (un statut figé n'est pas une mesure)",
      s.thermal_status() == "critical" and s.facteur_marche() == 0.0)
check("30 °C : normal", pas(30.0) == "normal" and s.facteur_marche() == 1.0)
check("60 °C : chaud, rien n'est réduit", pas(60.0) == "hot" and s.facteur_marche() == 1.0)
check("72,5 °C : bridé, marche à 75 %", pas(72.5) == "throttled"
      and abs(s.facteur_marche() - 0.75) < 1e-9)
check("80 °C : critique, plus de marche", pas(80.0) == "critical" and s.facteur_marche() == 0.0)
check("NaN : critique", pas(math.nan) == "critical")
check("Mesure vieille de 5 s : critique", pas(30.0, ts=time.time_ns() - 5 * 10**9) == "critical")
pas(30.0)
horloge[0] += 2.0
check("Service plus tické depuis 2 s : critique", s.thermal_status() == "critical"
      and s.facteur_marche() == 0.0)
check("Chaque changement est signalé aux écouteurs de l'OS",
      vus[:5] == ["normal", "hot", "throttled", "critical", "normal"], str(vus))
s2 = HardwareHealthService(FauxHAL(mesure=False), clock=lambda: horloge[0])
s2.tick()
check("Driver qui ne mesure pas la température : « unknown », rien d'inventé",
      s2.thermal_status() == "unknown" and s2.motors()[0].temperature_c is None
      and s2.motors()[0].margin_c is None and s2.facteur_marche() == 1.0)


def manifeste(perms, app_id="com.test.sante", **kw):
    return AppManifest(app_id=app_id, name="Santé", version="1.0.0", author="T",
                       author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
                       permissions=list(perms), **kw)


TOUT = [Permission.HARDWARE_HEALTH, Permission.SYSTEM_INFO, Permission.LOCOMOTION_WALK,
        Permission.ACTUATORS_WRITE_LEGS]


def contexte(robot, perms=None, sante=True):
    ctx = RobotContext.for_testing(robot.hal, robot.watchdog, manifeste(TOUT if perms is None
                                                                        else perms))
    ctx._set_clock(robot.clock)
    ctx._attach_services(locomotion=robot.locomotion, navigation=robot.navigation,
                         health=robot.health if sante else None)
    return ctx


section("3. API du SDK : battery, motors_health, thermal_status, on_thermal_change")
with SimulatedRobot(time_scale=ECHELLE, battery_percent=64.0) as robot:
    ctx = contexte(robot)
    b = ctx.battery()
    check("battery() : charge et charge en cours mesurées ; tension, courant, température, "
          "santé, cycles inconnus (None)", b.known and abs(b.percent - 64.0) < 1
          and b.charging is False and b.voltage is None and b.current_a is None
          and b.temperature_c is None and b.health_percent is None and b.cycles is None, str(b))
    time.sleep(0.5)
    m = {x.joint: x for x in ctx.motors_health()}
    check("motors_health() : température, seuil critique et marge par articulation",
          GENOU in m and m[GENOU].temperature_c == 25.0 and m[GENOU].critical_c == 80.0
          and m[GENOU].margin_c == 55.0 and m[GENOU].status == "normal", str(m.get(GENOU)))
    check("thermal_status() : normal", ctx.thermal_status() == "normal")
    changements = []
    check("on_thermal_change : abonné", ctx.on_thermal_change(changements.append))
    robot.set_motor_temperature(GENOU, 70.0)
    check("Moteur à 70 °C : « throttled » signalé à l'app",
          attendre(lambda: "throttled" in changements, 5) and ctx.thermal_status() == "throttled",
          str(changements))
    st = ctx.safety_status()
    check("safety_status : statut thermique et processeur (None si non mesuré)",
          st.thermal == "throttled" and "cpu_load_percent" in st.compute
          and "gpu_present" in st.compute, str(st))
    sans = contexte(robot, [Permission.SYSTEM_INFO])
    for nom, f in (("battery", sans.battery), ("motors_health", sans.motors_health),
                   ("thermal_status", sans.thermal_status),
                   ("on_thermal_change", lambda: sans.on_thermal_change(print))):
        check(f"{nom} sans HARDWARE_HEALTH : PermissionError", leve(PermissionError, f))
    check("on_thermal_change sans fonction : InvalidArgumentError",
          leve(InvalidArgumentError, ctx.on_thermal_change, 42))
    silencieux(robot.hal.emergency_stop_all)
    check("Sous arrêt d'urgence : lectures servies", ctx.battery().known
          and ctx.thermal_status() == "throttled" and len(ctx.motors_health()) > 0)
with SimulatedRobot(time_scale=ECHELLE) as robot:
    ctx = contexte(robot, sante=False)
    check("Sans service de santé : [], « unknown », pas d'abonnement",
          ctx.motors_health() == [] and ctx.thermal_status() == "unknown"
          and silencieux(ctx.on_thermal_change, print) is False)
    check("Batterie non mesurée : known=False, rien d'inventé",
          contexte(robot).battery() == BatteryInfo(known=False))


section("4. Marche bridée, puis arrêt sûr sous température critique")
with SimulatedRobot(time_scale=ECHELLE) as robot:
    ctx = contexte(robot)
    robot.set_motor_temperature(GENOU, 72.5)
    time.sleep(0.5)
    x0, t0 = robot.x, robot.clock()
    while robot.clock() - t0 < 2.0:
        ctx.walk(0.4, 0.0, 0.0)
        time.sleep(0.05)
    v = (robot.x - x0) / (robot.clock() - t0)
    check("72,5 °C : 0,4 m/s demandés, au plus 75 % parcourus", 0.0 < v <= 0.4 * 0.75 + 0.02,
          f"{v:.3f} m/s")
    robot.set_motor_temperature(GENOU, 82.0)
    time.sleep(0.5)
    x0 = robot.x
    for _ in range(20):
        ctx.walk(0.4, 0.0, 0.0)
        time.sleep(0.05)
    check("82 °C : plus aucune marche", abs(robot.x - x0) < 0.01, f"{robot.x - x0:.3f} m")
    st = silencieux(robot.raw_hal.send_joint_commands,
                    {GENOU: ActuatorCommand(mode=ControlMode.POSITION, position=0.1)})
    check("Garde-fou : plus aucune commande sur l'articulation trop chaude",
          st[GENOU].name == "ERROR_LIMIT", str(st))


class AppTetue(RobotApp):
    """Marche sans jamais regarder should_stop()."""

    def manifest(self):
        return manifeste([Permission.LOCOMOTION_WALK, Permission.ACTUATORS_WRITE_LEGS],
                         app_id="com.test.tetue", quota=ResourceQuota(max_runtime_s=120.0))

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        while True:
            ctx.walk(0.15, 0.0, 0.3)
            ctx.sleep(0.1)


def lancer(rt, app_id):
    import threading
    sortie = {}
    fil = threading.Thread(target=lambda: sortie.setdefault(
        "r", silencieux(rt.run_mission, app_id, {})), daemon=True)
    fil.start()
    return fil, sortie


with SimulatedRobot(time_scale=ECHELLE) as robot:
    rt = robot.make_runtime()
    rt.STOP_GRACE_S = 1.0
    silencieux(rt.install, AppTetue())
    silencieux(rt.start_app, "com.test.tetue")
    fil, sortie = lancer(rt, "com.test.tetue")
    attendre(lambda: rt.running_apps() == ["com.test.tetue"], 10)
    time.sleep(1.0)
    robot.set_motor_temperature(GENOU, 85.0)
    fil.join(15)
    r = sortie.get("r")
    check("Température critique : l'app têtue est arrêtée de force par l'OS",
          r is not None and not r.success and "Arrêt forcé par l'OS : température moteur "
          "critique" in (r.error_message or ""), str(r))
    recues = robot.notification_sink.delivered
    check("L'opérateur est prévenu (URGENT, par l'OS, articulation nommée)",
          any(n.source == "os" and n.priority == "urgent" and GENOU in n.message
              for n in recues), str(recues))
    r2 = silencieux(rt.run_mission, "com.test.tetue", {})
    check("Nouvelle mission refusée tant que c'est critique", not r2.success
          and "température moteur critique" in r2.error_message)
    robot.set_motor_temperature(GENOU, 40.0)
    attendre(lambda: robot.health.thermal_status() == "normal", 5)
    check("Refroidi : missions de nouveau permises", rt.mission_blocked_reason() is None)


section("5. Batterie faible : retour à la station, même contre une app têtue")
check("Carte : une station à 0,6 m d'une zone interdite (inatteignable) est refusée",
      leve(ValueError, SiteMap, [Place("station", -2.0, 0.0, kind=STATION)],
           [Zone("local", FORBIDDEN, ((-1.4, -0.5), (-0.6, -0.5), (-0.6, 0.5), (-1.4, 0.5)))]))
INTERDITE = Zone("local", FORBIDDEN, ((-1.4, -0.5), (-0.6, -0.5), (-0.6, 0.5), (-1.4, 0.5)))
carte = SiteMap([Place("station", -2.4, 0.0, 0.0, kind=STATION)], [INTERDITE])
with SimulatedRobot(time_scale=ECHELLE, site_map=carte, battery_percent=21.0,
                    battery_drain_per_s=0.4, charge_per_s=3.0) as robot:
    rt = robot.make_runtime()
    energie = EnergyService(rt, robot.navigation, robot.locomotion, robot.site_map,
                            follow=robot.follow_service, clock=robot.clock, grace_s=1.0)
    silencieux(rt.install, AppTetue())
    silencieux(rt.start_app, "com.test.tetue")
    energie.start(0.1)
    fil, sortie = lancer(rt, "com.test.tetue")
    d_min, etats = math.inf, set()
    fin = time.monotonic() + 40
    while time.monotonic() < fin and not ("charging" in etats and energie.state == "normal"):
        d_min = min(d_min, INTERDITE.distance(robot.x, robot.y))
        etats.add(energie.state)
        time.sleep(0.02)
    fil.join(10)
    r = sortie.get("r")
    check("Sous 20 % : la mission est arrêtée (coopératif puis forcé)", r is not None
          and not r.success and "batterie faible" in (r.error_message or ""), str(r))
    check("Le robot va à la station, charge, puis reprend (normal au-dessus de 30 %)",
          {"returning", "charging"} <= etats and energie.state == "normal"
          and robot.battery_percent >= 30.0 and math.hypot(robot.x + 2.4, robot.y) < 0.3,
          f"{etats} {energie.state} {robot.battery_percent} ({robot.x:.2f}, {robot.y:.2f})")
    check("En route, jamais dans la zone interdite", d_min > 0.0, f"{d_min:.3f} m")
    check("Opérateur prévenu (AVERTISSEMENT, par l'OS)", any(
        n.source == "os" and n.title == "Batterie faible" for n in robot.notification_sink.delivered))
    energie.stop()

with SimulatedRobot(time_scale=ECHELLE, site_map=carte, battery_percent=21.0) as robot:
    rt = robot.make_runtime()
    energie = EnergyService(rt, robot.navigation, robot.locomotion, robot.site_map,
                            clock=robot.clock)
    silencieux(rt.install, AppTetue())
    silencieux(rt.start_app, "com.test.tetue")
    energie.tick()
    robot.battery_percent = 19.0
    time.sleep(0.2)
    energie.tick()
    r = silencieux(rt.run_mission, "com.test.tetue", {})
    check("Pendant le retour : mission refusée", energie.state == "returning"
          and not r.success and "batterie faible" in r.error_message, energie.state)
    check("… et démarrage d'app refusé", silencieux(rt.start_app, "com.test.tetue") is False)
    for _ in range(3):
        time.sleep(0.25)
        energie.tick()
    check("go_to de l'OS vers la station", robot.navigation.etat.name == "EN_ROUTE")
    silencieux(robot.hal.emergency_stop_all)
    x0 = robot.x
    for _ in range(10):
        time.sleep(0.1)
        energie.tick()
    check("Arrêt d'urgence : plus aucun mouvement", abs(robot.x - x0) < 0.02)

with SimulatedRobot(time_scale=ECHELLE, site_map=carte, battery_percent=9.0,
                    battery_drain_per_s=0.5) as robot:
    rt = robot.make_runtime()
    energie = EnergyService(rt, robot.navigation, robot.locomotion, robot.site_map,
                            clock=robot.clock)
    silencieux(rt.install, AppTetue())
    silencieux(rt.start_app, "com.test.tetue")
    energie.start(0.1)
    fil, sortie = lancer(rt, "com.test.tetue")
    attendre(lambda: energie.state == "critical", 15)
    fil.join(10)
    x0 = robot.x
    time.sleep(1.0)
    check("Sous 8 % : arrêt sur place, app arrêtée, URGENT", energie.state == "critical"
          and sortie.get("r") is not None and not sortie["r"].success
          and abs(robot.x - x0) < 0.01 and any(n.priority == "urgent" and n.source == "os"
                                                for n in robot.notification_sink.delivered))
    energie.stop()

with SimulatedRobot(time_scale=ECHELLE, battery_percent=15.0) as robot:
    rt = robot.make_runtime()
    energie = EnergyService(rt, robot.navigation, robot.locomotion, robot.site_map,
                            clock=robot.clock)
    energie.tick()
    check("Pas de station sur la carte : « no_station », missions bloquées, URGENT",
          energie.state == "no_station" and rt.mission_blocked_reason() is not None
          and any(n.priority == "urgent" for n in robot.notification_sink.delivered))
with SimulatedRobot(time_scale=ECHELLE) as robot:
    rt = robot.make_runtime()
    energie = EnergyService(rt, robot.navigation, robot.locomotion, robot.site_map,
                            clock=robot.clock)
    energie.tick()
    check("Batterie non mesurée : « unknown », rien inventé, rien bloqué",
          energie.state == "unknown" and rt.mission_blocked_reason() is None)
    for seuils in ((20, 30), (3, 2), (95, 10), (math.nan, 5)):
        check(f"Seuils {seuils} refusés", leve(ValueError, energie.set_thresholds, *seuils))

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
