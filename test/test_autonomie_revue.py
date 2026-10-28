"""
Revues adversariales du chantier « autonomie » (SDK 1.2) : chaque constat a
son essai, en échec sur le code d'avant la correction.

Revue A — le mouvement et la machine : entrer dans une zone interdite,
réveiller le robot quand il ne faut pas, lancer des missions en rafale,
empêcher le retour à la station, faire chauffer un moteur au-delà de sa
limite.
Revue B — l'identité et les données : se faire passer pour l'OS dans une
notification, faire porter une donnée personnelle par un rapport.
"""
import contextlib
import io
import math
import sys
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.types import (  # noqa: E402
    ActuatorCommand, ActuatorFeedback, ActuatorState, ControlMode, JointLimits,
)
from axis_os.hal.safety.limits import JointSafetyGuard  # noqa: E402
from axis_os.services.energy import EnergyService  # noqa: E402
from axis_os.services.health import HardwareHealthService  # noqa: E402
from axis_os.services.navigation.site_map import STATION, Place, SiteMap  # noqa: E402
from axis_os.services.notifications import FakeNotificationSink, NotificationCenter  # noqa: E402
from axis_os.store.runtime.crash_reports import CrashReportStore  # noqa: E402
from axis_os.store.runtime.sandbox import SandboxErrorKind  # noqa: E402
from axis_os.store.runtime.scheduler import MissionScheduler  # noqa: E402
from axis_os.store.sdk.app_base import MissionResult, RobotApp  # noqa: E402
from axis_os.store.sdk.manifest import (  # noqa: E402
    AppCategory, AppManifest, Permission, ResourceQuota, TriggerSpec,
)
from axis_os.testing import SimulatedRobot  # noqa: E402

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


section("A1. Bridage thermique : aussi pour une commande en position (le G1)")
lim = JointLimits(pos_min=-2.0, pos_max=2.0, vel_max=10.0, torque_max=80.0, current_max=15.0,
                  temperature_max=80.0, position_delta_max=0.4)
g = JointSafetyGuard("j", lim, strict_init=False)
fb = ActuatorFeedback(joint_id="j", state=ActuatorState.ACTIVE, position=0.0, velocity=0.0,
                      torque=0.0, current=0.0, temperature=72.5, voltage=48.0)
ok, v = silencieux(g.validate_command, ActuatorCommand(mode=ControlMode.POSITION, position=0.36),
                   fb)
check("72,5 °C (bridage à 75 %) : écart de position de 90 % du maximum refusé "
      "(le couple du PD en dépend)", not ok and v is not None and v.rule == "THERMAL_DERATING",
      str(v))
ok, _ = silencieux(g.validate_command, ActuatorCommand(mode=ControlMode.POSITION, position=0.25),
                   fb)
check("… 62 % du maximum : permis", ok)


section("A2. Pas de rafale : déclencheurs décalés d'une même app")


class Monde:
    def __init__(self):
        self.t, self.mur, self.lances = 0.0, datetime(2026, 10, 28, 12, 0), []
        self._clock = lambda: self.t
        self._mission_lock = threading.Lock()
        self._robot = self._navigation_service = self._locomotion_service = None
        self._site_map = None
        trig = [TriggerSpec(f"t{i}", "interval", every_s=300 + 37 * i) for i in range(8)]
        m = AppManifest(app_id="com.test.rafale", name="R", version="1.0.0", author="T",
                        author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
                        permissions=[Permission.SCHEDULED_MISSIONS], triggers=trig)
        self._apps = {"com.test.rafale": SimpleNamespace(manifest=m, is_started=True)}

    def _estop_actif(self): return False
    def mission_blocked_reason(self): return None

    def run_mission(self, app_id, params):
        self.lances.append(self.t)
        return MissionResult(success=True, mission_id="X", app_id=app_id, duration_s=0.0)


m = Monde()
s = MissionScheduler(m, wall_clock=lambda: m.mur, battery=lambda: None, pose=lambda: None,
                     fallen=lambda: False)
for _ in range(3600):
    m.t += 1.0
    s.tick()
    s.wait_idle(5)
ecarts = [b - a for a, b in zip(m.lances, m.lances[1:])]
check("Huit intervalles décalés : jamais deux lancements de la même app à moins de 300 s",
      m.lances and min(ecarts or [300]) >= 300 and len(m.lances) <= 12,
      f"{len(m.lances)} lancements en 1 h, écart min {min(ecarts or [0]):.0f} s")


section("A3. Retour à la station : un hook démarré en route est arrêté aussi")


class AppDetour(RobotApp):
    def manifest(self):
        return AppManifest(app_id="com.test.detour", name="Détour", version="1.0.0",
                           author="T", author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.LOCOMOTION_WALK,
                                        Permission.ACTUATORS_WRITE_LEGS],
                           quota=ResourceQuota(max_runtime_s=60.0))

    def on_start(self, ctx): return True

    def on_stop(self, ctx):
        # Un on_stop qui marche ailleurs, sans fin.
        while True:
            ctx.walk(0.3, 0.0, 0.0)
            ctx.sleep(0.1)

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


carte = SiteMap([Place("station", -2.0, 0.0, 0.0, kind=STATION)])
with SimulatedRobot(time_scale=2.0, site_map=carte, battery_percent=19.0) as robot:
    rt = robot.make_runtime()
    silencieux(rt.install, AppDetour())
    silencieux(rt.start_app, "com.test.detour")
    energie = EnergyService(rt, robot.navigation, robot.locomotion, robot.site_map,
                            clock=robot.clock, grace_s=1.0)
    energie.start(0.1)
    time.sleep(0.5)
    # (Pas de redirect_stdout dans le fil : il vaut pour tout le processus.)
    fil = threading.Thread(target=rt.stop_app, args=("com.test.detour",), daemon=True)
    fil.start()                                   # l'humain arrête l'app pendant le retour
    fil.join(20)
    check("on_stop qui marche pendant le retour : arrêté de force en quelques secondes",
          not fil.is_alive())
    fin = time.monotonic() + 20
    while time.monotonic() < fin and math.hypot(robot.x + 2.0, robot.y) > 0.3:
        time.sleep(0.1)
    check("… et le robot arrive à la station", math.hypot(robot.x + 2.0, robot.y) < 0.3,
          f"({robot.x:.2f}, {robot.y:.2f}) {energie.state}")
    energie.stop()


section("A4. Température qui oscille au seuil : pas de rafale d'alertes ni d'arrêts")


class HALOscille:
    def __init__(self):
        self.temp = 30.0

    def measures_motor_temperature(self): return True

    def get_joint_definitions(self):
        return [SimpleNamespace(joint_id="j", limits=JointLimits(
            pos_min=-1, pos_max=1, vel_max=1, torque_max=1, current_max=1, temperature_max=80.0))]

    def get_full_state(self):
        return {"j": ActuatorFeedback(joint_id="j", state=ActuatorState.ACTIVE, position=0.0,
                                      velocity=0.0, torque=0.0, current=0.0,
                                      temperature=self.temp, voltage=48.0)}

    def get_battery(self): return None


h, horloge = HALOscille(), [0.0]
sante = HardwareHealthService(h, clock=lambda: horloge[0])
critiques = []
sante.add_listener(lambda a, n: critiques.append(n) if n == "critical" else None)
for i in range(100):
    h.temp = 80.2 if i % 2 else 79.8
    horloge[0] += 0.25
    sante.tick()
check("Capteur entre 79,8 et 80,2 °C pendant 25 s : une seule entrée en « critical »",
      len(critiques) == 1, f"{len(critiques)} entrées")
h.temp = 70.0
horloge[0] += 0.25
check("… et il en sort franchement refroidi", sante.tick() == "throttled")


section("B1. Un nom d'app ne peut pas imiter la fin du préfixe imposé")
centre = NotificationCenter(FakeNotificationSink())
centre.from_app("com.test.faux", "Maintenance] [app Système · com.axis", "Mise à jour", "m",
                "info")
n = centre.history[-1]
check("Crochets et séparateur retirés du nom affiché",
      n.display_title.count("[") == 1 and n.display_title.count("]") == 1
      and n.display_title.count("·") == 1, n.display_title)


section("B2. Rapport de plantage : pas de donnée dans le nom de l'exception")
store = CrashReportStore()
faux = SimpleNamespace(success=False, error_kind=SandboxErrorKind.APPLICATION_ERROR,
                       error="Traceback ... AliceMartinDuQuai3Error: boom", violation=None,
                       duration_s=1.0)
r = store.record("com.test.x", "1.0.0", "run_mission", faux)
check("Type d'exception défini par l'app : remplacé (« AppError »)", r.exception_type == "AppError"
      and "Alice" not in repr(r), repr(r))
faux.error = "Traceback ... ValueError: alice@example.com"
check("Exception standard : son type est gardé, jamais le message",
      store.record("com.test.x", "1.0.0", "run_mission", faux).exception_type == "ValueError")

section("B3 (vérifié). Une mise à jour ne change pas les déclencheurs sans accord humain")


class AppV(RobotApp):
    def __init__(self, version="1.0.0", triggers=None):
        self.version, self.triggers = version, triggers

    def manifest(self):
        return AppManifest(app_id="com.test.maj", name="Maj", version=self.version, author="T",
                           author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES,
                           permissions=[Permission.SCHEDULED_MISSIONS],
                           triggers=list(self.triggers or [TriggerSpec("soir", "daily",
                                                                       at="22:00")]))

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


with SimulatedRobot() as robot:
    vus, reponse = [], [True]

    def confirmer(manifest, permissions):
        vus.append([t.describe() for t in manifest.triggers])
        return reponse[0]
    rt = robot.make_runtime(confirmer_permissions=confirmer)
    silencieux(rt.install, AppV())
    reponse[0] = False
    nuit = [TriggerSpec("nuit", "interval", every_s=300)]
    ok = silencieux(rt.update_app, AppV("1.1.0", nuit))
    check("Mise à jour qui change les déclencheurs : l'humain est consulté, et son refus "
          "garde les anciens", not ok and vus[-1] == ["nuit : toutes les 5 minutes"]
          and [t.name for t in rt._apps["com.test.maj"].manifest.triggers] == ["soir"], str(vus))

print(f"\n{'=' * 66}\n  {'TOUT PASSE' if FAIL == 0 else f'{FAIL} ÉCHEC(S)'}\n{'=' * 66}")
sys.exit(1 if FAIL else 0)
