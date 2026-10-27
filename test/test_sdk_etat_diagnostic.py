"""
SDK, partie 2 : état, capacités, diagnostic et refus structurés.

Vérifié ici, en direct (RobotContext) et depuis le processus de l'app
(pont JSON) :
  1. robot_info() : modèle, versions, profil, capacités et leur état,
     services, limites, batterie inconnue quand le driver ne la mesure pas ;
  2. has_capability() : capacité matérielle disponible ou non, service
     présent ou non ; nom inconnu ou de mauvais type : InvalidArgumentError
     (code INVALID_ARGUMENT), de part et d'autre du pont ;
  3. safety_status() : arrêt d'urgence, gardien de proximité, chute, quotas
     restants (durée du hook, mémoire persistante) ;
  4. permission SYSTEM_INFO absente : PermissionError, code
     PERMISSION_DENIED ;
  5. pendant un arrêt d'urgence : ces lectures restent servies, et disent
     l'arrêt ;
  6. capacité dégradée ou absente, batterie mesurée par le driver.
"""
import contextlib
import io
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.capabilities import CapabilitySet, CapabilityStatus, RobotCapability  # noqa: E402
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer  # noqa: E402
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL  # noqa: E402
from axis_os.hal.core.types import BatteryState  # noqa: E402
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL  # noqa: E402
from axis_os.hal.watchdog.system import WatchdogSystem  # noqa: E402
from axis_os.services.locomotion.proximite import Verdict  # noqa: E402
from axis_os.services.locomotion.service import LocomotionState, Twist  # noqa: E402
from axis_os.services.memory.service import MemoryService  # noqa: E402
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION  # noqa: E402
from axis_os.store.runtime.profil import Profil  # noqa: E402
from axis_os.store.sdk.app_base import (  # noqa: E402
    EstopActiveError, MissionResult, PermissionError as SdkPermissionError, RobotApp,
    RobotContext,
)
from axis_os.store.sdk.manifest import AppCategory, AppManifest, Permission  # noqa: E402
from axis_os.store.sdk.results import (  # noqa: E402
    AxisRefusal, InvalidArgumentError, RefusalCode, RobotInfo, SafetyStatus,
)
from axis_os.store.sdk.version import API_VERSION  # noqa: E402

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


def robot(hal_cls=UnitreeG1HAL, nom="g1_diag"):
    with contextlib.redirect_stdout(io.StringIO()):
        r = NormalizedRobotHAL(hal_cls(nom), build_g1_normalizer())
        r.initialize_all()
    return r


def manifeste(perms, app_id="com.test.diag"):
    return AppManifest(app_id=app_id, name="Diag", version="1.0.0", author="T",
                       author_email="t@t.com", description="d",
                       category=AppCategory.UTILITIES, permissions=list(perms))


class LocomotionFactice:
    """Ce que le SDK lit d'un service de locomotion : vitesses maximales,
    gardien, dernier verdict, état."""
    def __init__(self, verdict=None, gardien=True, etat=LocomotionState.RUNNING):
        self.max_twist = Twist(0.5, 0.2, 0.8)
        self.is_guarded = gardien
        self.dernier_verdict = verdict
        self.state = etat


section("1. robot_info() en direct")
ROBOT = robot()
WD = WatchdogSystem(ROBOT)
ctx = RobotContext.for_testing(ROBOT, WD, manifeste([Permission.SYSTEM_INFO]))
ctx._attach_services(locomotion=LocomotionFactice())
ctx._set_os_info("DEVELOPPEMENT")
info = silencieux(ctx.robot_info)
check("Un RobotInfo, avec le modèle et le fabricant déclarés par le driver",
      isinstance(info, RobotInfo) and info.model == "G1" and "Unitree" in info.manufacturer,
      str(info))
import axis_os  # noqa: E402
check("Versions de l'OS et de l'API du SDK", info.os_version == axis_os.__version__
      and info.sdk_api_version == API_VERSION, f"{info.os_version} {info.sdk_api_version}")
check("Profil en anglais : « development »", info.profile == "development", info.profile)
check("Capacités matérielles du G1 avec leur état", info.capabilities.get("legs") == "available"
      and info.capabilities.get("wheels") == "absent" and info.capabilities.get("lidar") == "absent",
      str(info.capabilities))
check("Services : locomotion présente, perception absente",
      info.services.get("locomotion") == "available" and info.services.get("perception") == "absent",
      str(info.services))
check("Limites utiles : vitesse de marche, rotation, rapport de vitesse du manifeste",
      info.limits["max_walk_speed_mps"] == 0.5 and info.limits["max_turn_rate_radps"] == 0.8
      and info.limits["max_velocity_ratio"] == 0.8, str(info.limits))
check("Batterie que le driver ne mesure pas : inconnue, jamais inventée",
      info.battery.known is False and info.battery.percent is None
      and info.services.get("battery") == "absent", str(info.battery))

section("2. has_capability()")
check("« legs » (disponible) : vrai ; « wheels » (absente) : faux",
      silencieux(ctx.has_capability, "legs") is True
      and silencieux(ctx.has_capability, "wheels") is False)
check("Service présent (« locomotion ») : vrai ; absent (« perception ») : faux",
      silencieux(ctx.has_capability, "locomotion") is True
      and silencieux(ctx.has_capability, "perception") is False)
check("Casse et espaces sans importance (« LEGS »)", silencieux(ctx.has_capability, " LEGS ") is True)
check("Nom inconnu : InvalidArgumentError (code INVALID_ARGUMENT)",
      leve(RefusalCode.INVALID_ARGUMENT, ctx.has_capability, "lazer"))
check("Pas un texte : InvalidArgumentError", leve(RefusalCode.INVALID_ARGUMENT, ctx.has_capability, 3))
check("Texte vide : InvalidArgumentError", leve(RefusalCode.INVALID_ARGUMENT, ctx.has_capability, ""))
check("InvalidArgumentError reste une ValueError", issubclass(InvalidArgumentError, ValueError))

section("3. safety_status()")
st = silencieux(ctx.safety_status)
check("Rien à signaler : pas d'arrêt d'urgence, pas de chute, gardien sans mesure encore",
      isinstance(st, SafetyStatus) and st.estop_active is False and st.fall_detected is False
      and st.proximity == "unavailable", str(st))
for facteur, attendu in ((1.0, "clear"), (0.4, "slowing"), (0.0, "stopped")):
    ctx._attach_services(locomotion=LocomotionFactice(Verdict(facteur, 0.9, "personne devant")))
    st = silencieux(ctx.safety_status)
    check(f"Gardien, facteur {facteur} : « {attendu} », avec la distance et la raison",
          st.proximity == attendu and st.proximity_distance_m == 0.9
          and "personne" in st.proximity_reason, str(st))
ctx._attach_services(locomotion=LocomotionFactice(gardien=False))
check("Locomotion sans gardien : « unavailable »",
      silencieux(ctx.safety_status).proximity == "unavailable")
ctx._attach_services(locomotion=LocomotionFactice(etat=LocomotionState.FALLEN))
check("Chute détectée par la locomotion", silencieux(ctx.safety_status).fall_detected is True)
ctx._attach_services()
check("Robot sans locomotion : chute inconnue (None)",
      silencieux(ctx.safety_status).fall_detected is None)
memoire = MemoryService(Path(tempfile.mkdtemp(prefix="axis_diag_")))
silencieux(memoire.start)
ctx_m = RobotContext.for_testing(ROBOT, WD, manifeste([Permission.SYSTEM_INFO,
                                                       Permission.STORAGE_PERSISTENT]))
ctx_m._attach_services(memory=memoire)
avant = silencieux(ctx_m.safety_status)
silencieux(ctx_m.remember, "k", "v" * 100)
apres = silencieux(ctx_m.safety_status)
check("Mémoire persistante : clés et octets restants, qui baissent après remember()",
      avant.storage_keys_remaining == MemoryService.MAX_CLES_PAR_APP
      and apres.storage_keys_remaining == MemoryService.MAX_CLES_PAR_APP - 1
      and apres.storage_bytes_remaining < avant.storage_bytes_remaining,
      f"{avant} {apres}")

section("4. Permission SYSTEM_INFO absente")
sans = RobotContext.for_testing(ROBOT, WD, manifeste([]))
for nom, appel in (("robot_info", lambda: sans.robot_info()),
                   ("has_capability", lambda: sans.has_capability("legs")),
                   ("safety_status", lambda: sans.safety_status())):
    try:
        silencieux(appel)
        check(f"{nom} sans SYSTEM_INFO : refusé", False)
    except SdkPermissionError as exc:
        check(f"{nom} sans SYSTEM_INFO : PermissionError, code PERMISSION_DENIED",
              exc.code is RefusalCode.PERMISSION_DENIED)


class AppDiag(RobotApp):
    """Lit l'état du robot et rapporte ce qu'elle a reçu, types compris ;
    avec « apres_arret », attend l'arrêt d'urgence puis relit."""
    PERMISSIONS = [Permission.SYSTEM_INFO, Permission.STORAGE_PERSISTENT]
    APP_ID = "com.test.diag_app"

    def manifest(self):
        return AppManifest(app_id=self.APP_ID, name="Diag", version="1.0.0", author="T",
                           author_email="t@t.com", description="d",
                           category=AppCategory.UTILITIES, permissions=list(self.PERMISSIONS))

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        from axis_os.store.sdk.results import AxisRefusal, RobotInfo, SafetyStatus
        m = {}
        if params.get("apres_arret"):
            import time as _t
            fin = _t.monotonic() + 10.0
            while _t.monotonic() < fin:
                try:
                    ctx.now()
                except AxisRefusal as exc:
                    m["code_arret"] = exc.code.value
                    break
                _t.sleep(0.01)
        try:
            info = ctx.robot_info()
            st = ctx.safety_status()
            m.update(info_type=isinstance(info, RobotInfo), modele=info.model,
                     profil=info.profile, batterie=info.battery.known,
                     camera=info.capabilities.get("camera"),
                     st_type=isinstance(st, SafetyStatus), estop=st.estop_active,
                     temps=st.time_remaining_s, cles=st.storage_keys_remaining,
                     jambes=ctx.has_capability("legs"))
        except AxisRefusal as exc:
            m["refus"] = exc.code.value
        for nom, arg in (("inconnu", "lazer"), ("type", 12)):
            try:
                ctx.has_capability(arg)
                m[nom] = None
            except AxisRefusal as exc:
                m[nom] = [type(exc).__name__, exc.code.value]
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0, metrics=m)


class AppSansDroit(AppDiag):
    PERMISSIONS = []
    APP_ID = "com.test.diag_sans_droit"


def pile(app, **kw):
    r = robot(nom=f"g1_{app.APP_ID}")
    mem = MemoryService(Path(tempfile.mkdtemp(prefix="axis_diag_rt_")))
    silencieux(mem.start)
    rt = silencieux(PlayStoreRuntime, r, WatchdogSystem(r), memory_service=mem,
                    profil=Profil.SIMULATION, confirmer_permissions=accepter_tout_SIMULATION, **kw)
    silencieux(rt.install, app)
    silencieux(rt.start_app, app.APP_ID)
    return r, rt


section("5. Depuis le processus de l'app (pont JSON)")
r_app, rt = pile(AppDiag())
res = silencieux(rt.run_mission, AppDiag.APP_ID, {})
m = getattr(res, "metrics", {}) or {}
check("RobotInfo et SafetyStatus reconstruits, comme en direct",
      m.get("info_type") is True and m.get("st_type") is True and m.get("modele") == "G1", str(m))
check("Profil « simulation », batterie inconnue, caméra disponible, jambes : vrai",
      m.get("profil") == "simulation" and m.get("batterie") is False
      and m.get("camera") == "available" and m.get("jambes") is True, str(m))
check("Temps restant du hook : positif, au plus le quota de durée (30 s)",
      isinstance(m.get("temps"), float) and 0 < m["temps"] <= 30.0, str(m.get("temps")))
check("Mémoire persistante restante lue depuis l'app",
      m.get("cles") == MemoryService.MAX_CLES_PAR_APP, str(m.get("cles")))
check("Nom inconnu et mauvais type : InvalidArgumentError, code INVALID_ARGUMENT, à travers le pont",
      m.get("inconnu") == ["InvalidArgumentError", "INVALID_ARGUMENT"]
      and m.get("type") == ["InvalidArgumentError", "INVALID_ARGUMENT"], str(m))
r_sans, rt_sans = pile(AppSansDroit())
res = silencieux(rt_sans.run_mission, AppSansDroit.APP_ID, {})
check("Sans SYSTEM_INFO : PermissionError, code PERMISSION_DENIED, à travers le pont",
      (getattr(res, "metrics", {}) or {}).get("refus") == "PERMISSION_DENIED",
      str(getattr(res, "metrics", res)))

section("6. Pendant un arrêt d'urgence")
import threading  # noqa: E402
import time  # noqa: E402
r_arret, rt_arret = pile(AppDiag())
resultat = {}


def lancer():
    with contextlib.redirect_stdout(io.StringIO()):
        resultat["r"] = rt_arret.run_mission(AppDiag.APP_ID, {"apres_arret": True})


fil = threading.Thread(target=lancer)
fil.start()
time.sleep(1.0)
silencieux(r_arret.emergency_stop_all)
fil.join(timeout=30)
if fil.is_alive():
    sys.stdout = sys.__stdout__
m = getattr(resultat.get("r"), "metrics", {}) or {}
check("L'app reçoit l'arrêt (ESTOP_ACTIVE) sur un appel ordinaire",
      m.get("code_arret") == "ESTOP_ACTIVE", str(m))
check("… et lit encore l'état : safety_status().estop_active vrai, robot_info servi",
      m.get("estop") is True and m.get("modele") == "G1", str(m))
ctx_e = RobotContext.for_testing(r_arret, WatchdogSystem(r_arret), manifeste([Permission.SYSTEM_INFO]))
check("En direct aussi : safety_status() sous e-stop", silencieux(ctx_e.safety_status).estop_active)
check("EstopActiveError porte le code ESTOP_ACTIVE",
      EstopActiveError("x").code is RefusalCode.ESTOP_ACTIVE)


class HALDegradee(UnitreeG1HAL):
    """G1 dont la caméra est hors service et qui n'a pas de micro, mais qui
    mesure sa batterie."""
    def get_capabilities(self):
        caps = CapabilitySet.from_iterable([RobotCapability.LEGS, RobotCapability.ARMS,
                                            RobotCapability.CAMERA])
        return caps.set_status(RobotCapability.CAMERA, CapabilityStatus.DEGRADED)

    def get_battery(self):
        return BatteryState(percent=42.5, voltage=50.1, charging=False)


section("7. Capacité dégradée ou absente, batterie mesurée")
r_d = robot(HALDegradee, "g1_degrade")
ctx_d = RobotContext.for_testing(r_d, WatchdogSystem(r_d), manifeste([Permission.SYSTEM_INFO]))
info = silencieux(ctx_d.robot_info)
check("Caméra « degraded », micro « absent »", info.capabilities.get("camera") == "degraded"
      and info.capabilities.get("microphone") == "absent", str(info.capabilities))
check("has_capability : caméra dégradée -> faux (pas utilisable sans réserve), micro -> faux",
      silencieux(ctx_d.has_capability, "camera") is False
      and silencieux(ctx_d.has_capability, "microphone") is False)
check("Batterie mesurée par le driver : connue, 42.5 %, service « battery » présent",
      info.battery.known is True and info.battery.percent == 42.5
      and info.battery.charging is False and silencieux(ctx_d.has_capability, "battery") is True,
      str(info.battery))
check("Robot sans locomotion : limites de marche None",
      info.limits["max_walk_speed_mps"] is None, str(info.limits))

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
sys.exit(0 if FAIL == 0 else 1)
