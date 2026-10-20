"""
AXIS-OS — Tests audit externe, vague 2 (C5, E8, E9, E10, M2, M3, M10, M11)
============================================================================
Complète test_audit_fixes_wave1.py (si présent) / les correctifs déjà
poussés (C1-C4, E1-E7, M8) avec la suite des points trouvés par le
même audit externe (Claude Opus, 2026-09-14) — voir CHANGELOG.md pour
le détail complet par point.
"""

import sys
import time
from unittest.mock import MagicMock
sys.path.insert(0, ".")

from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.drivers.unitree_g1.g1_dds_actuator import G1DDSActuator
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.core.types import (
    JointDefinition, JointLimits, JointType, ActuatorCommand,
    ControlMode, HALStatus,
)
from axis_os.hal.watchdog.system import WatchdogSystem, WatchdogConfig
from axis_os.hal.watchdog.events import AlertKind
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
from axis_os.store.runtime.profil import Profil
from axis_os.store.runtime.hal_bridge import HALBridgeServer, HALCommand
from axis_os.store.sdk.app_base import RobotContext, PermissionError, MissionResult, RobotApp
from axis_os.store.sdk.manifest import AppManifest, AppCategory, PricingModel, Permission
from axis_os.apps.package_sorting.app import PackageSortingApp
import multiprocessing as mp
from dataclasses import asdict

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


def make_robot() -> UnitreeG1HAL:
    r = UnitreeG1HAL("g1_audit_w2")
    r.initialize_all()
    return r


# ══════════════════════════════════════════════════════════════════

section("C5 — borne sur le delta de position (couple réel du PD embarqué)")

definition = JointDefinition(
    joint_id="right_knee_joint", joint_type=JointType.REVOLUTE,
    limits=JointLimits(pos_min=-0.087, pos_max=2.88, vel_max=20.0,
                       torque_max=139.0, current_max=1000.0, temperature_max=80.0),
    parent_link="p", child_link="c", default_control_mode=ControlMode.POSITION,
)
hub = MagicMock()
hub.is_initialized = True


class _FakeMotorState:
    def __init__(self, q):
        self.q, self.dq, self.tau_est = q, 0.0, 0.0
        self.temperature, self.vol = [25.0, 25.0], 48.0


actuator = G1DDSActuator(definition, hub)
check("position_delta_max calculé == torque_max/kp (139/100=1.39)",
      abs(actuator._guard.limits.position_delta_max - 1.39) < 0.001)

hub.read_motor_state.return_value = _FakeMotorState(q=0.0)
r_big = actuator.send_command(ActuatorCommand(mode=ControlMode.POSITION, position=1.5))
check("Saut légal en amplitude mais brutal (1.5 rad depuis 0.0) → ERROR_LIMIT",
      r_big == HALStatus.ERROR_LIMIT)

r_small = actuator.send_command(ActuatorCommand(mode=ControlMode.POSITION, position=1.0))
check("Saut plus petit (1.0 < 1.39) → OK", r_small == HALStatus.OK)

# Rétrocompatibilité : sans feedback, le check est un no-op (pas de crash)
actuator2 = G1DDSActuator(definition, MagicMock(is_initialized=True, read_motor_state=lambda i: None))
r_no_fb = actuator2.send_command(ActuatorCommand(mode=ControlMode.POSITION, position=1.5))
check("Sans feedback disponible, delta-check no-op (pas de crash, comportement dégradé mais sûr)",
      r_no_fb in (HALStatus.OK,))

# ══════════════════════════════════════════════════════════════════

section("E8 — le pont HAL ne meurt plus sur un message malformé")

robot_e8 = NormalizedRobotHAL(UnitreeG1HAL("g1_e8_w2"), build_g1_normalizer())
robot_e8.initialize_all()
wd_e8 = WatchdogSystem(robot_e8)
manifest_e8 = PackageSortingApp().manifest()
ctx_e8 = RobotContext.for_testing(robot_e8, wd_e8, manifest_e8)

cq, rq = mp.Queue(), mp.Queue()
server_e8 = HALBridgeServer(ctx_e8, cq, rq)
server_e8.start()

cq.put({"foo": 1})  # message malformé
time.sleep(0.3)
check("Thread serveur survit à un message malformé", server_e8._thread.is_alive())

cq.put({"action": "get_imu", "payload": {}, "request_id": 1})
time.sleep(0.3)
resp = rq.get(timeout=1.0)
check("Le serveur répond toujours à une vraie requête après le message malformé",
      resp["success"] is True)
server_e8.stop()

# ══════════════════════════════════════════════════════════════════

section("E9 — SandboxRunner(strict=True) refuse de démarrer sans psutil")

import builtins
from axis_os.store.runtime.sandbox import SandboxRunner
_real_import = builtins.__import__


def _fake_import(name, *args, **kwargs):
    if name == "psutil":
        raise ImportError("simulé absent")
    return _real_import(name, *args, **kwargs)


builtins.__import__ = _fake_import
try:
    runner_default = SandboxRunner("com.test.e9")
    check("strict=False (défaut) construit quand même sans psutil",
          runner_default._psutil is None)
    try:
        SandboxRunner("com.test.e9b", strict=True)
        check("strict=True lève RuntimeError sans psutil", False)
    except RuntimeError:
        check("strict=True lève RuntimeError sans psutil", True)
finally:
    builtins.__import__ = _real_import

# ══════════════════════════════════════════════════════════════════

section("E10 — e-stop du Supervisor visible par run_mission()")

robot_e10 = NormalizedRobotHAL(UnitreeG1HAL("g1_e10_w2"), build_g1_normalizer())
robot_e10.initialize_all()
wd_e10 = WatchdogSystem(robot_e10)
runtime_e10 = PlayStoreRuntime(robot_e10, wd_e10, confirmer_permissions=accepter_tout_SIMULATION,
                               profil=Profil.SIMULATION)
runtime_e10.install(PackageSortingApp())
runtime_e10.start_app("com.acme.package_sorting")

check("Avant e-stop : robot.estop_active == False", robot_e10.estop_active is False)

# Simule EXACTEMENT le pattern Supervisor : appel HAL direct, jamais
# via le Watchdog (watchdog.estop_fired reste donc False).
robot_e10.emergency_stop_all()
check("robot.estop_active == True après emergency_stop_all() direct",
      robot_e10.estop_active is True)
check("watchdog.estop_fired reste False (jamais notifié, scénario réaliste)",
      wd_e10.estop_fired is False)

result_e10 = runtime_e10.run_mission("com.acme.package_sorting", params={})
check("Mission refusée malgré watchdog.estop_fired == False",
      result_e10 is not None and result_e10.success is False)

# ══════════════════════════════════════════════════════════════════

section("M2 — facturation respecte pricing_model (ONE_TIME, SUBSCRIPTION, PAY_PER_MISSION)")


class _PricingTestApp(RobotApp):
    def __init__(self, model):
        self._model = model

    def manifest(self):
        return AppManifest(
            app_id=f"com.test.pricing.{self._model.value}", name="T", version="1.0.0",
            author="T", author_email="t@t.com", description="d",
            category=AppCategory.UTILITIES, pricing_model=self._model,
            base_price_eur=1.40, permissions=[Permission.ACTUATORS_WRITE_ARMS],
        )

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass

    def run_mission(self, ctx, params):
        # Bouge RÉELLEMENT le robot — nécessaire depuis le correctif M3
        # (la facturation exige un mouvement réel, pas juste
        # success=True) pour tester M2 sans se heurter à M3.
        ctx.move_arms({"left_elbow_joint": ActuatorCommand(
            mode=ControlMode.POSITION, position=0.1)})
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


robot_m2 = NormalizedRobotHAL(UnitreeG1HAL("g1_m2_w2"), build_g1_normalizer())
robot_m2.initialize_all()
wd_m2 = WatchdogSystem(robot_m2)

for model, expected_bills in [
    (PricingModel.ONE_TIME, 1), (PricingModel.SUBSCRIPTION, 1),
    (PricingModel.PAY_PER_MISSION, 3),
]:
    runtime_m2 = PlayStoreRuntime(robot_m2, wd_m2, confirmer_permissions=accepter_tout_SIMULATION,
                                  profil=Profil.SIMULATION)
    app = _PricingTestApp(model)
    runtime_m2.install(app)
    runtime_m2.start_app(app.manifest().app_id)
    for _ in range(3):
        runtime_m2.run_mission(app.manifest().app_id, params={})
    check(f"{model.name} : {expected_bills} facture(s) sur 3 missions",
          len(runtime_m2._billing) == expected_bills)

# ══════════════════════════════════════════════════════════════════

section("M3 — la facturation exige un vrai mouvement du robot, pas juste success=True")


class _LyingApp(RobotApp):
    def manifest(self):
        return AppManifest(
            app_id="com.attacker.lying_w2", name="T", version="1.0.0",
            author="T", author_email="t@t.com", description="d",
            category=AppCategory.UTILITIES, pricing_model=PricingModel.PAY_PER_MISSION,
            base_price_eur=1.40,
        )

    def on_start(self, ctx): return True
    def on_stop(self, ctx): pass
    def run_mission(self, ctx, params):
        # Ne bouge JAMAIS le robot — ment sur le succès.
        return MissionResult(success=True, mission_id="", app_id="", duration_s=0.0)


robot_m3 = NormalizedRobotHAL(UnitreeG1HAL("g1_m3_w2"), build_g1_normalizer())
robot_m3.initialize_all()
wd_m3 = WatchdogSystem(robot_m3)
runtime_m3 = PlayStoreRuntime(robot_m3, wd_m3, confirmer_permissions=accepter_tout_SIMULATION,
                              profil=Profil.SIMULATION)
liar = _LyingApp()
runtime_m3.install(liar)
runtime_m3.start_app(liar.manifest().app_id)
result_m3 = runtime_m3.run_mission(liar.manifest().app_id, params={})
check("App qui ment sur success=True sans bouger le robot → aucune facture émise",
      result_m3.success is True and len(runtime_m3._billing) == 0)

# Non-régression : une vraie mission qui bouge le robot est facturée normalement.
robot_m3b = NormalizedRobotHAL(UnitreeG1HAL("g1_m3b_w2"), build_g1_normalizer())
robot_m3b.initialize_all()
wd_m3b = WatchdogSystem(robot_m3b)
runtime_m3b = PlayStoreRuntime(robot_m3b, wd_m3b, confirmer_permissions=accepter_tout_SIMULATION,
                               profil=Profil.SIMULATION)
runtime_m3b.install(PackageSortingApp())
runtime_m3b.start_app("com.acme.package_sorting")
result_m3b = runtime_m3b.run_mission("com.acme.package_sorting", params={})
check("Non-régression : une vraie mission (qui bouge le robot) est bien facturée",
      result_m3b is not None and result_m3b.success and len(runtime_m3b._billing) == 1)

# ══════════════════════════════════════════════════════════════════

section("M10 — le watchdog détecte une position hors des vraies limites physiques")

robot_m10 = make_robot()  # UnitreeG1HAL brut — même niveau d'objet que le watchdog attend
act_m10 = robot_m10.get_actuator("left_knee_joint")
cfg_m10 = WatchdogConfig.standard()
cfg_m10.poll_interval_s = 0.01
cfg_m10.confirm_samples = 2
wd_m10 = WatchdogSystem(robot_m10, cfg_m10)
alerts_m10 = []
wd_m10.subscribe(lambda a: alerts_m10.append(a))
wd_m10.start()
time.sleep(0.05)
act_m10._position = 99.0  # bien au-delà de pos_max (~2.88)
time.sleep(0.15)
wd_m10.stop()

breach_alerts = [a for a in alerts_m10 if a.kind == AlertKind.POSITION_LIMIT_BREACH]
check("Alerte POSITION_LIMIT_BREACH émise sur une position hors limites confirmée",
      len(breach_alerts) > 0)

# ══════════════════════════════════════════════════════════════════

section("M11 — une app peut déclencher un e-stop (avec la bonne permission)")

robot_m11 = NormalizedRobotHAL(UnitreeG1HAL("g1_m11_w2"), build_g1_normalizer())
robot_m11.initialize_all()
wd_m11 = WatchdogSystem(robot_m11)

manifest_no_perm = AppManifest(
    app_id="com.test.noperm_w2", name="T", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
)
ctx_no_perm = RobotContext.for_testing(robot_m11, wd_m11, manifest_no_perm)
try:
    ctx_no_perm.emergency_stop()
    check("Sans Permission.EMERGENCY_STOP → refusé", False)
except PermissionError:
    check("Sans Permission.EMERGENCY_STOP → refusé", True)

manifest_with_perm = AppManifest(
    app_id="com.test.withperm_w2", name="T", version="1.0.0", author="T",
    author_email="t@t.com", description="d", category=AppCategory.UTILITIES,
    permissions=[Permission.EMERGENCY_STOP],
)
ctx_with_perm = RobotContext.for_testing(robot_m11, wd_m11, manifest_with_perm)
status_m11 = ctx_with_perm.emergency_stop()
check("Avec la permission → emergency_stop() réussit (HALStatus.OK)",
      status_m11 == HALStatus.OK)
check("robot.estop_active bien positionné (cohérent avec E10)",
      robot_m11.estop_active is True)

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

sys.exit(0 if FAIL == 0 else 1)
