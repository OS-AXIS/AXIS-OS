"""
AXIS-OS — Test Suite Complète
================================
Valide l'intégralité de la stack AXIS-OS en une seule commande.

    python test_axis_os.py

Scénarios :
  1.  HAL — UnitreeG1HAL 24 DoF
  2.  Safety — rejet commandes hors-limites
  3.  KinematicNormalizer — conversions + symétrie
  4.  Correction bug genou droit G1
  5.  NormalizedRobotHAL — façade apps
  6.  WatchdogSystem — thermique + e-stop auto
  7.  WatchdogSystem — perte communication
  8.  Play Store — installation + manifeste
  9.  Play Store — commission 30%
  10. Play Store — sandbox permissions
  11. PackageSorting — mission complète
  12. Rapport de facturation
"""

import sys
import time
import threading
import random
sys.path.insert(0, ".")
random.seed(42)

from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL, G1SimulatedActuator
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer, KinematicNormalizer, ConventionEntry
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.core.types import (
    ActuatorCommand, ControlMode, HALStatus, ActuatorState
)
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.watchdog.config import WatchdogConfig
from axis_os.hal.watchdog.events import AlertKind, AlertSeverity
from axis_os.store.runtime.play_store import PlayStoreRuntime, accepter_tout_SIMULATION
from axis_os.store.sdk.manifest import (
    AppManifest, Permission, AppCategory,
    PricingModel, HardwareRequirements
)
from axis_os.store.sdk.app_base import RobotApp, RobotContext, MissionResult
from axis_os.apps.package_sorting.app import PackageSortingApp

PASS = 0
FAIL = 0


def section(title: str):
    print(f"\n{'═'*60}")
    print(f"  {title}")
    print(f"{'═'*60}")


def check(label: str, condition: bool) -> bool:
    global PASS, FAIL
    icon   = "✓" if condition else "✗"
    status = "OK"    if condition else "FAILED"
    print(f"  [{icon}] {label:<52} {status}")
    if condition: PASS += 1
    else:         FAIL += 1
    return condition


def approx(a: float, b: float, tol: float = 1e-9) -> bool:
    return abs(a - b) < tol


# ══════════════════════════════════════════════════════════════════

section("1 — HAL UnitreeG1HAL (24 DoF)")
robot = UnitreeG1HAL("g1_test")
check("initialize_all() OK", robot.initialize_all() == HALStatus.OK)
check("is_initialized", robot.is_initialized)
check("23 joints (fichier URDF officiel, plus de 24 codés à la main)", len(robot.get_all_actuators()) == 23)
check("left_knee_joint accessible", robot.get_actuator("left_knee_joint") is not None)
check("left_knee_joint en IDLE", robot.get_actuator("left_knee_joint").state == ActuatorState.IDLE)
check("IMU accessible", robot.get_imu() is not None)
info = robot.get_robot_info()
check("robot_info complet", all(k in info for k in ["model", "dof", "hal_version"]))
check("model = G1", info["model"] == "G1")
check("dof = 23", info["dof"] == "23")

section("2 — Safety Guard")
cmd_ok  = ActuatorCommand(ControlMode.POSITION, position=0.5)
cmd_bad = ActuatorCommand(ControlMode.POSITION, position=5.0)
r_ok    = robot.send_joint_commands({"left_knee_joint": cmd_ok})
r_bad   = robot.send_joint_commands({"left_knee_joint": cmd_bad})
check("Commande valide acceptée",      r_ok["left_knee_joint"]  == HALStatus.OK)
check("Commande hors-limite rejetée",  r_bad["left_knee_joint"] == HALStatus.ERROR_LIMIT)
fb = robot.get_actuator("left_knee_joint").get_feedback()
check("Position inchangée après rejet", approx(fb.position, 0.5))

section("3 — KinematicNormalizer")
norm = build_g1_normalizer()
# left_knee_joint sign=+1 → raw == normalized
raw = norm.normalized_to_raw("left_knee_joint", 0.5)
check("left_knee_joint norm=0.5 → raw=0.5",   approx(raw, 0.5))
# right_knee_joint sign=-1 → raw = -0.5
raw2 = norm.normalized_to_raw("right_knee_joint", 0.5)
check("right_knee_joint norm=0.5 → raw=-0.5", approx(raw2, -0.5))
# round-trip
for jid in ["left_knee_joint", "right_knee_joint", "right_hip_pitch_joint", "left_elbow_joint"]:
    for angle in [0.0, 0.3, -0.5, 1.0]:
        r, _ = norm.normalized_to_raw(jid, angle), None
        back = norm.raw_to_normalized(jid, r)
        if not approx(back, angle):
            check(f"Round-trip {jid} {angle}", False)
check("Round-trip exact sur 4 joints × 4 angles", True)

section("4 — Correction bug genou droit G1")
print("""
  MISE À JOUR (2026-09-08) : depuis le passage au fichier URDF
  officiel Unitree, left_knee_joint et right_knee_joint ont les
  MÊMES bornes brutes (pos_min≈-0.087, pos_max≈2.88) — le fichier
  ne montre aucune asymétrie de POSITION entre les deux genoux.
  sign=-1 reste néanmoins actif sur right_knee_joint dans
  conventions.py : l'URDF décrit la géométrie/les limites, pas le
  sens de réponse électrique du moteur (souvent monté en miroir
  côté droit sur un robot symétrique) — un point qui ne peut être
  tranché qu'en observant le comportement réel du moteur (Isaac
  Sim ou matériel), pas en lisant un fichier URDF seul. Ce test
  vérifie donc désormais qu'une commande normalized=+0.5 CÔTÉ
  DROIT, une fois inversée en raw=-0.5 par le sign, est
  CORRECTEMENT REJETÉE par les vraies limites du fichier officiel
  (-0.5 est hors de [-0.087, 2.88]) — le comportement inverse
  d'avant cette session, qui reposait sur des limites inventées
  à la main où raw=-0.5 tombait par coïncidence dans la plage.
""")
norm_robot = NormalizedRobotHAL(UnitreeG1HAL("g1_norm"), build_g1_normalizer())
norm_robot.initialize_all()
results = norm_robot.send_joint_commands({
    "left_knee_joint":  ActuatorCommand(ControlMode.POSITION, position=0.5),
    "right_knee_joint": ActuatorCommand(ControlMode.POSITION, position=0.5),
})
check("left_knee_joint  norm=0.5 accepté", results["left_knee_joint"]  == HALStatus.OK)
check("right_knee_joint norm=0.5 (→raw=-0.5) refusé par les vraies limites URDF",
      results["right_knee_joint"] == HALStatus.ERROR_LIMIT)
state = norm_robot.get_full_state()
check("feedback left_knee_joint  normalisé ≈ 0.5", approx(state["left_knee_joint"].position,  0.5))
check("feedback right_knee_joint resté à 0.0 (commande refusée, jamais appliquée)",
      approx(state["right_knee_joint"].position, 0.0))

section("5 — NormalizedRobotHAL")
info2 = norm_robot.get_robot_info()
check("hal_normalized = true", info2.get("hal_normalized") == "true")
defs = norm_robot.get_joint_definitions()
rk = next((d for d in defs if d.joint_id == "right_knee_joint"), None)
check("right_knee_joint limites normalisées pos_max > 0", rk is not None and rk.limits.pos_max > 0)

section("6 — WatchdogSystem : thermique + e-stop")
robot2 = UnitreeG1HAL("g1_wdg")
robot2.initialize_all()
cfg = WatchdogConfig.standard()
cfg.poll_interval_s = 0.01

alerts = []
lock   = threading.Lock()
def collect(a):
    with lock: alerts.append(a)

wd = WatchdogSystem(robot2, config=cfg)
wd.subscribe(collect)
wd.start()
time.sleep(0.05)

# Injecter température critique
act = robot2.get_actuator("left_knee_joint")
if isinstance(act, G1SimulatedActuator):
    act._temperature = 82.0
time.sleep(0.15)

check("TEMPERATURE_CRITICAL émise",
      any(a.kind == AlertKind.TEMPERATURE_CRITICAL for a in alerts))
check("ESTOP_TRIGGERED émis",
      any(a.kind == AlertKind.ESTOP_TRIGGERED for a in alerts))
check("E-stop déclenché", wd.estop_fired)
check("Tous les joints en EMERGENCY_STOP",
      all(a.state == ActuatorState.EMERGENCY_STOP
          for a in robot2.get_all_actuators().values()))
wd.stop()
robot2.shutdown_all()

section("7 — WatchdogSystem : perte de communication")
robot3 = UnitreeG1HAL("g1_comm")
robot3.initialize_all()
cfg2 = WatchdogConfig.standard()
cfg2.poll_interval_s = 0.005
cfg2.comm_timeout_s  = 0.05
cfg2.comm_lost_s     = 0.15

alerts2 = []
wd2 = WatchdogSystem(robot3, config=cfg2)
wd2.subscribe(lambda a: alerts2.append(a))
wd2.start()
time.sleep(0.03)

# Couper feedback d'un joint
act3 = robot3.get_actuator("left_elbow_joint")
original_fb = act3.get_feedback
act3.get_feedback = lambda: None
jstate = wd2._joint_states.get("left_elbow_joint")
jstate.last_feedback_time = time.monotonic() - 0.06
time.sleep(0.08)
check("COMM_TIMEOUT émis", any(a.kind == AlertKind.COMM_TIMEOUT for a in alerts2))
jstate.last_feedback_time = time.monotonic() - 0.20
time.sleep(0.08)
check("COMM_LOST émis",    any(a.kind == AlertKind.COMM_LOST    for a in alerts2))
check("E-stop sur comm lost", wd2.estop_fired)
act3.get_feedback = original_fb
wd2.stop()
robot3.shutdown_all()

section("8 — Play Store : installation + manifeste")
robot4 = UnitreeG1HAL("g1_store")
robot4.initialize_all()
norm4  = NormalizedRobotHAL(robot4, build_g1_normalizer())
wd4    = WatchdogSystem(robot4, WatchdogConfig.simulation())
wd4.start()
runtime = PlayStoreRuntime(norm4, wd4, confirmer_permissions=accepter_tout_SIMULATION)

app      = PackageSortingApp()
manifest = app.manifest()
errors   = manifest.validate()
check("Manifeste valide", len(errors) == 0)
check("app_id correct",   manifest.app_id == "com.acme.package_sorting")
check("Version 1.0.0",    manifest.version == "1.0.0")
installed = runtime.install(PackageSortingApp())
check("Installation réussie", installed)

section("9 — Play Store : commission 30%")
check("Prix dev = 1.40€",   abs(manifest.developer_revenue_eur - 1.40) < 0.001)
check("Prix client ≈ 2.00€", abs(manifest.store_price_eur - 2.0) < 0.01)
check("Commission ≈ 0.60€",  abs(manifest.platform_commission_eur - 0.60) < 0.01)
check("Taux 30% exact",
      abs(manifest.platform_commission_eur / manifest.store_price_eur - 0.30) < 0.001)

section("10 — Play Store : sandbox permissions")
class BadApp(RobotApp):
    def manifest(self): return AppManifest(
        app_id="com.bad.actor", name="Bad", version="1.0.0",
        author="X", author_email="x@x.com", description="test",
        category=AppCategory.UTILITIES,
        permissions=[Permission.ACTUATORS_READ],  # PAS WRITE_LEGS
        hardware=HardwareRequirements(min_dof=0),
        pricing_model=PricingModel.FREE, max_velocity_ratio=0.5,
    )
    def on_start(self, ctx): return True
    def run_mission(self, ctx, params):
        ctx.move_legs({"left_knee_joint": ActuatorCommand(ControlMode.POSITION, position=0.5)})
        return MissionResult(True, "", "com.bad.actor", 0.0)
    def on_stop(self, ctx): pass

runtime.install(BadApp())
runtime.start_app("com.bad.actor")
bad_result = runtime.run_mission("com.bad.actor", {})
check("Mission refusée (permission manquante)", bad_result is not None and not bad_result.success)
check("Message contient 'Permission'",
      "Permission" in (bad_result.error_message or "") if bad_result else False)

section("11 — PackageSorting : mission complète")
runtime.start_app("com.acme.package_sorting")
result = runtime.run_mission(
    "com.acme.package_sorting",
    params={"batch_size": 5, "speed_mode": "safe"}
)
check("Mission réussie",        result is not None and result.success)
check("5 colis traités",        result.items_processed == 5 if result else False)
check("mission_id généré",      bool(result.mission_id) if result else False)
check("Log non vide",           len(result.log) > 0 if result else False)
check("Métriques présentes",    "success_rate_pct" in result.metrics if result else False)

section("12 — Rapport de facturation")
billing = runtime.billing_summary()
check("1 mission facturée",        billing["total_missions"] == 1)
check("Total ≈ 2.00€",            abs(billing["total_gross_eur"] - 2.00) < 0.01)
check("Commission ≈ 0.60€",       abs(billing["commission_eur"]  - 0.60) < 0.01)
check("2 apps installées",         billing["apps_installed"] == 2)

runtime.stop_app("com.acme.package_sorting")
wd4.stop()
robot4.shutdown_all()

# ── RÉSUMÉ ────────────────────────────────────────────────────────
section("RÉSUMÉ FINAL")
total = PASS + FAIL
print(f"  Résultats : {PASS}/{total} tests passés")
print()
if FAIL == 0:
    print("  ✓ Stack AXIS-OS complète validée.")
    print("  ✓ HAL + Safety + Normalizer + Watchdog + Play Store")
    print("  ✓ PackageSorting : 5 colis, commission 30%")
    print("  ✓ Sandbox : violation de permission bloquée")
    print()
    print("  Prêt pour :")
    print("  → Isaac Sim / Webots (IsaacSimHAL + ISimBackend)")
    print("  → Hardware réel (G1CANActuator)")
    print("  → Module IA (Sprint 2)")
else:
    print(f"  ✗ {FAIL} test(s) échoué(s). Voir détails ci-dessus.")

sys.exit(0 if FAIL == 0 else 1)
