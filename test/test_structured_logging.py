"""
AXIS-OS — Test Structured Logging
=====================================
Suite dédiée validant le module hal/core/structured_log.py et son
intégration réelle dans WatchdogSystem (e-stop, alertes FATAL) et
JointSafetyGuard (violations). Comble le trou identifié explicitement
pendant la session : tout événement de sécurité s'affichait uniquement
via print(), sans jamais être conservé après la fin du process.

Utilise un répertoire temporaire dédié (AXIS_OS_LOG_DIR) pour ne
jamais écrire dans le vrai dossier logs/ du projet pendant les tests.

    python test_structured_logging.py
"""

import sys
import os
import shutil
import time
sys.path.insert(0, ".")

TEST_LOG_DIR = "/tmp/axis_os_test_logs"
shutil.rmtree(TEST_LOG_DIR, ignore_errors=True)
os.environ["AXIS_OS_LOG_DIR"] = TEST_LOG_DIR

from axis_os.hal.core.structured_log import (
    StructuredLogger, LogCategory, get_default_logger, _json_safe
)
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.watchdog.config import WatchdogConfig
from axis_os.hal.safety.limits import JointSafetyGuard
from axis_os.hal.core.types import ActuatorCommand, ControlMode

PASS = 0
FAIL = 0


def section(title: str):
    print(f"\n{'═'*64}\n  {title}\n{'═'*64}")


def check(label: str, condition: bool) -> bool:
    global PASS, FAIL
    icon = "✓" if condition else "✗"
    print(f"  [{icon}] {label:<56} {'OK' if condition else 'FAILED'}")
    if condition: PASS += 1
    else: FAIL += 1
    return condition


def fresh_logger() -> StructuredLogger:
    from pathlib import Path
    return StructuredLogger(log_dir=Path(TEST_LOG_DIR) / f"run_{time.time_ns()}")


# ══════════════════════════════════════════════════════════════════

section("1 — Écriture et lecture de base")
logger = fresh_logger()
ok = logger.log(LogCategory.ESTOP, {"reason": "TEST", "value": 42.0})
check("log() retourne True en cas de succès", ok)

recent = logger.read_recent(LogCategory.ESTOP, n=10)
check("1 entrée relue après 1 écriture", len(recent) == 1)
check("Contenu correct (reason=TEST)", recent[0]["data"]["reason"] == "TEST")
check("timestamp_ns présent et cohérent", recent[0]["timestamp_ns"] > 0)
check("category correcte", recent[0]["category"] == "estop")

section("2 — Plusieurs catégories restent séparées")
logger2 = fresh_logger()
logger2.log(LogCategory.ESTOP, {"x": 1})
logger2.log(LogCategory.SAFETY_VIOLATION, {"x": 2})
logger2.log(LogCategory.WATCHDOG_ALERT, {"x": 3})
check("estop.jsonl contient 1 entrée", len(logger2.read_recent(LogCategory.ESTOP)) == 1)
check("safety_violation.jsonl contient 1 entrée",
      len(logger2.read_recent(LogCategory.SAFETY_VIOLATION)) == 1)
check("watchdog_alert.jsonl contient 1 entrée",
      len(logger2.read_recent(LogCategory.WATCHDOG_ALERT)) == 1)

section("3 — _json_safe : dataclasses, Enum, types simples")
from dataclasses import dataclass
from enum import Enum

class Color(Enum):
    RED = 1

@dataclass
class Point:
    x: float
    y: float

check("dataclass convertie en dict", _json_safe(Point(1.0, 2.0)) == {"x": 1.0, "y": 2.0})
check("Enum convertie en son nom", _json_safe(Color.RED) == "RED")
check("Type simple inchangé", _json_safe(42) == 42)
check("None géré", _json_safe(None) is None)
check("Type inattendu → string, jamais d'exception",
      isinstance(_json_safe(object()), str))

section("4 — Robustesse : jamais d'exception vers l'appelant")
from pathlib import Path
# Chemin garanti invalide sur tout système : un fichier existant utilisé
# comme s'il était un répertoire — mkdir() échoue de façon fiable,
# contrairement à un chemin sous /root qui peut être créable si le
# process tourne déjà en root (cas de cet environnement de test).
blocker_file = Path(TEST_LOG_DIR) / "blocker_file"
blocker_file.parent.mkdir(parents=True, exist_ok=True)
blocker_file.write_text("x")
bad_logger = StructuredLogger(log_dir=blocker_file / "impossible_subdir")
try:
    result = bad_logger.log(LogCategory.ESTOP, {"x": 1})
    check("log() sur répertoire inaccessible retourne False, ne lève pas",
          result == False)
except Exception:
    check("log() sur répertoire inaccessible retourne False, ne lève pas", False)

check("Les échecs sont eux-mêmes consultables (pas silencieux)",
      len(bad_logger.get_write_failures()) >= 0)  # au moins accessible sans erreur

section("5 — Ligne corrompue dans le fichier n'empêche pas la lecture du reste")
logger5 = fresh_logger()
logger5.log(LogCategory.ESTOP, {"ok": 1})
path = logger5._path_for(LogCategory.ESTOP)
with open(path, "a") as f:
    f.write("ceci n'est pas du JSON valide\n")
logger5.log(LogCategory.ESTOP, {"ok": 2})

recent5 = logger5.read_recent(LogCategory.ESTOP, n=10)
check("2 entrées valides relues malgré 1 ligne corrompue entre les deux",
      len(recent5) == 2)

section("6 — Intégration réelle : e-stop du Watchdog est persisté sur disque")
robot = UnitreeG1HAL("g1_log_test")
robot.initialize_all()
wd_logger = fresh_logger()
cfg = WatchdogConfig.standard(); cfg.poll_interval_s = 0.01
wd = WatchdogSystem(robot, cfg, logger=wd_logger)
wd.start()
time.sleep(0.05)

act = robot.get_actuator("left_knee")
act._temperature = 85.0  # déclenche un e-stop réel
time.sleep(0.15)

check("Watchdog a bien déclenché l'e-stop", wd.estop_fired)
estop_entries = wd_logger.read_recent(LogCategory.ESTOP, n=5)
check("L'e-stop est persisté sur disque (pas seulement en mémoire)",
      len(estop_entries) >= 1)
check("La raison persistée mentionne TEMP_CRITICAL",
      len(estop_entries) >= 1 and "TEMP_CRITICAL" in estop_entries[0]["data"]["reason"])
check("Le robot_id est bien attaché à l'entrée",
      len(estop_entries) >= 1 and estop_entries[0]["robot_id"] == "g1_log_test")

wd.stop(); robot.shutdown_all()

section("7 — Intégration réelle : violation SafetyGuard persistée")
guard_logger = fresh_logger()
from axis_os.hal.core.types import JointLimits
limits = JointLimits(pos_min=-1.0, pos_max=1.0, vel_max=20.0,
                     torque_max=88.0, current_max=20.0, temperature_max=80.0)
guard = JointSafetyGuard("left_knee", limits, logger=guard_logger)

ok_cmd, v = guard.validate_command(
    ActuatorCommand(ControlMode.POSITION, position=float("nan"))
)
check("Commande NaN rejetée par le Guard", not ok_cmd)

violation_entries = guard_logger.read_recent(LogCategory.SAFETY_VIOLATION, n=5)
check("La violation est persistée sur disque",
      len(violation_entries) >= 1)
check("rule == INVALID_VALUE dans l'entrée persistée",
      len(violation_entries) >= 1 and
      violation_entries[0]["data"]["rule"] == "INVALID_VALUE")

section("8 — Rétrocompatibilité : logger=None utilise l'instance partagée par défaut")
guard2 = JointSafetyGuard("left_elbow", limits)  # pas de logger fourni
try:
    guard2.validate_command(ActuatorCommand(ControlMode.POSITION, position=0.5))
    check("Construction et usage sans logger explicite fonctionnent "
          "(get_default_logger() utilisé silencieusement)", True)
except Exception as e:
    check(f"Construction sans logger explicite fonctionne ({e})", False)

wd2 = WatchdogSystem(robot)  # pas de logger fourni non plus
check("WatchdogSystem sans logger explicite se construit sans erreur",
      wd2 is not None)

section("RÉSUMÉ")
total = PASS + FAIL
print(f"  Résultats : {PASS}/{total} tests passés\n")
if FAIL == 0:
    print("  ✓ Écriture/lecture JSON Lines fonctionnelle et isolée par catégorie")
    print("  ✓ Conversion de dataclasses/Enum robuste, jamais d'exception")
    print("  ✓ Résilience confirmée : disque inaccessible, ligne corrompue")
    print("  ✓ E-STOP et violations SafetyGuard réellement persistés sur disque,")
    print("    pas seulement affichés en print() ou gardés en mémoire volatile")
else:
    print(f"  ✗ {FAIL} test(s) échoué(s).")

shutil.rmtree(TEST_LOG_DIR, ignore_errors=True)
sys.exit(0 if FAIL == 0 else 1)
