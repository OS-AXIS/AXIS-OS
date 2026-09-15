"""
AXIS-OS — Tests moniteur de sécurité indépendant
=====================================================
Valide IndependentSafetyMonitor (point 3, recommandation externe) —
redondance DIVERSE (pas duplication) : lit rt/lowstate/feedback
directement depuis IRobotHAL, sans jamais passer par
NormalizedRobotHAL ni KinematicNormalizer, précisément pour attraper
des bugs de CONCEPTION du chemin de commande (comme C3, où
JointSafetyGuard validait un couple déjà inversé par le normalizer —
voir CHANGELOG.md) que la redondance de duplication ne peut pas
attraper.
"""

import sys
import ast
import time
sys.path.insert(0, ".")

from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL
from axis_os.hal.independent_monitor.monitor import IndependentSafetyMonitor

FAIL = 0


def check(description: str, condition: bool) -> None:
    global FAIL
    status = "✓" if condition else "✗"
    print(f"  [{status}] {description}")
    if not condition:
        FAIL += 1


def section(title: str) -> None:
    print(f"\n{'=' * 66}\n  {title}\n{'=' * 66}")


# ══════════════════════════════════════════════════════════════════

section("1 — CRITIQUE : isolation architecturale vérifiée statiquement")

# Vérifie les imports RÉELS du fichier source, pas seulement le
# comportement observé — garde contre un futur refactor qui
# réintroduirait silencieusement le couplage que ce module existe
# pour éviter.
with open("axis_os/hal/independent_monitor/monitor.py", encoding="utf-8") as f:
    tree = ast.parse(f.read())

imports = []
for node in ast.walk(tree):
    if isinstance(node, ast.ImportFrom):
        imports.append(node.module or "")
    elif isinstance(node, ast.Import):
        imports.extend(alias.name for alias in node.names)

forbidden = ["kinematic_normalizer", "unitree_g1.robot", "g1_dds_actuator"]
violations = [imp for imp in imports if any(f in imp for f in forbidden)]
check("CRITIQUE : aucun import vers kinematic_normalizer/robot.py/g1_dds_actuator",
      violations == [])
check("Importe bien urdf_loader directement (source de vérité indépendante)",
      any("urdf_loader" in imp for imp in imports))

# ══════════════════════════════════════════════════════════════════

section("2 — Détecte une anomalie et déclenche un e-stop, sans dépendance au guard normal")

hal = UnitreeG1HAL("g1_indep_w1")
hal.initialize_all()
monitor = IndependentSafetyMonitor(
    hal, "axis_os/hal/drivers/unitree_g1/urdf/g1_23dof_rev_1_0.urdf",
    poll_interval_s=0.02,
)
alerts = []
monitor.subscribe(lambda a: alerts.append(a))
monitor.start()

time.sleep(0.08)
check("Avant anomalie : estop_triggered == False", monitor.estop_triggered is False)

act = hal._actuators["left_knee_joint"]
act._position = 99.0  # bien au-delà de pos_max (~2.88)
time.sleep(0.1)
monitor.stop()

check("Après anomalie : estop_triggered == True", monitor.estop_triggered is True)
check("Au moins une alerte reçue via subscribe()", len(alerts) > 0)
check("L'alerte identifie le bon joint",
      len(alerts) > 0 and alerts[0].joint_id == "left_knee_joint")
check("L'e-stop a bien été déclenché sur le VRAI HAL (pas juste signalé)",
      hal.get_actuator("left_knee_joint").state.name == "EMERGENCY_STOP")

# ══════════════════════════════════════════════════════════════════

section("3 — Ne déclenche rien quand tout est dans les vraies limites")

hal2 = UnitreeG1HAL("g1_indep_w2")
hal2.initialize_all()
monitor2 = IndependentSafetyMonitor(
    hal2, "axis_os/hal/drivers/unitree_g1/urdf/g1_23dof_rev_1_0.urdf",
    poll_interval_s=0.02,
)
monitor2.start()
time.sleep(0.1)
monitor2.stop()
check("Aucun e-stop déclenché quand toutes les positions sont dans les limites",
      monitor2.estop_triggered is False)
check("Aucune alerte", len(monitor2.get_alerts()) == 0)

# ══════════════════════════════════════════════════════════════════

section("4 — margin_ratio personnalisable, indépendante des marges du reste du système")

hal3 = UnitreeG1HAL("g1_indep_w3")
hal3.initialize_all()
# margin_ratio=0.5 : moitié des limites officielles — doit déclencher
# BIEN AVANT que la position n'atteigne la vraie limite URDF.
monitor3 = IndependentSafetyMonitor(
    hal3, "axis_os/hal/drivers/unitree_g1/urdf/g1_23dof_rev_1_0.urdf",
    poll_interval_s=0.02, margin_ratio=0.5,
)
monitor3.start()
time.sleep(0.05)

act3 = hal3._actuators["left_knee_joint"]
# pos_max réel ~2.88, mais avec margin_ratio=0.5 la limite effective
# de ce moniteur est ~1.44 — une position de 2.0 (VALIDE selon
# JointSafetyGuard, qui utilise 2.88) doit déclencher CE moniteur.
act3._position = 2.0
time.sleep(0.1)
monitor3.stop()

check("margin_ratio=0.5 déclenche sur une position VALIDE selon les limites "
      "officielles (2.0 < 2.88) mais hors de SA PROPRE marge plus stricte",
      monitor3.estop_triggered is True)

# ══════════════════════════════════════════════════════════════════

section("5 — Une exception dans un abonné n'interrompt jamais le moniteur")

hal4 = UnitreeG1HAL("g1_indep_w4")
hal4.initialize_all()
monitor4 = IndependentSafetyMonitor(
    hal4, "axis_os/hal/drivers/unitree_g1/urdf/g1_23dof_rev_1_0.urdf",
    poll_interval_s=0.02,
)


def _bad_subscriber(alert):
    raise RuntimeError("abonné volontairement défaillant")


monitor4.subscribe(_bad_subscriber)
monitor4.start()
act4 = hal4._actuators["left_knee_joint"]
act4._position = 99.0
time.sleep(0.1)
monitor4.stop()
check("estop_triggered reste True malgré un abonné qui lève une exception",
      monitor4.estop_triggered is True)

# ══════════════════════════════════════════════════════════════════

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)

if FAIL == 0:
    print("\n  ✓ Isolation architecturale vérifiée STATIQUEMENT (pas seulement")
    print("    par comportement observé) — un refactor qui réintroduirait le")
    print("    couplage ferait échouer ce test immédiatement")
    print("  ✓ Détecte et déclenche un e-stop réel, indépendamment de")
    print("    JointSafetyGuard/WatchdogSystem")
    print("  ✓ Politique de marge propre, indépendante du reste du système")

sys.exit(0 if FAIL == 0 else 1)
