"""
Driver caméra RealSense du G1 — comportement SANS matériel.

Ce que ce test peut prouver : que le driver respecte le contrat ICamera,
qu'il dégrade proprement quand le SDK ou le capteur est absent, et qu'il
ne ment jamais sur ce qu'il ne sait pas.

Ce qu'il ne peut PAS prouver : qu'il lit correctement une vraie caméra.
Cela demande le matériel, et relève du protocole de mise sous tension
(docs/campagne_validation_isaac.md §9).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from axis_os.hal.core.interfaces import ICamera
from axis_os.hal.core.types import HALStatus, SensorType
from axis_os.hal.drivers.unitree_g1.g1_camera import G1RealSenseCamera

FAIL = 0


def check(label, cond):
    global FAIL
    print(f"  [{'✓' if cond else '✗'}] {label}")
    if not cond:
        FAIL += 1


def section(t):
    print(f"\n{'=' * 66}\n  {t}\n{'=' * 66}")


section("1 — Contrat ICamera")

cam = G1RealSenseCamera("cam_test")
check("G1RealSenseCamera implémente ICamera", isinstance(cam, ICamera))
check("Aucune méthode abstraite non implémentée",
      not getattr(cam, "__abstractmethods__", None))
check("Type de capteur RGB par défaut",
      cam.sensor_type is SensorType.CAMERA_RGB)
check("depth=True bascule le type de capteur",
      G1RealSenseCamera("c", depth=True).sensor_type is SensorType.CAMERA_DEPTH)

section("2 — Absence de SDK / de matériel : dégrader, pas planter")

st = cam.initialize()
check("initialize() sans pyrealsense2 rend ERROR_COMM, sans exception",
      st is HALStatus.ERROR_COMM)
check("is_healthy() est False après un échec d'initialisation",
      cam.is_healthy() is False)
check("capture() rend None plutôt que de lever",
      cam.capture() is None)
check("get_intrinsics() rend None — aucune calibration inventée",
      cam.get_intrinsics() is None)
check("shutdown() reste OK même sans initialisation réussie",
      cam.shutdown() is HALStatus.OK)
check("shutdown() est idempotent",
      cam.shutdown() is HALStatus.OK)

section("3 — capture() ne bloque jamais")

import time
t0 = time.perf_counter()
for _ in range(1000):
    cam.capture()
dt_ms = (time.perf_counter() - t0) * 1000
check(f"1000 appels à capture() en {dt_ms:.1f} ms (jamais bloquant)",
      dt_ms < 100)

section("4 — Aucune calibration codée en dur")

src = Path("axis_os/hal/drivers/unitree_g1/g1_camera.py").read_text()
# Une calibration codée en dur ressemblerait à fx=600.0 dans le driver.
# Les seules valeurs autorisées viennent de get_intrinsics() du capteur.
check("Le driver ne contient aucun fx=/fy= littéral",
      "fx=6" not in src and "fx=3" not in src and "fx = 6" not in src)
check("La calibration vient bien de intr.fx (le capteur)",
      "intr.fx" in src and "intr.ppx" in src)

print(f"\n{'=' * 66}")
print(f"  Résultats : {'Tous les tests passés' if FAIL == 0 else str(FAIL) + ' échec(s)'}")
print("=" * 66)
if FAIL == 0:
    print("\n  ✓ Contrat ICamera respecté, calibration lue sur le capteur")
    print("  ✓ Dégradation propre sans SDK ni matériel")
    print("  ⚠ La lecture d'une VRAIE caméra reste à valider sur le robot")

sys.exit(0 if FAIL == 0 else 1)
