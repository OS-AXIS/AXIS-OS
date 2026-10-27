"""Test de Teleop local, piloté par une manette factice (python test_app.py)."""
import sys
import time
from pathlib import Path

ICI = Path(__file__).resolve().parent
sys.path.insert(0, str(ICI))
sys.path.insert(0, str(ICI.parents[1]))

from app import TeleopLocalApp  # noqa: E402
from axis_os.testing import AppTestHarness  # noqa: E402


def test_teleop_local():
    # 1 s en avant à 0.15 m/s, bouton tenu ; puis bouton relâché.
    script = [[1.0, 0.15, 0.0, 0.0, True], [0.5, 0.15, 0.0, 0.0, False]]
    with AppTestHarness(TeleopLocalApp()) as h:
        result = h.run({"script": script})
        assert result is not None and result.success, result and result.error_message
        m = result.metrics
        assert m["accepted"] > 30 and m["safe_stops"] == 1, m
        assert m["state"] in ("STOPPING", "STOPPED"), m
        assert 0.08 < h.robot.pose[0] < 0.3, h.robot.pose
        x = h.robot.pose[0]
        time.sleep(0.5)
        assert abs(h.robot.pose[0] - x) < 0.01, "le robot doit rester arrêté"


def test_teleop_vitesse_trop_haute():
    # 0.3 m/s : au-delà de la vitesse du flux (0.5 × 0.4 m/s) : tout est rejeté.
    with AppTestHarness(TeleopLocalApp()) as h:
        result = h.run({"script": [[0.5, 0.3, 0.0, 0.0, True]]})
        assert result.metrics["accepted"] == 0
        assert result.metrics["rejected"].get("OUT_OF_BOUNDS", 0) > 10
        assert h.robot.pose[0] == 0.0


if __name__ == "__main__":
    test_teleop_local()
    test_teleop_vitesse_trop_haute()
    print("✓ teleop_local")
