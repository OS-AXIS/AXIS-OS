"""Test de Hello Robot, sur le robot simulé léger (python test_app.py)."""
import sys
from pathlib import Path

ICI = Path(__file__).resolve().parent
sys.path.insert(0, str(ICI))
sys.path.insert(0, str(ICI.parents[1]))

from app import HelloRobotApp  # noqa: E402
from axis_os.testing import AppTestHarness  # noqa: E402


def test_hello_robot():
    with AppTestHarness(HelloRobotApp()) as h:
        result = h.run()
        assert result is not None and result.success, result and result.error_message
        assert result.metrics["done"] == ["wave:PLAYED", "speak:SPOKEN"], result.metrics
        assert h.robot.voice.spoken == [("Hello! I am G1, nice to meet you.", "en")]

        # L'utilisateur choisit le français.
        assert h.runtime.set_setting(h.app_id, "language", "fr")
        result = h.run()
        assert result.success
        assert h.robot.voice.spoken[-1] == ("Bonjour ! Je suis G1, enchanté.", "fr")


if __name__ == "__main__":
    test_hello_robot()
    print("✓ hello_robot")
