"""Test de Fetch, sur le robot simulé léger (python test_app.py)."""
import math
import sys
from pathlib import Path

ICI = Path(__file__).resolve().parent
sys.path.insert(0, str(ICI))
sys.path.insert(0, str(ICI.parents[1]))

from app import FetchApp  # noqa: E402
from axis_os.testing import AppTestHarness, SimulatedObject  # noqa: E402


def test_fetch():
    box = SimulatedObject("red box", (1.2, 0.4, 0.90))
    with AppTestHarness(FetchApp(), objects=[box]) as h:
        result = h.run({"object": "red", "drop": [0.0, -0.5]})
        assert result is not None and result.success, result and result.error_message
        assert result.metrics["steps"] == ["go_to:ARRIVED", "pick:HELD", "go_to_drop:ARRIVED",
                                           "place:PLACED", "rest_pose:AT_REST"], result.metrics
        # La boîte est posée devant le robot, arrivé au point de dépose : à
        # environ 30 cm de ce point (les mains sont à 30 cm devant lui).
        assert math.dist(box.position[:2], (0.0, -0.5)) < 0.45, box.position
        assert h.runtime.get_progress(h.app_id).fraction == 1.0


def test_fetch_object_not_found():
    with AppTestHarness(FetchApp()) as h:
        result = h.run({"object": "unicorn"})
        assert result is not None and not result.success
        assert result.error_message == "objet introuvable"


if __name__ == "__main__":
    test_fetch()
    test_fetch_object_not_found()
    print("✓ fetch")
