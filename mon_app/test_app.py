"""
Tests pour Mon App
"""

import sys
sys.path.insert(0, ".")

from axis_os.hal.drivers.isaac_sim.hal import IsaacSimHAL
from axis_os.hal.core.kinematic_normalizer import build_g1_normalizer
from axis_os.hal.core.normalized_robot import NormalizedRobotHAL
from axis_os.hal.watchdog.system import WatchdogSystem
from axis_os.hal.watchdog.config import WatchdogConfig
from axis_os.store.runtime.play_store import PlayStoreRuntime
from app import MonAppApp


def test_app():
    hal = IsaacSimHAL(robot_id="test_mon_app")
    hal.initialize_all()
    norm = NormalizedRobotHAL(hal, build_g1_normalizer())
    wd   = WatchdogSystem(hal, WatchdogConfig.simulation())
    wd.start()

    runtime = PlayStoreRuntime(norm, wd)
    assert runtime.install(MonAppApp())
    assert runtime.start_app("com.kenzo.mon_app")

    result = runtime.run_mission("com.kenzo.mon_app", params={})
    assert result is not None
    print(f"Résultat : {result}")

    wd.stop()
    hal.shutdown_all()
    print("✓ Test passé")


if __name__ == "__main__":
    test_app()
