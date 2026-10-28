"""Test de Night patrol, sur le robot simulé léger (python test_app.py).

Un faux site : trois lieux de ronde, une station de charge, une zone
interdite avec quelqu'un dedans. La ronde est lancée par l'ordonnanceur de
l'OS à 22:00 ; puis une batterie qui baisse en route ; puis un moteur qui
chauffe avant le départ."""
import math
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

ICI = Path(__file__).resolve().parent
sys.path.insert(0, str(ICI))
sys.path.insert(0, str(ICI.parents[1]))

from app import NightPatrolApp  # noqa: E402
from axis_os.services.navigation.site_map import FORBIDDEN, STATION, Place, SiteMap, Zone  # noqa: E402
from axis_os.store.runtime.scheduler import MissionScheduler  # noqa: E402
from axis_os.testing import AppTestHarness  # noqa: E402

RESERVE = Zone("réserve", FORBIDDEN, ((-0.5, 2.2), (1.0, 2.2), (1.0, 4.0), (-0.5, 4.0)))


def site():
    return SiteMap([Place("ronde 1", 2.0, -1.5, 1.57), Place("ronde 2", 2.0, 1.5, 2.4),
                    Place("ronde 3", -2.0, 1.5, 0.0),
                    Place("station", -3.0, -2.5, 0.0, kind=STATION)], [RESERVE])


def watch_zone(robot):
    """Distance minimale du robot à la zone interdite, relevée en continu."""
    out = {"d_min": math.inf}
    stop = threading.Event()

    def run():
        while not stop.wait(0.02):
            out["d_min"] = min(out["d_min"], RESERVE.distance(robot.x, robot.y))
    threading.Thread(target=run, daemon=True).start()
    return out, stop


def test_scheduled_round_reports_intruder():
    with AppTestHarness(NightPatrolApp(), site_map=site(), time_scale=2.0) as h:
        h.robot.add_person(0.25, 3.3)                     # dans la réserve
        clock = [datetime(2026, 10, 28, 21, 59, 50)]
        scheduler = MissionScheduler(h.runtime, wall_clock=lambda: clock[0])
        scheduler.tick()
        out, stop = watch_zone(h.robot)
        clock[0] = datetime(2026, 10, 28, 22, 0, 5)
        assert [e.decision for e in scheduler.tick()] == ["launched"]
        assert scheduler.wait_idle(120)
        stop.set()
        finished = [e for e in scheduler.journal if e.decision == "finished"]
        assert finished and finished[-1].detail == "réussie", list(scheduler.journal)
        urgent = [n for n in h.robot.notification_sink.delivered if n.priority == "urgent"]
        assert len(urgent) == 1 and urgent[0].source == "app" \
            and urgent[0].app_id == "com.axis_os.examples.night_patrol" \
            and "réserve" in urgent[0].message, urgent
        assert out["d_min"] > 0.0, out


def test_low_battery_goes_home():
    with AppTestHarness(NightPatrolApp(), site_map=site(), time_scale=2.0,
                        battery_percent=31.0, battery_drain_per_s=0.15) as h:
        result = h.run()
        assert result.metrics["outcome"] == "LOW_BATTERY", result.metrics
        assert result.metrics["events"][-1] == "station:ARRIVED", result.metrics
        assert math.hypot(h.robot.x + 3.0, h.robot.y + 2.5) < 0.3
        assert any(n.title == "Ronde interrompue" for n in h.robot.notification_sink.delivered)


def test_pause_while_motor_is_hot():
    with AppTestHarness(NightPatrolApp(), site_map=site(), time_scale=2.0) as h:
        robot = h.robot
        robot.set_motor_temperature("left_knee_joint", 72.0)

        def cool_down_once_paused():
            # Le moteur refroidit 2 s après que l'app a annoncé sa pause.
            while not any(n.title == "Ronde en pause" for n in robot.notification_sink.delivered):
                time.sleep(0.05)
            time.sleep(2.0)
            robot.set_motor_temperature("left_knee_joint", 35.0)
        threading.Thread(target=cool_down_once_paused, daemon=True).start()
        result = h.run()
        assert result.metrics["outcome"] == "DONE", result.metrics
        assert result.metrics["events"][:3] == ["pause", "resume", "ronde 1:ARRIVED"], \
            result.metrics


if __name__ == "__main__":
    test_scheduled_round_reports_intruder()
    test_low_battery_goes_home()
    test_pause_while_motor_is_hot()
    print("✓ night_patrol")
