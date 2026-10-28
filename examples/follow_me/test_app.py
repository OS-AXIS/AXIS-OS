"""Test de Follow me, sur le robot simulé léger (python test_app.py).

Un faux monde : une personne qui marche, s'arrête, puis repart de côté ;
une deuxième qui passe entre elle et le robot ; un mur derrière lequel on
disparaît. Les commandes vocales sont « entendues » par le micro simulé."""
import math
import sys
import threading
import time
from pathlib import Path

ICI = Path(__file__).resolve().parent
sys.path.insert(0, str(ICI))
sys.path.insert(0, str(ICI.parents[1]))

from app import SAID, FollowMeApp  # noqa: E402
from axis_os.services.navigation.service import Carte, Rectangle  # noqa: E402
from axis_os.testing import AppTestHarness  # noqa: E402

RED, BLUE = (200, 30, 30), (30, 50, 200)


class Watch:
    """Relève, toutes les 50 ms, la distance vraie du robot à une personne et
    chaque consigne de marche."""

    def __init__(self, robot, person):
        self.robot, self.person, self.d_min, self.commands = robot, person, math.inf, []
        send = robot.locomotion.set_velocity
        robot.locomotion.set_velocity = lambda vx=0.0, vy=0.0, wz=0.0: (
            self.commands.append((vx, vy, wz)), send(vx, vy, wz))[1]
        self._stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self._stop.wait(0.05):
            if self.robot.follow_service.active:
                self.d_min = min(self.d_min, math.hypot(self.person.x - self.robot.x,
                                                        self.person.y - self.robot.y))

    def close(self):
        self._stop.set()


def test_follow_then_stop():
    with AppTestHarness(FollowMeApp(), speech=True, time_scale=2.0) as h:
        robot = h.robot
        a = robot.add_person(2.2, 0.0, torso=RED)
        a.wait(2.0).walk_to(4.5, 0.0, speed_mps=0.3).wait(8.0).walk_to(4.5, 1.2, speed_mps=0.3)
        b = robot.add_person(3.75, -2.0, torso=BLUE)
        b.wait(11.0).walk_to(3.75, 2.0, speed_mps=0.8)
        robot.hear("Suis-moi !", after_s=1.0)
        robot.hear("Arrête-toi.", after_s=24.0)
        watch = Watch(robot, a)
        result = h.run({"max_s": 40})
        watch.close()
        assert result.success, result.error_message
        assert result.metrics == {"events": ["start", "stop"], "outcome": "STOPPED_BY_APP"}, \
            result.metrics
        assert [t for t, _ in robot.voice.spoken] == [SAID["start"], SAID["stop"]]
        follow = robot.follow_service
        assert follow.stats["follows"] == 1 and follow.stats["lost"] == 0, follow.stats
        # La deuxième personne est passée tout près : le robot s'est
        # immobilisé, puis a repris LA MÊME personne.
        assert follow._issue.status == "STOPPED_BY_APP" and follow._issue.moved
        assert robot.x > 1.5, robot.x                # il a suivi
        assert watch.d_min >= 1.4, watch.d_min       # jamais plus près que prévu
        assert all(vx >= 0.0 and vy == 0.0 and vx <= 0.25 for vx, vy, _ in watch.commands)
        assert follow._reference is None and len(robot.people_service._tracker) == 0


def test_lost_behind_wall():
    wall = Carte(-5.0, -5.0, 8.0, 8.0, obstacles=[Rectangle(3.0, 0.8, 3.3, 5.0)])
    with AppTestHarness(FollowMeApp(), speech=True, time_scale=2.0, carte=wall) as h:
        a = h.robot.add_person(2.4, 0.0, torso=RED)
        a.wait(1.5).walk_to(4.0, 0.0, speed_mps=0.3).walk_to(4.0, 3.0, speed_mps=0.5)
        h.robot.hear("suis moi", after_s=0.5)
        t0 = time.monotonic()
        result = h.run({"max_s": 60})
        assert result.success, result.error_message
        assert result.metrics == {"events": ["start", "lost"], "outcome": "LOST"}, result.metrics
        assert h.robot.voice.spoken[-1][0] == SAID["lost"]
        assert time.monotonic() - t0 < 30.0          # la recherche est bornée


def test_nobody_in_front():
    with AppTestHarness(FollowMeApp(), speech=True, time_scale=2.0) as h:
        h.robot.add_person(-2.0, 0.0, torso=RED)     # derrière le robot
        h.robot.hear("Suis-moi", after_s=0.5)
        result = h.run({"max_s": 4})
        assert result.metrics == {"events": ["nobody"], "outcome": "END"}, result.metrics
        assert h.robot.follow_service.stats["follows"] == 0


if __name__ == "__main__":
    test_follow_then_stop()
    test_lost_behind_wall()
    test_nobody_in_front()
    print("✓ follow_me")
