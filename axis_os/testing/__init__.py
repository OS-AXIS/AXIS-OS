"""
AXIS-OS — Aides pour tester ses apps (axis_os.testing)
=======================================================
Un développeur d'app écrit ses propres tests en quelques lignes, sur un
robot simulé léger (pas de GPU, pas d'Isaac Sim) :

    from axis_os.testing import AppTestHarness, SimulatedObject

    with AppTestHarness(MonApp(), objects=[SimulatedObject("red box", (1.3, 0.0, 0.9))]) as h:
        result = h.run({"couleur": "red"})
        assert result.success, result.error_message

L'app tourne exactement comme sur un robot : installée par le Play Store,
dans son processus séparé, à travers le pont JSON, sous ses permissions et
ses quotas. Le profil est SIMULATION : les permissions DANGEROUS y sont
accordées sans humain, ce qui n'est jamais le cas sur un vrai robot.

Licence : Apache 2.0 (open-source — SDK public)
"""
from __future__ import annotations

import contextlib
import io
from typing import Any, Dict, Optional

from axis_os.testing.humain import ScriptedHuman
from axis_os.testing.robot_simule import (
    SimulatedObject, SimulatedPerception, SimulatedPerson, SimulatedPersonDetector,
    SimulatedRobot,
)

__all__ = ["AppTestHarness", "ScriptedHuman", "SimulatedObject", "SimulatedPerception",
           "SimulatedPerson", "SimulatedPersonDetector", "SimulatedRobot"]


class AppTestHarness:
    """Installe une app sur un robot simulé, la démarre, lance ses
    missions, puis arrête tout à la sortie du bloc `with`.

    app      : l'instance de l'app (sa classe doit être définie au niveau
               d'un module, comme toute app).
    robot    : un SimulatedRobot déjà construit ; sinon un neuf, construit
               avec les mots-clés de `robot_kw` (objects, battery_percent…).
    runtime  : mots-clés transmis à PlayStoreRuntime (voir
               SimulatedRobot.make_runtime).
    verbose  : affiche la sortie du Play Store (silencieuse par défaut).
    """

    def __init__(self, app, robot: Optional[SimulatedRobot] = None, *,
                 runtime: Optional[Dict[str, Any]] = None, verbose: bool = False,
                 **robot_kw):
        self.app = app
        self.robot = robot or SimulatedRobot(**robot_kw)
        self._robot_a_nous = robot is None
        self._runtime_kw = runtime or {}
        self._verbose = verbose
        self.runtime = None
        self.app_id = app.manifest().app_id

    def _sortie(self):
        return contextlib.nullcontext() if self._verbose \
            else contextlib.redirect_stdout(io.StringIO())

    def __enter__(self) -> "AppTestHarness":
        if self._robot_a_nous:
            self.robot.start()
        self.runtime = self.robot.make_runtime(**self._runtime_kw)
        with self._sortie():
            if not self.runtime.install(self.app):
                raise RuntimeError(f"installation de {self.app_id} refusée")
            if not self.runtime.start_app(self.app_id):
                raise RuntimeError(f"on_start de {self.app_id} a échoué")
        return self

    def run(self, params: Optional[Dict[str, Any]] = None):
        """Lance une mission et rend son MissionResult."""
        with self._sortie():
            return self.runtime.run_mission(self.app_id, params or {})

    def __exit__(self, *exc) -> None:
        try:
            if self.runtime is not None:
                with self._sortie():
                    self.runtime.stop_app(self.app_id)
        finally:
            if self._robot_a_nous:
                self.robot.stop()
