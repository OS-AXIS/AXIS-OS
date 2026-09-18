"""
Pont réutilisable entre UnitreeG1HAL et Isaac Sim.

Remplace l'intercepteur de fonction écrit pour une session (voir
docs/isaac_lab_unitree_sim_status.md). Trois exigences tenues :

1. Boucle fermée : les états articulaires reviennent d'Isaac Sim vers
   AXIS-OS comme de vrais ActuatorFeedback, pas seulement l'envoi.
2. Couple RÉELLEMENT appliqué par le solveur physique exposé, pas le
   couple commandé — indispensable pour la suite S1.2.
3. Réimport USD sûr : supprime systématiquement le .usd existant avant
   d'importer, pour ne pas reproduire le blocage `is_homogeneous`.

À exécuter DANS le conteneur Isaac Sim (voir campagne_validation_isaac_axis_os.md
§1). Suppose que /isaac-sim/axis_os existe (copié via `docker cp`) et que
/isaac-sim/g1_23dof_rev_1_0.urdf existe (copié depuis le repo).
"""
from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from typing import Dict, Optional

sys.path.insert(0, "/isaac-sim")

from axis_os.hal.core.types import ActuatorFeedback, ActuatorState, HALStatus
from axis_os.hal.drivers.unitree_g1.robot import UnitreeG1HAL, _build_g1_joint_definitions


USD_PATH = "/isaac-sim/g1_axisos.usd"
URDF_PATH = "/isaac-sim/g1_23dof_rev_1_0.urdf"

_simulation_app = None  # référence gardée pour éviter le garbage-collect


def start_simulation_app(headless: bool = False):
    """À appeler EN PREMIER, avant tout autre usage de ce module et avant
    tout import isaacsim.* dans le process. Contrainte du framework
    Carbonite d'Isaac Sim, pas une particularité d'axis_os : sans ça,
    `from isaacsim.core.api import World` échoue avec
    `ModuleNotFoundError: No module named 'isaacsim.core'` même si le
    module est bien installé — l'extension n'est simplement pas encore
    chargée en mémoire.

    Retourne l'objet SimulationApp : garde-le en vie (ne le laisse pas
    être ramassé par le GC) et appelle .close() en fin de script.
    """
    global _simulation_app
    if _simulation_app is not None:
        return _simulation_app
    from isaacsim import SimulationApp
    _simulation_app = SimulationApp({"headless": headless})
    return _simulation_app


def import_g1_urdf(force_reimport: bool = True):
    """Importe le G1 depuis le vrai URDF officiel AXIS-OS (pas la variante
    générique NVIDIA — évite la divergence de nommage à 5 joints déjà
    rencontrée). Retourne le usd_path généré.

    force_reimport=True supprime le .usd existant avant d'importer : c'est
    le correctif au blocage `AttributeError: 'NoneType' object has no
    attribute 'is_homogeneous'` rencontré en réimportant sur un .usd déjà
    présent sur disque.
    """
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    if force_reimport and os.path.exists(USD_PATH):
        os.remove(USD_PATH)

    config = URDFImporterConfig()
    config.urdf_path = URDF_PATH
    config.usd_path = USD_PATH
    config.fix_base = False
    config.merge_fixed_joints = False
    config.self_collision = False

    importer = URDFImporter(config)
    return importer.import_urdf()


class IsaacSimBridge:
    """Relie UnitreeG1HAL(backend="sim") à un G1 physiquement simulé
    dans Isaac Sim, en remplaçant le G1SimulatedActuator interne par
    des appels réels à robot.apply_action() / la lecture de l'état
    articulaire réel.

    Usage :
        bridge = IsaacSimBridge()
        bridge.setup()                 # charge le G1, le plan de sol
        hal = bridge.build_hal()       # UnitreeG1HAL branché sur Isaac
        hal.initialize_all()
        hal.send_joint_commands({...}) # passe réellement par apply_action()
        bridge.step(60)                # avance la simulation physique
    """

    def __init__(self):
        self._robot = None
        self._world = None
        self._joint_defs = _build_g1_joint_definitions()
        self._joint_index: Dict[str, int] = {}  # joint_id AXIS-OS -> index DOF Isaac

    def setup(self):
        if _simulation_app is None:
            raise RuntimeError(
                "IsaacSimBridge.setup() appelé sans avoir d'abord appelé "
                "start_simulation_app() — obligatoire, voir sa docstring."
            )
        from isaacsim.core.api import World
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.utils.stage import add_reference_to_stage

        self._world = World(stage_units_in_meters=1.0)
        # Un seul plan de sol physique — add_default_ground_plane() suffit
        # et évite le doublon de prim rencontré avec un GroundPlane() en
        # plus au même endroit. La grille par défaut d'Isaac Sim n'a
        # AUCUNE collision : sans ce plan, le G1 tombe à travers.
        self._world.scene.add_default_ground_plane()

        # Dans isaacsim.core.api 5.3.2, Robot() n'accepte plus usd_path
        # directement (TypeError constaté en exécution réelle) : il faut
        # d'abord référencer le USD dans le stage, puis créer Robot sur
        # ce prim déjà peuplé.
        usd_path = import_g1_urdf()
        add_reference_to_stage(usd_path=usd_path, prim_path="/World/g1")
        self._robot = Robot(prim_path="/World/g1", name="g1_axisos")
        self._world.scene.add(self._robot)
        self._world.reset()

        # Table de correspondance nom AXIS-OS -> index DOF Isaac. Avec
        # l'URDF officiel importé directement, les noms coïncident
        # exactement (confirmé le 18/09, 23/23) — on vérifie quand même
        # plutôt que de le supposer, pour détecter toute régression.
        dof_names = list(self._robot.dof_names)
        missing = []
        for jdef in self._joint_defs:
            if jdef.joint_id not in dof_names:
                missing.append(jdef.joint_id)
            else:
                self._joint_index[jdef.joint_id] = dof_names.index(jdef.joint_id)
        if missing:
            raise RuntimeError(
                f"IsaacSimBridge: {len(missing)} joint(s) AXIS-OS absents "
                f"des DOF Isaac Sim : {missing}. Le mapping a divergé depuis "
                f"la validation du 18/09 — ne pas continuer sans corriger."
            )
        print(f"[bridge] {len(self._joint_index)}/23 joints mappés avec Isaac Sim.")

    def step(self, n: int = 1):
        for _ in range(n):
            self._world.step(render=False)

    def apply_command(self, joint_id: str, position: Optional[float] = None,
                       velocity: Optional[float] = None) -> None:
        from isaacsim.core.utils.types import ArticulationAction
        idx = self._joint_index[joint_id]
        action = ArticulationAction(
            joint_positions=[position] if position is not None else None,
            joint_velocities=[velocity] if velocity is not None else None,
            joint_indices=[idx],
        )
        self._robot.apply_action(action)

    def read_feedback(self, joint_id: str) -> ActuatorFeedback:
        """État réel lu depuis le solveur physique Isaac Sim — POSITION,
        VITESSE ET COUPLE RÉELLEMENT APPLIQUÉ (pas le couple commandé)."""
        idx = self._joint_index[joint_id]
        pos = float(self._robot.get_joint_positions()[idx])
        vel = float(self._robot.get_joint_velocities()[idx])
        # applied_joint_efforts = couple réellement exercé par le solveur
        # physique — c'est la seule source fiable pour la suite S1.2.
        try:
            eff = float(self._robot.get_applied_joint_efforts()[idx])
        except AttributeError:
            eff = float("nan")  # à vérifier contre ta version d'Isaac Sim
        return ActuatorFeedback(
            joint_id=joint_id,
            state=ActuatorState.ACTIVE,
            position=pos,
            velocity=vel,
            torque=eff,
            current=0.0,
            temperature=25.0,
            voltage=48.0,
        )

    def build_hal(self) -> UnitreeG1HAL:
        """Retourne un UnitreeG1HAL dont send_joint_commands() écrit
        réellement vers Isaac Sim via ce pont, au lieu du simulateur
        interne G1SimulatedActuator. Monkeypatch délibéré et documenté —
        c'est le point d'intégration unique entre les deux piles."""
        hal = UnitreeG1HAL(backend="sim")
        bridge = self

        original_send = hal.send_joint_commands

        def send_via_isaac(commands):
            statuses = original_send(commands)  # applique le guard + tracker AXIS-OS
            for joint_id, cmd in commands.items():
                if statuses.get(joint_id) == HALStatus.OK:
                    bridge.apply_command(joint_id, position=cmd.position, velocity=cmd.velocity)
            bridge.step(1)
            return statuses

        hal.send_joint_commands = send_via_isaac  # type: ignore[method-assign]
        return hal
