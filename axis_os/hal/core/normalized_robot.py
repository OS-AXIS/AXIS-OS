"""
AXIS-OS — NormalizedRobotHAL
==============================
Facade normalisée : les apps appellent cette classe, jamais le driver brut.
Toutes les positions/vitesses sont en convention normalisée (positif = flexion).

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from dataclasses import replace
from typing import Optional, Dict, List

from .interfaces import IRobotHAL, IIMU, IForceTorqueSensor
from .types import (
    ActuatorCommand, ActuatorFeedback, HALStatus,
    JointDefinition, JointLimits
)
from .kinematic_normalizer import KinematicNormalizer
from .capabilities import CapabilitySet


class NormalizedRobotHAL:
    """
    Wrapper qui enveloppe n'importe quel IRobotHAL et expose
    une interface 100% normalisée aux applications du Play Store.
    """

    def __init__(self, robot_hal: IRobotHAL, normalizer: KinematicNormalizer):
        self._hal  = robot_hal
        self._norm = normalizer

    # ── Délégation directe ────────────────────────────────────────

    def initialize_all(self) -> HALStatus:
        return self._hal.initialize_all()

    def shutdown_all(self) -> HALStatus:
        return self._hal.shutdown_all()

    def emergency_stop_all(self) -> HALStatus:
        return self._hal.emergency_stop_all()

    def get_imu(self, imu_id: str = "base_imu") -> Optional[IIMU]:
        return self._hal.get_imu(imu_id)

    def get_motion_tracker(self):
        """
        Délègue à IRobotHAL.get_motion_tracker() — trouvé manquant par
        un test qui exerçait pour la première fois WatchdogSystem avec
        un NormalizedRobotHAL (plutôt qu'un UnitreeG1HAL brut, le seul
        chemin exercé jusqu'ici) : WatchdogSystem._check_acceleration()
        appelle self._hal.get_motion_tracker(), et NormalizedRobotHAL
        ne relayait jamais cet appel à son IRobotHAL sous-jacent —
        levait AttributeError à chaque cycle, capturé silencieusement
        comme un [BUG INTERNE WATCHDOG] plutôt que de jamais
        apparaître en usage normal (chaque test existant utilisait
        UnitreeG1HAL directement avec WatchdogSystem, sans jamais
        passer par cette couche de normalisation).
        """
        return self._hal.get_motion_tracker()

    def get_robot_info(self) -> Dict[str, str]:
        info = self._hal.get_robot_info()
        info["hal_normalized"] = "true"
        return info

    def get_capabilities(self) -> CapabilitySet:
        return self._hal.get_capabilities()

    # ── Commandes normalisées → brutes ────────────────────────────

    def send_joint_commands(
        self, normalized_commands: Dict[str, ActuatorCommand]
    ) -> Dict[str, HALStatus]:
        raw_commands = self._norm.commands_batch_to_raw(normalized_commands)
        return self._hal.send_joint_commands(raw_commands)

    # ── Feedbacks bruts → normalisés ─────────────────────────────

    def get_full_state(self) -> Dict[str, ActuatorFeedback]:
        raw_state = self._hal.get_full_state()
        return self._norm.feedbacks_batch_to_normalized(raw_state)

    # ── Limites normalisées ───────────────────────────────────────

    def get_joint_definitions(self) -> List[JointDefinition]:
        raw_defs = self._hal.get_joint_definitions()
        normalized_defs = []
        for jdef in raw_defs:
            try:
                norm_min, norm_max = self._norm.get_normalized_limits(
                    jdef.joint_id, jdef.limits.pos_min, jdef.limits.pos_max
                )
                norm_limits = JointLimits(
                    pos_min=norm_min, pos_max=norm_max,
                    vel_max=jdef.limits.vel_max,
                    torque_max=jdef.limits.torque_max,
                    current_max=jdef.limits.current_max,
                    temperature_max=jdef.limits.temperature_max,
                )
                # replace() plutôt qu'une reconstruction champ par champ :
                # seules les LIMITES sont normalisées ici, tout le reste
                # doit traverser intact. L'énumération manuelle perdait
                # silencieusement tout champ ajouté depuis — c'est ce qui
                # est arrivé à `group`, rendant le routage bras/jambes
                # inopérant dès que NormalizedRobotHAL était dans la
                # chaîne (c'est-à-dire toujours, en production).
                normalized_defs.append(replace(jdef, limits=norm_limits))
            except KeyError:
                normalized_defs.append(jdef)
        return normalized_defs

    @property
    def robot_id(self) -> str:
        return self._hal.robot_id

    @property
    def estop_active(self) -> bool:
        """Délègue à IRobotHAL.estop_active — voir son ajout (correctif
        E10, audit de sécurité) pour le contexte complet."""
        return self._hal.estop_active

    @property
    def is_initialized(self) -> bool:
        return self._hal.is_initialized
