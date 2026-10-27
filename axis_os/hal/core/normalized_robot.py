"""
AXIS-OS — NormalizedRobotHAL
==============================
Façade normalisée du HAL (Hardware Abstraction Layer, la couche qui cache
aux apps les différences entre robots) : les apps appellent cette classe,
jamais le pilote brut du robot. Toutes les positions et vitesses y sont
exprimées dans la convention normalisée (positif = flexion), traduite par
KinematicNormalizer (kinematic_normalizer.py).

Licence : Apache 2.0 (open-source)
"""

from __future__ import annotations
from dataclasses import replace
from typing import Optional, Dict, List

from .interfaces import IRobotHAL, IIMU, IForceTorqueSensor
from .types import (
    ActuatorCommand, ActuatorFeedback, BatteryState, HALStatus,
    JointDefinition, JointLimits
)
from .kinematic_normalizer import KinematicNormalizer
from .capabilities import CapabilitySet


class NormalizedRobotHAL:
    """
    Enveloppe n'importe quel IRobotHAL (l'interface commune à tous les
    pilotes de robot) et expose une interface entièrement normalisée aux
    applications du Play Store (la boutique d'apps d'AXIS-OS).

    Ce qui ne porte pas d'angle est relayé sans conversion :
    initialisation, arrêt, e-stop (arrêt d'urgence), IMU (centrale
    inertielle), informations et capacités du robot. Les commandes, les
    états et les limites des joints (les articulations) sont convertis au
    passage.
    """

    def __init__(self, robot_hal: IRobotHAL, normalizer: KinematicNormalizer):
        self._hal  = robot_hal
        self._norm = normalizer

    # ── Délégation directe, sans conversion ───────────────────────

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
        Relaie IRobotHAL.get_motion_tracker(), qui donne l'historique de
        mouvement de chaque joint (voir motion_state.py).

        Ce relais manquait. Un test a fait tourner pour la première fois
        WatchdogSystem (le chien de garde qui surveille le robot en
        continu) au-dessus d'un NormalizedRobotHAL, et non d'un
        UnitreeG1HAL brut, seul chemin exercé jusque-là. Or WatchdogSystem
        appelle self._hal.get_motion_tracker() (à l'époque depuis
        _check_acceleration(), aujourd'hui depuis _check_feedback_drift()),
        et cette couche ne relayait pas l'appel au IRobotHAL sous-jacent.
        Chaque cycle levait AttributeError, capturé silencieusement et
        signalé comme [BUG INTERNE WATCHDOG].
        Le défaut n'apparaissait jamais en usage normal, car chaque test
        existant branchait WatchdogSystem directement sur UnitreeG1HAL,
        sans passer par cette couche de normalisation.
        """
        return self._hal.get_motion_tracker()

    def get_robot_info(self) -> Dict[str, str]:
        """Informations du robot, plus hal_normalized="true" pour signaler
        que cette couche de normalisation est présente."""
        info = self._hal.get_robot_info()
        info["hal_normalized"] = "true"
        return info

    def get_capabilities(self) -> CapabilitySet:
        return self._hal.get_capabilities()

    def get_battery(self) -> Optional[BatteryState]:
        """Relaie IRobotHAL.get_battery() : la charge mesurée, ou None
        (inconnue)."""
        return self._hal.get_battery()

    # ── Commandes normalisées → brutes ────────────────────────────

    def send_joint_commands(
        self, normalized_commands: Dict[str, ActuatorCommand]
    ) -> Dict[str, HALStatus]:
        """Convertit les commandes normalisées en commandes brutes, puis
        les transmet au pilote."""
        raw_commands = self._norm.commands_batch_to_raw(normalized_commands)
        return self._hal.send_joint_commands(raw_commands)

    # ── Feedbacks bruts → normalisés ─────────────────────────────

    def get_full_state(self) -> Dict[str, ActuatorFeedback]:
        """État de tous les joints (retours capteurs, ou feedbacks), en
        convention normalisée."""
        raw_state = self._hal.get_full_state()
        return self._norm.feedbacks_batch_to_normalized(raw_state)

    # ── Limites normalisées ───────────────────────────────────────

    def get_joint_definitions(self) -> List[JointDefinition]:
        """Définitions des joints, avec des limites de position
        normalisées.

        Un joint absent de la table de conventions (KeyError) est rendu
        tel quel, sans conversion.
        """
        raw_defs = self._hal.get_joint_definitions()
        normalized_defs = []
        for jdef in raw_defs:
            try:
                norm_min, norm_max = self._norm.get_normalized_limits(
                    jdef.joint_id, jdef.limits.pos_min, jdef.limits.pos_max
                )
                # À savoir : JointLimits est, lui, reconstruit champ par
                # champ. Ses champs facultatifs (accel_max,
                # position_delta_max, current_is_measured) n'y sont pas
                # recopiés et reprennent leur valeur par défaut.
                norm_limits = JointLimits(
                    pos_min=norm_min, pos_max=norm_max,
                    vel_max=jdef.limits.vel_max,
                    torque_max=jdef.limits.torque_max,
                    current_max=jdef.limits.current_max,
                    temperature_max=jdef.limits.temperature_max,
                )
                # replace() plutôt qu'une reconstruction champ par champ :
                # seules les limites sont normalisées ici, tout le reste
                # doit traverser intact. L'énumération manuelle perdait
                # silencieusement tout champ ajouté depuis à
                # JointDefinition. C'est arrivé à `group` : le routage
                # bras/jambes ne fonctionnait plus dès que
                # NormalizedRobotHAL était dans la chaîne, c'est-à-dire
                # toujours, en production.
                normalized_defs.append(replace(jdef, limits=norm_limits))
            except KeyError:
                normalized_defs.append(jdef)
        return normalized_defs

    @property
    def robot_id(self) -> str:
        return self._hal.robot_id

    @property
    def estop_active(self) -> bool:
        """Relaie IRobotHAL.estop_active : l'état d'arrêt d'urgence
        (e-stop) global, positionné par emergency_stop_all().

        Cet état a été ajouté après qu'un audit de sécurité extérieur a
        montré qu'un e-stop déclenché par le Supervisor (qui prend le
        relais quand le watchdog est mort ou bloqué) restait invisible
        pour le reste du système. Contexte complet : IRobotHAL.estop_active
        dans interfaces.py, CHANGELOG.md (« audit externe, vague 2 ») et
        docs/audit_externe_2026-09-14.md (section sur l'e-stop du
        Supervisor).
        """
        return self._hal.estop_active

    @property
    def is_initialized(self) -> bool:
        return self._hal.is_initialized
