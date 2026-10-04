"""
Configuration de modalités GR00T pour le G1 d'AXIS-OS (corps « NEW_EMBODIMENT »).

À passer à l'entraînement de NVIDIA Isaac GR00T :
    --modality-config-path <AXIS-OS>/axis_os/vla/config_gr00t_g1.py

Calquée sur examples/SO100/so100_config.py du dépôt Isaac-GR00T (N1.7).
Les clés DOIVENT correspondre à meta/modality.json écrit par vers_lerobot.py :
    vidéo  : ego_view (caméra de tête)
    état   : bras_gauche (7), bras_droit (7)
    action : bras_gauche, bras_droit (positions RELATIVES à l'état : meilleure
             généralisation, recommandé par NVIDIA), base (vitesses vx, vy, wz,
             ABSOLUES)
Horizon : 16 pas à 10 i/s, soit 1.6 s de mouvement prédit à chaque appel.

Ce fichier importe gr00t : il ne s'exécute que dans l'environnement
d'entraînement de GR00T, pas dans AXIS-OS ni dans Isaac Sim.
"""
from gr00t.configs.data.embodiment_configs import register_modality_config
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.data.types import (
    ActionConfig,
    ActionFormat,
    ActionRepresentation,
    ActionType,
    ModalityConfig,
)

_BRAS = ActionConfig(rep=ActionRepresentation.RELATIVE, type=ActionType.NON_EEF,
                     format=ActionFormat.DEFAULT)
_BASE = ActionConfig(rep=ActionRepresentation.ABSOLUTE, type=ActionType.NON_EEF,
                     format=ActionFormat.DEFAULT)

g1_axis_os_config = {
    "video": ModalityConfig(delta_indices=[0], modality_keys=["ego_view"]),
    "state": ModalityConfig(delta_indices=[0], modality_keys=["bras_gauche", "bras_droit"]),
    "action": ModalityConfig(
        delta_indices=list(range(0, 16)),
        modality_keys=["bras_gauche", "bras_droit", "base"],
        action_configs=[_BRAS, _BRAS, _BASE],
    ),
    "language": ModalityConfig(delta_indices=[0],
                               modality_keys=["annotation.human.task_description"]),
}

register_modality_config(g1_axis_os_config, embodiment_tag=EmbodimentTag.NEW_EMBODIMENT)
