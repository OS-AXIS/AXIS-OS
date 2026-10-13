"""
Configuration de modalités GR00T pour le G1 d'AXIS-OS.

GR00T, le modèle VLA (Vision-Language-Action) de NVIDIA pour les robots
humanoïdes, apprend à partir de plusieurs « modalités » : la vidéo d'une
caméra, l'état du robot, les actions à produire et la consigne en langage
naturel. Ce fichier lui indique, pour l'humanoïde Unitree G1 piloté par
AXIS-OS, quelles données lire et comment interpréter les actions. Le G1
est déclaré comme un corps nouveau, que GR00T ne connaît pas d'avance
(étiquette « NEW_EMBODIMENT »).

À passer à l'entraînement de NVIDIA Isaac GR00T :
    --modality-config-path <AXIS-OS>/axis_os/vla/config_gr00t_g1.py

Le fichier est calqué sur examples/SO100/so100_config.py du dépôt
Isaac-GR00T (version N1.7). Les clés doivent correspondre exactement à
celles de meta/modality.json, écrit par vers_lerobot.py :
    vidéo  : ego_view (caméra de tête) ;
    état   : bras_gauche (7 valeurs), bras_droit (7 valeurs) ;
    action : bras_gauche et bras_droit, en positions RELATIVES à l'état
             (meilleure généralisation, recommandé par NVIDIA), et base,
             en vitesses ABSOLUES (vx, vy, wz : vitesse vers l'avant,
             vitesse latérale, vitesse de rotation).
Horizon : 16 pas à 10 images par seconde, soit 1,6 s de mouvement prédit
à chaque appel du modèle.

Ce fichier importe gr00t : il ne s'exécute que dans l'environnement
d'entraînement de GR00T, pas dans AXIS-OS ni dans Isaac Sim (le simulateur
de NVIDIA).
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

# Bras en positions relatives à l'état, base en vitesses absolues (voir
# l'en-tête). NON_EEF : actions exprimées articulation par articulation,
# et non en position de la main (EEF, « end effector », l'organe terminal).
_BRAS = ActionConfig(rep=ActionRepresentation.RELATIVE, type=ActionType.NON_EEF,
                     format=ActionFormat.DEFAULT)
_BASE = ActionConfig(rep=ActionRepresentation.ABSOLUTE, type=ActionType.NON_EEF,
                     format=ActionFormat.DEFAULT)

# delta_indices : les instants lus, en pas comptés à partir de l'instant
# courant. [0] : l'instant présent seul ; range(0, 16) : les 16 pas
# d'action prédits (l'horizon). La consigne en langage naturel est
# retrouvée par la clé "annotation.human.task_description" de
# meta/modality.json (voir vers_lerobot.py).
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
