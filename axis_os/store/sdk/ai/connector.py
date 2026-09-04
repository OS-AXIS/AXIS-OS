"""
AXIS-OS — VLA Connector
==========================
Interface optionnelle pour brancher un modèle Vision-Language-Action
(style Figure Helix, OpenVLA, RT-2) dans une app Play Store.

Principe validé avec l'utilisateur :
  - L'IA n'est JAMAIS dans l'OS/HAL. Elle est optionnelle, dans le SDK.
  - Une app scriptée (PackageSorting) n'a besoin d'aucun de ces fichiers.
  - Une app qui veut de l'IA connecte IVLAConnector à son modèle de choix.
  - Le HAL/Safety/Watchdog protègent le robot quelle que soit la source
    des commandes (script ou IA) — rien ici ne contourne la sécurité.

Usage typique dans une app :

    from axis_os.store.sdk.ai.connector import OpenVLAConnector

    class MyAIApp(RobotApp):
        def on_start(self, ctx):
            self._vla = OpenVLAConnector()
            return self._vla.connect()

        def run_mission(self, ctx, params):
            image = params.get("camera_frame")
            action = self._vla.predict_action(image, "range le carton rouge")
            # Les angles de l'IA passent quand même par le Safety Guard
            ctx.move_arms(action.to_commands(arm_joint_ids))

Licence : Apache 2.0 (SDK public)
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from axis_os.hal.core.types import ActuatorCommand, ControlMode


@dataclass
class VLAAction:
    """
    Résultat générique produit par un modèle VLA.

    target_angles : dict {joint_id: angle_normalisé} — pas une liste brute,
                    pour éviter toute ambiguïté sur l'ordre des joints.
    gripper_closed : consigne de préhension simple (ouvert/fermé).
    confidence     : score de confiance du modèle, si disponible (0.0-1.0).
    """
    target_angles:  Dict[str, float] = field(default_factory=dict)
    gripper_closed: bool             = False
    confidence:     Optional[float]  = None

    def to_commands(self, velocity: float = 1.0) -> Dict[str, ActuatorCommand]:
        """
        Convertit l'action IA en commandes HAL normalisées.
        Ces commandes passeront quand même par le JointSafetyGuard —
        une hallucination du modèle ne peut pas casser le robot.
        """
        return {
            joint_id: ActuatorCommand(
                mode=ControlMode.POSITION,
                position=angle,
                velocity=velocity,
            )
            for joint_id, angle in self.target_angles.items()
        }


class IVLAConnector(ABC):
    """
    Contrat universel pour tout modèle Vision-Language-Action.
    Une app peut brancher n'importe quel modèle (OpenVLA, Helix, GPT-4V,
    Llama local) tant qu'il implémente ce contrat.
    """

    @abstractmethod
    def connect(self) -> bool:
        """Initialise le modèle (chargement RAM/VRAM ou connexion API)."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Libère les ressources du modèle."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Health check — le modèle est-il prêt à répondre ?"""
        ...

    @abstractmethod
    def predict_action(self, image: Any, instruction: str) -> VLAAction:
        """
        Prend une image (caméra) et une consigne en langage naturel.
        Retourne l'action physique à effectuer.
        Lève RuntimeError si le connecteur n'est pas connecté.
        """
        ...


class OpenVLAConnector(IVLAConnector):
    """
    Connecteur pour OpenVLA (ou tout modèle compatible).

    Mode stub (par défaut) : aucune dépendance externe, retourne une
    action neutre déterministe — permet de développer et tester une
    app IA sans avoir le vrai modèle installé.

    Mode production : en Sprint 4, ce connecteur chargera le vrai
    modèle via transformers/torch. L'interface ne changera pas —
    les apps qui utilisent OpenVLAConnector n'auront rien à modifier.
    """

    def __init__(self, endpoint: str = "local_stub"):
        self._endpoint     = endpoint
        self._connected    = False

    def connect(self) -> bool:
        print(f"[VLAConnector] Connexion à OpenVLA sur '{self._endpoint}'")
        # Sprint 4 : chargement réel du modèle HuggingFace/PyTorch ici.
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def predict_action(self, image: Any, instruction: str) -> VLAAction:
        if not self._connected:
            raise RuntimeError(
                "OpenVLAConnector non connecté — appelez connect() d'abord."
            )

        print(f"[OpenVLA] Analyse de l'image pour la consigne : '{instruction}'")

        # Stub déterministe : action neutre, aucun mouvement réel.
        # Remplacé par une vraie inférence modèle en Sprint 4.
        should_grasp = any(
            kw in instruction.lower() for kw in ["attrape", "prends", "saisis", "trie"]
        )

        return VLAAction(
            target_angles={},   # neutre : le stub ne bouge rien par défaut
            gripper_closed=should_grasp,
            confidence=1.0 if should_grasp else 0.5,
        )
