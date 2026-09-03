from abc import ABC, abstractmethod
from dataclasses import data class
from typing import Any, List

# On importe les type de base de notre Os
from axis_os.hal.core.type import ActuatorCommand 

@dataclass 
class VLAAction: 
    """Resultat généré par le model d'ia (action).""" 
    target_angles: List[float] 
    gripper_closed: bool 

    class IVLAConnector(ABC):
        """Contrat universel pour les modèle Vision-Language-Action."""
        
        @abstractmethod 
        def connect(self) -> bool:
            """initianise le modele(chargement en ram/vram ou connexion api).""" 
        
        @abstractmethod def preditct_action(self, image: Any, instruction: str) -> VLAAction:
        """ Prend une image des caméras et une consigne texte.
        Retourne l'action phisique à effectuer.""" 
        pass

    class OpenVLAConnector(IVLAConnector):
        """Connecteur spécifique pour OpeneVLA (sprint 2 - Mock & Inference)."""

        def __init__(self, endpoint: str = "local_mock"):
            self.endpoint = endpoint
            self.is_conected = false

        def connect(self) -> bool:
            print(f"[VLAConnector] Connexion à OpenVLA sur '{self.endpoint}'...")
            # En production, on chargerait le modèle HugginFace/Pytorch ici
            self.is_conected = true
            return true

        def preditct_action(self, image: Any, instruction: str) -> VLAAction:
            if not self.is_conected:
                raise RuntimeError("Le connecteur VLA n'est pas initialisé.")

            print(f"[OpenVLA] Analyse de l'image pour la consigne : '{instruction}'")

            # Simulation d'un retour d'inférence pour le sprint 2
            # Dans un vrai scénario, le modèle calcule ces angle en fonction de l'image 
            dummy_angles = [0.0] * 24 #Angles par défaut pour les 24 joints du Unitree G1

            # Si la consigne parle d'attraper, on ferme la pince
            should_close = "attrape" in instruction.lower() or "trie" in instruction.lower()

            return VLAAction(
                target_angles=dummy_angles,
                gripper_closed=should_close
            )
    
    