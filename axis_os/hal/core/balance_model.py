"""
AXIS-OS — Balance Model Interface
====================================
Contrat universel pour le "cerveau d'équilibre" d'un robot.

Contexte de conception (discuté avec l'utilisateur) : l'équilibre d'un
robot humanoïde n'est jamais codé à la main — Boston Dynamics, Figure AI,
Tesla, Unitree entraînent tous un modèle (RL en simulation, MPC, ou
hybride) SPÉCIFIQUE à la géométrie et au poids de LEUR robot. Un modèle
entraîné pour le G1 ne fonctionne pas sur un Digit — poids, longueur de
jambes et puissance moteur différents cassent les calculs.

AXIS-OS ne doit donc PAS essayer de fournir un modèle d'équilibre
universel (impossible), mais l'INTERFACE que chaque fabricant remplit
avec SON modèle entraîné — exactement le même principe que ISimBackend
pour les simulateurs, ou IVLAConnector pour les modèles IA d'app.

Niveau d'exécution (hiérarchie définie avec l'utilisateur) :
  Niveau 0 — Réflexes purs (Watchdog, Safety)      — jamais désactivable
  Niveau 1 — Équilibre actif (CE MODULE)            — tourne en continu,
             jamais interrompu par une app, 10-1000Hz
  Niveau 2 — Locomotion (marche vers un point)       — futur, Nav2/SLAM
  Niveau 3 — Tâches applicatives (apps Play Store)    — RobotApp
  Niveau 4 — Décision haut niveau (VLA/LLM)            — IVLAConnector

Portée volontairement limitée à cette version (documentée honnêtement) :
  - Ce module fournit l'INTERFACE et un STUB déterministe, exactement
    comme OpenVLAConnector. Aucun vrai modèle entraîné n'existe encore —
    ça nécessite Isaac Sim/Isaac Gym pour l'entraînement RL, hors de
    portée sans serveur GPU.
  - Le stub ne fait AUCUNE correction réelle (renvoie toujours une
    correction nulle) — il existe pour valider que l'interface s'intègre
    proprement au reste du HAL, pas pour simuler un vrai comportement
    d'équilibre.

Licence : Apache 2.0 (SDK public — chaque fabricant implémente le sien)
"""

from __future__ import annotations
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Optional

from .types import IMUData, ActuatorFeedback


@dataclass
class BalanceCorrection:
    """
    Sortie d'un modèle d'équilibre : corrections de posture à appliquer,
    exprimées en angles cibles normalisés par joint — jamais en commandes
    brutes. Ces corrections passent quand même par JointSafetyGuard avant
    d'atteindre le HAL, exactement comme n'importe quelle autre commande —
    un modèle d'équilibre qui hallucine ne peut pas casser le robot.
    """
    joint_corrections: Dict[str, float] = field(default_factory=dict)
    confidence:        Optional[float]  = None
    is_stable:         bool             = True
    """
    is_stable=False signale une situation où le modèle estime le robot
    proche de la chute. Consommé par le Watchdog pour élever health_state,
    indépendamment de toute autre vérification — c'est un signal de
    sécurité, pas juste une donnée de posture.
    """


class IBalanceModel(ABC):
    """
    Contrat universel que CHAQUE fabricant implémente pour SON robot.

    AXIS-OS n'entraîne jamais ce modèle lui-même — voir la note de
    conception en tête de fichier. Le rôle d'AXIS-OS est de garantir
    que n'importe quelle implémentation de cette interface s'intègre
    au même pipeline de sécurité (SafetyGuard, Watchdog), quel que
    soit le fabricant qui l'a fournie.
    """

    @abstractmethod
    def load(self, model_path: str) -> bool:
        """
        Charge le modèle entraîné (fichier fourni par le fabricant,
        format libre côté implémentation — ONNX, TorchScript, etc.).
        Retourne False si le chargement échoue ; ne doit jamais lever
        d'exception pour un fichier absent ou corrompu — c'est une
        condition attendue, pas une erreur de programmation.
        """
        ...

    @abstractmethod
    def is_loaded(self) -> bool:
        """Health check — le modèle est-il prêt à produire des corrections ?"""
        ...

    @abstractmethod
    def compute_correction(
        self,
        imu_data: IMUData,
        joint_feedback: Dict[str, ActuatorFeedback],
    ) -> BalanceCorrection:
        """
        Prend l'état actuel du robot (IMU + feedback de tous les joints
        pertinents) et retourne les corrections de posture.

        Doit être appelable à haute fréquence (viser < 5ms d'exécution)
        puisque ceci tourne au Niveau 1, jamais interrompu. Une
        implémentation lente dégraderait la sécurité de tout le système,
        pas seulement ses propres performances.

        Ne doit jamais lever d'exception sur une donnée d'entrée
        malformée (NaN, capteur en panne) — retourner une
        BalanceCorrection avec is_stable=False est le comportement
        attendu, jamais un crash qui interromprait le Niveau 1.
        """
        ...


class NullBalanceModel(IBalanceModel):
    """
    Stub déterministe — équivalent de OpenVLAConnector pour l'équilibre.

    Ne fait AUCUNE correction réelle : renvoie toujours des corrections
    vides (joint_corrections={}), is_stable=True. Existe pour :
      - valider que l'interface s'intègre proprement au reste du HAL
        sans dépendre d'un vrai modèle entraîné
      - servir de valeur par défaut sûre : un robot sans modèle
        d'équilibre chargé ne doit jamais planter, juste ne rien
        corriger (le reste du pipeline de sécurité reste actif)

    Ce N'EST PAS un modèle d'équilibre fonctionnel — un robot utilisant
    NullBalanceModel ne recevra aucune assistance réelle au maintien
    de l'équilibre. Voir la note de portée en tête de fichier.
    """

    def __init__(self):
        self._loaded = False

    def load(self, model_path: str) -> bool:
        # Le stub "charge" toujours avec succès, quel que soit le chemin —
        # il n'y a rien à charger réellement. Un vrai fabricant vérifierait
        # ici l'existence et l'intégrité du fichier modèle.
        self._loaded = True
        return True

    def is_loaded(self) -> bool:
        return self._loaded

    def compute_correction(
        self, imu_data: IMUData, joint_feedback: Dict[str, ActuatorFeedback]
    ) -> BalanceCorrection:
        if not self._loaded:
            return BalanceCorrection(joint_corrections={}, is_stable=False,
                                     confidence=0.0)

        # Détection minimale de non-finitude sur l'IMU — même si ce stub
        # ne corrige rien, il ne doit jamais masquer une donnée capteur
        # invalide en la traitant comme une situation stable.
        acc = imu_data.linear_acceleration
        if not all(math.isfinite(v) for v in (acc.x, acc.y, acc.z)):
            return BalanceCorrection(joint_corrections={}, is_stable=False,
                                     confidence=0.0)

        return BalanceCorrection(joint_corrections={}, is_stable=True,
                                 confidence=1.0)
