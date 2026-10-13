"""
AXIS-OS — Balance Model Interface
====================================
Contrat commun pour le « cerveau d'équilibre » d'un robot : le modèle qui
calcule les corrections de posture qui l'empêchent de tomber.

Pourquoi une interface et pas un modèle : l'équilibre d'un robot
humanoïde n'est jamais codé à la main. Boston Dynamics, Figure AI, Tesla
et Unitree entraînent tous un modèle (par apprentissage par
renforcement, ou RL, en simulation ; par commande prédictive, ou MPC ;
ou par un hybride des deux) propre à la géométrie et au poids de leur
robot. Un modèle entraîné pour le Unitree G1 ne fonctionne pas sur un
Digit (le robot d'Agility Robotics) : un poids, une longueur de jambes
et une puissance moteur différents faussent les calculs.

AXIS-OS ne cherche donc pas à fournir un modèle d'équilibre universel,
ce qui est impossible, mais l'interface que chaque fabricant remplit
avec son propre modèle entraîné. C'est exactement le principe
d'ISimBackend pour les simulateurs, ou d'IVLAConnector pour les modèles
d'IA des apps (VLA : modèle vision-langage-action, qui transforme ce que
voit le robot et une consigne en langage naturel en actions).

Niveaux d'exécution (hiérarchie retenue pour AXIS-OS) :
  Niveau 0 — Réflexes purs (Watchdog, Safety)      — jamais désactivable
  Niveau 1 — Équilibre actif (ce module)           — tourne en continu,
             jamais interrompu par une app, de 10 à 1000 Hz
  Niveau 2 — Locomotion (marcher vers un point)    — prévue pour plus
             tard, avec Nav2/SLAM, quand ce module a été écrit ; pour
             l'état actuel, voir axis_os/services/locomotion/ et
             axis_os/services/navigation/
  Niveau 3 — Tâches applicatives (apps Play Store) — RobotApp
  Niveau 4 — Décision de haut niveau (VLA/LLM)     — IVLAConnector

Le Watchdog (chien de garde) surveille le robot en continu et la couche
Safety vérifie chaque commande ; Nav2 est la pile de navigation de ROS 2
(Robot Operating System) ; SLAM désigne la cartographie et la
localisation simultanées ; le Play Store est la boutique d'apps
d'AXIS-OS ; un LLM est un grand modèle de langage.

Portée volontairement limitée dans cette version (documentée
honnêtement) :
  - Ce module fournit l'interface et un stub (une implémentation
    minimale et déterministe, qui ne fait rien de réel), exactement comme
    OpenVLAConnector. Aucun vrai modèle entraîné n'existe encore pour
    cette interface : il faudrait l'entraîner par RL dans Isaac Sim ou
    Isaac Gym (les simulateurs de NVIDIA), ce qui est hors de portée
    sans serveur GPU (processeur graphique de calcul).
  - Le stub ne fait aucune correction réelle (il renvoie toujours une
    correction nulle). Il existe pour valider que l'interface s'intègre
    proprement au reste du HAL (Hardware Abstraction Layer, la couche qui
    cache aux apps les différences entre robots), pas pour simuler un
    vrai comportement d'équilibre.
  - Aucun composant ne consomme encore BalanceCorrection en dehors des
    tests (test/test_balance_model.py) : son passage par JointSafetyGuard
    et la lecture de is_stable par le Watchdog, décrits plus bas, sont le
    branchement prévu.

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
    Sortie d'un modèle d'équilibre : les corrections de posture à
    appliquer, exprimées en angles cibles normalisés par joint (positif =
    flexion, voir kinematic_normalizer.py), jamais en commandes brutes.

    Ces corrections passent quand même par JointSafetyGuard (le garde qui
    vérifie chaque commande contre les limites du joint) avant d'atteindre
    le HAL, exactement comme n'importe quelle autre commande : un modèle
    d'équilibre qui hallucine (produit une sortie aberrante) ne peut pas
    casser le robot.

    confidence : confiance du modèle dans sa correction (le stub rend 1.0
    ou 0.0) ; None si le modèle n'en fournit pas.
    """
    joint_corrections: Dict[str, float] = field(default_factory=dict)
    confidence:        Optional[float]  = None
    is_stable:         bool             = True
    """
    is_stable=False signale que le modèle estime le robot proche de la
    chute. C'est un signal de sécurité, pas une simple donnée de posture :
    il est destiné au Watchdog, pour élever health_state (l'état de santé
    global du robot), indépendamment de toute autre vérification.
    """


class IBalanceModel(ABC):
    """
    Contrat que chaque fabricant implémente pour son robot.

    AXIS-OS n'entraîne jamais ce modèle lui-même (voir la note de
    conception en tête de fichier). Son rôle est de garantir que
    n'importe quelle implémentation de cette interface s'intègre au même
    pipeline de sécurité (SafetyGuard, Watchdog), quel que soit le
    fabricant qui l'a fournie.
    """

    @abstractmethod
    def load(self, model_path: str) -> bool:
        """
        Charge le modèle entraîné : un fichier fourni par le fabricant,
        dans le format de son choix (ONNX, format d'échange standard de
        réseaux de neurones ; TorchScript, format de PyTorch ; etc.).

        Retourne False si le chargement échoue. Ne doit jamais lever
        d'exception pour un fichier absent ou corrompu : c'est une
        condition attendue, pas une erreur de programmation.
        """
        ...

    @abstractmethod
    def is_loaded(self) -> bool:
        """Contrôle de santé : le modèle est-il prêt à produire des
        corrections ?"""
        ...

    @abstractmethod
    def compute_correction(
        self,
        imu_data: IMUData,
        joint_feedback: Dict[str, ActuatorFeedback],
    ) -> BalanceCorrection:
        """
        Prend l'état actuel du robot (IMU, la centrale inertielle qui
        mesure accélérations et rotations du corps, et retours capteurs de
        tous les joints pertinents) et retourne les corrections de
        posture.

        Doit pouvoir être appelée à haute fréquence (viser moins de 5 ms
        d'exécution), puisqu'elle tourne au niveau 1, jamais interrompu.
        Une implémentation lente dégraderait la sécurité de tout le
        système, pas seulement ses propres performances.

        Ne doit jamais lever d'exception sur une donnée d'entrée malformée
        (NaN, « Not a Number », capteur en panne) : le comportement
        attendu est de retourner une BalanceCorrection avec
        is_stable=False, jamais un plantage qui interromprait le niveau 1.
        """
        ...


class NullBalanceModel(IBalanceModel):
    """
    Stub déterministe : l'équivalent d'OpenVLAConnector pour l'équilibre.

    Il ne fait aucune correction réelle : une fois chargé, il renvoie
    toujours des corrections vides (joint_corrections={}) avec
    is_stable=True. S'il n'est pas chargé, ou si l'accélération mesurée
    par l'IMU n'est pas finie, il renvoie is_stable=False et une
    confiance de 0.0.

    Il existe pour :
      - valider que l'interface s'intègre proprement au reste du HAL sans
        dépendre d'un vrai modèle entraîné ;
      - servir de valeur par défaut sûre : un robot sans modèle
        d'équilibre chargé ne doit jamais planter, seulement ne rien
        corriger (le reste du pipeline de sécurité reste actif).

    Ce n'est PAS un modèle d'équilibre fonctionnel : un robot qui utilise
    NullBalanceModel ne reçoit aucune aide réelle au maintien de
    l'équilibre. Voir la note de portée en tête de fichier.
    """

    def __init__(self):
        self._loaded = False

    def load(self, model_path: str) -> bool:
        # Le stub « charge » toujours avec succès, quel que soit le
        # chemin : il n'y a rien à charger. Un vrai fabricant vérifierait
        # ici l'existence et l'intégrité du fichier modèle.
        self._loaded = True
        return True

    def is_loaded(self) -> bool:
        return self._loaded

    def compute_correction(
        self, imu_data: IMUData, joint_feedback: Dict[str, ActuatorFeedback]
    ) -> BalanceCorrection:
        # Sans modèle chargé, rien ne permet d'affirmer que le robot est
        # stable.
        if not self._loaded:
            return BalanceCorrection(joint_corrections={}, is_stable=False,
                                     confidence=0.0)

        # Contrôle minimal des valeurs non finies (NaN, infini) de l'IMU,
        # limité à l'accélération linéaire : même si ce stub ne corrige
        # rien, il ne doit jamais masquer une donnée capteur invalide en
        # la traitant comme une situation stable.
        acc = imu_data.linear_acceleration
        if not all(math.isfinite(v) for v in (acc.x, acc.y, acc.z)):
            return BalanceCorrection(joint_corrections={}, is_stable=False,
                                     confidence=0.0)

        return BalanceCorrection(joint_corrections={}, is_stable=True,
                                 confidence=1.0)
