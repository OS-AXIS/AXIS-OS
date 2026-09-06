"""
AXIS-OS — Locomotion Policy Connector
========================================
Interface optionnelle pour brancher une politique de LOCOMOTION
pré-entraînée (marche, équilibre) dans une app Play Store.

Distincte de IVLAConnector (connector.py) : une politique de
locomotion ne prend PAS d'image caméra ni d'instruction en langage
naturel — elle prend l'état des capteurs proprioceptifs du robot
(positions/vitesses des joints, IMU) et retourne des cibles de joint
à chaque pas de contrôle, généralement à 30-60 Hz.

Origine : découverte le 2026-09-06 qu'un checkpoint de marche
pré-entraîné pour le G1 existe déjà publiquement (PPO, entraîné en
Isaac Gym, jambes+torse uniquement, bras figés) — voir
hardware-pathon-ai/unitree-g1-phase1-locomotion sur Hugging Face.
Plutôt que d'entraîner nous-mêmes une politique (semaines de calcul
GPU), ce connecteur permet à une app d'utiliser un checkpoint externe
existant, quelle qu'en soit la source (fabricant, communauté,
entraînement propre futur).

Principe repris de IVLAConnector, inchangé :
  - L'IA n'est JAMAIS dans l'OS/HAL. Elle est optionnelle, dans le SDK.
  - Une app scriptée n'a besoin d'aucun de ces fichiers.
  - Une app qui veut de la locomotion pilotée par IA connecte
    ILocomotionPolicy à son checkpoint de choix.
  - Le HAL/Safety/Watchdog protègent le robot quelle que soit la
    source des commandes (script, VLA, ou politique de locomotion) —
    rien ici ne contourne la sécurité. Une politique qui recommande
    une position hors limites est bloquée par JointSafetyGuard comme
    n'importe quelle autre commande.

Usage typique dans une app :

    from axis_os.store.sdk.ai.locomotion_connector import PPOLocomotionPolicy

    class WalkingApp(RobotApp):
        def on_start(self, ctx):
            self._policy = PPOLocomotionPolicy(checkpoint_path="model_5000.pt")
            return self._policy.connect()

        def run_mission(self, ctx, params):
            sensor_state = LocomotionObservation(
                joint_positions=ctx.get_full_state(),  # à adapter au vrai format
                ...
            )
            targets = self._policy.predict_joint_targets(sensor_state)
            # Les cibles de l'IA passent quand même par le Safety Guard
            ctx.move_legs(targets.to_commands())

Portée volontairement limitée à cette version :
  - Contrat pensé pour des politiques qui contrôlent jambes+torse
    (cohérent avec le checkpoint "phase1" trouvé, qui garde les bras
    figés). Un contrôle bras+mains complet nécessiterait un contrat
    différent ou étendu, non fait ici.
  - Aucune implémentation de production n'est encore branchée à un
    vrai fichier .pt dans cette version — PPOLocomotionPolicy reste
    un stub déterministe (voir sa docstring), exactement comme
    OpenVLAConnector l'était avant que son intégration réelle ne soit
    faite. Le chargement réel via torch.load() est le prochain travail
    à faire, une fois le mapping de joints entre le checkpoint et
    AXIS-OS confirmé (voir NOTE_JOINT_MISMATCH dans backend.py —
    un checkpoint entraîné sur 29 DOF pourrait ne pas correspondre
    directement aux 23-24 joints qu'AXIS-OS gère actuellement).

Licence : Apache 2.0 (SDK public)
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Optional

from axis_os.hal.core.types import ActuatorCommand, ControlMode


@dataclass
class LocomotionObservation:
    """
    État proprioceptif donné en entrée à une politique de locomotion,
    à chaque pas de contrôle.

    joint_positions/velocities : dict {joint_id: valeur}, mêmes clés
        que celles retournées par IRobotHAL.get_full_state() — pas
        une liste brute, pour éviter toute ambiguïté sur l'ordre.
    imu_orientation : quaternion (w, x, y, z) de l'IMU du robot, si
        disponible — nécessaire pour la plupart des politiques de
        marche modernes (elles ont besoin de savoir si le robot est
        en train de basculer).
    command_velocity : consigne de vitesse souhaitée (vx, vy, yaw_rate)
        — typiquement ce qu'une app ou un joystick demanderait
        ("avance à 0.3 m/s"), pas ce que le robot fait déjà.
    """
    joint_positions:  Dict[str, float] = field(default_factory=dict)
    joint_velocities: Dict[str, float] = field(default_factory=dict)
    imu_orientation:  Optional[tuple[float, float, float, float]] = None
    command_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class LocomotionAction:
    """
    Résultat produit par une politique de locomotion à chaque pas.

    target_positions : dict {joint_id: angle cible} — même format que
                        VLAAction.target_angles, pour cohérence avec
                        le reste du SDK.
    """
    target_positions: Dict[str, float] = field(default_factory=dict)

    def to_commands(self, velocity: float = 1.0) -> Dict[str, ActuatorCommand]:
        """
        Convertit l'action en commandes HAL normalisées. Comme pour
        VLAAction, ces commandes passeront quand même par le
        JointSafetyGuard — une politique qui recommande une position
        dangereuse (divergence d'entraînement, checkpoint corrompu,
        joint hors mapping) ne peut pas casser le robot.
        """
        return {
            joint_id: ActuatorCommand(
                mode=ControlMode.POSITION,
                position=angle,
                velocity=velocity,
            )
            for joint_id, angle in self.target_positions.items()
        }


class ILocomotionPolicy(ABC):
    """
    Contrat universel pour toute politique de locomotion pré-entraînée.
    Une app peut brancher n'importe quel checkpoint (PPO, imitation
    learning, règle codée à la main) tant qu'il implémente ce contrat.
    """

    @abstractmethod
    def connect(self) -> bool:
        """Charge le checkpoint (RAM/VRAM) ou initialise la politique."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Libère les ressources de la politique."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Health check — la politique est-elle prête à répondre ?"""
        ...

    @abstractmethod
    def predict_joint_targets(self, observation: LocomotionObservation) -> LocomotionAction:
        """
        Prend l'état proprioceptif courant du robot et retourne les
        cibles de joint pour le prochain pas de contrôle.
        Lève RuntimeError si la politique n'est pas connectée.
        """
        ...

    @abstractmethod
    def get_control_frequency_hz(self) -> float:
        """
        Fréquence à laquelle cette politique attend d'être appelée
        (typiquement 30-60 Hz pour une politique de marche RL). Une
        app doit respecter cette fréquence — appeler trop lentement
        dégrade la stabilité de la marche, appeler trop vite n'apporte
        rien (la politique retourne la même chose entre deux pas
        d'entraînement).
        """
        ...


class PPOLocomotionPolicy(ILocomotionPolicy):
    """
    Connecteur pour une politique de locomotion entraînée par PPO
    (Proximal Policy Optimization) et sauvegardée comme checkpoint
    PyTorch (.pt) — le format utilisé par la plupart des politiques
    Isaac Lab/Isaac Gym, dont hardware-pathon-ai/unitree-g1-phase1-locomotion.

    Mode stub (par défaut) : aucune dépendance externe, retourne les
    positions ACTUELLES inchangées (le robot reste immobile) — permet
    de développer et tester une app de locomotion sans avoir le vrai
    checkpoint chargé, exactement comme OpenVLAConnector le fait pour
    la manipulation.

    Mode production : chargera le vrai checkpoint via torch.load() et
    fera l'inférence réelle du réseau de neurones à chaque pas. NON
    IMPLÉMENTÉ dans cette version — le travail restant, documenté
    honnêtement plutôt que deviné :
      1. Confirmer le nombre et l'ORDRE exact des DOF attendus par le
         checkpoint (probablement 29 pour un G1 complet, à comparer
         aux 23-24 gérés par AXIS-OS — voir NOTE_JOINT_MISMATCH dans
         backend.py).
      2. Confirmer le format d'observation exact attendu (certains
         checkpoints Isaac Gym normalisent les entrées d'une façon
         précise — appeler predict_joint_targets() avec des valeurs
         brutes non normalisées produirait des résultats incohérents
         sans lever d'erreur explicite).
      3. Écrire la boucle d'inférence réelle (torch.no_grad(), passage
         du tenseur d'observation, extraction de l'action).
    Rien de tout cela n'a été vérifié contre le vrai checkpoint —
    seule son existence et sa description publique ont été consultées.
    """

    def __init__(self, checkpoint_path: str = "local_stub", control_frequency_hz: float = 50.0):
        self._checkpoint_path = checkpoint_path
        self._control_frequency_hz = control_frequency_hz
        self._connected = False

    def connect(self) -> bool:
        print(f"[LocomotionPolicy] Connexion au checkpoint '{self._checkpoint_path}'")
        # Production : torch.load(self._checkpoint_path) ici, une fois
        # le mapping de DOF confirmé (voir docstring de classe).
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def get_control_frequency_hz(self) -> float:
        return self._control_frequency_hz

    def predict_joint_targets(self, observation: LocomotionObservation) -> LocomotionAction:
        if not self._connected:
            raise RuntimeError(
                "PPOLocomotionPolicy non connectée — appelez connect() d'abord."
            )

        # Stub déterministe : retourne les positions ACTUELLES sans
        # changement — le robot reste immobile mais stable, jamais de
        # mouvement inventé. Remplacé par une vraie inférence PyTorch
        # une fois le mapping de DOF confirmé contre le vrai checkpoint.
        return LocomotionAction(target_positions=dict(observation.joint_positions))
