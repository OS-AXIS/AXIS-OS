"""
AXIS-OS — Locomotion Policy Connector
========================================
Interface optionnelle pour brancher, dans une app du Play Store (la
boutique d'apps installables sur le robot), une politique de LOCOMOTION
pré-entraînée, c'est-à-dire un programme de contrôle appris qui fait
marcher le robot et garde son équilibre.

Elle est distincte de IVLAConnector (connector.py), le contrat des modèles
VLA (vision-langage-action, qui reçoivent une image et une consigne en
langage naturel). Une politique de locomotion ne prend PAS d'image de
caméra ni de consigne en langage naturel : elle reçoit l'état des capteurs
proprioceptifs du robot (ceux qui mesurent son propre corps : positions et
vitesses des articulations, IMU, la centrale inertielle qui mesure
l'orientation) et renvoie des positions cibles pour les articulations à
chaque pas de contrôle, généralement 30 à 60 fois par seconde (30-60 Hz).

Origine : le 2026-09-06, nous avons découvert qu'un checkpoint de marche
(les poids d'un modèle sauvegardés après entraînement) existe déjà
publiquement pour le robot Unitree G1. Il a été entraîné par PPO (une
méthode d'apprentissage par renforcement) dans le simulateur Isaac Gym, et
ne commande que les jambes et le torse, bras figés : voir
hardware-pathon-ai/unitree-g1-phase1-locomotion sur Hugging Face (site de
partage de modèles). Plutôt que d'entraîner nous-mêmes une politique (des
semaines de calcul sur GPU), ce connecteur permet à une app d'utiliser un
checkpoint externe existant, quelle qu'en soit la source : fabricant,
communauté, ou futur entraînement propre.

Principes repris de IVLAConnector, inchangés :
  - L'IA ne fait JAMAIS partie de l'OS ni du HAL (la couche d'abstraction
    matérielle, seul chemin vers les moteurs). Elle est optionnelle et vit
    dans le SDK (le kit de développement fourni aux apps).
  - Une app scriptée n'a besoin d'aucun de ces fichiers.
  - Une app qui veut une locomotion pilotée par IA branche
    ILocomotionPolicy sur le checkpoint de son choix.
  - Le HAL, les gardes de sécurité et le Watchdog (la surveillance temps
    réel du robot, qui déclenche l'arrêt d'urgence quand un seuil
    dangereux est franchi) le protègent quelle que soit la source des
    commandes (script, modèle VLA ou politique de locomotion) : rien ici
    ne contourne la sécurité. Une politique qui recommande une position
    hors limites est bloquée par JointSafetyGuard (le garde de sécurité
    des articulations), comme n'importe quelle autre commande.

Usage typique dans une app :

    from axis_os.store.sdk.ai.locomotion_connector import (
        LocomotionObservation, PPOLocomotionPolicy)

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
            # Les cibles de l'IA passent quand même par le JointSafetyGuard
            ctx.move_legs(targets.to_commands())

Portée volontairement limitée dans cette version :
  - Le contrat est pensé pour des politiques qui commandent les jambes et le
    torse, comme le checkpoint « phase1 » trouvé, qui garde les bras figés.
    Commander aussi les bras et les mains demanderait un contrat différent
    ou étendu, non fait ici.
  - Aucune implémentation de production n'est encore branchée sur un vrai
    fichier .pt (format de sauvegarde de PyTorch) : PPOLocomotionPolicy
    reste un bouchon (stub), une version factice et déterministe (voir sa
    docstring), exactement comme OpenVLAConnector (le connecteur du modèle
    VLA OpenVLA, dans connector.py) l'était avant son intégration réelle.
    Le chargement réel par torch.load() est la prochaine étape, une fois
    confirmée la correspondance des articulations entre le checkpoint et
    AXIS-OS. Voir NOTE_JOINT_MISMATCH dans
    axis_os/hal/drivers/isaac_sim/backend.py : un checkpoint entraîné sur
    29 DOF (degrés de liberté, ici des articulations motorisées) pourrait
    ne pas correspondre directement aux 23-24 articulations qu'AXIS-OS gère
    actuellement.

Licence : Apache 2.0 (SDK public)
"""

from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from axis_os.hal.core.types import ActuatorCommand, ControlMode, ORIGINE_IA


@dataclass
class LocomotionObservation:
    """
    État proprioceptif (mesures du corps du robot) donné en entrée à une
    politique de locomotion, à chaque pas de contrôle.

    joint_positions/velocities : dictionnaires {joint_id: valeur}, où
        joint_id est l'identifiant d'une articulation, avec les mêmes clés
        que celles renvoyées par IRobotHAL.get_full_state() (l'interface du
        robot dans le HAL). Un dictionnaire plutôt qu'une liste brute, pour
        éviter toute ambiguïté sur l'ordre des articulations.
    imu_orientation : orientation du robot mesurée par l'IMU, sous forme de
        quaternion (w, x, y, z), une représentation de l'orientation en
        quatre nombres ; None si elle n'est pas disponible. La plupart des
        politiques de marche modernes en ont besoin pour savoir si le robot
        est en train de basculer.
    command_velocity : consigne de vitesse souhaitée (vx, vy, yaw_rate) :
        avance, déplacement latéral et vitesse de rotation autour de l'axe
        vertical. C'est typiquement ce qu'une app ou un joystick
        demanderait (« avance à 0.3 m/s »), et non ce que le robot fait
        déjà.
    """
    joint_positions:  Dict[str, float] = field(default_factory=dict)
    joint_velocities: Dict[str, float] = field(default_factory=dict)
    imu_orientation:  Optional[tuple[float, float, float, float]] = None
    command_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class LocomotionAction:
    """
    Résultat produit par une politique de locomotion à chaque pas.

    target_positions : dictionnaire {joint_id: angle cible}, au même format
                       que VLAAction.target_angles (l'action des modèles
                       VLA), par cohérence avec le reste du SDK.
    base_velocity    : vitesse de déplacement (vx, vy, wz) quand la
                       politique en propose une plutôt que des angles
                       (navigation) ; None sinon. Voir walk().

    Toute consigne tirée de cette action porte la marque ORIGINE_IA :
    to_commands(), move_to_pose() et walk() la posent, et l'OS exige alors
    ACTUATORS_AI_DRIVEN. Une app qui recopie les nombres à la main pour
    appeler ctx.walk() ou ctx.move_to_pose() sans la marque n'est pas
    détectée (SECURITY.md, limite n° 2).
    """
    target_positions: Dict[str, float] = field(default_factory=dict)
    base_velocity:    Optional[Tuple[float, float, float]] = None

    def move_to_pose(self, ctx, duration_s: float = 2.0, step_s: float = 0.05) -> bool:
        """Rejoint target_positions par ctx.move_to_pose(), marque IA
        comprise (ACTUATORS_AI_DRIVEN exigée)."""
        return ctx.move_to_pose(dict(self.target_positions), duration_s, step_s,
                                origine=ORIGINE_IA)

    def walk(self, ctx) -> bool:
        """Transmet base_velocity par ctx.walk(), marque IA comprise
        (ACTUATORS_AI_DRIVEN exigée). ValueError si l'action ne porte pas
        de vitesse."""
        if self.base_velocity is None:
            raise ValueError("cette action ne porte pas de vitesse (base_velocity)")
        vx, vy, wz = self.base_velocity
        return ctx.walk(vx, vy, wz, origine=ORIGINE_IA)

    def to_commands(self, velocity: float = 1.0) -> Dict[str, ActuatorCommand]:
        """
        Convertit l'action en commandes normalisées du HAL, en mode
        position, avec la vitesse `velocity` pour chaque articulation.

        Comme pour VLAAction, ces commandes passeront quand même par le
        JointSafetyGuard. Ainsi, une politique qui recommande une position
        dangereuse ne peut pas casser le robot, quelle qu'en soit la cause :
        entraînement qui a divergé, checkpoint corrompu, ou articulation
        absente de la table de correspondance entre le checkpoint et le
        robot.
        """
        return {
            joint_id: ActuatorCommand(
                mode=ControlMode.POSITION,
                position=angle,
                velocity=velocity,
                origine=ORIGINE_IA,
            )
            for joint_id, angle in self.target_positions.items()
        }


class ILocomotionPolicy(ABC):
    """
    Contrat universel pour toute politique de locomotion pré-entraînée.
    Une app peut brancher n'importe quel checkpoint (entraîné par PPO, par
    apprentissage par imitation de démonstrations, ou même une règle codée
    à la main) tant qu'il respecte ce contrat.
    """

    @abstractmethod
    def connect(self) -> bool:
        """Charge le checkpoint en mémoire (mémoire vive, ou mémoire du GPU)
        ou initialise la politique. Renvoie True si elle est prête."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Libère les ressources de la politique."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Contrôle de santé : la politique est-elle prête à répondre ?"""
        ...

    @abstractmethod
    def predict_joint_targets(self, observation: LocomotionObservation) -> LocomotionAction:
        """
        Prend l'état proprioceptif courant du robot et renvoie les
        positions cibles des articulations pour le prochain pas de
        contrôle. Lève RuntimeError si la politique n'est pas connectée.
        """
        ...

    @abstractmethod
    def get_control_frequency_hz(self) -> float:
        """
        Fréquence, en hertz (appels par seconde), à laquelle cette
        politique attend d'être appelée : typiquement 30 à 60 Hz pour une
        politique de marche entraînée par apprentissage par renforcement.
        Une app doit respecter cette fréquence. Appeler trop lentement
        dégrade la stabilité de la marche ; appeler trop vite n'apporte
        rien, car la politique renvoie la même chose entre deux pas
        d'entraînement.
        """
        ...


class PPOLocomotionPolicy(ILocomotionPolicy):
    """
    Connecteur pour une politique de locomotion entraînée par PPO
    (Proximal Policy Optimization, une méthode d'apprentissage par
    renforcement) et sauvegardée comme checkpoint PyTorch (.pt). C'est le
    format de la plupart des politiques entraînées dans les simulateurs
    Isaac Lab et Isaac Gym de NVIDIA, dont
    hardware-pathon-ai/unitree-g1-phase1-locomotion.

    Mode bouchon (stub ; c'est le mode par défaut, et pour l'instant le
    seul disponible) : aucune dépendance externe ; la politique renvoie
    les positions ACTUELLES inchangées, si bien que le robot reste
    immobile. Cela permet de développer et de tester une app de
    locomotion sans avoir chargé le vrai checkpoint, exactement comme
    OpenVLAConnector le fait pour la manipulation.

    Mode production : il chargera le vrai checkpoint par torch.load() et
    calculera à chaque pas la sortie réelle du réseau de neurones
    (l'inférence). Il n'est PAS IMPLÉMENTÉ dans cette version. Le travail
    restant est décrit tel quel plutôt que deviné :
      1. Confirmer le nombre et l'ORDRE exact des DOF (degrés de liberté,
         ici les articulations motorisées) attendus par le checkpoint :
         probablement 29 pour un G1 complet, à comparer aux 23-24 gérés
         par AXIS-OS (voir NOTE_JOINT_MISMATCH dans
         axis_os/hal/drivers/isaac_sim/backend.py).
      2. Confirmer le format d'observation exact attendu. Piège : certains
         checkpoints Isaac Gym normalisent leurs entrées d'une façon
         précise (mise à l'échelle des valeurs) ; appeler
         predict_joint_targets() avec des valeurs brutes non normalisées
         produirait des résultats incohérents, sans aucune erreur
         explicite.
      3. Écrire la boucle d'inférence réelle : calcul sous
         torch.no_grad() (sans mémoriser de quoi réentraîner le réseau),
         passage du tenseur d'observation (le tableau de nombres donné au
         réseau), extraction de l'action.
    Rien de tout cela n'a été vérifié contre le vrai checkpoint : seules
    son existence et sa description publique ont été consultées.
    """

    def __init__(self, checkpoint_path: str = "local_stub", control_frequency_hz: float = 50.0):
        self._checkpoint_path = checkpoint_path
        self._control_frequency_hz = control_frequency_hz
        self._connected = False

    def connect(self) -> bool:
        print(f"[LocomotionPolicy] Connexion au checkpoint '{self._checkpoint_path}'")
        # En production, torch.load(self._checkpoint_path) viendra ici, une
        # fois confirmée la correspondance des DOF entre le checkpoint et
        # AXIS-OS (voir la docstring de la classe).
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

        # Bouchon déterministe : un bouchon ne doit jamais inventer de
        # mouvement. Il renvoie donc les positions ACTUELLES sans
        # changement : le robot reste immobile, mais stable. Il sera
        # remplacé par une vraie inférence PyTorch une fois la
        # correspondance des DOF confirmée contre le vrai checkpoint.
        return LocomotionAction(target_positions=dict(observation.joint_positions))
