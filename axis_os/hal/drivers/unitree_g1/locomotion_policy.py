"""
AXIS-OS — Politique de locomotion pré-entraînée du Unitree G1.

Ce module enveloppe la politique de marche officielle d'Unitree. Une
« politique » est ici un réseau de neurones entraîné par apprentissage
par renforcement : il reçoit l'état du robot et rend des consignes pour
les moteurs. Celle-ci vient du dépôt unitree_rl_gym (fichier
deploy/pre_train/g1/motion.pt) : elle prend en entrée une consigne de
vitesse et rend des positions cibles pour les 12 articulations des
jambes.

C'est l'implémentation G1 de ce que le document d'architecture appelle
l'adaptateur de locomotion : Nav2 (la pile de navigation de ROS 2) ou un
joystick décide où aller, cette politique décide comment poser les
pieds.

Une politique entraînée se comporte comme une clé, pas comme un
programme : si un seul des paramètres ci-dessous diffère de celui de
son entraînement, le robot tombe, sans qu'aucune erreur ne soit levée.
Tous sont repris textuellement de
unitree_rl_gym/deploy/deploy_mujoco/configs/g1.yaml et du code de
déploiement d'Unitree (commit 276801e). On ne les modifie que pour les
faire correspondre à une autre politique :

  - ordre des 12 articulations   (celui du modèle 12 DOF, c'est-à-dire à
                                  12 degrés de liberté : les jambes seules)
  - pose par défaut              (légèrement accroupie)
  - gains PD                     (raideur kp 100/150/40 et amortissement
                                  kd 2/4/2 de l'asservissement en position
                                  de chaque moteur)
  - échelles des observations    (0.25, 1.0, 0.05, ... : facteurs appliqués
                                  à chaque mesure avant de la donner au
                                  réseau)
  - fréquences                   (commande à 50 Hz, physique à 500 Hz)
  - phase de marche              (horloge interne du pas, période 0.8 s)

Trois points où une erreur ne lève aucune exception :

1. Repère de la vitesse angulaire. La politique attend la vitesse
   angulaire de la base (le bassin) exprimée dans le repère du robot :
   c'est la convention du simulateur MuJoCo, dans lequel elle a été
   vérifiée. Le simulateur Isaac Sim (NVIDIA) la rend dans le repère du
   monde. Les deux coïncident presque tant que le robot regarde droit
   devant ; dès qu'il tourne, les observations deviennent fausses et il
   tombe. body_angular_velocity() fait la conversion.

2. Convention des angles. La politique travaille sur les angles bruts de
   l'URDF (le fichier qui décrit le robot : articulations, axes,
   butées), et non dans la convention normalisée d'AXIS-OS, où le sens
   de 9 articulations est inversé. Il a été vérifié que les axes, les
   origines et les limites de position des 12 articulations sont
   identiques entre l'URDF d'entraînement et celui d'AXIS-OS. Les
   consignes produites sont donc des angles bruts, à envoyer au HAL brut
   (couche d'abstraction matérielle : l'interface qui parle aux
   moteurs), et surtout pas au NormalizedRobotHAL, qui attend des angles
   dans la convention normalisée.

3. Effort des chevilles. Unitree publie deux valeurs contradictoires de
   couple maximal pour la même articulation, sous le même nom de
   fichier : unitree_ros indique 35 N·m (newton-mètres), unitree_rl_gym
   (sur lequel cette politique a été entraînée) indique 50 N·m. AXIS-OS
   garde 35 N·m, la valeur prudente. Si le robot tombe en simulation
   faute de couple aux chevilles, c'est cette contradiction qui en est
   la cause ; elle doit être tranchée par la fiche technique du moteur
   avant tout essai sur le robot réel.

Licence de la politique : BSD 3-Clause, Unitree Robotics.
"""
from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

# Empreinte SHA-256 (somme de contrôle qui identifie le contenu exact d'un
# fichier) de motion.pt au commit 276801e d'unitree_rl_gym. Pourquoi
# l'épingler : une autre politique publiée sous le même nom ferait marcher
# le robot autrement, ou le ferait tomber, sans qu'une seule ligne
# d'AXIS-OS ait changé. Même principe que pour la bibliothèque
# transformers et les poids du détecteur d'objets OWL-ViT, eux aussi
# épinglés (voir axis_os/services/perception/owlvit_detector.py).
G1_MOTION_PT_SHA256 = (
    "cf668f75b90d1abf73d2b87612a6e76bccc61ff7e083b63582d3f6aaa3c1759d"
)
G1_MOTION_PT_SOURCE = (
    "github.com/unitreerobotics/unitree_rl_gym @ 276801e "
    "deploy/pre_train/g1/motion.pt"
)


@dataclass(frozen=True)
class G1LocomotionConfig:
    """Paramètres d'entraînement de la politique. Ne pas les modifier sans
    changer de politique : chacun est une dent de la clé (voir l'en-tête
    du module).

    Les vitesses de max_cmd sont, dans l'ordre, vx et vy en m/s et wz en
    rad/s. control_dt et physics_dt sont en secondes (50 Hz et 500 Hz).
    """

    leg_joints: Tuple[str, ...] = (
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    )
    kps: Tuple[float, ...] = (100, 100, 100, 150, 40, 40,
                              100, 100, 100, 150, 40, 40)
    kds: Tuple[float, ...] = (2, 2, 2, 4, 2, 2,
                              2, 2, 2, 4, 2, 2)
    default_angles: Tuple[float, ...] = (-0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
                                         -0.1, 0.0, 0.0, 0.3, -0.2, 0.0)
    ang_vel_scale: float = 0.25
    dof_pos_scale: float = 1.0
    dof_vel_scale: float = 0.05
    action_scale: float = 0.25
    cmd_scale: Tuple[float, float, float] = (2.0, 2.0, 0.25)
    max_cmd: Tuple[float, float, float] = (0.8, 0.5, 1.57)
    control_dt: float = 0.02
    physics_dt: float = 0.002
    gait_period: float = 0.8
    num_obs: int = 47

    # Haut du corps, tenu rigidement pendant la marche. Pourquoi : la
    # politique a été entraînée sur un G1 à 12 articulations ; pour elle,
    # bras et taille n'existaient pas. Les gains sont repris du déploiement
    # réel d'Unitree (deploy_real/configs/g1.yaml), ramenés aux
    # articulations du G1 23 DOF. Chaque entrée : (articulation, kp, kd,
    # position cible en radians).
    upper_body_hold: Tuple[Tuple[str, float, float, float], ...] = (
        ("waist_yaw_joint",            300.0, 3.0, 0.0),
        ("left_shoulder_pitch_joint",  100.0, 2.0, 0.0),
        ("left_shoulder_roll_joint",   100.0, 2.0, 0.0),
        ("left_shoulder_yaw_joint",     50.0, 2.0, 0.0),
        ("left_elbow_joint",            50.0, 2.0, 0.0),
        ("left_wrist_roll_joint",       20.0, 1.0, 0.0),
        ("right_shoulder_pitch_joint", 100.0, 2.0, 0.0),
        ("right_shoulder_roll_joint",  100.0, 2.0, 0.0),
        ("right_shoulder_yaw_joint",    50.0, 2.0, 0.0),
        ("right_elbow_joint",           50.0, 2.0, 0.0),
        ("right_wrist_roll_joint",      20.0, 1.0, 0.0),
    )

    @property
    def num_actions(self) -> int:
        """Nombre de sorties du réseau : une par articulation de jambe."""
        return len(self.leg_joints)

    @property
    def decimation(self) -> int:
        """Pas de physique par pas de commande (10 : 500 Hz / 50 Hz)."""
        return int(round(self.control_dt / self.physics_dt))


def quat_wxyz_to_matrix(q: Sequence[float]) -> np.ndarray:
    """Quaternion (w, x, y, z) -> matrice de rotation monde <- corps.

    Un quaternion est une représentation usuelle d'une orientation en 3D.
    Le quaternion est d'abord normalisé ; un quaternion nul donne
    l'identité plutôt qu'une division par zéro.
    """
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def projected_gravity(q_wxyz: Sequence[float]) -> np.ndarray:
    """Direction de la gravité exprimée dans le repère du robot.

    Équivalent exact de get_gravity_orientation() du code d'Unitree
    (vérifié par test), écrit sous la forme R^T·(0, 0, -1), qui rend
    l'intention lisible : robot droit -> (0, 0, -1). La composante z sert
    aussi à détecter une chute (voir G1LocomotionAdapter.is_fallen).
    """
    return quat_wxyz_to_matrix(q_wxyz).T @ np.array([0.0, 0.0, -1.0])


def body_angular_velocity(q_wxyz: Sequence[float],
                          omega_world: Sequence[float]) -> np.ndarray:
    """Vitesse angulaire ω, du repère du monde vers celui du robot : R^T·ω.

    Isaac Sim rend ω dans le repère du monde ; la politique l'attend dans
    celui du robot. Sans cette conversion, tout fonctionne tant que le
    robot regarde droit devant, puis il tombe dès qu'il tourne.
    """
    return quat_wxyz_to_matrix(q_wxyz).T @ np.asarray(omega_world, dtype=float)


def file_sha256(path: Path) -> str:
    """Empreinte SHA-256 d'un fichier, lu par blocs de 64 Kio pour ne pas
    le charger en mémoire d'un seul coup."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


class G1LocomotionPolicy:
    """Consigne de vitesse -> positions cibles brutes des jambes (angles de
    l'URDF, voir « Convention des angles » dans l'en-tête du module)."""

    def __init__(self, policy_path: Optional[Path] = None,
                 cfg: G1LocomotionConfig = G1LocomotionConfig(),
                 expected_sha256: Optional[str] = G1_MOTION_PT_SHA256,
                 joint_limits: Optional[Dict[str, Tuple[float, float]]] = None):
        """
        policy_path : chemin du fichier motion.pt.
        expected_sha256 : empreinte attendue du fichier ; None ou une
            chaîne vide désactive la vérification.
        joint_limits : {articulation: (min, max)}. S'il est fourni, les
            consignes sont ramenées aux butées avant envoi, et chaque
            bornage est compté (clamp_counts). Voir action_to_targets().
        """
        self.cfg = cfg
        self._policy_path = Path(policy_path) if policy_path else None
        self._expected_sha256 = expected_sha256
        self._policy = None
        self._default = np.array(cfg.default_angles, dtype=float)
        self._limits = dict(joint_limits) if joint_limits else None
        self.clamp_counts: Dict[str, int] = {}
        self.reset()

    # ---- cycle de vie ----------------------------------------------------

    def load(self) -> None:
        """Charge la politique TorchScript (format de PyTorch qui enregistre
        un réseau exécutable sans son code Python d'origine), après avoir
        vérifié son empreinte.

        Lève une exception plutôt que de passer en mode dégradé : une
        politique de marche qui n'est pas celle attendue n'a pas de mode
        dégradé acceptable, elle fait tomber le robot.
        """
        if self._policy_path is None or not self._policy_path.exists():
            raise FileNotFoundError(
                f"politique introuvable : {self._policy_path}. Source : "
                f"{G1_MOTION_PT_SOURCE}")
        if self._expected_sha256:
            actual = file_sha256(self._policy_path)
            if actual != self._expected_sha256:
                raise ValueError(
                    f"empreinte de la politique inattendue ({actual[:12]}… "
                    f"au lieu de {self._expected_sha256[:12]}…). Ce n'est pas "
                    f"la politique pour laquelle cette configuration a été "
                    f"écrite — refus de charger.")
        import torch
        self._policy = torch.jit.load(str(self._policy_path), map_location="cpu")
        self._policy.eval()

    def reset(self) -> None:
        """Remet à zéro l'action précédente, l'horloge de la phase de
        marche et les compteurs de bornage."""
        self.last_action = np.zeros(self.cfg.num_actions, dtype=float)
        self.t = 0.0
        self.clamp_counts = {}

    # ---- observation ------------------------------------------------------

    def clamp_command(self, cmd: Sequence[float]) -> np.ndarray:
        """Borne la consigne aux vitesses vues à l'entraînement (max_cmd).
        Pourquoi : au-delà, la politique extrapole hors de son domaine,
        sans le signaler."""
        c = np.asarray(cmd, dtype=float)
        m = np.asarray(self.cfg.max_cmd, dtype=float)
        return np.clip(c, -m, m)

    def build_observation(
        self,
        base_quat_wxyz: Sequence[float],
        base_ang_vel: Sequence[float],
        joint_pos: Dict[str, float],
        joint_vel: Dict[str, float],
        command: Sequence[float],
        *,
        ang_vel_frame: str,
    ) -> np.ndarray:
        """Assemble les 47 valeurs de l'observation (le vecteur d'entrée du
        réseau), dans l'ordre exact de l'entraînement :

            [0:3]   vitesse angulaire, repère robot       x 0.25
            [3:6]   gravité projetée, repère robot
            [6:9]   consigne (vx, vy, wz)                 x (2, 2, 0.25)
            [9:21]  position articulaire - pose défaut    x 1.0
            [21:33] vitesse articulaire                   x 0.05
            [33:45] action précédente (brute)
            [45:47] sin, cos de la phase de marche

        ang_vel_frame : repère dans lequel base_ang_vel est exprimée,
        'world' (monde) ou 'body' (robot).
        """
        c = self.cfg
        # ang_vel_frame est obligatoire et n'a pas de valeur par défaut :
        # l'appelant doit déclarer le repère de la vitesse angulaire.
        # Pourquoi : en simulation, Isaac Sim la rend dans le repère du
        # monde ; une vraie centrale inertielle (le capteur de rotation et
        # d'accélération du robot) la rend dans celui du robot. Deviner, ou
        # supposer le monde par défaut, ferait convertir deux fois sur le
        # vrai robot, ce qui le fait tomber en tournant (mesuré).
        if ang_vel_frame == "world":
            omega_body = body_angular_velocity(base_quat_wxyz, base_ang_vel)
        elif ang_vel_frame == "body":
            omega_body = np.asarray(base_ang_vel, dtype=float)
        else:
            raise ValueError(
                f"ang_vel_frame doit valoir 'world' ou 'body', pas "
                f"{ang_vel_frame!r} — le repère ne se devine pas")
        missing = [j for j in c.leg_joints if j not in joint_pos or j not in joint_vel]
        if missing:
            raise KeyError(f"articulations absentes de l'état : {missing}")

        qj = np.array([joint_pos[j] for j in c.leg_joints], dtype=float)
        dqj = np.array([joint_vel[j] for j in c.leg_joints], dtype=float)
        phase = (self.t % c.gait_period) / c.gait_period

        obs = np.zeros(c.num_obs, dtype=np.float32)
        n = c.num_actions
        obs[0:3] = omega_body * c.ang_vel_scale
        obs[3:6] = projected_gravity(base_quat_wxyz)
        obs[6:9] = self.clamp_command(command) * np.asarray(c.cmd_scale)
        obs[9:9 + n] = (qj - self._default) * c.dof_pos_scale
        obs[9 + n:9 + 2 * n] = dqj * c.dof_vel_scale
        obs[9 + 2 * n:9 + 3 * n] = self.last_action
        obs[9 + 3 * n:9 + 3 * n + 2] = (math.sin(2 * math.pi * phase),
                                        math.cos(2 * math.pi * phase))
        return obs

    # ---- action -----------------------------------------------------------

    def action_to_targets(self, action: Sequence[float]) -> Dict[str, float]:
        """Action brute du réseau -> positions cibles brutes des jambes
        (cible = action x action_scale + pose par défaut).

        Bornage aux butées, si joint_limits a été fourni. Pourquoi :
        constaté en simulation, cette politique vise régulièrement au-delà
        des butées des chevilles, jusqu'à 0.326 rad pour un roulis limité
        à 0.262 rad, un pas sur dix pour la cheville gauche. À
        l'entraînement, viser au-delà ne coûtait rien, puisque la physique
        bloquait l'articulation : la politique l'a donc appris.

        Sans bornage, JointSafetyGuard (le garde-fou d'AXIS-OS qui refuse
        toute consigne hors des butées) refuse ces consignes. C'est
        correct, et sans effet mesurable sur la marche (2.15 m parcourus,
        contre 2.17 m sans garde-fou). Mais il en journalise plus de six
        par seconde, toutes bénignes : une violation réellement dangereuse
        pendant une marche serait noyée dans ce bruit.

        Le garde-fou reste strict pour toutes les apps (les applications
        qui tournent sur AXIS-OS). C'est ici, dans l'adaptateur qui connaît
        cette politique, que la consigne est ramenée à la butée, ce qui
        correspond exactement à ce que la politique a vécu à
        l'entraînement. Et chaque bornage est compté (clamp_counts),
        jamais silencieux.
        """
        a = np.asarray(action, dtype=float)
        targets = a * self.cfg.action_scale + self._default
        out = {}
        for j, v in zip(self.cfg.leg_joints, targets):
            v = float(v)
            if self._limits is not None and j in self._limits:
                lo, hi = self._limits[j]
                if v < lo or v > hi:
                    self.clamp_counts[j] = self.clamp_counts.get(j, 0) + 1
                    v = min(max(v, lo), hi)
            out[j] = v
        return out

    def step(self, base_quat_wxyz, base_ang_vel, joint_pos, joint_vel,
             command, *, ang_vel_frame: str) -> Dict[str, float]:
        """Un pas de commande (50 Hz) : état -> cibles des jambes.

        Une action non finie, c'est-à-dire contenant NaN (« not a number »,
        résultat d'un calcul invalide) ou une valeur infinie, lève
        FloatingPointError : de telles valeurs ne doivent jamais atteindre
        les moteurs.
        """
        if self._policy is None:
            raise RuntimeError("politique non chargée — appeler load()")
        import torch
        obs = self.build_observation(base_quat_wxyz, base_ang_vel,
                                     joint_pos, joint_vel, command,
                                     ang_vel_frame=ang_vel_frame)
        with torch.no_grad():
            action = self._policy(torch.from_numpy(obs).unsqueeze(0))
        action = action.numpy().squeeze().astype(float)
        if not np.all(np.isfinite(action)):
            raise FloatingPointError(
                "la politique a produit une action non finie — refus de "
                "transmettre des consignes NaN/inf aux moteurs")
        self.last_action = action
        self.t += self.cfg.control_dt
        return self.action_to_targets(action)

    # ---- gains --------------------------------------------------------------

    def gains(self) -> Dict[str, Tuple[float, float]]:
        """(kp, kd) par articulation : jambes et haut du corps tenu."""
        c = self.cfg
        out = {j: (float(kp), float(kd))
               for j, kp, kd in zip(c.leg_joints, c.kps, c.kds)}
        for j, kp, kd, _t in c.upper_body_hold:
            out[j] = (kp, kd)
        return out

    def upper_body_targets(self) -> Dict[str, float]:
        """Positions fixes du haut du corps (voir upper_body_hold)."""
        return {j: t for j, _kp, _kd, t in self.cfg.upper_body_hold}

    def default_pose(self) -> Dict[str, float]:
        """Pose par défaut complète : jambes légèrement accroupies, haut du
        corps tenu."""
        pose = {j: float(v) for j, v in zip(self.cfg.leg_joints, self._default)}
        pose.update(self.upper_body_targets())
        return pose


class G1LocomotionAdapter:
    """ILocomotionAdapter du G1 : enveloppe G1LocomotionPolicy.

    ILocomotionAdapter est l'interface par laquelle le service de
    locomotion d'AXIS-OS confie à chaque modèle de robot la traduction
    d'une consigne de vitesse en consignes articulaires. C'est le driver
    qui la fournit, conformément au principe d'universalité : le service
    de locomotion ne sait rien de la marche bipède.

    Un twist nul (consigne de vitesse linéaire et angulaire nulle) fait
    tourner la politique avec une consigne nulle : le robot s'immobilise
    en gardant l'équilibre. C'est l'arrêt sûr d'un bipède ; couper les
    moteurs le ferait tomber.
    """

    def __init__(self, policy: "G1LocomotionPolicy", tilt_limit: float = -0.7,
                 gain_cap: float = 1.5, rotation_max: float = 0.6):
        self._policy = policy
        # Détection de chute : la composante z de la gravité projetée vaut
        # -1 robot droit et remonte vers 0 quand il se couche. Au-dessus de
        # -0.7, l'inclinaison dépasse environ 45°. Même seuil que les
        # essais de marche et de virage en simulation
        # (sim/validation/test_isaac_l1_locomotion.py et
        # test_isaac_l2_turning.py).
        self._tilt_limit = tilt_limit
        # Maintien de cap, voir _maintien_cap() : gain_cap est la rotation
        # commandée (rad/s) par radian de dérive, rotation_max le plafond
        # de cette correction (rad/s).
        self._gain_cap = gain_cap
        self._rotation_max = rotation_max
        self._cap_ref: Optional[float] = None
        self.correction_cap = 0.0          # dernière correction, pour affichage

    @property
    def control_dt(self) -> float:
        return self._policy.cfg.control_dt

    @property
    def max_twist(self):
        """Vitesses maximales acceptées : celles vues à l'entraînement."""
        from axis_os.services.locomotion.service import Twist
        m = self._policy.cfg.max_cmd
        return Twist(m[0], m[1], m[2])

    def reset(self) -> None:
        self._policy.reset()
        self._cap_ref = None
        self.correction_cap = 0.0

    @staticmethod
    def _lacet(q) -> float:
        """Lacet (rotation autour de l'axe vertical, c'est-à-dire le cap),
        en radians, tiré du quaternion (w, x, y, z)."""
        w, x, y, z = (float(v) for v in q)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _maintien_cap(self, state, wz: float) -> float:
        """Rotation à commander pour tenir le cap quand aucune rotation
        n'est demandée.

        Pourquoi, constaté en vidéo : arrêté par l'OS, le robot pivotait
        sur lui-même, de plus de 90° en quatre secondes. Cette politique ne
        s'arrête jamais vraiment : sa phase de marche continue, donc le
        robot piétine sur place. Et elle ne connaît pas son cap : ses
        observations contiennent la vitesse de rotation et la gravité,
        jamais l'orientation absolue. Une petite asymétrie du piétinement
        s'accumule donc sans qu'elle puisse la percevoir.

        Correction : quand la consigne de rotation est nulle, on mémorise
        le cap, puis on commande une rotation inverse proportionnelle à la
        dérive, plafonnée à rotation_max. On passe par l'interface normale
        de la politique, une consigne de vitesse, sans toucher à ses
        sorties.

        Dès qu'une rotation est demandée, on la transmet telle quelle et on
        oublie le cap de référence : le robot doit pouvoir tourner quand on
        le lui demande.
        """
        if abs(wz) > 1e-6:
            self._cap_ref = None
            self.correction_cap = 0.0
            return wz
        cap = self._lacet(state.quat_wxyz)
        if self._cap_ref is None:
            self._cap_ref = cap
        ecart = (self._cap_ref - cap + math.pi) % (2.0 * math.pi) - math.pi
        if not math.isfinite(ecart):
            # Orientation non finie (NaN ou infinie : centrale inertielle en
            # défaut) : aucune correction. Pourquoi : le bornage ci-dessous
            # transformerait NaN en +rotation_max (min(m, nan) rend m), et
            # le robot aurait pivoté à la rotation maximale sur une mesure
            # invalide. Le cap de référence est oublié : il sera repris sur
            # une mesure valide. Voir test/test_locomotion_non_finie.py.
            self._cap_ref = None
            self.correction_cap = 0.0
            return 0.0
        c = max(-self._rotation_max, min(self._rotation_max,
                                         self._gain_cap * ecart))
        self.correction_cap = c
        return c

    def step(self, state, twist) -> Dict[str, float]:
        """Un pas de commande : cibles des jambes calculées par la
        politique, haut du corps tenu à sa position fixe."""
        wz = self._maintien_cap(state, twist.wz)
        targets = self._policy.step(
            state.quat_wxyz, state.ang_vel_body,
            state.joint_pos, state.joint_vel,
            (twist.vx, twist.vy, wz),
            # BaseState (l'état du robot transmis par le service) porte,
            # par contrat, la vitesse angulaire dans le repère du corps :
            # une vraie centrale inertielle la fournit telle quelle, et la
            # source de simulation la convertit avant de remplir l'état.
            ang_vel_frame="body",
        )
        targets.update(self._policy.upper_body_targets())
        return targets

    def is_fallen(self, state) -> bool:
        """Vrai si l'inclinaison dépasse le seuil de chute (tilt_limit)."""
        return float(projected_gravity(state.quat_wxyz)[2]) > self._tilt_limit
