"""
AXIS-OS — Politique de locomotion pré-entraînée du Unitree G1.

Enveloppe la politique officielle d'Unitree (unitree_rl_gym,
deploy/pre_train/g1/motion.pt) : une consigne de vitesse en entrée, des
positions cibles pour les 12 articulations des jambes en sortie.

C'est l'implémentation G1 de ce que le document d'architecture appelle
l'adaptateur de locomotion : Nav2 (ou un joystick) décide OÙ aller, cette
politique décide COMMENT poser les pieds.

UNE POLITIQUE ENTRAÎNÉE EST UNE CLÉ, PAS UN PROGRAMME. Si un seul des
paramètres ci-dessous diffère de son entraînement, le robot tombe — sans
aucune erreur. Tous proviennent textuellement de
unitree_rl_gym/deploy/deploy_mujoco/configs/g1.yaml et du code de
déploiement, commit 276801e, et ne doivent être modifiés que pour
correspondre à une AUTRE politique :

  - ordre des 12 articulations        (celui du modèle 12 DOF)
  - pose par défaut                    (légèrement accroupie)
  - gains PD                           (kp 100/150/40, kd 2/4/2)
  - échelles des observations          (0.25, 1.0, 0.05, ...)
  - fréquence : commande à 50 Hz, physique à 500 Hz
  - phase de marche                    (période 0.8 s)

TROIS POINTS OÙ UNE ERREUR NE LÈVE AUCUNE EXCEPTION :

1. Repère de la vitesse angulaire. La politique attend la vitesse
   angulaire de la base DANS LE REPÈRE DU ROBOT (convention MuJoCo, où
   elle a été vérifiée). Isaac Sim la rend DANS LE REPÈRE DU MONDE. Les
   deux coïncident presque tant que le robot regarde droit devant —
   puis dès qu'il tourne, les observations deviennent fausses et il
   tombe. body_angular_velocity() fait la conversion.

2. Convention des angles. La politique travaille en angles BRUTS de
   l'URDF, pas dans la convention normalisée d'AXIS-OS (9 articulations
   inversées). Vérifié : axes, origines et limites de position des 12
   articulations sont identiques entre l'URDF d'entraînement et celui
   d'AXIS-OS. Les consignes produites sont donc des angles bruts, à
   envoyer au HAL brut — PAS au NormalizedRobotHAL.

3. Effort des chevilles. Unitree publie deux valeurs contradictoires
   pour la même articulation, sous le même nom de fichier :
   unitree_ros dit 35 N·m, unitree_rl_gym (sur lequel cette politique
   a été entraînée) dit 50 N·m. AXIS-OS garde 35 N·m, la valeur
   prudente. Si le robot tombe en simulation faute de couple aux
   chevilles, c'est cette contradiction qui en est la cause, et elle
   doit être tranchée par la fiche technique du moteur avant tout essai
   réel.

Licence de la politique : BSD 3-Clause, Unitree Robotics.
"""
from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

# Empreinte de motion.pt au commit 276801e d'unitree_rl_gym. Une
# politique différente sous le même nom ferait marcher le robot
# autrement — ou le ferait tomber — sans qu'une ligne d'AXIS-OS bouge.
# Même principe que l'épinglage de transformers et des poids OWL-ViT.
G1_MOTION_PT_SHA256 = (
    "cf668f75b90d1abf73d2b87612a6e76bccc61ff7e083b63582d3f6aaa3c1759d"
)
G1_MOTION_PT_SOURCE = (
    "github.com/unitreerobotics/unitree_rl_gym @ 276801e "
    "deploy/pre_train/g1/motion.pt"
)


@dataclass(frozen=True)
class G1LocomotionConfig:
    """Paramètres d'entraînement de la politique. Ne pas modifier sans
    changer de politique : chacun est une dent de la clé."""

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

    # Haut du corps, TENU RIGIDEMENT pendant la marche. La politique a été
    # entraînée sur un G1 à 12 articulations : bras et taille n'existaient
    # pas pour elle. Gains repris du déploiement réel d'Unitree
    # (deploy_real/configs/g1.yaml), ramenés aux articulations du 23 DOF.
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
        return len(self.leg_joints)

    @property
    def decimation(self) -> int:
        return int(round(self.control_dt / self.physics_dt))


def quat_wxyz_to_matrix(q: Sequence[float]) -> np.ndarray:
    """Quaternion (w, x, y, z) -> matrice de rotation monde <- corps."""
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
    (vérifié par test), écrit sous la forme R^T·(0,0,-1) qui rend
    l'intention lisible : robot droit -> (0, 0, -1).
    """
    return quat_wxyz_to_matrix(q_wxyz).T @ np.array([0.0, 0.0, -1.0])


def body_angular_velocity(q_wxyz: Sequence[float],
                          omega_world: Sequence[float]) -> np.ndarray:
    """Vitesse angulaire du MONDE vers le repère du ROBOT : R^T·ω.

    Isaac Sim rend ω dans le repère du monde ; la politique l'attend
    dans celui du robot. Sans cette conversion, tout fonctionne tant que
    le robot regarde droit devant, puis il tombe dès qu'il tourne.
    """
    return quat_wxyz_to_matrix(q_wxyz).T @ np.asarray(omega_world, dtype=float)


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


class G1LocomotionPolicy:
    """Consigne de vitesse -> positions cibles BRUTES des jambes."""

    def __init__(self, policy_path: Optional[Path] = None,
                 cfg: G1LocomotionConfig = G1LocomotionConfig(),
                 expected_sha256: Optional[str] = G1_MOTION_PT_SHA256,
                 joint_limits: Optional[Dict[str, Tuple[float, float]]] = None):
        """
        joint_limits : {articulation: (min, max)} — si fourni, les
            consignes sont ramenées aux butées AVANT envoi, et chaque
            bornage est COMPTÉ (clamp_counts). Voir action_to_targets().
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
        """Charge la politique TorchScript après avoir vérifié son empreinte.

        Lève plutôt que de dégrader : une politique de marche qui n'est
        pas celle attendue n'a pas de mode dégradé acceptable — elle fait
        tomber le robot.
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
        self.last_action = np.zeros(self.cfg.num_actions, dtype=float)
        self.t = 0.0
        self.clamp_counts = {}

    # ---- observation ------------------------------------------------------

    def clamp_command(self, cmd: Sequence[float]) -> np.ndarray:
        """Borne la consigne aux vitesses vues à l'entraînement. Au-delà,
        la politique extrapole hors de son domaine — sans le dire."""
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
        """Assemble les 47 valeurs, dans l'ordre exact de l'entraînement :

            [0:3]   vitesse angulaire, repère robot       x 0.25
            [3:6]   gravité projetée, repère robot
            [6:9]   consigne (vx, vy, wz)                 x (2, 2, 0.25)
            [9:21]  position articulaire - pose défaut    x 1.0
            [21:33] vitesse articulaire                   x 0.05
            [33:45] action précédente (brute)
            [45:47] sin, cos de la phase de marche
        """
        c = self.cfg
        # ang_vel_frame est OBLIGATOIRE et sans valeur par défaut : le
        # repère de la vitesse angulaire doit être DÉCLARÉ par l'appelant.
        # En simulation, Isaac Sim rend le repère du monde ; une vraie
        # centrale inertielle rend celui du robot. Deviner — ou supposer
        # le monde par défaut — ferait convertir deux fois sur le vrai
        # robot, ce qui le fait tomber en tournant (mesuré).
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
        """Action brute du réseau -> positions cibles BRUTES des jambes.

        BORNAGE AUX BUTÉES, si joint_limits a été fourni. Constaté en
        simulation : cette politique vise régulièrement AU-DELÀ des
        butées des chevilles — jusqu'à 0.326 rad pour un roulis limité à
        0.262, un pas sur dix pour la cheville gauche. À l'entraînement,
        viser au-delà ne coûtait rien : la physique bloquait
        l'articulation. La politique l'a donc appris.

        Sans bornage, JointSafetyGuard refuse ces consignes — ce qui est
        correct, et sans effet mesurable sur la marche (2.15 m contre
        2.17 m sans garde-fou). Mais il en journalise plus de six par
        seconde, toutes bénignes : une violation réellement dangereuse
        pendant une marche serait noyée dans le bruit.

        Le garde-fou RESTE STRICT pour toutes les apps. C'est ici, dans
        l'adaptateur qui connaît cette politique, que la consigne est
        ramenée à la butée — ce qui correspond exactement à ce qu'elle a
        vécu à l'entraînement. Et chaque bornage est compté
        (clamp_counts), jamais silencieux.
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
        """Un pas de commande (50 Hz) : état -> cibles des jambes."""
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
        """(kp, kd) par articulation, jambes ET haut du corps tenu."""
        c = self.cfg
        out = {j: (float(kp), float(kd))
               for j, kp, kd in zip(c.leg_joints, c.kps, c.kds)}
        for j, kp, kd, _t in c.upper_body_hold:
            out[j] = (kp, kd)
        return out

    def upper_body_targets(self) -> Dict[str, float]:
        return {j: t for j, _kp, _kd, t in self.cfg.upper_body_hold}

    def default_pose(self) -> Dict[str, float]:
        pose = {j: float(v) for j, v in zip(self.cfg.leg_joints, self._default)}
        pose.update(self.upper_body_targets())
        return pose


class G1LocomotionAdapter:
    """ILocomotionAdapter du G1 : enveloppe G1LocomotionPolicy.

    Déclaré par le driver, conformément au principe d'universalité : le
    service de locomotion ne sait rien de la marche bipède.

    Un twist nul fait tourner la politique avec une consigne nulle : le
    robot s'immobilise en gardant l'équilibre. C'est l'arrêt sûr d'un
    bipède — couper les moteurs le ferait tomber.
    """

    def __init__(self, policy: "G1LocomotionPolicy", tilt_limit: float = -0.7,
                 gain_cap: float = 1.5, rotation_max: float = 0.6):
        self._policy = policy
        # Chute : gravité projetée z au-dessus de -0.7, soit plus de ~45°
        # d'inclinaison. Même seuil que les suites L1 et L2.
        self._tilt_limit = tilt_limit
        # MAINTIEN DE CAP — voir step().
        self._gain_cap = gain_cap
        self._rotation_max = rotation_max
        self._cap_ref: Optional[float] = None
        self.correction_cap = 0.0          # dernière correction, pour affichage

    @property
    def control_dt(self) -> float:
        return self._policy.cfg.control_dt

    @property
    def max_twist(self):
        from axis_os.services.locomotion.service import Twist
        m = self._policy.cfg.max_cmd
        return Twist(m[0], m[1], m[2])

    def reset(self) -> None:
        self._policy.reset()
        self._cap_ref = None
        self.correction_cap = 0.0

    @staticmethod
    def _lacet(q) -> float:
        w, x, y, z = (float(v) for v in q)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _maintien_cap(self, state, wz: float) -> float:
        """Rotation à commander pour TENIR LE CAP quand aucune rotation
        n'est demandée.

        POURQUOI, constaté en vidéo : arrêté par l'OS, le robot pivotait
        sur lui-même — plus de 90° en quatre secondes. Cette politique ne
        s'arrête jamais vraiment : sa phase de marche continue, donc le
        robot PIÉTINE sur place. Et elle ne connaît pas son cap — ses
        observations contiennent la vitesse de rotation et la gravité,
        jamais l'orientation absolue. Une petite asymétrie de piétinement
        s'accumule sans qu'elle puisse la percevoir.

        Correction : quand la consigne de rotation est nulle, on mémorise
        le cap, puis on commande une rotation inverse proportionnelle à la
        dérive. On passe par l'interface NORMALE de la politique — une
        consigne de vitesse —, sans toucher à ses sorties.

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
            # Orientation NON FINIE (centrale inertielle en défaut) : aucune
            # correction. POURQUOI : le bornage ci-dessous transforme NaN en
            # +rotation_max (min(m, nan) rend m) — le robot aurait pivoté à
            # la rotation maximale sur une mesure invalide. Le cap de
            # référence est oublié : il sera repris sur une mesure valide.
            self._cap_ref = None
            self.correction_cap = 0.0
            return 0.0
        c = max(-self._rotation_max, min(self._rotation_max,
                                         self._gain_cap * ecart))
        self.correction_cap = c
        return c

    def step(self, state, twist) -> Dict[str, float]:
        wz = self._maintien_cap(state, twist.wz)
        targets = self._policy.step(
            state.quat_wxyz, state.ang_vel_body,
            state.joint_pos, state.joint_vel,
            (twist.vx, twist.vy, wz),
            # BaseState porte le repère CORPS par contrat : une vraie
            # centrale inertielle le fournit tel quel, et la source de
            # simulation convertit avant de remplir l'état.
            ang_vel_frame="body",
        )
        targets.update(self._policy.upper_body_targets())
        return targets

    def is_fallen(self, state) -> bool:
        return float(projected_gravity(state.quat_wxyz)[2]) > self._tilt_limit
