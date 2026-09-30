"""
AXIS-OS — Politique de loco-manipulation GR00T Whole-Body Control (NVIDIA)
pour le Unitree G1 29 DOF.

Remplace, pour le 29 DOF, la politique de marche d'Unitree : elle
commande les jambes ET la taille (15 articulations), en OBSERVANT les 29
articulations du corps — bras compris. C'est ce qui lui permet de
compenser une charge tenue devant le corps. Les bras restent commandés
par l'appelant.

Analyse préalable et sources : docs/integration_gr00t_wbc.md.
Référence : NVlabs/GR00T-WholeBodyControl @ b042411,
decoupled_wbc/control/policy/g1_gear_wbc_policy.py — le code de
DÉPLOIEMENT, pas le script de démonstration MuJoCo, incohérent (il
mélange les 14 articulations des doigts aux 29 du corps).

Licence des poids : NVIDIA Open Model License — différente de la BSD
d'Unitree, à lire avant tout usage commercial.

QUATRE PIÈGES, chacun silencieux, chacun traité et testé :

1. ORDRE DE L'HISTORIQUE. Six observations empilées, la plus ANCIENNE en
   premier. Au démarrage, les cases manquantes sont des zéros À GAUCHE.
   Inverser l'ordre donne à la politique un passé à l'envers.

2. HISTORIQUE À RÉINITIALISER à chaque reprise. Sinon la politique reçoit
   un passé qui appartient à une autre situation — un robot soulevé puis
   reposé, par exemple.

3. 29 ARTICULATIONS OBSERVÉES, 15 COMMANDÉES. Les positions sont
   centrées sur la pose par défaut pour les 15 commandées, et sur ZÉRO
   pour les 14 des bras. Centrer les bras sur autre chose décale tout ce
   que la politique croit savoir d'eux.

4. REPÈRE DE LA VITESSE ANGULAIRE, obligatoire à déclarer — même règle
   que pour la politique d'Unitree, pour la même raison mesurée : une
   erreur de repère fait tomber le robot en tournant.
"""
from __future__ import annotations

import collections
import hashlib
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, Optional, Sequence, Tuple

import numpy as np

from axis_os.hal.drivers.unitree_g1.locomotion_policy import (
    body_angular_velocity, projected_gravity,
)

# Ordre moteur Unitree des 29 articulations du corps — celui de GR00T WBC
# et, vérifié par test, celui de l'URDF 29 DOF chargé par AXIS-OS.
G1_29_BODY_JOINTS: Tuple[str, ...] = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)


# Empreintes des politiques VALIDÉES en simulation (test W1, 2026-09-25 :
# debout, posture de port, marche en port, arrêt — tout tenu). Un fichier
# différent est refusé au chargement : ce n'est plus celui qu'on a validé.
GR00T_WBC_SHA256 = {
    "balance": "f645da599d4ca3d29ed273c8f4712620bb680d34977469ca3aeabe5bb9631c18",
    "walk":    "7c82255b6905ffcc4468fa7f8ddcf7b70db168cf1042107ccab887cb6a8e5407",
}


@dataclass(frozen=True)
class GR00TWBCConfig:
    """Paramètres d'ENTRAÎNEMENT — à ne pas modifier sans changer de
    politique. Relevés dans g1_gear_wbc.yaml."""
    body_joints: Tuple[str, ...] = G1_29_BODY_JOINTS
    num_actions: int = 15                      # 12 jambes + 3 taille
    default_angles: Tuple[float, ...] = (
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
        0.0, 0.0, 0.0)
    kps: Tuple[float, ...] = (150, 150, 150, 200, 40, 40,
                              150, 150, 150, 200, 40, 40,
                              250, 250, 250)
    kds: Tuple[float, ...] = (2, 2, 2, 4, 2, 2,
                              2, 2, 2, 4, 2, 2,
                              5, 5, 5)
    ang_vel_scale: float = 0.5
    dof_pos_scale: float = 1.0
    dof_vel_scale: float = 0.05
    action_scale: float = 0.25
    cmd_scale: Tuple[float, float, float] = (2.0, 2.0, 0.5)
    single_obs_dim: int = 86
    history_len: int = 6
    height_default: float = 0.74
    physics_dt: float = 0.005                  # 200 Hz
    decimation: int = 4                        # -> 50 Hz
    # En dessous de cette norme de consigne, la politique d'ÉQUILIBRE est
    # utilisée — un vrai mode debout, contrairement à la politique
    # d'Unitree qui piétinait en permanence.
    seuil_equilibre: float = 0.05
    # Vitesses maximales : NON documentées par NVIDIA. Valeurs prudentes,
    # à confirmer en simulation. Au-delà du domaine d'entraînement, une
    # politique apprise extrapole sans le dire.
    max_cmd: Tuple[float, float, float] = (0.6, 0.3, 0.8)

    @property
    def control_dt(self) -> float:
        return self.physics_dt * self.decimation

    @property
    def controlled_joints(self) -> Tuple[str, ...]:
        return self.body_joints[:self.num_actions]

    @property
    def num_obs(self) -> int:
        return self.single_obs_dim * self.history_len


class GR00TWBCPolicy:
    """Deux réseaux ONNX — Balance et Walk — et leur observation."""

    def __init__(self, balance_path: Optional[Path] = None,
                 walk_path: Optional[Path] = None,
                 cfg: GR00TWBCConfig = GR00TWBCConfig(),
                 sha256: Optional[Dict[str, str]] = None):
        """
        sha256 : empreintes attendues {"balance": ..., "walk": ...}. None
            tant qu'elles n'ont pas été relevées sur les fichiers
            téléchargés : le chargement AVERTIT alors, sans refuser.
        """
        self.cfg = cfg
        self._paths = {"balance": balance_path, "walk": walk_path}
        self._sha256 = GR00T_WBC_SHA256 if sha256 is None else sha256
        self._nets: Dict[str, Callable[[np.ndarray], np.ndarray]] = {}
        n = len(cfg.body_joints)
        # Centrage des positions : pose par défaut pour les 15 commandées,
        # ZÉRO pour les 14 des bras — voir piège 3.
        self._centre = np.zeros(n, dtype=np.float32)
        self._centre[:cfg.num_actions] = cfg.default_angles
        self.reset()

    # ---- cycle de vie ----------------------------------------------------

    def reset(self) -> None:
        """À appeler à CHAQUE reprise — voir piège 2."""
        self.last_action = np.zeros(self.cfg.num_actions, dtype=np.float32)
        self._history: collections.deque = collections.deque(
            maxlen=self.cfg.history_len)
        self.derniere_politique: Optional[str] = None

    def load(self) -> None:
        import onnxruntime as ort
        for nom, chemin in self._paths.items():
            if chemin is None:
                raise FileNotFoundError(f"chemin de la politique '{nom}' absent")
            chemin = Path(chemin)
            empreinte = hashlib.sha256(chemin.read_bytes()).hexdigest()
            attendue = self._sha256.get(nom)
            if attendue and empreinte != attendue:
                raise ValueError(
                    f"politique '{nom}' : empreinte {empreinte} au lieu de "
                    f"{attendue} — fichier différent de celui validé, refusé")
            if not attendue:
                print(f"  [gr00t-wbc] AVERTISSEMENT : empreinte de '{nom}' non "
                      f"épinglée — relevée : {empreinte}")
            sess = ort.InferenceSession(str(chemin))
            entree = sess.get_inputs()[0]
            dim = entree.shape[-1]
            if isinstance(dim, int) and dim != self.cfg.num_obs:
                raise ValueError(
                    f"politique '{nom}' attend {dim} entrées, la configuration "
                    f"en prévoit {self.cfg.num_obs} — refusée plutôt que "
                    f"nourrie de travers")
            nom_entree = entree.name
            self._nets[nom] = (lambda s, n: lambda x: s.run(
                None, {n: x.astype(np.float32)[None, :]})[0][0])(sess, nom_entree)

    def utiliser_reseaux(self, balance: Callable, walk: Callable) -> None:
        """Injection de réseaux — pour les tests, sans fichier ONNX."""
        self._nets = {"balance": balance, "walk": walk}

    # ---- observation -----------------------------------------------------

    def build_observation(self, base_quat_wxyz: Sequence[float],
                          base_ang_vel: Sequence[float],
                          joint_pos: Dict[str, float],
                          joint_vel: Dict[str, float],
                          command: Sequence[float],
                          height: float,
                          rpy: Sequence[float],
                          *, ang_vel_frame: str) -> np.ndarray:
        """Observation UNITAIRE de 86 valeurs."""
        c = self.cfg
        if ang_vel_frame == "world":
            omega = body_angular_velocity(base_quat_wxyz, base_ang_vel)
        elif ang_vel_frame == "body":
            omega = np.asarray(base_ang_vel, dtype=float)
        else:
            raise ValueError(f"ang_vel_frame doit valoir 'world' ou 'body', "
                             f"pas {ang_vel_frame!r} — le repère ne se devine pas")
        manquantes = [j for j in c.body_joints
                      if j not in joint_pos or j not in joint_vel]
        if manquantes:
            raise KeyError(f"articulations absentes de l'état : {manquantes} — "
                           f"les 29 sont observées, bras compris")

        q = np.array([joint_pos[j] for j in c.body_joints], dtype=np.float32)
        dq = np.array([joint_vel[j] for j in c.body_joints], dtype=np.float32)
        n = len(c.body_joints)

        obs = np.zeros(c.single_obs_dim, dtype=np.float32)
        obs[0:3] = np.asarray(command, dtype=np.float32) * np.asarray(c.cmd_scale)
        obs[3] = height
        obs[4:7] = rpy
        obs[7:10] = omega * c.ang_vel_scale
        obs[10:13] = projected_gravity(base_quat_wxyz)
        obs[13:13 + n] = (q - self._centre) * c.dof_pos_scale
        obs[13 + n:13 + 2 * n] = dq * c.dof_vel_scale
        obs[13 + 2 * n:13 + 2 * n + c.num_actions] = self.last_action
        return obs

    def empiler(self, obs: np.ndarray) -> np.ndarray:
        """Ajoute une observation à l'historique et rend l'entrée complète.

        Plus ANCIENNE en premier ; au démarrage, zéros À GAUCHE — voir
        piège 1. Relevé dans set_observation() du code de déploiement.
        """
        self._history.append(np.asarray(obs, dtype=np.float32))
        while len(self._history) < self.cfg.history_len:
            self._history.appendleft(np.zeros_like(self._history[-1]))
        return np.concatenate(list(self._history))

    # ---- un pas de commande ---------------------------------------------

    def step(self, base_quat_wxyz, base_ang_vel, joint_pos, joint_vel,
             command, height: Optional[float] = None,
             rpy: Sequence[float] = (0.0, 0.0, 0.0),
             *, ang_vel_frame: str) -> Dict[str, float]:
        """Un pas à 50 Hz : état -> cibles des 15 articulations commandées."""
        if not self._nets:
            raise RuntimeError("politiques non chargées — appeler load()")
        c = self.cfg
        h = c.height_default if height is None else float(height)
        obs = self.build_observation(base_quat_wxyz, base_ang_vel, joint_pos,
                                     joint_vel, command, h, rpy,
                                     ang_vel_frame=ang_vel_frame)
        entree = self.empiler(obs)
        # Même critère que le code de déploiement : norme de la consigne
        # BRUTE (avant mise à l'échelle).
        nom = ("balance" if float(np.linalg.norm(np.asarray(command)))
               < c.seuil_equilibre else "walk")
        self.derniere_politique = nom
        action = np.asarray(self._nets[nom](entree), dtype=np.float32).reshape(-1)
        if action.size != c.num_actions:
            raise ValueError(f"la politique a rendu {action.size} actions, "
                             f"{c.num_actions} attendues")
        self.last_action = action
        cibles = action * c.action_scale + np.asarray(c.default_angles)
        return {j: float(v) for j, v in zip(c.controlled_joints, cibles)}

    def gains(self) -> Dict[str, Tuple[float, float]]:
        return {j: (float(kp), float(kd)) for j, kp, kd in
                zip(self.cfg.controlled_joints, self.cfg.kps, self.cfg.kds)}


class GR00TWBCAdapter:
    """ILocomotionAdapter du G1 29 DOF, sur GR00T WBC.

    Rend les cibles des jambes et de la taille. Les BRAS, eux, sont tenus à
    `pose_bras` — en attendant l'arbitrage des actionneurs entre services,
    qui laissera l'app en disposer (docs/integration_gr00t_wbc.md §6).

    Consigne nulle = politique d'ÉQUILIBRE : un vrai arrêt, sans le
    piétinement de la politique d'Unitree — donc sans le maintien de cap
    qu'il avait fallu ajouter. À vérifier en simulation avant de s'y fier.
    """

    def __init__(self, policy: GR00TWBCPolicy, tilt_limit: float = -0.7,
                 pose_bras: Optional[Dict[str, float]] = None,
                 limites: Optional[Dict[str, Tuple[float, float]]] = None):
        """limites : {articulation: (min, max)} — les butées du robot (URDF).
        Les cibles de la politique sont RAMENÉES dans ces butées avant
        envoi, comme le fait tout code de déploiement : constaté sur GPU, au
        démarrage (robot qui se pose), GR00T demandait jusqu'à 0.61 rad à la
        cheville pour une butée à 0.52 ; le JointSafetyGuard refusait ces
        pas (9 refus). Le garde reste la barrière finale et n'est PAS
        contourné : il revérifie chaque cible. Chaque écrêtage est compté
        (stats), pour qu'un écrêtage fréquent se voie."""
        self._limites = dict(limites or {})
        self.stats: Dict[str, object] = {"ecretages": 0, "ecretages_par_articulation": {}}
        self._policy = policy
        self._tilt_limit = tilt_limit
        bras = policy.cfg.body_joints[policy.cfg.num_actions:]
        self.pose_bras: Dict[str, float] = {j: 0.0 for j in bras}
        if pose_bras:
            inconnues = set(pose_bras) - set(bras)
            if inconnues:
                raise KeyError(f"articulations de bras inconnues : {inconnues}")
            self.pose_bras.update(pose_bras)
        self.hauteur: Optional[float] = None
        self.rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)

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

    def posture(self, hauteur: Optional[float] = None,
                rpy: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        """Hauteur du bassin et orientation du torse commandées — ce qui
        permet au robot de s'abaisser vers un objet."""
        self.hauteur = hauteur
        self.rpy = tuple(float(v) for v in rpy)

    def step(self, state, twist) -> Dict[str, float]:
        cibles = self._policy.step(
            state.quat_wxyz, state.ang_vel_body,
            state.joint_pos, state.joint_vel,
            (twist.vx, twist.vy, twist.wz),
            self.hauteur, self.rpy,
            # BaseState porte le repère CORPS par contrat.
            ang_vel_frame="body",
        )
        cibles.update(self.pose_bras)
        for j, v in cibles.items():
            lim = self._limites.get(j)
            if lim is None:
                continue
            c = min(max(v, lim[0]), lim[1])
            if c != v:
                cibles[j] = c
                self.stats["ecretages"] += 1
                par = self.stats["ecretages_par_articulation"]
                par[j] = par.get(j, 0) + 1
        return cibles

    def is_fallen(self, state) -> bool:
        return float(projected_gravity(state.quat_wxyz)[2]) > self._tilt_limit
