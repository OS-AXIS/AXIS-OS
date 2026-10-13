"""
AXIS-OS — IsaacSimBackend
===========================
Implémentation d'ISimBackend (l'interface commune aux ponts vers un
simulateur) pour NVIDIA Isaac Sim : le pont entre la HAL simulée
(IsaacSimHAL ; HAL pour Hardware Abstraction Layer, la couche
d'abstraction matérielle d'AXIS-OS) et le simulateur. Il détecte
automatiquement Isaac Sim et, s'il est absent ou si le chargement du robot
échoue, bascule sur un stub : une physique simplifiée intégrée
(_StubPhysics), qui permet de faire tourner AXIS-OS sans simulateur.

Le mode production (Isaac Sim réellement chargé) a été implémenté après
avoir vérifié à la main que le G1 officiel d'Unitree existe dans la
bibliothèque d'assets (modèles prêts à l'emploi) d'Isaac Sim, au chemin
Isaac/Robots/Unitree/G1/g1.usd, et que son squelette réel comporte 37
articulations : 24 correspondant à ISAAC_G1_JOINT_MAP et 13 de plus pour
les doigts, non gérées par AXIS-OS à ce stade (portée volontairement
limitée aux membres, voir plus bas). Ce décompte date d'avant la
correction de la table décrite sous ISAAC_G1_JOINT_MAP, qui compte
désormais 23 entrées.

Historique : ce fichier ne contenait auparavant qu'une coquille. connect()
détectait bien Isaac Sim (l'import réussissait) mais ne chargeait jamais
rien de réel ; toutes les méthodes de lecture et d'écriture ne
fonctionnaient qu'en mode stub, jamais en mode réellement connecté. Le
vrai pont vers Isaac Sim (chargement du G1, lecture et écriture via la
classe Articulation d'Isaac Sim) a été implémenté ici pour la première
fois.

Limites connues de cette version, dont la portée est volontairement
restreinte :
  - Seules les articulations de ISAAC_G1_JOINT_MAP sont lues et écrites
    (24 à l'origine, 23 depuis la correction de la table). Les 13
    articulations des doigts du vrai G1 (left_five_joint,
    left_three_joint, etc.) existent dans le squelette mais ne sont pas
    exposées par ce backend, ce qui était cohérent avec conventions.py,
    qui ne définissait alors que 24 JointDefinition.
  - Un seul robot par instance de SimulationApp (l'application Isaac Sim
    lancée depuis Python) : c'est une limite d'Isaac Sim lui-même, pas de
    ce backend.
  - Le courant reste dérivé du couple (abs(torque)/10.0), comme dans le
    stub : le G1 d'Isaac Sim n'expose pas par défaut de capteur de
    courant électrique séparé.

Licence : Apache 2.0
"""

from __future__ import annotations
import time
import threading
from typing import Optional, Dict, List

from axis_os.hal.drivers.sim_common.interfaces import (
    ISimBackend, SimJointState, SimIMUState, SimCommand
)

ISAAC_G1_JOINT_MAP: Dict[str, str] = {
    "left_hip_roll": "left_hip_roll_joint",
    "left_hip_yaw": "left_hip_yaw_joint",
    "left_hip_pitch": "left_hip_pitch_joint",
    "left_knee": "left_knee_joint",
    "left_ankle_pitch": "left_ankle_pitch_joint",
    "left_ankle_roll": "left_ankle_roll_joint",
    "right_hip_roll": "right_hip_roll_joint",
    "right_hip_yaw": "right_hip_yaw_joint",
    "right_hip_pitch": "right_hip_pitch_joint",
    "right_knee": "right_knee_joint",
    "right_ankle_pitch": "right_ankle_pitch_joint",
    "right_ankle_roll": "right_ankle_roll_joint",
    "torso": "torso_joint",
    "left_shoulder_pitch": "left_shoulder_pitch_joint",
    "left_shoulder_roll": "left_shoulder_roll_joint",
    "left_shoulder_yaw": "left_shoulder_yaw_joint",
    "left_elbow_pitch": "left_elbow_pitch_joint",
    "left_elbow_roll": "left_elbow_roll_joint",
    "right_shoulder_pitch": "right_shoulder_pitch_joint",
    "right_shoulder_roll": "right_shoulder_roll_joint",
    "right_shoulder_yaw": "right_shoulder_yaw_joint",
    "right_elbow_pitch": "right_elbow_pitch_joint",
    "right_elbow_roll": "right_elbow_roll_joint",
}
"""
Correspondance nom d'articulation AXIS-OS -> nom dans le modèle G1
d'Isaac Sim.

Table corrigée le 2026-09-06, après vérification directe contre les 37
vrais dof_names du G1 (les noms de ses degrés de liberté, DOF en
anglais), lus la veille depuis Isaac Sim :

  - "waist_yaw" et "waist_roll" (deux articulations supposées) ont été
    remplacés par un seul "torso" -> "torso_joint" : le G1 chargé dans
    Isaac Sim n'a qu'une articulation de torse, pas deux.
  - "left_elbow" et "right_elbow" (une articulation chacun, supposée) ont
    été remplacés par deux articulations chacun, "_elbow_pitch" et
    "_elbow_roll" : c'est confirmé, le coude de ce G1 a deux degrés de
    liberté, pas un seul.
  - "left_wrist_roll" et "right_wrist_roll" ont été supprimés. Aucune
    articulation de poignet n'apparaît dans les 37 dof_names réels : ce
    modèle Isaac Sim n'a pas de poignet articulé séparé (le mouvement du
    poignet, s'il existe, doit être porté par les articulations des
    doigts, hors de la portée actuelle d'AXIS-OS).

Total : de 24 à 23 articulations (torse -1, coudes +2, poignets -2). Ce
changement concerne aussi conventions.py et tous les fichiers qui
désignent le coude ou le torse par leurs anciens noms ("left_elbow",
"waist_yaw", "waist_roll"). Cela n'a pas été fait lors de cette
correction, et reste à traiter avant tout usage réel des bras et du
torse. Depuis, conventions.py (correction du 2026-09-08, voir son en-tête)
suit le fichier URDF officiel du G1 à 23 DOF (URDF, Unified Robot
Description Format : le fichier standard qui décrit les articulations
d'un robot). Ses identifiants diffèrent des clés de cette table (suffixe
"_joint" : "left_knee_joint" au lieu de "left_knee"), et son modèle
diffère de celui d'Isaac Sim (waist_yaw_joint au lieu de torso_joint, une
seule articulation de coude par bras au lieu de deux) : les deux tables
ne sont toujours pas mises en cohérence. Conséquence constatée :
IsaacSimHAL, qui nomme ses articulations d'après ce même URDF, ne retrouve
aucune d'elles dans le stub (voir la limite connue décrite dans hal.py).
"""


class _StubPhysics:
    """Physique simplifiée (le stub), utilisée à la place d'Isaac Sim.

    Chaque articulation est simulée seule, avec les mêmes constantes pour
    toutes. Trois modes de commande : en position, un régulateur PD
    (proportionnel-dérivé : un couple proportionnel à l'écart de position,
    freiné par un terme proportionnel à la vitesse) ; en vitesse, un
    couple proportionnel à l'écart de vitesse ; en couple, le couple
    demandé est appliqué directement. Dans tous les modes, un frottement
    proportionnel à la vitesse (DAMPING) s'y ajoute, et l'accélération
    tient compte de l'inertie INERTIA. Un fil d'exécution de fond (thread)
    intègre le mouvement tous les DT = 4 ms."""
    INERTIA = 0.05; DAMPING = 2.0; STIFFNESS = 50.0; DT = 0.004

    def __init__(self, joint_names: list[str]):
        self._joints = {n: {"pos": 0.0, "vel": 0.0, "torque": 0.0,
                            "target_pos": 0.0, "target_vel": 0.0,
                            "mode": "position"} for n in joint_names}
        self._time   = 0.0
        self._lock   = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True, name="stub_physics").start()

    def _loop(self):
        while self._running:
            t0 = time.monotonic()
            with self._lock:
                for s in self._joints.values():
                    if s["mode"] == "position":
                        torque = self.STIFFNESS * (s["target_pos"] - s["pos"]) - self.DAMPING * s["vel"]
                    elif s["mode"] == "velocity":
                        torque = self.STIFFNESS * 0.1 * (s["target_vel"] - s["vel"])
                    else:
                        torque = s["torque"]
                    acc = (torque - self.DAMPING * s["vel"]) / self.INERTIA
                    s["vel"] += acc * self.DT
                    s["pos"] += s["vel"] * self.DT
                    s["torque"] = torque
                self._time += self.DT
            sleep = self.DT - (time.monotonic() - t0)
            if sleep > 0: time.sleep(sleep)

    def set(self, name: str, mode: str, value: float, velocity: Optional[float] = None) -> bool:
        with self._lock:
            if name not in self._joints: return False
            s = self._joints[name]; s["mode"] = mode
            if mode == "position": s["target_pos"] = value
            elif mode == "velocity": s["target_vel"] = value
            else: s["torque"] = value
            return True

    def get(self, name: str) -> Optional[dict]:
        with self._lock:
            return dict(self._joints[name]) if name in self._joints else None

    def get_time(self) -> float:
        with self._lock: return self._time

    def reset(self):
        with self._lock:
            for s in self._joints.values():
                s["pos"] = s["vel"] = s["torque"] = s["target_pos"] = s["target_vel"] = 0.0

    def stop(self): self._running = False


class IsaacSimBackend(ISimBackend):
    """Pont entre AXIS-OS et Isaac Sim, avec repli automatique sur le stub.

    robot_prim_path : chemin du robot dans la scène USD (Universal Scene
        Description, le format de scène d'Isaac Sim).
    force_stub : True pour utiliser le stub même si Isaac Sim est
        installé.
    usd_path : fichier USD du robot ; par défaut, le G1 de la bibliothèque
        d'assets d'Isaac Sim.
    """
    # Tension et température fictives, utilisées dans les deux modes : le
    # backend ne lit ni l'une ni l'autre dans Isaac Sim. La température
    # rendue part de STUB_TEMP et augmente de 0.02 par unité de couple.
    STUB_VOLTAGE = 48.0; STUB_TEMP = 25.0

    def __init__(self, robot_prim_path: str = "/World/G1", force_stub: bool = False,
                usd_path: Optional[str] = None):
        self._prim_path  = robot_prim_path
        self._force_stub = force_stub
        self._usd_path   = usd_path
        self._connected  = False
        self._stub_mode  = False
        self._stub: Optional[_StubPhysics] = None
        self._joint_map  = ISAAC_G1_JOINT_MAP

        self._world = None
        self._articulation = None
        self._joint_indices: Dict[str, int] = {}

    def connect(self) -> bool:
        # Isaac Sim est détecté par l'import de omni.isaac.core ; s'il
        # échoue, on bascule sur le stub.
        if self._force_stub:
            return self._activate_stub("Stub forcé.")
        try:
            import omni.isaac.core as _
            return self._connect_production()
        except ImportError:
            return self._activate_stub("omni.isaac.core non trouvé.")

    def _connect_production(self) -> bool:
        """
        Charge réellement le G1 dans Isaac Sim et construit la table de
        correspondance nom AXIS-OS -> index d'articulation. Toute erreur
        ici fait retomber sur le stub, plutôt que de laisser AXIS-OS sans
        backend fonctionnel. Les articulations de la table absentes du
        squelette chargé sont signalées par un message, puis ignorées.
        """
        try:
            import isaacsim.storage.native as nucleus
            from isaacsim.core.api import World
            from isaacsim.core.utils.stage import add_reference_to_stage
            from isaacsim.core.prims import Articulation

            usd_path = self._usd_path
            if usd_path is None:
                assets_root = nucleus.get_assets_root_path()
                if assets_root is None:
                    return self._activate_stub("assets_root introuvable.")
                usd_path = assets_root + "/Isaac/Robots/Unitree/G1/g1.usd"

            self._world = World()
            self._world.scene.add_default_ground_plane()
            add_reference_to_stage(usd_path=usd_path, prim_path=self._prim_path)
            self._world.reset()

            self._articulation = Articulation(prim_paths_expr=self._prim_path)
            self._articulation.initialize()

            real_joint_names = list(self._articulation.dof_names)
            self._joint_indices = {}
            missing = []
            for axis_name, isaac_name in self._joint_map.items():
                if isaac_name in real_joint_names:
                    self._joint_indices[axis_name] = real_joint_names.index(isaac_name)
                else:
                    missing.append((axis_name, isaac_name))

            if missing:
                print(f"[IsaacBackend] {len(missing)} joint(s) de "
                      f"ISAAC_G1_JOINT_MAP absent(s) du vrai squelette G1 : "
                      f"{[a for a, _ in missing]}")

            self._connected = True
            self._stub_mode = False
            print(f"[IsaacBackend] Connecté en mode PRODUCTION | "
                  f"prim={self._prim_path} | "
                  f"{len(self._joint_indices)}/{len(self._joint_map)} joints résolus")
            return True

        except Exception as exc:
            return self._activate_stub(f"Échec connexion production : {exc}")

    def disconnect(self) -> None:
        if self._stub: self._stub.stop(); self._stub = None
        self._articulation = None
        self._world = None
        self._connected = False

    def is_connected(self) -> bool: return self._connected

    def get_joint_state(self, joint_name: str) -> Optional[SimJointState]:
        if not self._connected: return None
        if self._stub_mode:
            s = self._stub.get(joint_name)
            if s is None: return None
            return SimJointState(name=joint_name, position=s["pos"], velocity=s["vel"],
                                 torque=s["torque"],
                                 temperature=self.STUB_TEMP + abs(s["torque"]) * 0.02,
                                 voltage=self.STUB_VOLTAGE)
        return self._get_joint_state_production(joint_name)

    def _get_joint_state_production(self, joint_name: str) -> Optional[SimJointState]:
        idx = self._joint_indices.get(joint_name)
        if idx is None or self._articulation is None:
            return None
        try:
            positions = self._articulation.get_joint_positions()
            velocities = self._articulation.get_joint_velocities()
            efforts = self._articulation.get_applied_joint_efforts()
            pos = float(positions[0][idx])
            vel = float(velocities[0][idx])
            torque = float(efforts[0][idx]) if efforts is not None else 0.0
            return SimJointState(
                name=joint_name, position=pos, velocity=vel, torque=torque,
                temperature=self.STUB_TEMP + abs(torque) * 0.02,
                voltage=self.STUB_VOLTAGE,
            )
        except Exception:
            return None

    def get_all_joint_states(self) -> Dict[str, SimJointState]:
        return {n: self.get_joint_state(n) for n in self._joint_map
                if self.get_joint_state(n) is not None}

    def send_command(self, command: SimCommand) -> bool:
        if not self._connected: return False
        if self._stub_mode:
            return self._stub.set(command.name, command.mode, command.value, command.velocity)
        return self._send_command_production(command)

    def _send_command_production(self, command: SimCommand) -> bool:
        idx = self._joint_indices.get(command.name)
        if idx is None or self._articulation is None:
            return False
        try:
            import numpy as np
            if command.mode == "position":
                positions = self._articulation.get_joint_positions()
                positions[0][idx] = command.value
                self._articulation.set_joint_positions(positions)
            elif command.mode == "velocity":
                velocities = self._articulation.get_joint_velocities()
                velocities[0][idx] = command.value
                self._articulation.set_joint_velocities(velocities)
            elif command.mode == "torque":
                efforts = self._articulation.get_applied_joint_efforts()
                if efforts is None:
                    efforts = np.zeros((1, len(self._joint_map)))
                efforts[0][idx] = command.value
                self._articulation.set_joint_efforts(efforts)
            else:
                return False
            return True
        except Exception:
            return False

    def send_commands_batch(self, commands: list[SimCommand]) -> bool:
        return all(self.send_command(c) for c in commands)

    def get_imu_state(self, imu_name: str = "imu") -> Optional[SimIMUState]:
        # Centrale inertielle (IMU) : valeurs fixes dans les deux modes,
        # celles d'un robot immobile et à plat (seule la gravité, 9.81 m/s²,
        # selon z). Aucune mesure n'est lue dans Isaac Sim.
        if not self._connected: return None
        return SimIMUState(acc_x=0.0, acc_y=0.0, acc_z=9.81,
                           gyr_x=0.0, gyr_y=0.0, gyr_z=0.0,
                           quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0)

    def reset(self) -> bool:
        if self._stub:
            self._stub.reset()
            return True
        if self._world is not None:
            try:
                self._world.reset()
                return True
            except Exception:
                return False
        return False

    def get_sim_time(self) -> float:
        if self._stub:
            return self._stub.get_time()
        if self._world is not None:
            try:
                return float(self._world.current_time)
            except Exception:
                return 0.0
        return 0.0

    def get_backend_info(self) -> Dict[str, str]:
        # En mode stub, toutes les articulations de la table existent par
        # construction : elles sont toutes « résolues ».
        return {"name": "IsaacSimBackend", "version": "0.2.0",
                "mode": "stub" if self._stub_mode else "production",
                "robot_model": "Unitree G1", "joints": str(len(self._joint_map)),
                "joints_resolved": str(len(self._joint_indices)) if not self._stub_mode
                                   else str(len(self._joint_map))}

    def _activate_stub(self, reason: str) -> bool:
        print(f"[IsaacBackend] Stub — {reason}")
        self._stub_mode = True; self._connected = True
        self._stub = _StubPhysics(list(self._joint_map.keys()))
        return True


# NOTE_JOINT_MISMATCH. Cette note décrit l'état antérieur à la correction
# de la table datée du 2026-09-06 (voir la docstring sous
# ISAAC_G1_JOINT_MAP). En inspectant le vrai G1 chargé dans Isaac Sim (37
# articulations réelles trouvées via Articulation.dof_names), certains
# noms différaient de ceux supposés dans ISAAC_G1_JOINT_MAP, notamment le
# coude, qui semblait avoir deux articulations réelles (pitch et roll
# séparés) plutôt qu'un seul "elbow_joint" comme supposé au départ. Ce
# backend tolère ce genre d'écart sans planter : les articulations non
# résolues sont simplement absentes, et _connect_production affiche un
# avertissement. La table elle-même n'avait pas encore été corrigée à ce
# moment-là : il fallait valider chaque nom un par un contre la vraie
# liste des dof_names.
