"""
AXIS-OS — WebotsBackend
=========================
Implémentation d'ISimBackend (le contrat commun aux ponts vers un
simulateur, voir sim_common/interfaces.py) pour Webots, un simulateur de
robots open source. Il est entièrement interchangeable avec
IsaacSimBackend, le pont vers Isaac Sim de NVIDIA : la HAL simulée
(IsaacSimHAL ; HAL pour Hardware Abstraction Layer, la couche
d'abstraction matérielle d'AXIS-OS) fonctionne avec l'un ou l'autre sans
modification.

Le robot modélisé est l'Unitree G1, le robot humanoïde d'Unitree
Robotics. Ce backend passe par l'API Python de Webots, le module
« controller », que Webots fournit aux programmes qui pilotent un robot
simulé. Si ce module est absent, ou si la connexion échoue, il bascule
sur un stub : la physique simplifiée _StubPhysics d'isaac_sim/backend.py,
qui permet de faire tourner AXIS-OS sans simulateur. Dans les tests du
dépôt, le module controller n'est jamais disponible : seul le chemin stub
y est exercé (voir test/test_audit_fixes_wave3.py).

Usage :
    hal = IsaacSimHAL(backend=WebotsBackend(timestep_ms=32))

Licence : Apache 2.0
"""

from __future__ import annotations
from typing import Optional, Dict

from axis_os.hal.drivers.sim_common.interfaces import (
    ISimBackend, SimJointState, SimIMUState, SimCommand
)
from axis_os.hal.drivers.isaac_sim.backend import _StubPhysics

# Correspondance entre les noms d'articulation (joint) d'AXIS-OS et les
# noms des moteurs attendus dans le modèle Webots du G1 : 24
# articulations (6 par jambe, 2 pour la taille, 5 par bras). Le capteur
# de position de chaque moteur porte le même nom suivi de « S » (voir
# connect).
WEBOTS_G1_JOINT_MAP: Dict[str, str] = {
    "left_hip_roll": "LeftHipRoll", "left_hip_yaw": "LeftHipYaw",
    "left_hip_pitch": "LeftHipPitch", "left_knee": "LeftKnee",
    "left_ankle_pitch": "LeftAnklePitch", "left_ankle_roll": "LeftAnkleRoll",
    "right_hip_roll": "RightHipRoll", "right_hip_yaw": "RightHipYaw",
    "right_hip_pitch": "RightHipPitch", "right_knee": "RightKnee",
    "right_ankle_pitch": "RightAnklePitch", "right_ankle_roll": "RightAnkleRoll",
    "waist_yaw": "WaistYaw", "waist_roll": "WaistRoll",
    "left_shoulder_pitch": "LeftShoulderPitch", "left_shoulder_roll": "LeftShoulderRoll",
    "left_shoulder_yaw": "LeftShoulderYaw", "left_elbow": "LeftElbow",
    "left_wrist_roll": "LeftWristRoll",
    "right_shoulder_pitch": "RightShoulderPitch", "right_shoulder_roll": "RightShoulderRoll",
    "right_shoulder_yaw": "RightShoulderYaw", "right_elbow": "RightElbow",
    "right_wrist_roll": "RightWristRoll",
}


class WebotsBackend(ISimBackend):
    """Pont entre AXIS-OS et Webots pour le G1, avec repli automatique sur
    le stub. Le mode réellement utilisé (« stub » ou « production »,
    c'est-à-dire Webots réellement connecté) se lit dans
    get_backend_info()["mode"]."""

    # Valeurs de remplacement, faute de capteur, utilisées en stub comme
    # en production : tension fixe de 48 V et température de base de
    # 25 °C, à laquelle s'ajoute une estimation liée au couple (voir
    # get_joint_state).
    STUB_VOLTAGE = 48.0; STUB_TEMP = 25.0

    def __init__(self, timestep_ms: int = 32, force_stub: bool = False):
        """timestep_ms : pas de simulation en millisecondes, c'est-à-dire
        la durée dont Webots avance à chaque pas et la période de lecture
        des capteurs. force_stub=True utilise directement le stub, sans
        chercher Webots."""
        self._timestep_ms = timestep_ms
        self._force_stub  = force_stub
        self._connected   = False
        self._stub_mode   = False
        self._joint_map   = WEBOTS_G1_JOINT_MAP
        self._robot_node  = None
        self._motors: dict = {}
        self._sensors: dict = {}
        self._imu_device  = None
        self._stub: Optional[_StubPhysics] = None

    def connect(self) -> bool:
        """Lie chaque articulation de WEBOTS_G1_JOINT_MAP à son moteur et
        à son capteur Webots, active la centrale inertielle, puis rend
        True. Si le module controller est absent ou si une erreur survient,
        bascule sur le stub et rend True aussi.

        Une articulation qui ne se lie pas n'empêche pas la connexion,
        mais chaque échec est signalé bruyamment (voir plus bas)."""
        if self._force_stub:
            return self._activate_stub("Stub forcé.")
        try:
            from controller import Robot, Motor, PositionSensor, InertialUnit
            self._robot_node = Robot()
            failed_joints = []
            for hal_name, wb_name in self._joint_map.items():
                try:
                    motor = self._robot_node.getDevice(wb_name)
                    if motor:
                        # Position infinie : dans Webots, le moteur passe
                        # en commande de vitesse ; avec une vitesse nulle,
                        # l'articulation reste immobile en attendant la
                        # première commande.
                        motor.setPosition(float("inf")); motor.setVelocity(0.0)
                        # Capteur de position associé (nom du moteur suivi
                        # de « S »). Un moteur sans capteur est quand même
                        # enregistré : il accepte des commandes, mais
                        # get_joint_state rendra None pour lui.
                        sensor = self._robot_node.getDevice(f"{wb_name}S")
                        if sensor:
                            sensor.enable(self._timestep_ms)
                            self._sensors[hal_name] = sensor
                        self._motors[hal_name] = motor
                    else:
                        failed_joints.append(hal_name)
                except Exception as exc:
                    failed_joints.append(hal_name)
                    # Un audit de sécurité a montré qu'un joint qui
                    # échouait à se lier disparaissait silencieusement de
                    # _motors, alors que connect() rendait quand même
                    # True. Les commandes suivantes sur ce joint échouaient
                    # ensuite sans bruit (send_command rend simplement
                    # False), ce qui peut faire exécuter partiellement un
                    # mouvement coordonné ; sur un robot bipède, cela peut
                    # provoquer une chute. Chaque échec est donc signalé
                    # dès la connexion, pour qu'un joint manquant soit
                    # visible avant le premier mouvement, et non découvert
                    # au milieu d'une mission.
                    print(f"  [WARNING] WebotsBackend: liaison du joint "
                          f"'{hal_name}' ('{wb_name}') échouée : {exc}")
            if failed_joints:
                print(f"  [WARNING] WebotsBackend: {len(failed_joints)}/"
                      f"{len(self._joint_map)} joint(s) non liés : "
                      f"{failed_joints} — les commandes sur ces joints "
                      f"échoueront (send_command retourne False).")
            # Centrale inertielle (IMU, Inertial Measurement Unit : le
            # capteur d'orientation du robot). Son absence ou un échec
            # d'activation n'empêche pas la connexion, mais est signalé.
            try:
                imu = self._robot_node.getDevice("inertial_unit")
                if imu: imu.enable(self._timestep_ms); self._imu_device = imu
                else:
                    print("  [WARNING] WebotsBackend: device 'inertial_unit' "
                          "introuvable — get_imu_state() retournera None.")
            except Exception as exc:
                print(f"  [WARNING] WebotsBackend: échec activation IMU : {exc}")
            self._connected = True; self._stub_mode = False
            print(f"[WebotsBackend] Connecté | timestep={self._timestep_ms}ms | "
                  f"{len(self._motors)}/{len(self._joint_map)} joints liés")
            return True
        except ImportError:
            return self._activate_stub("module 'controller' non trouvé.")
        except Exception as e:
            return self._activate_stub(f"Erreur: {e}")

    def disconnect(self) -> None:
        # Arrête le fil d'exécution du stub s'il tourne, puis oublie tous
        # les moteurs et capteurs liés.
        if self._stub: self._stub.stop(); self._stub = None
        self._connected = False; self._robot_node = None
        self._motors.clear(); self._sensors.clear()

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
        # Mode production : il faut à la fois le capteur (position) et le
        # moteur (vitesse, couple).
        sensor = self._sensors.get(joint_name)
        motor  = self._motors.get(joint_name)
        if sensor is None or motor is None: return None
        try:
            # Couple mesuré si l'API Webots fournit getTorqueFeedback,
            # sinon 0.
            torque = (motor.getTorqueFeedback()
                     if hasattr(motor, "getTorqueFeedback") else 0.0)
            # Un audit de sécurité a relevé que Webots n'expose pas
            # nativement de vrai capteur de température moteur, et que la
            # version précédente rendait STUB_TEMP, une constante, même en
            # mode production (moteurs et capteurs réellement liés). Le
            # Watchdog (le système de surveillance de la HAL, qui alerte
            # notamment en cas de surchauffe, voir axis_os/hal/watchdog/)
            # était ainsi structurellement aveugle à toute surchauffe
            # réelle. Faute de vrai capteur, on applique la même
            # estimation liée au couple que le stub, plutôt qu'une
            # constante : un signal imparfait, mais qui varie avec
            # l'activité réelle du moteur. C'est une estimation, jamais
            # une vraie mesure.
            # À noter : SimActuator (sim_common/sim_actuator.py), qui
            # transmet ces états à la HAL, calcule sa propre estimation de
            # température et n'utilise pas ce champ.
            temperature = self.STUB_TEMP + abs(torque) * 0.02
            return SimJointState(name=joint_name, position=sensor.getValue(),
                                 velocity=motor.getVelocity(),
                                 torque=torque,
                                 temperature=temperature, voltage=self.STUB_VOLTAGE)
        except Exception: return None

    def get_all_joint_states(self) -> Dict[str, SimJointState]:
        # En mode production, Webots n'avance que lorsque le contrôleur
        # appelle step() : c'est ici (et dans send_commands_batch) que la
        # simulation progresse d'un pas de timestep_ms. Un échec de step()
        # n'empêche pas de lire les derniers états connus.
        if not self._stub_mode and self._robot_node:
            try: self._robot_node.step(self._timestep_ms)
            except Exception: pass
        return {n: s for n in self._joint_map
                if (s := self.get_joint_state(n)) is not None}

    def send_command(self, command: SimCommand) -> bool:
        if not self._connected: return False
        if self._stub_mode:
            return self._stub.set(command.name, command.mode,
                                  command.value, command.velocity)
        motor = self._motors.get(command.name)
        if motor is None: return False
        # Traduction vers l'API Webots :
        #   - position : position cible ; la vitesse, si elle est fournie
        #     et non nulle, fixe la vitesse de déplacement vers la cible
        #     (sinon la vitesse réglée précédemment reste en vigueur) ;
        #   - vitesse : position infinie, qui passe le moteur en commande
        #     de vitesse, puis vitesse demandée ;
        #   - couple : couple appliqué directement.
        # SimActuator ne produit que ces trois modes (voir
        # _to_sim_command) ; un autre mode ne déclencherait aucune action,
        # et la méthode rendrait quand même True.
        try:
            if command.mode == "position":
                motor.setPosition(command.value)
                if command.velocity: motor.setVelocity(abs(command.velocity))
            elif command.mode == "velocity":
                motor.setPosition(float("inf")); motor.setVelocity(command.value)
            elif command.mode == "torque":
                motor.setTorque(command.value)
            return True
        except Exception: return False

    def send_commands_batch(self, commands: list[SimCommand]) -> bool:
        # Toutes les consignes sont envoyées, même si l'une échoue, puis
        # la simulation avance d'un seul pas : elles s'appliquent ainsi au
        # même instant simulé. Rend True seulement si toutes ont été
        # acceptées.
        results = [self.send_command(c) for c in commands]
        if not self._stub_mode and self._robot_node:
            try: self._robot_node.step(self._timestep_ms)
            except Exception: pass
        return all(results)

    def get_imu_state(self, imu_name: str = "imu") -> Optional[SimIMUState]:
        # Seule l'orientation vient de Webots, et seulement en mode
        # production. L'accélération et la vitesse angulaire gardent des
        # valeurs fixes (gravité seule, aucune rotation), comme si le
        # robot était immobile ; en stub, l'orientation est aussi fixe
        # (robot droit).
        if not self._connected: return None
        if self._stub_mode:
            return SimIMUState(acc_x=0.0, acc_y=0.0, acc_z=9.81,
                               gyr_x=0.0, gyr_y=0.0, gyr_z=0.0,
                               quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0)
        if self._imu_device is None: return None
        try:
            # Conversion classique des angles de roulis, tangage et lacet
            # (roll, pitch, yaw) fournis par Webots en quaternion (quatre
            # nombres w, x, y, z qui décrivent l'orientation).
            import math
            r, p, y = self._imu_device.getRollPitchYaw()
            cy, sy = math.cos(y/2), math.sin(y/2)
            cp, sp = math.cos(p/2), math.sin(p/2)
            cr, sr = math.cos(r/2), math.sin(r/2)
            return SimIMUState(acc_x=0.0, acc_y=0.0, acc_z=9.81,
                               gyr_x=0.0, gyr_y=0.0, gyr_z=0.0,
                               quat_w=cr*cp*cy + sr*sp*sy,
                               quat_x=sr*cp*cy - cr*sp*sy,
                               quat_y=cr*sp*cy + sr*cp*sy,
                               quat_z=cr*cp*sy - sr*sp*cy)
        except Exception: return None

    def reset(self) -> bool:
        # En production, simulationReset() remet tout le monde simulé de
        # Webots dans son état initial.
        if self._stub_mode and self._stub: self._stub.reset(); return True
        if self._robot_node:
            try: self._robot_node.simulationReset(); return True
            except Exception: pass
        return False

    def get_sim_time(self) -> float:
        if self._stub_mode and self._stub: return self._stub.get_time()
        if self._robot_node:
            try: return self._robot_node.getTime()
            except Exception: pass
        return 0.0

    def get_backend_info(self) -> Dict[str, str]:
        # « joints » compte les articulations de la table de
        # correspondance, pas celles réellement liées : ce second nombre
        # n'est affiché qu'à la connexion (voir connect).
        return {"name": "WebotsBackend", "version": "0.1.0",
                "mode": "stub" if self._stub_mode else "production",
                "robot_model": "Unitree G1",
                "timestep_ms": str(self._timestep_ms),
                "joints": str(len(self._joint_map))}

    def _activate_stub(self, reason: str) -> bool:
        # Bascule sur la physique simplifiée et rend True : connect()
        # réussit donc toujours, et le mode réel se lit dans
        # get_backend_info()["mode"].
        print(f"[WebotsBackend] Stub — {reason}")
        self._stub_mode = True; self._connected = True
        self._stub = _StubPhysics(list(self._joint_map.keys()))
        return True
