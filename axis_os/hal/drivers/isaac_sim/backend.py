from axis_os.hal.core.types import ActuatorCommand, ActuatorState
from axis_os.hal.drivers.sim_common.interfaces import ISimBackend
import time


class IsaacSimBackend(ISimBackend):
    """Connecteur vers le moteur physique NVIDIA Omniverse / Isaac Sim."""
    
    def __init__(self):
        self.is_running = False
        self.dummy_states = {} # Simule la mémoire pour nos tests


    def initialize(self) -> bool:
        print("[IsaacSim] Connexion au bridge Omniverse établie...")
        self.is_running = True
        return True


    def step(self, dt: float) -> None:
        # Avance la physique 3D d'un cran
        pass


    def send_command(self, joint_id: str, command: ActuatorCommand) -> None:
        # En production : modifiera les propriétés USD dans Isaac Sim
        if command.position is not None:
            print(f"[IsaacSim] Mouvement 3D : {joint_id} -> {command.position} rad")
            
            if joint_id not in self.dummy_states:
                self.dummy_states[joint_id] = ActuatorState(
                    position=0.0, velocity=0.0, torque=0.0, temperature=30.0
                )
            self.dummy_states[joint_id].position = command.position


    def read_state(self, joint_id: str) -> ActuatorState:
        # Retourne l'état 3D actuel du moteur
        return self.dummy_states.get(
            joint_id, 
            ActuatorState(position=0.0, velocity=0.0, torque=0.0, temperature=30.0)
        )

