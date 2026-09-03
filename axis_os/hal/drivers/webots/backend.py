from axis_os.hal.core.types import ActuatorCommand, ActuatorState
from axis_os.hal.drivers.sim_common.interfaces import ISimBackend


class WebotsBackend(ISimBackend):
    """Connecteur vers le simulateur Webots (Cyberbotics)."""
    
    def __init__(self):
        self.is_running = False


    def initialize(self) -> bool:
        print("[Webots] Démarrage du contrôleur Webots Python API...")
        self.is_running = True
        return True


    def step(self, dt: float) -> None:
        pass


    def send_command(self, joint_id: str, command: ActuatorCommand) -> None:
        pass


    def read_state(self, joint_id: str) -> ActuatorState:
        return ActuatorState(position=0.0, velocity=0.0, torque=0.0, temperature=25.0)