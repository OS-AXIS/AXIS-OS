from abc import ABC, abstractmethod
from axis_os.hal.core.types import ActuatorCommand, ActuatorState


class ISimBackend(ABC):
    """Contrat universel que tout simulateur (Isaac, Webots) doit respecter."""
    
    @abstractmethod
    def initialize(self) -> bool:
        """Démarre la connexion au simulateur."""
        pass


    @abstractmethod
    def step(self, dt: float) -> None:
        """Fait avancer le moteur physique d'un pas (ex: 0.01s)."""
        pass


    @abstractmethod
    def send_command(self, joint_id: str, command: ActuatorCommand) -> None:
        """Envoie une commande moteur au simulateur."""
        pass


    @abstractmethod
    def read_state(self, joint_id: str) -> ActuatorState:
        """Lit l'état actuel du moteur dans le simulateur."""
        pass

