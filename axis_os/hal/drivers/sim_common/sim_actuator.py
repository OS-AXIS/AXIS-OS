from axis_os.hal.core.interfaces import IActuator
from axis_os.hal.core.types import ActuatorCommand, ActuatorState, JointType
from axis_os.hal.drivers.sim_common.interfaces import ISimBackend


class SimActuator(IActuator):
    """Moteur virtuel qui relaie les ordres vers le simulateur (ISimBackend)."""
    
    def __init__(self, joint_id: str, joint_type: JointType, backend: ISimBackend):
        self.joint_id = joint_id
        self.joint_type = joint_type
        self.backend = backend


    def get_state(self) -> ActuatorState:
        return self.backend.read_state(self.joint_id)


    def send_command(self, command: ActuatorCommand) -> bool:
        self.backend.send_command(self.joint_id, command)
        return True
