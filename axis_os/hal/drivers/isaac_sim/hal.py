from axis_os.hal.core.interfaces import IRobotHAL
from axis_os.hal.core.types import JointType
from axis_os.hal.drivers.sim_common.sim_actuator import SimActuator
from axis_os.hal.drivers.isaac_sim.backend import IsaacSimBackend


class IsaacSimHAL(IRobotHAL):
    """Couche matérielle virtuelle pour un robot dans Isaac Sim."""
    
    def __init__(self):
        self.backend = IsaacSimBackend()
        self.actuators = {}


    def initialize(self) -> bool:
        success = self.backend.initialize()
        if success:
            # On simule un robot basique (ex: un bras avec 2 moteurs pour le test)
            self.actuators["joint_1"] = SimActuator("joint_1", JointType.REVOLUTE, self.backend)
            self.actuators["joint_2"] = SimActuator("joint_2", JointType.REVOLUTE, self.backend)
        return success


    def get_actuator(self, joint_id: str) -> SimActuator:
        return self.actuators.get(joint_id)


    def get_sensor(self, sensor_id: str):
        # Les capteurs virtuels (caméras 3D, IMU) iront ici
        pass
