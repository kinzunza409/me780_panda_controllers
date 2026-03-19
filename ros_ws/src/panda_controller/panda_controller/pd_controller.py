import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState

class PDController(Node):

    _DEFAULT_N_JOINTS = 7
    _DEFAULT_KP = 1000 * np.ones(_DEFAULT_N_JOINTS, dtype=np.float64)
    _DEFAULT_KD = 200 * np.ones(_DEFAULT_N_JOINTS, dtype=np.float64)

    def __init__(self):
        super().__init__("pd_controller")

        # Parameters
        self.freq = self.declare_parameter("control_loop_frequency", 1000)
        self.n_joints = self.declare_parameter("n_joints", 7).get_parameter_value().integer_value
        self.declare_parameter("Kp", self._DEFAULT_KP)
        self.declare_parameter("Kd", self._DEFAULT_KD)
        self.Kp = self.param_to_array("Kp")
        self.Kd = self.param_to_array("Kd")
        self.add_on_set_parameters_callback(self.params_callback)

        # Timers
        self.timer_ctrl_loop = self.create_timer(1/self.freq, self.ctrl_loop_callback)

        # Publishers
        self.pub_joint_torques = self.create_publisher(JointState, "joint_torques", 10)

        # Subscribers
        self.sub_joint_states = self.create_subscription(JointState, "joint_states", self.joint_states_callback, 10)

    def ctrl_loop_callback(self):
        pass
    
    def joint_states_callback(self):
        pass

    def params_callback(self, params : list[Parameter]):
        for p in params:
            if p.name == "Kp":
                self.Kp = self.param_to_array("Kp")
            elif p.name == "Kd":
                self.Kd = self.param_to_array("Kd")

        return SetParametersResult(successful=True)

    # retrieves parameter value and returns as numpy array
    def param_to_array(self, name : str) -> np.ndarray[np.float64]:
        return np.array(self.get_parameter(name).get_parameter_value().double_array_value, dtype=np.float64)


def main(args=None):
    rclpy.init(args=args)
    node = PDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()