import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
import builtin_interfaces.msg

class PDController(Node):

    _DEFAULT_STALE_TIME_NS = 100*1e6 # 100ms
    _DEFAULT_N_JOINTS = 7
    _DEFAULT_KP = 1000 * np.ones(_DEFAULT_N_JOINTS, dtype=np.float64)
    _DEFAULT_KD = 200 * np.ones(_DEFAULT_N_JOINTS, dtype=np.float64)
    _DEFAULT_Q_D = np.ones(_DEFAULT_N_JOINTS)
    _DEFAULT_Q_D[3] = -1
    _DEFAULT_QDOT_D = np.zeros(_DEFAULT_N_JOINTS)

    def __init__(self):
        super().__init__("pd_controller")

        # Util Parameters
        self.freq = self.declare_parameter("control_loop_frequency", 1000).get_parameter_value().integer_value
        self.n_joints = self.declare_parameter("n_joints", 7).get_parameter_value().integer_value

        # Control Coefficient Params
        self.declare_parameter("Kp", self._DEFAULT_KP.tolist())
        self.declare_parameter("Kd", self._DEFAULT_KD.tolist())
        self.Kp = self.param_to_array("Kp")
        self.Kd = self.param_to_array("Kd")

        # State Params
        self.declare_parameter("q_d", self._DEFAULT_Q_D.tolist())
        self.declare_parameter("qdot_d", self._DEFAULT_QDOT_D.tolist())
        self.q_d = self.param_to_array("q_d")
        self.qdot_d = self.param_to_array("qdot_d")
        self.q = None
        self.qdot = None

        # Param Callbacks
        self.add_on_set_parameters_callback(self.params_callback)

        # Timers
        self.timer_ctrl_loop = self.create_timer(1/self.freq, self.ctrl_loop_callback)

        # Publishers
        self.pub_joint_torques = self.create_publisher(JointState, "joint_torques", 10)

        # Subscribers
        self.sub_joint_states = self.create_subscription(JointState, "joint_states", self.joint_states_callback, 10)

    def ctrl_loop_callback(self):
        
        if self.q is None or self.qdot is None:
            self.get_logger().warn("Node will not publish joint torques until joint state is recieved", throttle_duration_sec=60)
            return
        
        diff_time = self.diff_time_from_stamp(self.last_state_time)
        if diff_time >= self._DEFAULT_STALE_TIME_NS:
            self.get_logger().warn(f"Stale joint data by {diff_time*1e6}ms")
        
        j = JointState()
        j.header.stamp = self.get_clock().now().to_msg()

        # control law
        tau =  self.Kp * (self.q_d - self.q) + self.Kd * (self.qdot_d - self.qdot)

        j.effort = tau.tolist()
        self.pub_joint_torques.publish(j)
    
    def joint_states_callback(self, msg : JointState):
        self.last_state_time = msg.header.stamp
        self.q = msg.position[:self.n_joints]
        self.qdot = msg.velocity[:self.n_joints]
        

    def params_callback(self, params : list[Parameter]):
        for p in params:
            if p.name == "Kp":
                self.Kp = self.param_to_array("Kp")
            elif p.name == "Kd":
                self.Kd = self.param_to_array("Kd")

        return SetParametersResult(successful=True)

    def diff_time_from_stamp(self, stamp: builtin_interfaces.msg.Time) -> int:
        now = self.get_clock().now()
        msg_time = Time.from_msg(stamp)
        return (now - msg_time).nanoseconds
    
    # retrieves parameter value and returns as numpy array
    def param_to_array(self, name : str):
        return np.array(self.get_parameter(name).get_parameter_value().double_array_value, dtype=np.float64)


def main(args=None):
    rclpy.init(args=args)
    node = PDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()