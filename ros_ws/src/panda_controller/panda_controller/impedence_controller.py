import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ImpedenceController(Node):

    _DEFAULT_STALE_TIME_NS = 100*1e6 # 100ms
    _DEFAULT_N_JOINTS = 7
    _DEFAULT_CTRL_LOOP_FREQ_HZ = 1000

    def __init__(self):
        super().__init__("impedence_controller")

        self.get_logger().info("Impedence Controller node started...")

        # Params
        self.freq = self.declare_parameter("control_loop_frequency", self._DEFAULT_CTRL_LOOP_FREQ_HZ).get_parameter_value().integer_value
        self.n_joints = self.declare_parameter("n_joints", 7).get_parameter_value().integer_value

        # Timers
        self.timer_ctrl_loop = self.create_timer(1/self.freq, self.ctrl_loop_callback)

        # Publishers
        self.pub_joint_torques = self.create_publisher(JointState, "joint_torques", 10)

        # Subscribers
        self.sub_joint_states = self.create_subscription(JointState, "joint_states", self.joint_states_callback, 10)

        # TODO: Setup pinnochio to calculate jacoboian

    def ctrl_loop_callback(self):
        self

    def joint_states_callback(self, msg : JointState):
        self.last_state_time = msg.header.stamp
        self.q = msg.position[:self.n_joints]
        self.qdot = msg.velocity[:self.n_joints]

def main(args=None):
    rclpy.init(args=args)
    node = ImpedenceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()