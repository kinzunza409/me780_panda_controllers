import debugpy
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

import pinocchio as pin


class ImpedenceController(Node):

    _DEFAULT_STALE_TIME_NS = 100*1e6 # 100ms
    _DEFAULT_N_JOINTS = 7
    _DEFAULT_CTRL_LOOP_FREQ_HZ = 1024

    _STARTING_Q = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.035, 0.035])

    _DEFAULT_LAMDA_D = np.diag([1.5, 1.5, 1.5, 0.1, 0.1, 0.1])
    _DEFAULT_K_D = np.diag([150, 150, 150, 8, 8, 8])
    _DEFAULT_D_D = np.diag([30, 30, 30, 3, 3, 3])

    _DEFAULT_x_d = np.concatenate([
        np.array([0.5545, 0.0, 0.6245]),
        np.array([2.2216, 2.2214, 0.0])
    ])
    _DEFAULT_x_d_dot = np.zeros(6)
    _DEFAULT_x_d_ddot = np.zeros(6)

    def __init__(self):
        super().__init__("impedence_controller")
        self.get_logger().info("Impedence Controller node started...")

        # Debug
        self.debug_mode = self.declare_parameter('debug_mode', False).get_parameter_value().bool_value
        if self.debug_mode:
            debugpy.listen(("0.0.0.0", 5678))
            self.get_logger().warn("Debug enabled! Waiting for connection...")
            debugpy.wait_for_client()
            self.get_logger().info("Debugger connected!")
            

        # Params
        self.freq = self.declare_parameter("control_loop_frequency", self._DEFAULT_CTRL_LOOP_FREQ_HZ).get_parameter_value().integer_value
        self.n_joints = self.declare_parameter("n_joints", 7).get_parameter_value().integer_value

        # Desired end effector kinematics
        self.x_d = self._DEFAULT_x_d
        self.x_d_dot = self._DEFAULT_x_d_dot
        self.x_d_ddot = self._DEFAULT_x_d_ddot

        # Controller Paramters
        self.Lambda_d = self._DEFAULT_LAMDA_D
        self.K_d = self._DEFAULT_K_D
        self.D_d = self._DEFAULT_D_D
        self.F_ext = np.zeros(6)

        # Initialize Pinocchio
        model_path = "/workspace/assets/models/franka_emika_panda/panda.xml"
        full_model, *_ = pin.buildModelsFromMJCF(model_path)
        self.model = pin.buildReducedModel(full_model, [8, 9], self._STARTING_Q) # lock gripper joints in place
        self.data = self.model.createData()


        # Timers
        self.timer_ctrl_loop = self.create_timer(1/self.freq, self.ctrl_loop_callback)
        self.start_ctrl_loop = False

        # Publishers
        self.pub_joint_torques = self.create_publisher(JointState, "joint_torques", 10)

        # Subscribers
        self.sub_joint_states = self.create_subscription(JointState, "joint_states", self.joint_states_callback, 10)
        self.sub_external_wrench = self.create_subscription(WrenchStamped, "wrench_external", self.wrench_callback, 10)


    def ctrl_loop_callback(self):

        if self.start_ctrl_loop:
            # update Pinocchio data
            pin.forwardKinematics(self.model, self.data, self.q, self.qdot)
            frame_id = self.model.getFrameId("hand")
            J = pin.computeFrameJacobian(self.model, self.data, self.q, 
                frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            pin.computeJointJacobiansTimeVariation(self.model, self.data, self.q, self.qdot)
            J_dot = pin.getFrameJacobianTimeVariation(self.model, self.data,
                frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            C = pin.computeCoriolisMatrix(self.model, self.data, self.q, self.qdot)
            M = pin.crba(self.model, self.data, self.q) # mass matrix (only upper triangular)
            M = np.triu(M) + np.triu(M, 1).T  # symmetrize
            g = pin.computeGeneralizedGravity(self.model, self.data, self.q)

            # get setpoints
            x_d = self.x_d
            x_d_dot = self.x_d_dot
            x_d_ddot = self.x_d_ddot
            
            # get sim data
            q = self.q
            q_dot = self.qdot
            tau = self.tau
            F_ext = self.F_ext

            # get controller parameters
            Lambda_d = self.Lambda_d
            K_d = self.K_d
            D_d = self.D_d

            T_ee = self.data.oMf[frame_id] # transform from world to EE

            # matrix operations
            J_T = np.transpose(J)
            M_inv = np.linalg.inv(M)

            # forward kinematics
            x_pos = T_ee.translation
            x_rot = pin.Quaternion(T_ee.rotation)
            x_dot = J @ q_dot

            # calculate errors
            x_d_rot = pin.Quaternion(pin.exp3(x_d[3:]))
            x_d_rot = x_d_rot if x_rot.dot(x_d_rot) > 0 else pin.Quaternion(-x_d_rot.coeffs())
            e = np.concatenate([
                x_pos - x_d[:3],                       # translation
                (x_d_rot.inverse() * x_rot).vec()      # rotation
            ])
            e_dot = x_dot - x_d_dot

            self.get_logger().info(
                f"e_pos: {np.linalg.norm(e[:3]):.3f}  "
                f"e_rot: {np.linalg.norm(e[3:]):.3f}  "
                f"e_dot_pos: {np.linalg.norm(e_dot[:3]):.3f}  "
                f"e_dot_rot: {np.linalg.norm(e_dot[3:]):.3f}",
                throttle_duration_sec=10
            )

            # calculate task space inertia
            Lambda = np.linalg.inv(J @ M_inv @ J_T)

            # calculate virtual cartesian wrench
            Lambda_d_inv = np.linalg.inv(Lambda_d)
            F_tau = Lambda @ x_d_ddot - Lambda @ Lambda_d_inv @ (D_d @ e_dot + K_d @ e) + (Lambda @ Lambda_d_inv - np.eye(6)) @ F_ext - Lambda @ J_dot @ q_dot

            # calculate desired torques
            tau_d = J_T @ F_tau + C @ q_dot + g

            msg = JointState()
            msg.effort = tau_d.tolist()
            self.pub_joint_torques.publish(msg)

            self.get_logger().info(f"F_ext: {self.F_ext}", throttle_duration_sec=10)
            self.get_logger().info(f"q: {np.round(q, 3)}", throttle_duration_sec=10)
            self.get_logger().info(f"tau: {np.round(tau_d, 3)}", throttle_duration_sec=10)
        

    def joint_states_callback(self, msg : JointState):
        self.last_state_time = msg.header.stamp

        # truncate to only first 7 joints (delete EE)
        self.q      =   np.array(msg.position[:7], dtype=np.float64)
        self.qdot   =   np.array(msg.velocity[:7], dtype=np.float64)
        self.tau    =   np.array(msg.effort[:7], dtype=np.float64)

        if not self.start_ctrl_loop:
            self.start_ctrl_loop = True


    def wrench_callback(self, msg : WrenchStamped):
        
        self.F_ext = np.array([
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, 
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
            ])
        

def main(args=None):
    rclpy.init(args=args)
    node = ImpedenceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()