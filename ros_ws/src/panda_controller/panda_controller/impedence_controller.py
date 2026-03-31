import debugpy
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, PoseStamped

import pinocchio as pin


class ImpedenceController(Node):

    _DEFAULT_STALE_TIME_NS = 100*1e6 # 100ms
    _DEFAULT_N_JOINTS = 7
    _DEFAULT_CTRL_LOOP_FREQ_HZ = 1024

    #_STARTING_Q = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.035, 0.035])

    _DEFAULT_LAMDA_D = np.diag([1.5, 1.5, 1.5, 0.1, 0.1, 0.1])
    _DEFAULT_K_D = np.diag([150, 150, 150, 8, 8, 8])
    _DEFAULT_D_D = np.diag([30, 30, 30, 3, 3, 3])
    
    _DEFAULT_K_N = np.diag([10, 10, 10, 10, 10 ,10, 10])
    _DEFAULT_D_N = 2*np.sqrt(_DEFAULT_K_N) # critical damping

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
        #self.x_d = self._DEFAULT_x_d
        self.x_d_dot = self._DEFAULT_x_d_dot
        self.x_d_ddot = self._DEFAULT_x_d_ddot

        # Controller Paramters
        self.Lambda_d = self._DEFAULT_LAMDA_D
        self.K_d = self._DEFAULT_K_D
        self.D_d = self._DEFAULT_D_D
        self.K_N = self._DEFAULT_K_N
        self.D_N = self._DEFAULT_D_N
        self.F_ext = np.zeros(6)

        # Initialize Pinocchio
        # build reduced model
        model_path = "/workspace/assets/models/franka_emika_panda/panda.xml"
        full_model, *_ = pin.buildModelsFromMJCF(model_path)
        q_home_full = full_model.referenceConfigurations["home"]
        self.model = pin.buildReducedModel(full_model, [8, 9], q_home_full)
        self.data = self.model.createData()

        # set initial x_d to be home position
        q_home = q_home_full[:7]
        pin.forwardKinematics(self.model, self.data, q_home)
        pin.updateFramePlacements(self.model, self.data)
        frame_id = self.model.getFrameId("hand")
        T_home = self.data.oMf[frame_id]
        #self.x_d = np.concatenate([T_home.translation.copy(), pin.log3(T_home.rotation.copy())])
        #self.get_logger().info(f"x_d initialized from keyframe: {np.round(self.x_d, 3)}")
        
        # consider joint limits in null joint set points
        self.q_min = self.model.lowerPositionLimit
        self.q_max = self.model.upperPositionLimit
        #self.q_N = 0.5 * (self.q_min + self.q_max) # set desired null space config to be between joint limits
        self.q_N = q_home.copy()

        # Timers
        self.timer_ctrl_loop = self.create_timer(1/self.freq, self.ctrl_loop_callback)
        self.start_ctrl_loop = False

        # Publishers
        self.pub_joint_torques = self.create_publisher(JointState, "joint_torques", 10)

        # Subscribers
        self.sub_joint_states = self.create_subscription(JointState, "joint_states", self.joint_states_callback, 10)
        self.sub_external_wrench = self.create_subscription(WrenchStamped, "wrench_external", self.wrench_callback, 10)
        self.sub_desired_pose = self.create_subscription(PoseStamped, "desired_ee_pose", self.pose_callback, 10)


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
            x_d_pos = self.x_d_pos
            x_d_rot = self.x_d_quat
            

            x_d_dot = self.x_d_dot
            x_d_ddot = self.x_d_ddot
            
            # TODO: set q_N as q for x_d (run IK)
            q_N = self.q_N
            
            # get sim data
            q = self.q
            q_dot = self.qdot
            tau = self.tau
            F_ext = self.F_ext

            self.check_joint_limits(q) # this is for debugging if null torques are doing their job

            # get controller parameters
            Lambda_d = self.Lambda_d
            K_d = self.K_d
            D_d = self.D_d
            K_N = self.K_N
            D_N = self.D_N

            T_ee = self.data.oMf[frame_id] # transform from world to EE

            # matrix operations
            J_T = np.transpose(J)
            M_inv = np.linalg.inv(M)

            # forward kinematics
            x_pos = T_ee.translation
            x_rot = pin.Quaternion(T_ee.rotation)
            x_dot = J @ q_dot

            # calculate errors
            x_d_rot = self.x_d_quat
            x_d_rot = x_d_rot if x_rot.dot(x_d_rot) > 0 else pin.Quaternion(-x_d_rot.coeffs())

            e_quat = x_d_rot.inverse() * x_rot          # error in desired frame
            e_rot_world = x_d_rot.toRotationMatrix() @ e_quat.vec()  # rotate to world frame

            e = np.concatenate([
                x_pos - self.x_d_pos,
                e_rot_world
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

            # calculate desired torques (cartesian)
            tau_d_cart = J_T @ F_tau + C @ q_dot + g

            # find null torques
            tau_d_null = -K_N @ (q - q_N) - D_N @ q_dot
            
            # find null projector
            _, _, Vh_T = np.linalg.svd(J, full_matrices=True)
            V_T = Vh_T[6:,:] # extract null space basis
            N_1 = V_T.T @ V_T

            # get torques to send to robot
            tau_d = tau_d_cart + N_1 @ tau_d_null

            msg = JointState()
            msg.effort = tau_d.tolist()
            self.pub_joint_torques.publish(msg)

            #self.get_logger().info(f"F_ext: {self.F_ext}", throttle_duration_sec=10)
            #self.get_logger().info(f"q: {np.round(q, 3)}", throttle_duration_sec=10)
            #self.get_logger().info(f"tau: {np.round(tau_d, 3)}", throttle_duration_sec=10)
        

    def joint_states_callback(self, msg : JointState):
        self.last_state_time = msg.header.stamp

        # truncate to only first 7 joints (delete EE)
        self.q      =   np.array(msg.position[:7], dtype=np.float64)
        self.qdot   =   np.array(msg.velocity[:7], dtype=np.float64)
        self.tau    =   np.array(msg.effort[:7], dtype=np.float64)

        if not self.start_ctrl_loop:
            pin.forwardKinematics(self.model, self.data, self.q)
            pin.updateFramePlacements(self.model, self.data)
            frame_id = self.model.getFrameId("hand")
            T = self.data.oMf[frame_id]
            self.x_d_pos = T.translation.copy()
            self.x_d_quat = pin.Quaternion(T.rotation.copy())
            self.q_N = self.q.copy()
            self.get_logger().info(f"x_d_pos initialized from current state: {np.round(self.x_d_pos, 4)}")
            self.get_logger().info(f"x_d_quat initialized from current state: {np.round(self.x_d_quat.coeffs(), 4)}")
            self.start_ctrl_loop = True


    def wrench_callback(self, msg : WrenchStamped):
        
        self.F_ext = np.array([
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, 
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
            ])
        
    def pose_callback(self, msg : PoseStamped):
        
        self.x_d_pos, self.x_d_quat = self.pose_stamped_to_xd(msg)
        self.q_N = self.ik(self.x_d_pos, self.x_d_quat, self.q) 

    @staticmethod
    def pose_stamped_to_xd(msg: PoseStamped) -> tuple[np.ndarray, pin.Quaternion]:
        
        x_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        quat = msg.pose.orientation
        x_rot = pin.Quaternion(quat.w, quat.x, quat.y, quat.z).normalize() # normalize to fix floating point accumulation
    
        return x_pos, x_rot
    

    
    # inverse kinematics using CLIK
    def ik(self, x_pos : np.ndarray, x_rot : pin.Quaternion, q0 : np.ndarray, max_iter=1000, tol=1e-4, step_size=0.5):
        ee_id = self.model.getFrameId("hand")

        target = pin.SE3(
            x_rot.toRotationMatrix(),
            np.array(x_pos)
        )

        q = q0.copy()

        for i in range(max_iter):
            pin.framesForwardKinematics(self.model, self.data, q)
            T_current = self.data.oMf[ee_id]

            err = pin.log6(T_current.inverse() * target).vector

            if np.linalg.norm(err) < tol:
                return q

            pin.computeJointJacobians(self.model, self.data, q)
            J = pin.getFrameJacobian(self.model, self.data, ee_id, pin.ReferenceFrame.LOCAL)

            J_pinv = np.linalg.pinv(J)
            dq = step_size * J_pinv @ err

            q = pin.integrate(self.model, q, dq)
            q = pin.normalize(self.model, q)

        raise RuntimeError(f"IK failed to converge. Final error: {np.linalg.norm(err):.6f}")
        
    # checks if given joint state q is close to joint limits
    def check_joint_limits(self, q: np.ndarray, threshold: float = 0.1) -> None:
        margin = threshold * (self.q_max - self.q_min)  # per-joint absolute margin

        near_lower = q < (self.q_min + margin)
        near_upper = q > (self.q_max - margin)

        for i in range(len(q)):
            if near_lower[i]:
                self.get_logger().warn(
                    f"Joint {i+1} is near its LOWER limit: "
                    f"q={q[i]:.3f} rad, limit={self.q_min[i]:.3f} rad, "
                    f"margin={margin[i]:.3f} rad",
                    throttle_duration_sec=10
                )
            if near_upper[i]:
                self.get_logger().warn(
                    f"Joint {i+1} is near its UPPER limit: "
                    f"q={q[i]:.3f} rad, limit={self.q_max[i]:.3f} rad, "
                    f"margin={margin[i]:.3f} rad",
                    throttle_duration_sec=10
                )
        

def main(args=None):
    rclpy.init(args=args)
    node = ImpedenceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()