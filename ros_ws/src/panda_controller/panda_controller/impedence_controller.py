import debugpy
debugpy.listen(("0.0.0.0", 5678))

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import pinocchio as pin


class ImpedenceController(Node):

    _DEFAULT_STALE_TIME_NS = 100*1e6 # 100ms
    _DEFAULT_N_JOINTS = 7
    _DEFAULT_CTRL_LOOP_FREQ_HZ = 1024

    _STARTING_Q = np.array([0.0, -np.pi/4, 0.0, -3*np.pi/4, 0.0, np.pi/2, np.pi/4, 0.035, 0.035])

    _DEFAULT_LAMDA_D = np.diag([1.5, 1.5, 1.5, 0.1, 0.1, 0.1])
    _DEFAULT_K_D = np.diag([150, 150, 150, 8, 8, 8])
    _DEFAULT_D_D = np.diag([30, 30, 30, 3, 3, 3])

    _DEFAULT_x_d = np.array([0.3, 0.0, 0.4, np.pi, 0, 0])
    _DEFAULT_x_d_dot = np.zeros(6)
    _DEFAULT_x_d_ddot = np.zeros(6)

    def __init__(self):
        super().__init__("impedence_controller")

        self.get_logger().info("Impedence Controller node started...")

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

        # Initialize Pinocchio
        model_path = "/workspace/assets/models/franka_emika_panda/panda.xml"
        # TODO: fix the end effector to reduce the model
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


    def ctrl_loop_callback(self):

        if self.start_ctrl_loop:
            # update Pinocchio data
            pin.forwardKinematics(self.model, self.data, self.q, self.qdot)
            J = pin.computeFrameJacobian(self.model, self.data, self.q,
                    self.model.nframes - 1, 
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J_dot = pin.computeJointJacobiansTimeVariation(self.model, self.data, self.q, self.qdot)
            C = pin.computeCoriolisMatrix(self.model, self.data, self.q, self.qdot)
            M = pin.crba(self.model, self.data, self.q) # mass matrix (only upper triangular)
            g = pin.computeGeneralizedGravity(self.model, self.data, self.q)

            # get joint data
            q = self.q
            q_dot = self.qdot
            tau = self.tau

            # get controller parameters
            Lambda_d = self.Lambda_d
            K_d = self.K_d
            D_d = self.D_d

            T_ee = self.data.oMf[-1] # transform from world to EE

            # forward kinematics
            x = np.concatenate([
                T_ee.translation,
                pin.rpy.matrixToRpy(T_ee.rotation)])
            x_dot = J @ q_dot

            # forward dynamics
            q_ddot = np.linalg.solve(M, tau - C@q_dot - g) # use np solver to avoid calcuating M_inverse
            #x_ddot = J @ q_ddot + J_dot @ q_dot
            x_ddot = q_ddot[:6]
            
            # get errors
            # TODO: Lectures show e = x - x_d but logically it should be the other way around. Look into this
            e       =   self.x_d - x
            e_dot   =   self.x_d_dot - x_dot
            #e_ddot  =   x_ddot - self.x_d_ddot

            self.get_logger().info(f"e: {np.linalg.norm(e):.4f}  e_dot: {np.linalg.norm(e_dot):.4f}", throttle_duration_sec=10)

            # Calculate F_ext
            #F_ext = Lambda_d @ e_ddot + D_d @ e_dot + K_d @ e
            #self.get_logger().info(f'F_ext: [Fx={F_ext[0]:.3f}, Fy={F_ext[1]:.3f}, Fz={F_ext[2]:.3f}] N, M_ext: [Mx={F_ext[3]:.3f}, My={F_ext[4]:.3f}, Mz={F_ext[5]:.3f}] Nm', throttle_duration_sec=30)
            

            # TODO: Re-read paper, F_ext and F_d are probably different - I think I am finding x_ddot the wrong way
            
            # Basic impedence control from lecture for now
            F = K_d @ e + D_d @ e_dot
            tau_d = np.transpose(J) @ F + C @ q_dot + g
            msg = JointState()
            msg.effort = tau_d.tolist()

            self.pub_joint_torques.publish(msg)
        

    def joint_states_callback(self, msg : JointState):
        self.last_state_time = msg.header.stamp

        # truncate to only first 7 joints (delete EE)
        self.q      =   np.array(msg.position[:7], dtype=np.float64)
        self.qdot   =   np.array(msg.velocity[:7], dtype=np.float64)
        self.tau    =   np.array(msg.effort[:7], dtype=np.float64)

        if not self.start_ctrl_loop:
            self.start_ctrl_loop = True

def main(args=None):
    rclpy.init(args=args)
    node = ImpedenceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()