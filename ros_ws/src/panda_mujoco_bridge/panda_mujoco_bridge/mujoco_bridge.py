import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, Vector3

import mujoco
import mujoco.viewer
from dm_control import mjcf

class MuJoCoBridge(Node):
    def __init__(self):
        super().__init__('mujoco_bridge')
        self.get_logger().info('MuJoCoBridge started...')

        self.display_on = self.declare_parameter("display_on", True)
        self.axes_on = self.declare_parameter("axes_on", False)
        self.display_freq = self.declare_parameter("display_freq", 60)

        self.model_path = self.declare_parameter("model_path", "/workspace/assets/models/custom/scene_torque.xml").get_parameter_value().string_value
        self.model = self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        
        self.hand_id         = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "hand")
        self.left_finger_id  = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_finger")
        self.right_finger_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_finger")

        if self.display_on.get_parameter_value().bool_value:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.get_logger().info('MuJoCo display opened...')

            if self.axes_on.get_parameter_value().bool_value:
                self.viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY
        else:
            self.viewer = None



        self.joint_names = [self.model.joint(i).name for i in range(self.model.njnt)]
        
        self.pub_joint_states = self.create_publisher(JointState, "joint_states", 10)
        self.pub_wrench = self.create_publisher(WrenchStamped, "wrench_external", 10)
        self.pub_timer = self.create_timer(self.model.opt.timestep, self.publisher_callback)
        self.viewer_timer = self.create_timer(1/self.display_freq.get_parameter_value().integer_value, self.viewer_callback)

        self.sub_joint_torques = self.create_subscription(JointState, "joint_torques", self.subscriber_callback, 10)

    def publisher_callback(self):
        
        # step sim forward
        if self.viewer is not None:
            with self.viewer.lock():
                mujoco.mj_step(self.model, self.data)
                mujoco.mj_rnePostConstraint(self.model, self.data)
        else:
            mujoco.mj_step(self.model, self.data)

        # publish new joint states
        j = JointState()

        j.header.stamp = self.get_clock().now().to_msg()

        j.name = self.joint_names
        j.position = self.data.qpos.tolist()
        j.velocity = self.data.qvel.tolist()
        j.effort = self.data.qfrc_actuator.tolist()

        self.pub_joint_states.publish(j)

        # publish external wrench
        w = WrenchStamped()
        w.header.stamp = self.get_clock().now().to_msg()
        w.header.frame_id = "world"

        # get external wrench for all components of end effector
        wrench = (
            self.data.cfrc_ext[self.hand_id] +
            self.data.cfrc_ext[self.left_finger_id] +
            self.data.cfrc_ext[self.right_finger_id] +
            self.data.xfrc_applied[self.hand_id] +
            self.data.xfrc_applied[self.left_finger_id] +
            self.data.xfrc_applied[self.right_finger_id]
        ).copy()

        w.wrench.force = self.list_to_vector3(wrench[3:6])
        w.wrench.torque = self.list_to_vector3(wrench[0:3])


        self.pub_wrench.publish(w)


    def subscriber_callback(self, msg : JointState):

        # TODO: add code to check if efforts are above torque limits and add a warn
        
        self.data.ctrl[:len(msg.effort)] = msg.effort.tolist()
        #print(msg.effort)

    
    def viewer_callback(self):
        
        if self.viewer is not None:
            self.viewer.sync()


    @staticmethod
    def list_to_vector3(v):
        if len(v) != 3:
            raise ValueError(f"Expected length 3, got {len(v)}")
        return Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))
    
    def destroy_node(self):
        if self.viewer is not None:
            self.viewer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()