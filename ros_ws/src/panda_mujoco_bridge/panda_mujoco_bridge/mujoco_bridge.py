import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import mujoco
import mujoco.viewer
from dm_control import mjcf

class MuJoCoBridge(Node):
    def __init__(self):
        super().__init__('mujoco_bridge')
        self.get_logger().info('MuJoCoBridge started...')

        self.display_on = self.declare_parameter("display_on", True)
        self.display_freq = self.declare_parameter("display_freq", 60)

        self.model_path = self.declare_parameter("model_path", "/workspace/assets/models/franka_emika_panda/scene.xml").get_parameter_value().string_value
        self.model = self.model_to_torque_control(self.model_path)
        self.data = mujoco.MjData(self.model)

        if self.display_on.get_parameter_value().bool_value:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.get_logger().info('MuJoCo display opened...')
        else:
            self.viewer = None

        self.joint_names = [self.model.joint(i).name for i in range(self.model.njnt)]
        
        self.pub_joint_states = self.create_publisher(JointState, "joint_states", 10)
        self.pub_timer = self.create_timer(self.model.opt.timestep, self.publisher_callback)
        self.viewer_timer = self.create_timer(1/self.display_freq.get_parameter_value().integer_value, self.viewer_callback)

        self.sub_joint_torques = self.create_subscription(JointState, "joint_torques", self.subscriber_callback, 10)

    def publisher_callback(self):
        
        # step sim forward
        if self.viewer is not None:
            with self.viewer.lock():
                mujoco.mj_step(self.model, self.data)
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



    def subscriber_callback(self, msg : JointState):
        
        self.data.ctrl[:] = msg.effort

    
    def viewer_callback(self):
        
        if self.viewer is not None:
            self.viewer.sync()


    # converts Panda Model from position input (with built in PID) to torque input
    def model_to_torque_control(self, path):
        
        #root = mjcf.from_path("/workspace/assets/models/franka_emika_panda/scene.xml")
        root = mjcf.from_path(path)
    
        for actuator in root.find_all('actuator'):
            actuator.remove()

        # Add motor actuators for arm joints only
        root.actuator.add('motor', name='actuator1', joint='joint1', gear=[1], ctrllimited=True, ctrlrange=[-87, 87])
        root.actuator.add('motor', name='actuator2', joint='joint2', gear=[1], ctrllimited=True, ctrlrange=[-87, 87])
        root.actuator.add('motor', name='actuator3', joint='joint3', gear=[1], ctrllimited=True, ctrlrange=[-87, 87])
        root.actuator.add('motor', name='actuator4', joint='joint4', gear=[1], ctrllimited=True, ctrlrange=[-87, 87])
        root.actuator.add('motor', name='actuator5', joint='joint5', gear=[1], ctrllimited=True, ctrlrange=[-12, 12])
        root.actuator.add('motor', name='actuator6', joint='joint6', gear=[1], ctrllimited=True, ctrlrange=[-12, 12])
        root.actuator.add('motor', name='actuator7', joint='joint7', gear=[1], ctrllimited=True, ctrlrange=[-12, 12])
        root.actuator.add('motor', name='actuator8', tendon='split', gear=[1], ctrllimited=True, ctrlrange=[-100, 100])

        physics = mjcf.Physics.from_mjcf_model(root)
        model = physics.model.ptr

        return model
    
    def destro_node(self):
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