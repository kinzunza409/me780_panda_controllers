import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class SimpleTrajectory(Node):
    def __init__(self):
        super().__init__('simple_trajectory')
        
        # Declare parameters
        self.n_seconds = self.declare_parameter('n_seconds', 5).value
        trajectory = self.declare_parameter('trajectory', 'home').value
        
        # Publisher
        self.publisher = self.create_publisher(PoseStamped, '/desired_ee_pose', 10)
        
        # Set pose based on trajectory
        if trajectory == 'home':
            self.set_pose(0.5, 0.0, 0.2, 1/np.sqrt(2), 1/np.sqrt(2), 0.0, 0.0)
        else:
            self.get_logger().error(f"Unknown trajectory: {trajectory}")
            raise RuntimeError()
    
        self.timer = self.create_timer(self.n_seconds, self.publish_pose)

    
    def set_pose(self, px, py, pz, qx, qy, qz, qw):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        self.pose = pose
    
    def publish_pose(self):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.pose)
        self.destroy_timer(self.timer)
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()