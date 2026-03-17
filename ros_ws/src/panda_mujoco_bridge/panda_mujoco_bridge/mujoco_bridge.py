import rclpy
from rclpy.node import Node


class MuJoCoBridge(Node):
    def __init__(self):
        super().__init__('mujoco_bridge')
        self.get_logger().info('MuJoCoBridge started')

        # TODO: Create subscriber for joint states
        # TODO: Create publisher for joint torques


def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()