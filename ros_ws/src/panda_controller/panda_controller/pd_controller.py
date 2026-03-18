import rclpy
from rclpy.node import Node

class PDController(Node):
    def __init__(self):
        super().__init__("pd_controller")



def main(args=None):
    rclpy.init(args=args)
    node = PDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()