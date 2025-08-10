import rclpy
from rclpy.node import Node

class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')
        self.get_logger().info('Autonomous Transport Bot Node Started')

def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
