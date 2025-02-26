import rclpy
from rclpy.node import Node

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__("ObstacleDetectorNode")
        self.get_logger().info('ObstacleDetectorNode has been started.')


def main(args=None):
    rclpy.init(args=args)
    detector_node = ObstacleDetectorNode()
    rclpy.spin(detector_node)
    rclpy.shutdown(detector_node)

if __name__ == '__main__':
    main()
