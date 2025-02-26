#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ObstacleDetectorNode(Node):
    def __init__(self, node_name="obstacle_detector_node"):
        self._node_name = node_name
        super().__init__(self._node_name)

        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.get_logger().info(self._node_name + " Ready...")

    def laserscan_callback(self, msg):
        sectors = {
            "Right_Rear": (0, 33),
            "Right": (34, 66),
            "Front_Right": (67, 100),
            "Front_Left": (101, 133),
            "Left": (134, 166),
            "Left_Rear": (167, 199)
        }

        min_distances = {key: float('inf') for key in sectors.keys()}

        for sector, (start_idx, end_idx) in sectors.items():
            if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
                sector_ranges = msg.ranges[start_idx:end_idx + 1]
                if sector_ranges:
                    min_distances[sector] = min(sector_ranges)

        for sector, min_distance in min_distances.items():
            self.get_logger().info(f'{sector}: {min_distance:.2f} meters')

        obstacle_threshold = 0.8 

        detections = {sector: min_distance < obstacle_threshold for sector, min_distance in min_distances.items()}

        if detections["Front_Left"] and detections["Front_Right"]:
            arbitrary_direction = "Right"
            action = "Selected Turn Arbitrary Direction "+arbitrary_direction        
        elif detections["Front_Left"] and not detections["Front_Right"]:
            action = "Turn Right to avoid obstacle on the front-left"
        elif detections["Front_Right"] and not detections["Front_Left"]:
            action = "Turn Left to avoid obstacle on the front-right"

        elif detections["Left"]:
            action = "Go Forwards turning slightly right to avoid obstacle on the left"
        elif detections["Right"]:
            action = "Go Forwards turning slightly left to avoid obstacle on the right"

        elif detections["Right_Rear"]:
            action = "Go Forwards, BUT DONT reverse Right"
        elif detections["Left_Rear"]:
            action = "Go Forwards, BUT DONT reverse left"
        else:
            action = "Go Forwards"

        self.get_logger().info(f'Suggested action: {action}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()