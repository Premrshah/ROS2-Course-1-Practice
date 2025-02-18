#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import random

class TemperatureNode(Node):
    def __init__(self):
        # call super() in the constructor to initialize the Node object
        # the parameter we pass is the node name
        super().__init__("Temperature_node")
        self.get_logger().info('Temperature Monitor Node has been started.')
        # create a timer sending two parameters:
        # - the duration between two callbacks (0.2 seconds)
        # - the timer function (timer_callback)
        self.create_timer(1, self.timer_callback)
        
    def timer_callback(self):
        temperature = random.uniform(20.0,100.0)
        if temperature > 70:
            self.get_logger().warn(str(temperature))
        else:
            self.get_logger().info(str(temperature))

def start_monitor(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = TemperatureNode()
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclpy.spin(node)
    # shutdown the ROS2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    start_monitor()