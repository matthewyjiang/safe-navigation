#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from reactive_planner_lib import polygonDifeo

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.declare_parameter('robot_radius', 0.1)
        self.goal_point = None
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        
        
        self.create_subscription(PointArray, 'safe_set', self.safe_set_callback, 10)
    
    def safe_set_callback(self, msg):
        self.get_logger().info('Received safe set')
        self.safe_set = PointArray()
        self.safe_set.points = msg.points
        
    
if __name__ == '__main__':
    rclpy.init()
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    rclpy.shutdown()
    
       