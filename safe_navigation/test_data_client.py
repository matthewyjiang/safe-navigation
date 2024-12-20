#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
from safe_navigation.srv import AddDataPoint
from ament_index_python.packages import get_package_share_directory
import sys
import select
import os
import time

PACKAGE_NAME = 'safe_navigation'
PACKAGE_SHARE_DIRECTORY = get_package_share_directory(PACKAGE_NAME)

class TestDataClient(Node):
    def __init__(self):
        super().__init__('safeopt_publisher_node')
        self.cli = self.create_client(AddDataPoint, 'add_data_point')
        self.declare_parameter('data_file', "data/testvalues.csv")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddDataPoint.Request()
    
    def send_request(self, x1, x2, y):
        self.req.x1 = float(x1)
        self.req.x2 = float(x2)
        self.req.y = y
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
        

def main(args=None):
    rclpy.init(args=args)

    testdataclient_node = TestDataClient()
    
    data_file = testdataclient_node.get_parameter('data_file').get_parameter_value().string_value
    file_path = os.path.join(PACKAGE_SHARE_DIRECTORY, data_file)
    
    Y = np.loadtxt(file_path, delimiter=',', skiprows=1)
    # Get the shape of Y
    Y_shape = Y.shape
    
    while rclpy.ok():
        randX1 = np.random.randint(0, Y_shape[0])
        randX2 = np.random.randint(0, Y_shape[1])
        Y_random = Y[randX1, randX2]
        testdataclient_node.get_logger().info(f"Request: {randX1}, {randX2}, {Y_random}")
        
        response = testdataclient_node.send_request(randX1, randX2, Y_random)
        testdataclient_node.get_logger().info(f"Response: {response}")
        
        time.sleep(3)
    
    

    rclpy.spin(testdataclient_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
