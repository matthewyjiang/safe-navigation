#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import safeopt
import matplotlib.pyplot as plt
import GPy
import numpy as np
import pandas as pd
from safe_navigation.srv import AddDataPoint
from ament_index_python.packages import get_package_share_directory
import os

from std_msgs.msg import String

PACKAGE_NAME = 'safe_navigation'
PACKAGE_SHARE_DIRECTORY = get_package_share_directory(PACKAGE_NAME)

class SafeOptNode(Node):
    def __init__(self):
        super().__init__('safeopt_publisher_node')
        
        self.declare_parameter('kernel_variance', 1.0)
        self.declare_parameter('kernel_length_scale', 1.0)
        self.declare_parameter('lipschitz', None)
        self.declare_parameter('safety_threshold', 0.0)
        self.declare_parameter('bounds', None)
        self.declare_parameter('beta', 2.0)
        self.declare_parameter('safeparameters_filename', 'safe_parameters.yaml')
        
        self.data_file = self.get_parameter('safeparameters_filename').get_parameter_value().string_value
        file_path = os.path.join(PACKAGE_SHARE_DIRECTORY, self.data_file)
        
        # read file into pandas dataframe
        
        starting_data = pd.read_csv(file_path)
        self.X = starting_data[['X1', 'X2']].values
        self.Y = starting_data['Y'].values.reshape(-1, 1)
        
        self.bounds = self.get_parameter('bounds').get_parameter_value().integer_array_value
        self.bounds = list(self.bounds)
        
        self.xmin = self.bounds[0]
        self.xmax = self.bounds[1]
        self.ymin = self.bounds[2]
        self.ymax = self.bounds[3]
        max_dim = max(self.xmax - self.xmin, self.ymax - self.ymin)
        
        self.parameter_set = safeopt.linearly_spaced_combinations([(self.xmin, self.xmax), (self.ymin, self.ymax)], max_dim)
        self.beta = self.get_parameter('beta').get_parameter_value().double_value
        self.lipschitz = self.get_parameter('lipschitz').get_parameter_value().double_value
        self.safety_threshold = self.get_parameter('safety_threshold').get_parameter_value().double_value
        self.add_data_point_srv = self.create_service(AddDataPoint, 'add_data_point', self.add_data_point_callback)

        self.variance = self.get_parameter('kernel_variance').get_parameter_value().double_value
        self.lengthscale = self.get_parameter('kernel_length_scale').get_parameter_value().double_value
        
        # self.X = np.empty((0, 2))
        # self.Y = np.empty((0, 1))
        
        self.kernel = GPy.kern.RBF(input_dim=2, variance=self.variance, lengthscale=self.lengthscale)
        self.model = GPy.models.GPRegression(self.X, self.Y, self.kernel)
        self.opt = safeopt.SafeOpt(self.model, self.parameter_set, self.safety_threshold, self.lipschitz, self.beta)

        
    def add_data_point_callback(self, request, response):
        point = np.array([request.x1, request.x2])
        self.X = np.append(self.X, [point], axis=0)
        self.Y = np.append(self.Y, [[request.y]], axis=0)
        
        if (len(self.X.shape) == 1):
            self.X = self.X.reshape(1, -1)
        if (len(self.Y.shape) == 1):
            self.Y = self.Y.reshape(-1, 1)
            
        self.opt.add_new_data_point(point, request.y)
        

        # Plotting the GP
        plt.figure(figsize=(10, 5))
        self.opt.plot(50)
        plt.scatter(self.X[:, 0], self.X[:, 1], c=self.Y, s=50, zorder=10, edgecolors=(0, 0, 0))
        plt.title('Gaussian Process with SafeOpt')
        plt.xlabel('X1')
        plt.ylabel('X2')
        plt.colorbar(label='Y')
        plt.show()
        
        
        response.success = True
        return response
    
    

        


def main(args=None):
    rclpy.init(args=args)

    safeopt_node = SafeOptNode()
    
    

    rclpy.spin(safeopt_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
