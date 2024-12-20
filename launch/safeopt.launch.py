import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(
        os.path.dirname(os.path.dirname(__file__)), 'config', 'config.yaml'  # Adjust the relative path
        ),
        description='Path to the parameter file to use'
    )
    
    param_file = LaunchConfiguration('param_file')
    

    return LaunchDescription([param_file_arg, Node(
            package='safe_navigation',  
            executable='safeopt_node.py',             
            name='safeopt_node',
            parameters=[
                param_file
            ],
        ),
            Node(
            package='safe_navigation',  
            executable='test_data_client.py',             
            name='test_data_client',
            parameters=[
                param_file
            ],
        )
    ])
