cmake_minimum_required(VERSION 3.8)
project(safe_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts (e.g., entry points)
install(
    PROGRAMS
        safe_navigation/safeopt_node.py
        safe_navigation/test_data_client.py
    DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/AddDataPoint.srv"
    "msg/PointArray.msg"
    DEPENDENCIES geometry_msgs
)

# Add dependencies
ament_export_dependencies(rosidl_default_runtime)

# Optionally install data files (e.g., config or launch files)
install(
    DIRECTORY config launch data
    DESTINATION share/${PROJECT_NAME}
)

# Package export
ament_package()