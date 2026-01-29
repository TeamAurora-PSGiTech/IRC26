cmake_minimum_required(VERSION 3.8)
project(arm_ctrl)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msgs/Arm.msg"
  DEPENDENCIES geometry_msgs
)

# Install Python modules
install(DIRECTORY arm_ctrl
  DESTINATION lib/python3.10/site-packages
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
