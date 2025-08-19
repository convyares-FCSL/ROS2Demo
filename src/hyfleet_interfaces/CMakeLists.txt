cmake_minimum_required(VERSION 3.8)
project(hyfleet_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate code from our action definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FillToTarget.action"
)

# Install the .action file so `ros2 interface show` can find it
install(DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

# Export runtime dependency for downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()