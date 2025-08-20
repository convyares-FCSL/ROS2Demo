cmake_minimum_required(VERSION 3.8)
project(tank_pressure_demo_cpp)

add_compile_options(-Wall -Wextra -Wpedantic)

# -------------------- Finds --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)

# >>> The generated interfaces we depend on <<<
find_package(hyfleet_interfaces REQUIRED)

# -------------------- Executables --------------------
include_directories(include)

add_executable(pressure_publisher src/pressure_publisher.cpp)
ament_target_dependencies(pressure_publisher rclcpp std_msgs rcl_interfaces)

add_executable(pressure_subscriber src/pressure_subscriber.cpp)
ament_target_dependencies(pressure_subscriber rclcpp std_msgs)

add_executable(compressor_service src/compressor_service.cpp)
ament_target_dependencies(compressor_service rclcpp std_srvs)

add_executable(scheduler_node src/scheduler_node.cpp)
ament_target_dependencies(scheduler_node rclcpp std_msgs std_srvs)

add_executable(fill_action_server src/fill_action_server.cpp)
target_include_directories(fill_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fill_action_server
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  action_msgs
  hyfleet_interfaces        # <<— add this so headers + typesupport link in
)

add_executable(fill_action_client src/fill_action_client.cpp)
target_include_directories(fill_action_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fill_action_client
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  action_msgs
  hyfleet_interfaces        # <<— same here
)

# -------------------- Install --------------------
install(TARGETS
  pressure_publisher
  pressure_subscriber
  compressor_service
  scheduler_node
  fill_action_server
  fill_action_client
  DESTINATION lib/${PROJECT_NAME}
)

# headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

# launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()