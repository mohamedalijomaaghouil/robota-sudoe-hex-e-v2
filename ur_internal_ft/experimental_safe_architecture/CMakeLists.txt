cmake_minimum_required(VERSION 3.8)
project(admittance_control_ros2)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(admittance_control_node src/admittance_control_ros2.cpp)
ament_target_dependencies(admittance_control_node
  rclcpp geometry_msgs std_msgs std_srvs sensor_msgs tf2_ros tf2_geometry_msgs urdf kdl_parser orocos_kdl)
target_link_libraries(admittance_control_node Eigen3::Eigen)
install(TARGETS admittance_control_node DESTINATION lib/${PROJECT_NAME})

add_executable(admittance_preview_node src/admittance_preview.cpp)
ament_target_dependencies(admittance_preview_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  urdf
  kdl_parser
  orocos_kdl
)
target_link_libraries(admittance_preview_node Eigen3::Eigen)
target_compile_features(admittance_preview_node PRIVATE cxx_std_17)
install(TARGETS admittance_preview_node
  DESTINATION lib/${PROJECT_NAME})


add_executable(
  admittance_preview_gate_node
  src/admittance_preview_gate.cpp
)
ament_target_dependencies(
  admittance_preview_gate_node
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
)
target_compile_features(
  admittance_preview_gate_node
  PRIVATE cxx_std_17
)
install(
  TARGETS admittance_preview_gate_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  admittance_controller_guard_node
  src/admittance_controller_guard.cpp
)
ament_target_dependencies(
  admittance_controller_guard_node
  rclcpp
  std_msgs
  controller_manager_msgs
)
target_compile_features(
  admittance_controller_guard_node
  PRIVATE cxx_std_17
)
install(
  TARGETS admittance_controller_guard_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  admittance_zero_command_bridge_node
  src/admittance_zero_command_bridge.cpp
)
ament_target_dependencies(
  admittance_zero_command_bridge_node
  rclcpp
  std_msgs
)
target_compile_features(
  admittance_zero_command_bridge_node
  PRIVATE cxx_std_17
)
install(
  TARGETS admittance_zero_command_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  admittance_execution_interlock_node
  src/admittance_execution_interlock.cpp
)
ament_target_dependencies(
  admittance_execution_interlock_node
  rclcpp
  std_msgs
  controller_manager_msgs
  ur_dashboard_msgs
)
target_compile_features(
  admittance_execution_interlock_node
  PRIVATE cxx_std_17
)
install(
  TARGETS admittance_execution_interlock_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  admittance_execution_bridge_node
  src/admittance_execution_bridge.cpp
)
ament_target_dependencies(
  admittance_execution_bridge_node
  rclcpp
  std_msgs
  std_srvs
)
target_compile_features(
  admittance_execution_bridge_node
  PRIVATE cxx_std_17
)
install(
  TARGETS admittance_execution_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
