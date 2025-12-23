# IMU node executable
add_executable(imu_node src/imu.cpp)
ament_target_dependencies(imu_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  corgi_msgs
)
target_link_libraries(imu_node Eigen3::Eigen mip)

# IMU listener node executable
add_executable(imu_node_listener src/imu_listener.cpp)
ament_target_dependencies(imu_node_listener
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  corgi_msgs
)
target_link_libraries(imu_node_listener Eigen3::Eigen mip)

# Install executables
install(TARGETS
  imu_node
  imu_node_listener
  DESTINATION lib/${PROJECT_NAME}
)