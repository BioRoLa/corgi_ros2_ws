# IMU node executable
add_executable(imu_node imu.cpp)
ament_target_dependencies(imu_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  corgi_msgs
)
target_link_libraries(imu_node Eigen3::Eigen mip)

# Install executables
install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)