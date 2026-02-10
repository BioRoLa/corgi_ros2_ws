# Define the executable and link required libraries
add_executable(corgi_odometry ./corgi_odometry.cpp)
target_link_libraries(corgi_odometry Eigen3::Eigen Kinematic IF)
ament_target_dependencies(corgi_odometry rclcpp std_msgs geometry_msgs sensor_msgs corgi_msgs corgi_utils)

add_executable(corgi_z_position ./corgi_z_position.cpp)
target_link_libraries(corgi_z_position Eigen3::Eigen IF)
ament_target_dependencies(corgi_z_position rclcpp std_msgs geometry_msgs sensor_msgs corgi_msgs corgi_utils)

install(TARGETS
  corgi_odometry
  corgi_z_position
  DESTINATION lib/${PROJECT_NAME}
)