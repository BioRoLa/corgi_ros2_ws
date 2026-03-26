# Define the executable and link required libraries
add_executable(corgi_odometry_legacy ./corgi_odometry_legacy.cpp)
target_link_libraries(corgi_odometry_legacy Eigen3::Eigen Kinematic IF)
ament_target_dependencies(corgi_odometry_legacy rclcpp std_msgs geometry_msgs sensor_msgs corgi_msgs corgi_utils)

add_executable(corgi_z_position_legacy ./corgi_z_position_legacy.cpp)
target_link_libraries(corgi_z_position_legacy Eigen3::Eigen IF)
ament_target_dependencies(corgi_z_position_legacy rclcpp std_msgs geometry_msgs sensor_msgs corgi_msgs corgi_utils)

install(TARGETS
  corgi_odometry_legacy
  corgi_z_position_legacy
  DESTINATION lib/${PROJECT_NAME}
)