add_executable(display display_node.cpp)
add_executable(publisher publisher.cpp)
add_executable(teleop teleop_twist_keyboard.cpp)
add_executable(wheeled_test wheeled_test.cpp wheeled_gen.cpp wheeled_cmd.cpp)

# Link display node with rclcpp and message dependencies
ament_target_dependencies(display
  rclcpp
  std_msgs
  corgi_msgs
)

# Link publisher node with rclcpp and message dependencies
ament_target_dependencies(publisher
  rclcpp
  std_msgs
  sensor_msgs
  corgi_msgs
)

# Link teleop node with rclcpp and message dependencies
ament_target_dependencies(teleop
  rclcpp
  std_msgs
)

# Link wheeled_test with library and ROS dependencies
target_link_libraries(wheeled_test
  ${PROJECT_NAME}
  corgi_utils
  Eigen3::Eigen
  ${CMAKE_DL_LIBS}
)
ament_target_dependencies(wheeled_test
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
  corgi_msgs
  corgi_gait_selector
)



