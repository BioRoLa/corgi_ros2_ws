# Main executable combining joystick control and wheeled control
add_executable(wheel_control_node wheeled.cpp joystick_control.cpp)
ament_target_dependencies(wheel_control_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  corgi_msgs
)
target_link_libraries(wheel_control_node wheeled_control)

install(TARGETS
  wheel_control_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(wheeled_state_publisher publisher.cpp)
ament_target_dependencies(wheeled_state_publisher rclcpp sensor_msgs std_msgs corgi_msgs)
target_link_libraries(wheeled_state_publisher wheeled_control)

add_executable(wheeled_display display_node.cpp)
ament_target_dependencies(wheeled_display rclcpp std_msgs corgi_msgs)
target_link_libraries(wheeled_display wheeled_control)

add_executable(wheeled_simple simple_test.cpp)
ament_target_dependencies(wheeled_simple rclcpp sensor_msgs std_msgs corgi_msgs)
target_link_libraries(wheeled_simple wheeled_control)

install(TARGETS
  wheeled_state_publisher
  wheeled_display
  wheeled_simple
  DESTINATION lib/${PROJECT_NAME}
)


