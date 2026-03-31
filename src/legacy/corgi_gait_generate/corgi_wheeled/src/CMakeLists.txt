# Display node
add_executable(display_node display_node.cpp)
target_link_libraries(display_node ${PROJECT_NAME})
ament_target_dependencies(display_node
  rclcpp
  std_msgs
  corgi_msgs
)

# Publisher node
add_executable(publisher_node publisher.cpp)
target_link_libraries(publisher_node ${PROJECT_NAME})
ament_target_dependencies(publisher_node
  rclcpp
  std_msgs
  sensor_msgs
  corgi_msgs
)

# Teleop node
add_executable(teleop_node teleop_twist_keyboard.cpp)
target_link_libraries(teleop_node ${PROJECT_NAME})
ament_target_dependencies(teleop_node
  rclcpp
  std_msgs
)

# Test node
add_executable(wheeled_test wheeled_test.cpp)
target_link_libraries(wheeled_test ${PROJECT_NAME})
ament_target_dependencies(wheeled_test
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
  corgi_msgs
)

# Install nodes
install(TARGETS
  display_node
  publisher_node
  teleop_node
  wheeled_test
  DESTINATION lib/${PROJECT_NAME}
)
