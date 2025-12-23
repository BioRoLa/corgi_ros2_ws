# Main control node
add_executable(wheel_control_node
  $<TARGET_OBJECTS:wheeled_control_obj>
)

ament_target_dependencies(wheel_control_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  corgi_msgs
)

target_link_libraries(wheel_control_node
  ${PROJECT_NAME}
)

install(TARGETS
  wheel_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Other nodes
add_executable(wheeled_state_publisher publisher.cpp)
ament_target_dependencies(wheeled_state_publisher
  rclcpp 
  sensor_msgs 
  std_msgs 
  corgi_msgs
)
target_link_libraries(wheeled_state_publisher ${PROJECT_NAME})

add_executable(wheeled_display display_node.cpp)
ament_target_dependencies(wheeled_display
  rclcpp 
  std_msgs corgi_msgs
)
target_link_libraries(wheeled_display ${PROJECT_NAME})

add_executable(wheeled_simple simple_test.cpp)
ament_target_dependencies(wheeled_simple
  rclcpp 
  sensor_msgs 
  std_msgs 
  corgi_msgs
)
target_link_libraries(wheeled_simple ${PROJECT_NAME})

install(TARGETS
  wheeled_state_publisher
  wheeled_display
  wheeled_simple
  DESTINATION lib/${PROJECT_NAME}
)
