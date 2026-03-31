# Main node
add_executable(wlw_node
  wlw.cpp
)

target_link_libraries(wlw_node
  ${PROJECT_NAME}
)

ament_target_dependencies(wlw_node
  rclcpp
  std_msgs
  geometry_msgs
  corgi_msgs
)

# Test node
add_executable(wlw_test
  test.cpp
)

target_link_libraries(wlw_test
  ${PROJECT_NAME}
)

ament_target_dependencies(wlw_test
  rclcpp
  std_msgs
  geometry_msgs
  corgi_msgs
)

# Install
install(TARGETS
  wlw_node
  wlw_test
  DESTINATION lib/${PROJECT_NAME}
)
