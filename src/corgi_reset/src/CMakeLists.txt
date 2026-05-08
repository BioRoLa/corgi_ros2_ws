add_executable(reset
  reset.cpp
)

ament_target_dependencies(reset
  rclcpp
  std_msgs
  geometry_msgs
  corgi_msgs
  corgi_utils
)

# Install
install(TARGETS
  reset
  DESTINATION lib/${PROJECT_NAME}
)
