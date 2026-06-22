add_executable(homing
  homing.cpp
)

ament_target_dependencies(homing
  rclcpp
  std_msgs
  geometry_msgs
  corgi_msgs
  corgi_utils
)

# Install
install(TARGETS
  homing
  DESTINATION lib/${PROJECT_NAME}
)
