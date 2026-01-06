add_executable(set_zero
  set_zero.cpp
)

ament_target_dependencies(set_zero
  rclcpp
  std_msgs
  geometry_msgs
  corgi_msgs
  corgi_utils
)

# Install
install(TARGETS
  corgi_rt_control
  set_zero
  DESTINATION lib/${PROJECT_NAME}
)
