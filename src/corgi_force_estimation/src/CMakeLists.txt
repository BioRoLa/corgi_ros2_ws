# Force estimation node executable
add_executable(force_estimation_node
  force_estimation.cpp
)

ament_target_dependencies(force_estimation_node
  rclcpp
  Eigen3
  corgi_utils
  corgi_msgs
)

# Wheel experiment node executable
add_executable(exp_wheel_node
  exp_wheel.cpp
)

target_link_libraries(exp_wheel_node
  ${PROJECT_NAME}
)

ament_target_dependencies(exp_wheel_node
  rclcpp
  Eigen3
  corgi_utils
  corgi_msgs
)

# Walk experiment node executable
add_executable(exp_walk_node
  exp_walk.cpp
)

target_link_libraries(exp_walk_node
  ${PROJECT_NAME}
  ${CORGI_WALK}
)

ament_target_dependencies(exp_walk_node
  rclcpp
  Eigen3
  corgi_utils
  corgi_msgs
  corgi_walk
)

# Install executables
install(TARGETS
  force_estimation_node
  exp_wheel_node
  exp_walk_node
  DESTINATION lib/${PROJECT_NAME}
)
