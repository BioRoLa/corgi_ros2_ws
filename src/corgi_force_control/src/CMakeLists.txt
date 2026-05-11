# Admittance control node
add_executable(admittance_control_node admittance_control.cpp)
ament_target_dependencies(admittance_control_node
  rclcpp
  corgi_msgs
  Eigen3
)
target_link_libraries(admittance_control_node
  corgi_force_estimation::corgi_force_estimation
)

# Force control node
add_executable(force_control_node force_control.cpp)
ament_target_dependencies(force_control_node
  rclcpp
  corgi_msgs
  sensor_msgs
  Eigen3
)
target_link_libraries(force_control_node
  corgi_force_estimation::corgi_force_estimation
)

# Impedance command publisher (exp_h24) node
add_executable(exp_h24_node exp_h24.cpp)
ament_target_dependencies(exp_h24_node
  rclcpp
  corgi_msgs
  Eigen3
)
target_link_libraries(exp_h24_node
  corgi_force_estimation::corgi_force_estimation
)

add_executable(exp_height_node exp_height.cpp)
ament_target_dependencies(exp_height_node
  rclcpp
  corgi_msgs
  Eigen3
)
target_link_libraries(exp_height_node
  corgi_force_estimation::corgi_force_estimation
)

# Admittance control experiment node
add_executable(exp_admittance_node exp_admittance.cpp)
ament_target_dependencies(exp_admittance_node
  rclcpp
  corgi_msgs
  Eigen3
)
target_link_libraries(exp_admittance_node
  corgi_force_estimation::corgi_force_estimation
)

# Install
install(TARGETS
  admittance_control_node
  force_control_node
  exp_h24_node
  exp_height_node
  exp_admittance_node
  DESTINATION lib/${PROJECT_NAME}
)
