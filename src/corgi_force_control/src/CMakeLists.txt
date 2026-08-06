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

# Impedance command publisher (exp_sim_stay) node
add_executable(exp_sim_stay_node exp_sim_stay.cpp)
ament_target_dependencies(exp_sim_stay_node
  rclcpp
  corgi_msgs
  Eigen3
)
target_link_libraries(exp_sim_stay_node
  corgi_force_estimation::corgi_force_estimation
)

# G-SLIP pronk controller node
add_executable(gslip_pronk_node gslip_pronk.cpp)
ament_target_dependencies(gslip_pronk_node
  rclcpp
  corgi_msgs
  ament_index_cpp
  Eigen3
)

# Install
install(TARGETS
  force_control_node
  exp_h24_node
  exp_sim_stay_node
  gslip_pronk_node
  DESTINATION lib/${PROJECT_NAME}
)
