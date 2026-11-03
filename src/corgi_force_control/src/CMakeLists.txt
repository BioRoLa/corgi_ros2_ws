add_executable(force_control_node force_control.cpp)
ament_target_dependencies(force_control_node
  rclcpp
  std_msgs
  corgi_msgs
  sensor_msgs
  Eigen3
)
target_link_libraries(force_control_node
  corgi_force_estimation::corgi_force_estimation
)

# Impedance command publishers for each experiment height
foreach(height h15 h18 h21 h24)
  add_executable(exp_${height}_node exp_${height}.cpp)
  ament_target_dependencies(exp_${height}_node
    rclcpp
    std_msgs
    corgi_msgs
    Eigen3
  )
  target_link_libraries(exp_${height}_node
    corgi_force_estimation::corgi_force_estimation
  )
endforeach()

# Impedance command publisher (exp_sim_stay) node
add_executable(exp_sim_stay_node exp_sim_stay.cpp)
ament_target_dependencies(exp_sim_stay_node
  rclcpp
  std_msgs
  corgi_msgs
  Eigen3
)
target_link_libraries(exp_sim_stay_node
  corgi_force_estimation::corgi_force_estimation
)

# Install
install(TARGETS
  force_control_node
  exp_h15_node
  exp_h18_node
  exp_h21_node
  exp_h24_node
  exp_sim_stay_node
  DESTINATION lib/${PROJECT_NAME}
)
