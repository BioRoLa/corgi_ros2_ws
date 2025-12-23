# Main executables
add_executable(force_estimation force_estimation.cpp)
target_link_libraries(force_estimation ${PROJECT_NAME})
ament_target_dependencies(force_estimation
  rclcpp
  std_msgs
  rcl_interfaces
  geometry_msgs
  sensor_msgs
  corgi_msgs
  corgi_utils
)

add_executable(kt_test kt_test.cpp)
target_link_libraries(kt_test ${PROJECT_NAME})
ament_target_dependencies(kt_test
  rclcpp
  std_msgs
  rcl_interfaces
  geometry_msgs
  sensor_msgs
  corgi_msgs
  corgi_utils
)

add_executable(exp_walk exp_walk.cpp)
target_link_libraries(exp_walk ${PROJECT_NAME})
ament_target_dependencies(exp_walk
  rclcpp
  std_msgs
  rcl_interfaces
  geometry_msgs
  sensor_msgs
  corgi_msgs
  corgi_utils
  corgi_walk
)

add_executable(exp_trot exp_trot.cpp)
target_link_libraries(exp_trot ${PROJECT_NAME})
ament_target_dependencies(exp_trot
  rclcpp
  std_msgs
  rcl_interfaces
  geometry_msgs
  sensor_msgs
  corgi_msgs
  corgi_utils
  corgi_walk
)

add_executable(exp_sim_leg exp_sim_leg.cpp)
target_link_libraries(exp_sim_leg ${PROJECT_NAME})
ament_target_dependencies(exp_sim_leg
  rclcpp
  std_msgs
  rcl_interfaces
  geometry_msgs
  sensor_msgs
  corgi_msgs
  corgi_utils
)

add_executable(exp_wheel exp_wheel.cpp)
target_link_libraries(exp_wheel ${PROJECT_NAME})
ament_target_dependencies(exp_wheel
  rclcpp
  std_msgs
  rcl_interfaces
  geometry_msgs
  sensor_msgs
  corgi_msgs
  corgi_utils
)

# Install executables
install(TARGETS
  force_estimation
  kt_test
  exp_walk
  exp_trot
  exp_sim_leg
  exp_wheel
  DESTINATION lib/${PROJECT_NAME}
)