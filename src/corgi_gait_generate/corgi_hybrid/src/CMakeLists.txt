# Executables
add_executable(hybrid_test hybrid_test.cpp)
target_link_libraries(hybrid_test ${PROJECT_NAME})
ament_target_dependencies(hybrid_test
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
  visualization_msgs
  corgi_msgs
)

add_executable(single_leg_traj Single_leg_traj.cpp)
target_link_libraries(single_leg_traj ${PROJECT_NAME})
ament_target_dependencies(single_leg_traj
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
  visualization_msgs
  corgi_msgs
)

add_executable(alpha_calculation alpha_calculation.cpp)
target_link_libraries(alpha_calculation ${PROJECT_NAME})
ament_target_dependencies(alpha_calculation
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
  visualization_msgs
  corgi_msgs
)

add_executable(stability stability.cpp)
target_link_libraries(stability ${PROJECT_NAME})
ament_target_dependencies(stability
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
  visualization_msgs
  corgi_msgs
)

add_executable(four Four_leg.cpp)
target_link_libraries(four ${PROJECT_NAME})
ament_target_dependencies(four
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
  visualization_msgs
  corgi_msgs
)

# Install executables
install(TARGETS
  hybrid_test
  single_leg_traj
  alpha_calculation
  stability
  four
  DESTINATION lib/${PROJECT_NAME}
)