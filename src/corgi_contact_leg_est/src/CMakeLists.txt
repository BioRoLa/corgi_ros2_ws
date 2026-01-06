# Build library for dynamics and observer
add_library(quadruped_dynamics
  quadruped_dynamics.cpp
  DisturbanceObserver.cpp
)

target_link_libraries(quadruped_dynamics
  Eigen3::Eigen
)

# Build executable
add_executable(offline_test 
  offline_test.cpp
)

ament_target_dependencies(offline_test 
  rclcpp 
  std_msgs 
  rosgraph_msgs 
  geometry_msgs 
  corgi_msgs
)

target_link_libraries(offline_test
  quadruped_dynamics
  Eigen3::Eigen
)

install(TARGETS offline_test
  DESTINATION lib/${PROJECT_NAME})