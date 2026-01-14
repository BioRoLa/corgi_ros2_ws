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

# Build executable
add_executable(corgi_contact_leg_est 
  corgi_contact_leg_est.cpp
)

ament_target_dependencies(corgi_contact_leg_est 
  rclcpp 
  std_msgs 
  rosgraph_msgs 
  geometry_msgs 
  corgi_msgs
)

target_link_libraries(corgi_contact_leg_est
  quadruped_dynamics
  Eigen3::Eigen
)

install(TARGETS corgi_contact_leg_est
  DESTINATION lib/${PROJECT_NAME})

# Build executable for velocity estimator
add_executable(velocity_estimator 
  velocity_estimator.cpp
)

ament_target_dependencies(velocity_estimator 
  rclcpp 
  std_msgs 
  geometry_msgs 
  corgi_msgs
  sensor_msgs
)

target_link_libraries(velocity_estimator
  Eigen3::Eigen
)

install(TARGETS velocity_estimator
  DESTINATION lib/${PROJECT_NAME})