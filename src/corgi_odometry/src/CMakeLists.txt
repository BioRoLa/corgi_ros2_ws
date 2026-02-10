# Build library for dynamics and observer
add_library(simplify_dynamics
  general_momentum_observer/simplify_dynamics.cpp
  general_momentum_observer/DisturbanceObserver.cpp
  general_momentum_observer/DataProcessor.cpp
)

ament_target_dependencies(simplify_dynamics
  rclcpp
  geometry_msgs
  corgi_msgs
)

target_link_libraries(simplify_dynamics
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
  simplify_dynamics
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
  simplify_dynamics
  Eigen3::Eigen
)

install(TARGETS corgi_contact_leg_est
  DESTINATION lib/${PROJECT_NAME})

# ES-EKF estimation library (leg kinematics + filter)
add_library(esekf_estimation
  es_ekf/ESEKF.cpp
  kinematic/Leg.cpp
  kinematic/LinkLegModel.cpp
)

target_link_libraries(esekf_estimation
  Eigen3::Eigen
)

# Build executable
add_executable(corgi_leg_odom 
  corgi_leg_odom.cpp
)

ament_target_dependencies(corgi_leg_odom 
  rclcpp 
  std_msgs 
  rosgraph_msgs 
  geometry_msgs 
  corgi_msgs
)

target_link_libraries(corgi_leg_odom
  simplify_dynamics
  esekf_estimation
  Eigen3::Eigen
)

install(TARGETS corgi_leg_odom
  DESTINATION lib/${PROJECT_NAME})

# Build executable for velocity estimator
add_executable(velocity_estimator 
  sim/velocity_estimator.cpp
)

ament_target_dependencies(velocity_estimator 
  rclcpp 
  std_msgs 
  geometry_msgs 
  corgi_msgs
  sensor_msgs
  tf2_ros
)

target_link_libraries(velocity_estimator
  Eigen3::Eigen
)

install(TARGETS velocity_estimator
  DESTINATION lib/${PROJECT_NAME})