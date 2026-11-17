# MPC core, without ROS, so it can be linked into tests and other nodes.
add_library(corgi_mpc_core STATIC mpc.cpp)
target_include_directories(corgi_mpc_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include)
ament_target_dependencies(corgi_mpc_core corgi_utils Eigen3)
target_link_libraries(corgi_mpc_core osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)

add_executable(mpc_walk mpc_walk.cpp)
ament_target_dependencies(mpc_walk
  rclcpp
  std_msgs
  geometry_msgs
  corgi_msgs
  corgi_utils
  corgi_walk
)
target_link_libraries(mpc_walk corgi_mpc_core)

install(TARGETS
  mpc_walk
  DESTINATION lib/${PROJECT_NAME}
)
