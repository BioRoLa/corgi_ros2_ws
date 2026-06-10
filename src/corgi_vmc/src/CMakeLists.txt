set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  corgi_msgs
  tf2_msgs
  tf2_ros
  tf2
  corgi_utils
  corgi_walk
  corgi_force_estimation
  corgi_gait_selector
  corgi_hybrid
  Eigen3
  osqp
  OsqpEigen
  yaml-cpp
)

add_executable(walk_vmc walk_vmc.cpp vmc.cpp)
ament_target_dependencies(walk_vmc ${dependencies})
target_link_libraries(walk_vmc ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)
set_target_properties(walk_vmc PROPERTIES
  INSTALL_RPATH "/usr/local/lib"
)

install(TARGETS
  walk_vmc
  DESTINATION lib/${PROJECT_NAME}
)
