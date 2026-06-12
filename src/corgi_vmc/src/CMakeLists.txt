set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
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

function(add_vmc_walk_target target source_file)
  add_executable(${target} ${source_file} vmc.cpp)
  ament_target_dependencies(${target} ${dependencies})
  target_link_libraries(${target} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)
  set_target_properties(${target} PROPERTIES
    INSTALL_RPATH "/usr/local/lib"
  )
endfunction()

add_vmc_walk_target(walk_vmc_time walk_vmc_time.cpp)
add_vmc_walk_target(walk_vmc_dist walk_vmc_dist.cpp)

install(TARGETS
  walk_vmc_time
  walk_vmc_dist
  DESTINATION lib/${PROJECT_NAME}
)
