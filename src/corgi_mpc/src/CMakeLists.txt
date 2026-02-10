set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  corgi_msgs
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

# Walk h20 v10
add_executable(walk_h20_v10_open walk_h20_v10_open.cpp mpc.cpp)
ament_target_dependencies(walk_h20_v10_open ${dependencies})
target_link_libraries(walk_h20_v10_open ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)

add_executable(walk_h20_v10_closed walk_h20_v10_closed.cpp mpc.cpp)
ament_target_dependencies(walk_h20_v10_closed ${dependencies})
target_link_libraries(walk_h20_v10_closed ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)

# Walk h20 v15
add_executable(walk_h20_v15_open walk_h20_v15_open.cpp mpc.cpp)
ament_target_dependencies(walk_h20_v15_open ${dependencies})
target_link_libraries(walk_h20_v15_open ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)

add_executable(walk_h20_v15_closed walk_h20_v15_closed.cpp mpc.cpp)
ament_target_dependencies(walk_h20_v15_closed ${dependencies})
target_link_libraries(walk_h20_v15_closed ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)

# Walk h25 v10
add_executable(walk_h25_v10_open walk_h25_v10_open.cpp mpc.cpp)
ament_target_dependencies(walk_h25_v10_open ${dependencies})
target_link_libraries(walk_h25_v10_open ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)

add_executable(walk_h25_v10_closed walk_h25_v10_closed.cpp mpc.cpp)
ament_target_dependencies(walk_h25_v10_closed ${dependencies})
target_link_libraries(walk_h25_v10_closed ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)

# Walk h25 v15
add_executable(walk_h25_v15_open walk_h25_v15_open.cpp mpc.cpp)
ament_target_dependencies(walk_h25_v15_open ${dependencies})
target_link_libraries(walk_h25_v15_open ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)

add_executable(walk_h25_v15_closed walk_h25_v15_closed.cpp mpc.cpp)
ament_target_dependencies(walk_h25_v15_closed ${dependencies})
target_link_libraries(walk_h25_v15_closed ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)

# Walk obstacle variants
add_executable(walk_obs_closed walk_obs_closed.cpp mpc.cpp)
ament_target_dependencies(walk_obs_closed ${dependencies})
target_link_libraries(walk_obs_closed ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)

add_executable(walk_obs_open walk_obs_open.cpp mpc.cpp)
ament_target_dependencies(walk_obs_open ${dependencies})
target_link_libraries(walk_obs_open ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)

# Install all executables
install(TARGETS
  walk_h20_v10_open
  walk_h20_v10_closed
  walk_h20_v15_open
  walk_h20_v15_closed
  walk_h25_v10_open
  walk_h25_v10_closed
  walk_h25_v15_open
  walk_h25_v15_closed
  walk_obs_closed
  walk_obs_open
  DESTINATION lib/${PROJECT_NAME}
)