set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  corgi_msgs
  tf2_msgs
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

# ---------------------------------------------------------------------------
# Internal static library: compile mpc.cpp exactly once, shared by all targets
# ---------------------------------------------------------------------------
add_library(corgi_mpc_lib STATIC mpc.cpp)
ament_target_dependencies(corgi_mpc_lib ${dependencies})
target_link_libraries(corgi_mpc_lib
  ${CMAKE_DL_LIBS}
  ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR}
  osqp::osqp OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp
)

# Precompiled headers on the library — compiled once and reused by every executable
target_precompile_headers(corgi_mpc_lib PRIVATE
  <OsqpEigen/OsqpEigen.h>
  <Eigen/Dense>
  <Eigen/Geometry>
  <unsupported/Eigen/MatrixFunctions>
  <yaml-cpp/yaml.h>
  <rclcpp/rclcpp.hpp>
  <vector>
  <array>
  <cmath>
)

# Helper macro to avoid repeating the same 3 lines for every executable
macro(add_walk_executable target_name source_file)
  add_executable(${target_name} ${source_file})
  ament_target_dependencies(${target_name} ${dependencies})
  target_link_libraries(${target_name} corgi_mpc_lib)
  target_precompile_headers(${target_name} REUSE_FROM corgi_mpc_lib)
endmacro()

# Walk h20 v10
add_walk_executable(walk_h20_v10_open  walk_h20_v10_open.cpp)

# WLW open-loop (Hybrid gait, params from config/config.yaml common.wlw_open)
add_walk_executable(wlw_open           wlw_open.cpp)

# Closed-loop walk controllers (time-driven and distance-driven)
add_walk_executable(walk_closed_time   walk_closed_time.cpp)
add_walk_executable(walk_closed_dist   walk_closed_dist.cpp)

# Install all executables
install(TARGETS
  walk_h20_v10_open
  wlw_open
  walk_closed_time
  walk_closed_dist
  DESTINATION lib/${PROJECT_NAME}
)