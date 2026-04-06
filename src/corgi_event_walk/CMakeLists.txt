cmake_minimum_required(VERSION 3.8)
project(corgi_event_walk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# --- dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(corgi_utils REQUIRED)

# --- library: event_walk_gait (ROS-free gait engine) ---
add_library(event_walk_gait SHARED
  src/event_walk_gait.cpp
)

target_include_directories(event_walk_gait
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(event_walk_gait
  corgi_utils
)

# --- executable: event_walk_node ---
add_executable(event_walk_node
  src/event_walk_node.cpp
  src/event_walk_main.cpp
)

target_include_directories(event_walk_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(event_walk_node
  rclcpp
  std_msgs
  corgi_msgs
  corgi_utils
)

target_link_libraries(event_walk_node
  event_walk_gait
)

# --- executable: event_walk_exp ---
add_executable(event_walk_exp
  src/event_walk_exp.cpp
)

target_include_directories(event_walk_exp
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(event_walk_exp
  rclcpp
  std_msgs
  corgi_msgs
  corgi_utils
)

target_link_libraries(event_walk_exp
  event_walk_gait
)

# --- install ---
install(TARGETS event_walk_gait
  EXPORT export_event_walk_gait
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS event_walk_node event_walk_exp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# --- export library for downstream packages (e.g. corgi_mpc) ---
ament_export_targets(export_event_walk_gait HAS_LIBRARY_TARGET)
ament_export_dependencies(corgi_utils)

ament_package()
