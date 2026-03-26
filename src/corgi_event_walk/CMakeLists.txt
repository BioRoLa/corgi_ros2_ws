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

# --- install ---
install(TARGETS
  event_walk_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
