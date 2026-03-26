cmake_minimum_required(VERSION 3.8)
project(corgi_attitude_control)

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
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# --- body_leveling library ---
add_library(body_leveling_lib
  src/body_leveling.cpp
)

target_include_directories(body_leveling_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(body_leveling_lib
  corgi_utils
)

target_link_libraries(body_leveling_lib
  Eigen3::Eigen
)

# --- attitude_node executable ---
add_executable(attitude_node
  src/attitude_node.cpp
)

target_include_directories(attitude_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(attitude_node
  rclcpp
  std_msgs
  corgi_msgs
  corgi_utils
)

target_link_libraries(attitude_node
  body_leveling_lib
)

# --- install ---
install(TARGETS body_leveling_lib
  EXPORT export_body_leveling_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS attitude_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# --- export library for downstream packages ---
ament_export_targets(export_body_leveling_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(corgi_utils Eigen3)

ament_package()
