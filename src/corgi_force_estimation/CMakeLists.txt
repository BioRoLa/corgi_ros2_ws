cmake_minimum_required(VERSION 3.8)
project(corgi_force_estimation)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(corgi_utils REQUIRED)
find_package(corgi_walk REQUIRED)
find_package(corgi_gait_selector REQUIRED)
find_package(corgi_hybrid REQUIRED)

# Library
add_library(${PROJECT_NAME}
  src/force_estimation.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  rcl_interfaces
  geometry_msgs
  sensor_msgs
  corgi_msgs
  corgi_utils
  corgi_walk
  corgi_gait_selector
  corgi_hybrid
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  rclcpp
  std_msgs
  rcl_interfaces
  geometry_msgs
  sensor_msgs
  corgi_msgs
  corgi_utils
  corgi_walk
  corgi_gait_selector
  corgi_hybrid
)

# Subdirectory (executables)
add_subdirectory(src)

ament_package()