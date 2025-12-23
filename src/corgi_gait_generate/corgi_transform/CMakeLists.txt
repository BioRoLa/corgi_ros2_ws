cmake_minimum_required(VERSION 3.8)
project(corgi_transformation LANGUAGES C CXX)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(corgi_msgs REQUIRED)
find_package(corgi_utils REQUIRED)
find_package(corgi_gait_selector REQUIRED)
find_package(corgi_hybrid REQUIRED)
find_package(corgi_wheeled REQUIRED)
find_package(corgi_legged REQUIRED)

# Library
add_library(${PROJECT_NAME}
  src/HybridToLegged.cpp
  src/WheeledToHybrid.cpp
  src/WheeledToLegged.cpp
  src/transform_gen.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
  visualization_msgs
  corgi_msgs
  corgi_utils
  corgi_gait_selector
  corgi_hybrid
  corgi_wheeled
  corgi_legged
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

# Nodes
add_subdirectory(src)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
  visualization_msgs
  corgi_msgs
  corgi_utils
  corgi_gait_selector
  corgi_hybrid
  corgi_wheeled
  corgi_legged
)

ament_package()
