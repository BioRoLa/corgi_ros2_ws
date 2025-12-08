cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

project(corgi_hybrid LANGUAGES C CXX)

set(CMAKE_EXE_LINKER_FLAGS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)

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

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  include
  ${corgi_utils_DIR}/../../../include
  ${corgi_gait_selector_DIR}/../../../include
)

# Link directories for corgi_utils library
if(corgi_utils_DIR)
  link_directories(${corgi_utils_DIR}/../../../lib)
endif()
if(corgi_gait_selector_DIR)
  link_directories(${corgi_gait_selector_DIR}/../../../lib)
endif()

add_subdirectory(src)

add_library(${PROJECT_NAME}
  src/hybrid_gen.cpp
  src/hybrid_swing.cpp
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
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  corgi_utils
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
  visualization_msgs
  corgi_msgs
  corgi_utils
  corgi_gait_selector
  eigen3_cmake_module
  Eigen3
)

ament_package()