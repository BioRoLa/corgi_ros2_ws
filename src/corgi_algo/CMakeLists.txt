cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

project(corgi_algo LANGUAGES C CXX)

set (CMAKE_EXE_LINKER_FLAGS)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(corgi_utils REQUIRED)
find_package(corgi_gait_selector REQUIRED)
find_package(corgi_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  include
  ${corgi_utils_DIR}/../../../include
  ${corgi_gait_selector_DIR}/../../../include
)

if(corgi_utils_DIR)
  link_directories(${corgi_utils_DIR}/../../../lib)
endif()
if(corgi_gait_selector_DIR)
  link_directories(${corgi_gait_selector_DIR}/../../../lib)
endif()

add_executable(algo src/algo.cpp)
target_link_libraries(algo
  corgi_utils
  corgi_gait_selector
  Eigen3::Eigen
  ${CMAKE_DL_LIBS}
)

ament_target_dependencies(algo
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  corgi_msgs
  corgi_utils
  corgi_gait_selector
  Eigen3
)

install(TARGETS
  algo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(Eigen3 rclcpp std_msgs geometry_msgs sensor_msgs corgi_msgs corgi_utils corgi_gait_selector)
ament_package()
