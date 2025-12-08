cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

project(corgi_transformation LANGUAGES C CXX)

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
find_package(corgi_hybrid REQUIRED)
find_package(corgi_wheeled REQUIRED)
find_package(corgi_legged REQUIRED)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  include
  ${corgi_utils_DIR}/../../../include
  ${corgi_gait_selector_DIR}/../../../include
  ${corgi_hybrid_DIR}/../../../include
  ${corgi_wheeled_DIR}/../../../include
  ${corgi_legged_DIR}/../../../include
)

# Link directories for dependency libraries
if(corgi_utils_DIR)
  link_directories(${corgi_utils_DIR}/../../../lib)
endif()
if(corgi_gait_selector_DIR)
  link_directories(${corgi_gait_selector_DIR}/../../../lib)
endif()
if(corgi_hybrid_DIR)
  link_directories(${corgi_hybrid_DIR}/../../../lib)
endif()
if(corgi_wheeled_DIR)
  link_directories(${corgi_wheeled_DIR}/../../../lib)
endif()
if(corgi_legged_DIR)
  link_directories(${corgi_legged_DIR}/../../../lib)
endif()

add_subdirectory(src)

add_library(${PROJECT_NAME}
  src/HybridToLegged.cpp
  src/WheeledToHybrid.cpp
  src/WheeledToLegged.cpp
  src/transform_gen.cpp
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
  corgi_utils
  corgi_gait_selector
  corgi_hybrid
  corgi_wheeled
  corgi_legged
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
  corgi_hybrid
  corgi_wheeled
  corgi_legged
  eigen3_cmake_module
  Eigen3
)

ament_package()