cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

project(corgi_stair LANGUAGES C CXX)

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
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(corgi_walk REQUIRED)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  include
)

add_library(${PROJECT_NAME}
  src/stair_climb.cpp
  src/leg_info.cpp
  src/plane_segmentation.cpp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  corgi_msgs
  corgi_walk
)

# Find corgi_walk library and include directories
find_library(CORGI_WALK corgi_walk PATHS ${corgi_walk_DIR}/../../../lib NO_DEFAULT_PATH)
set(CORGI_WALK_INCLUDE_DIR ${corgi_walk_DIR}/../../../include)

# Executables
add_executable(stair_main_indoor src/stair_main_indoor.cpp)
target_include_directories(stair_main_indoor PRIVATE ${CORGI_WALK_INCLUDE_DIR})
target_link_libraries(stair_main_indoor ${PROJECT_NAME} ${CORGI_WALK} Eigen3::Eigen)
ament_target_dependencies(stair_main_indoor rclcpp corgi_msgs geometry_msgs tf2_ros tf2_geometry_msgs corgi_walk corgi_utils)

add_executable(stair_main_indoor_open src/stair_main_indoor_open.cpp)
target_include_directories(stair_main_indoor_open PRIVATE ${CORGI_WALK_INCLUDE_DIR})
target_link_libraries(stair_main_indoor_open ${PROJECT_NAME} ${CORGI_WALK} Eigen3::Eigen)
ament_target_dependencies(stair_main_indoor_open rclcpp corgi_msgs geometry_msgs tf2_ros tf2_geometry_msgs corgi_walk corgi_utils)

add_executable(stair_main_outdoor src/stair_main_outdoor.cpp)
target_include_directories(stair_main_outdoor PRIVATE ${CORGI_WALK_INCLUDE_DIR})
target_link_libraries(stair_main_outdoor ${PROJECT_NAME} ${CORGI_WALK} Eigen3::Eigen)
ament_target_dependencies(stair_main_outdoor rclcpp corgi_msgs geometry_msgs tf2_ros tf2_geometry_msgs corgi_walk corgi_utils)

add_executable(stair_main_outdoor2 src/stair_main_outdoor2.cpp)
target_include_directories(stair_main_outdoor2 PRIVATE ${CORGI_WALK_INCLUDE_DIR})
target_link_libraries(stair_main_outdoor2 ${PROJECT_NAME} ${CORGI_WALK} Eigen3::Eigen)
ament_target_dependencies(stair_main_outdoor2 rclcpp corgi_msgs geometry_msgs tf2_ros tf2_geometry_msgs corgi_walk corgi_utils)

add_executable(stair_main_outdoor3 src/stair_main_outdoor3.cpp)
target_include_directories(stair_main_outdoor3 PRIVATE ${CORGI_WALK_INCLUDE_DIR})
target_link_libraries(stair_main_outdoor3 ${PROJECT_NAME} ${CORGI_WALK} Eigen3::Eigen)
ament_target_dependencies(stair_main_outdoor3 rclcpp corgi_msgs geometry_msgs tf2_ros tf2_geometry_msgs corgi_walk corgi_utils)

add_executable(stair_main_outdoor_open src/stair_main_outdoor_open.cpp)
target_include_directories(stair_main_outdoor_open PRIVATE ${CORGI_WALK_INCLUDE_DIR})
target_link_libraries(stair_main_outdoor_open ${PROJECT_NAME} ${CORGI_WALK} Eigen3::Eigen)
ament_target_dependencies(stair_main_outdoor_open rclcpp corgi_msgs geometry_msgs tf2_ros tf2_geometry_msgs corgi_walk corgi_utils)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  stair_main_indoor
  stair_main_indoor_open
  stair_main_outdoor
  stair_main_outdoor2
  stair_main_outdoor3
  stair_main_outdoor_open
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp std_msgs geometry_msgs sensor_msgs visualization_msgs tf2_ros tf2_eigen tf2_geometry_msgs pcl_conversions pcl_ros corgi_msgs corgi_walk corgi_utils)
ament_package()