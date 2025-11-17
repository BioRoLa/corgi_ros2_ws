cmake_minimum_required(VERSION 3.8)
project(corgi_walk LANGUAGES C CXX)

# Compiler settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(corgi_utils REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Find corgi_utils library
find_library(CORGI_UTILS corgi_utils PATHS ${corgi_utils_DIR}/../../../lib NO_DEFAULT_PATH)

# Library: walk and trot gait algorithms
add_library(${PROJECT_NAME}
  src/walk_gait.cpp
  src/trot_gait.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${CORGI_UTILS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  corgi_msgs
  corgi_utils
)

# Executables
add_executable(walk_main src/walk_main.cpp)
target_link_libraries(walk_main ${PROJECT_NAME} ${CORGI_UTILS})
ament_target_dependencies(walk_main rclcpp corgi_msgs)

add_executable(trot_main src/trot_main.cpp)
target_link_libraries(trot_main ${PROJECT_NAME} ${CORGI_UTILS})
ament_target_dependencies(trot_main rclcpp corgi_msgs)

add_executable(walk_exp src/walk_exp.cpp)
target_link_libraries(walk_exp ${PROJECT_NAME} ${CORGI_UTILS})
ament_target_dependencies(walk_exp rclcpp corgi_msgs)

# Installation
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS walk_main trot_main walk_exp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Export information
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs geometry_msgs corgi_msgs corgi_utils)

ament_package()