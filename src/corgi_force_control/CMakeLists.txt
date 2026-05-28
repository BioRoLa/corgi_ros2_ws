cmake_minimum_required(VERSION 3.8)
project(corgi_force_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(corgi_force_estimation REQUIRED)
find_package(corgi_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${corgi_force_estimation_INCLUDE_DIRS}
  ${corgi_utils_INCLUDE_DIRS}
)

# Add subdirectory for executables
add_subdirectory(src)

# Install
install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
