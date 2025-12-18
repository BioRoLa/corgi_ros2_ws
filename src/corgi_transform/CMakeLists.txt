cmake_minimum_required(VERSION 3.8)
project(corgi_transform LANGUAGES C CXX)

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

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(corgi_msgs REQUIRED)
find_package(corgi_utils REQUIRED)

# Library: wheel <-> leg transforms
add_library(${PROJECT_NAME}
  src/wheel_to_leg.cpp
)

# Include directories (Modern CMake)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  corgi_msgs
  corgi_utils
  Eigen3
)

# Link Eigen (header-only)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

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

# Export for downstream packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp corgi_msgs corgi_utils Eigen3)
ament_export_libraries(${PROJECT_NAME})

# Add src subdirectory for executables
add_subdirectory(src)

ament_package()
