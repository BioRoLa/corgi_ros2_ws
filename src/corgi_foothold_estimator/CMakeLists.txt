cmake_minimum_required(VERSION 3.8)
project(corgi_foothold_estimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ─── Dependencies ────────────────────────────────────────────────────────────
find_package(ament_cmake  REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(corgi_msgs   REQUIRED)
find_package(std_msgs     REQUIRED)

# ─── Pure C++ Library ────────────────────────────────────────────────────────
add_library(foothold_detector_lib src/foothold_detector.cpp)

target_include_directories(foothold_detector_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ─── ROS 2 Node Executable ───────────────────────────────────────────────────
add_executable(detection_node src/detection_node.cpp)

target_include_directories(detection_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(detection_node
  rclcpp
  corgi_msgs
  std_msgs
)

target_link_libraries(detection_node foothold_detector_lib)

# ─── Install Rules ───────────────────────────────────────────────────────────
install(TARGETS foothold_detector_lib
  EXPORT  export_foothold_detector_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ROS 2 executables must be installed to lib/<package_name>/
install(TARGETS detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ─── Ament Export ────────────────────────────────────────────────────────────
ament_export_targets(export_foothold_detector_lib HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
