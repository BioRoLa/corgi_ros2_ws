cmake_minimum_required(VERSION 3.8)
project(corgi_csv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(LOCAL_PACKAGE_PATH ${CMAKE_PREFIX_PATH} CACHE STRING "Path to grpc and core installation")
set(CORE_PREFIX_PATH ${LOCAL_PACKAGE_PATH})
set(CMAKE_PREFIX_PATH "/opt/ros/humble;${LOCAL_PACKAGE_PATH}")

include(${CMAKE_CURRENT_LIST_DIR}/cmake/common.cmake)

add_subdirectory(src)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

#[[
cmake_minimum_required(VERSION 3.8)

project(corgi_csv_control LANGUAGES C CXX)

set(CMAKE_EXE_LINKER_FLAGS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)

set(corgi_msgs_DIR "${CMAKE_CURRENT_LIST_DIR}/../../devel/share/corgi_msgs/cmake")
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs)
catkin_package(CATKIN_DEPENDS roscpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}
)

add_subdirectory(src)
#]]