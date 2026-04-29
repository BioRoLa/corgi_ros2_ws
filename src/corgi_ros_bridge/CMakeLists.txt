cmake_minimum_required(VERSION 3.8)

project(corgi_ros_bridge LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)

#set(CMAKE_EXE_LINKER_FLAGS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")


if(DEFINED ENV{LOCAL_PACKAGE_PATH})
  # Environment variable always takes priority (handles stale cmake cache)
  set(LOCAL_PACKAGE_PATH "$ENV{LOCAL_PACKAGE_PATH}" CACHE PATH "Path to grpc and core installation" FORCE)
elseif(NOT DEFINED LOCAL_PACKAGE_PATH OR "${LOCAL_PACKAGE_PATH}" STREQUAL "")
  # No cache variable and no env var: use default
  set(LOCAL_PACKAGE_PATH "$ENV{HOME}/corgi_ws/install" CACHE PATH "Path to grpc and core installation")
endif()
# If -DLOCAL_PACKAGE_PATH was passed explicitly and no env var: preserve the -D value
set(CORE_PREFIX_PATH ${LOCAL_PACKAGE_PATH})
set(CMAKE_PREFIX_PATH "/opt/ros/humble;${LOCAL_PACKAGE_PATH}")

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
)

ament_export_dependencies(rclcpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/common.cmake)

add_subdirectory(src)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()