cmake_minimum_required(VERSION 3.8)
project(corgi_virtual_agent LANGUAGES C CXX)

# ----------------------------------------------------------------------
# 1. Find required packages
# ----------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ----------------------------------------------------------------------
# 2. Compiler / build settings
# ----------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

# ----------------------------------------------------------------------
# 3. gRPC and Core library setup
# ----------------------------------------------------------------------
set(LOCAL_PACKAGE_PATH "$ENV{HOME}/.grpc_local" CACHE STRING "Path to grpc and core installation")
set(CORE_PREFIX_PATH ${LOCAL_PACKAGE_PATH})

# Update CMAKE_PREFIX_PATH to include gRPC installation
list(APPEND CMAKE_PREFIX_PATH ${LOCAL_PACKAGE_PATH})

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/common.cmake)

# ----------------------------------------------------------------------
# 4. Build subdirectories
# ----------------------------------------------------------------------
add_subdirectory(src)

# ----------------------------------------------------------------------
# 5. Install and export
# ----------------------------------------------------------------------
ament_package()
