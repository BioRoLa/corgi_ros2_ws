cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

project(corgi_vmc LANGUAGES C CXX)

set (CMAKE_EXE_LINKER_FLAGS)
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

if(NOT DEFINED LOCAL_PACKAGE_PATH)
  if(DEFINED ENV{LOCAL_PACKAGE_PATH})
    set(LOCAL_PACKAGE_PATH "$ENV{LOCAL_PACKAGE_PATH}")
  else()
    set(LOCAL_PACKAGE_PATH "$ENV{HOME}/corgi_ws/install")
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(corgi_utils REQUIRED)
find_package(corgi_walk REQUIRED)
find_package(corgi_force_estimation REQUIRED)
find_package(corgi_gait_selector REQUIRED)
find_package(corgi_hybrid REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(yaml-cpp REQUIRED)

set(OSQP_INCLUDE_DIRS "${LOCAL_PACKAGE_PATH}/include/osqp")
set(OSQP_EIGEN_INCLUDE_DIRS "${LOCAL_PACKAGE_PATH}/include/OsqpEigen")

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  ${OSQP_INCLUDE_DIRS}
  ${OSQP_EIGEN_INCLUDE_DIRS}
  include
)

find_library(CORGI_UTILS corgi_utils)
find_library(CORGI_WALK corgi_walk)
find_library(CORGI_FORCE_EST corgi_force_estimation)
find_library(CORGI_GAIT_SELECTOR corgi_gait_selector)
find_library(CORGI_HYBRID corgi_hybrid)

add_subdirectory(src)

ament_package()
