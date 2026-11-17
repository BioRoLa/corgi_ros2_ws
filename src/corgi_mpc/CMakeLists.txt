cmake_minimum_required(VERSION 3.8)
project(corgi_mpc LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# OSQP, OsqpEigen and yaml-cpp are installed outside the ROS workspace.
if(NOT DEFINED LOCAL_PACKAGE_PATH)
  if(DEFINED ENV{LOCAL_PACKAGE_PATH})
    set(LOCAL_PACKAGE_PATH "$ENV{LOCAL_PACKAGE_PATH}")
  else()
    set(LOCAL_PACKAGE_PATH "$ENV{HOME}/corgi_ws/install")
  endif()
endif()
list(APPEND CMAKE_PREFIX_PATH "${LOCAL_PACKAGE_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(corgi_utils REQUIRED)
find_package(corgi_walk REQUIRED)
find_package(corgi_force_estimation REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${corgi_force_estimation_INCLUDE_DIRS}
)

# OsqpEigen passes Eigen types across its library boundary, so this package
# must compile against the same Eigen that OsqpEigen was built with. That is
# the one find_package(Eigen3) resolves here; put it ahead of any other Eigen
# that the ROS dependencies add to the include path (e.g. /usr/include/eigen3).
get_target_property(CORGI_MPC_EIGEN_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
include_directories(BEFORE SYSTEM ${CORGI_MPC_EIGEN_INCLUDE_DIRS})

add_subdirectory(src)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
