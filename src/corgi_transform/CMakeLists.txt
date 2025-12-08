cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

project(corgi_transform LANGUAGES C CXX)

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
find_package(corgi_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  include
  ${CMAKE_CURRENT_LIST_DIR}/../corgi_utils/include
)

find_library(CORGI_UTILS corgi_utils
  PATHS
    ${CMAKE_CURRENT_LIST_DIR}/../corgi_utils/build
    ${CMAKE_CURRENT_LIST_DIR}/../../build/corgi_utils
    ${CMAKE_CURRENT_LIST_DIR}/../../install/corgi_utils/lib
    ${CMAKE_CURRENT_LIST_DIR}/../../devel/lib
    ${LOCAL_PACKAGE_PATH}/lib
    ${CMAKE_INSTALL_PREFIX}/lib
  PATH_SUFFIXES "" lib
)

if(NOT CORGI_UTILS)
  message(FATAL_ERROR "corgi_utils library not found. Please build the corgi_utils package first: colcon build --packages-select corgi_utils")
endif()

add_subdirectory(src)

add_library(${PROJECT_NAME}
  src/wheel_to_leg.cpp
  src/leg_to_wheel.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${CORGI_UTILS}
  Eigen3::Eigen
)

ament_export_include_directories(include)
ament_export_dependencies(Eigen3)
ament_export_libraries(${PROJECT_NAME})
ament_package()