cmake_minimum_required(VERSION 3.8)

project(wlw_gen LANGUAGES C CXX)

set(CMAKE_EXE_LINKER_FLAGS)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)

set(corgi_msgs_DIR "${CMAKE_CURRENT_LIST_DIR}/../../devel/share/corgi_msgs/cmake")
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(NLopt REQUIRED)
find_package(catkin REQUIRED COMPONENTS corgi_walk roscpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs corgi_utils)
catkin_package(CATKIN_DEPENDS roscpp std_msgs rosgraph_msgs corgi_walk geometry_msgs corgi_msgs corgi_utils)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}
 
  include
)
# ${NLopt_INCLUDE_DIRS}
find_library(CORGI_UTILS corgi_utils)
find_library(CORGI_WALK corgi_walk)
add_subdirectory(src)