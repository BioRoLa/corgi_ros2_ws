cmake_minimum_required(VERSION 3.8)
project(corgi_sim LANGUAGES C CXX)

# ----------------------------------------------------------------------
# 1. Find required packages
# ----------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(corgi_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rosidl_default_generators REQUIRED)

# ----------------------------------------------------------------------
# 2. Compiler / build settings
# ----------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

set(msg_files
  "msg/ContactPoint.msg"
  "msg/Float64Stamped.msg"  
)
set(srv_files
  "srv/NodeGetField.srv"
  "srv/NodeEnableContactPointsTracking.srv"
  "srv/GetUint64.srv"
  "srv/FieldGetString.srv"
  "srv/NodeGetContactPoints.srv"
  "srv/SetInt.srv"
  "srv/SupervisorGetFromId.srv"
  "srv/SetFloat.srv"
  "srv/NodeGetOrientation.srv"
  "srv/NodeGetPosition.srv"
  "srv/MotorSetControlPid.srv"
  "srv/NodeGetDef.srv"
)

# ----------------------------------------------------------------------
# 3. Generate messages / services
# ----------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES geometry_msgs std_msgs rosgraph_msgs
)

# ----------------------------------------------------------------------
# 4. Include directories
# ----------------------------------------------------------------------
include_directories(
  ${CMAKE_CURRENT_LIST_DIR}
  include
)

# ----------------------------------------------------------------------
# 5. Export dependencies
# ----------------------------------------------------------------------
ament_export_dependencies(
  rclcpp rclpy std_msgs rosgraph_msgs geometry_msgs corgi_msgs rosidl_default_runtime)

# ----------------------------------------------------------------------
# 6. Build executables
# ----------------------------------------------------------------------

# Both use topic-based control (ROS2 pattern)
add_executable(corgi_sim_pos src/corgi_sim_pos.cpp)
ament_target_dependencies(corgi_sim_pos
  rclcpp corgi_msgs geometry_msgs std_msgs rosgraph_msgs)
add_dependencies(corgi_sim_pos ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(corgi_sim_pos "${cpp_typesupport_target}" Eigen3::Eigen ${CMAKE_DL_LIBS})

add_executable(corgi_sim_trq src/corgi_sim_trq.cpp)
ament_target_dependencies(corgi_sim_trq
  rclcpp corgi_msgs geometry_msgs std_msgs rosgraph_msgs)
add_dependencies(corgi_sim_trq ${PROJECT_NAME}__rosidl_generator_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target_trq
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(corgi_sim_trq "${cpp_typesupport_target_trq}" Eigen3::Eigen ${CMAKE_DL_LIBS})

# ----------------------------------------------------------------------
# 7. Install
# ----------------------------------------------------------------------
install(DIRECTORY 
  launch
  worlds
  controllers
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  corgi_sim_pos
  corgi_sim_trq 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()