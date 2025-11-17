# Find required custom libraries
find_library(GRPC_PROTO_LIB grpc_proto_lib)
if (NOT GRPC_PROTO_LIB)
    message(FATAL_ERROR "Not found GRPC_PROTO Library.")
endif()

# Define the executable
add_executable(corgi_virtual_agent ./corgi_virtual_agent.cpp)

# Add ROS2 dependencies
ament_target_dependencies(corgi_virtual_agent
  rclcpp
  std_msgs
  rosgraph_msgs
  geometry_msgs
)

# Link required libraries
target_link_libraries(corgi_virtual_agent ${CMAKE_DL_LIBS} ${_CORE_LIBRARIES} ${GRPC_PROTO_LIB})

# Install the executable
install(TARGETS
  corgi_virtual_agent
  DESTINATION lib/${PROJECT_NAME}
)
