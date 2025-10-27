find_library(GRPC_PROTO_LIB grpc_proto_lib)
if (NOT GRPC_PROTO_LIB)
  message(FATAL_ERROR "Not found GRPC_PROTO Library.")
endif()

add_executable(corgi_csv_control corgi_csv_control.cpp)

ament_target_dependencies(corgi_csv_control rclcpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs)

target_include_directories(corgi_csv_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(corgi_csv_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(corgi_csv_control
  ${CMAKE_DL_LIBS}
  ${GRPC_PROTO_LIB}
  ${_CORE_LIBRARIES}  # Optional: Only include if this variable is defined
)
  
target_link_libraries(corgi_csv_control
  ${CMAKE_DL_LIBS}
  ${GRPC_PROTO_LIB}
  ${_CORE_LIBRARIES}  # Optional: Only include if this variable is defined
)

install(TARGETS corgi_csv_control
  DESTINATION lib/${PROJECT_NAME})

#[[
# Define the executable and link required libraries
add_executable(corgi_csv_control ./corgi_csv_control.cpp)
target_link_libraries(corgi_csv_control ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
#]]

# Find required custom libraries