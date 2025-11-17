add_executable(corgi_data_recorder corgi_data_recorder.cpp)

ament_target_dependencies(corgi_data_recorder
  rclcpp
  std_msgs
  rosgraph_msgs
  geometry_msgs
  sensor_msgs
  corgi_msgs
)

install(TARGETS corgi_data_recorder
  DESTINATION lib/${PROJECT_NAME}
)
