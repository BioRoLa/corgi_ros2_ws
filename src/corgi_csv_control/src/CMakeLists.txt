add_executable(corgi_csv_control corgi_csv_control.cpp)

ament_target_dependencies(corgi_csv_control rclcpp std_msgs rosgraph_msgs geometry_msgs corgi_msgs)

install(TARGETS corgi_csv_control
  DESTINATION lib/${PROJECT_NAME})