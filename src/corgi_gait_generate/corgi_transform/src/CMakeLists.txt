add_executable(transform_test transform_test.cpp)

target_link_libraries(transform_test
  ${PROJECT_NAME}
)

ament_target_dependencies(transform_test
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  joy
  visualization_msgs
  corgi_msgs
)

install(TARGETS
  transform_test
  DESTINATION lib/${PROJECT_NAME}
)
