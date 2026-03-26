add_executable(transform_main transform_main.cpp)

# Link corgi_transform library
target_link_libraries(transform_main ${PROJECT_NAME})

# Dependencies
ament_target_dependencies(transform_main
  rclcpp
  corgi_msgs
  corgi_utils
)

# Install executable
install(TARGETS transform_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
