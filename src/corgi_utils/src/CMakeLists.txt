add_executable(leg_model leg_model_main.cpp)
ament_target_dependencies(leg_model rclcpp)
target_link_libraries(leg_model ${PROJECT_NAME})

install(TARGETS leg_model
  DESTINATION lib/${PROJECT_NAME}
)