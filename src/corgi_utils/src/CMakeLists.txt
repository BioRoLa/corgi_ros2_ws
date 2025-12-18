add_executable(leg_model_main
  leg_model_main.cpp
)

target_link_libraries(leg_model_main
  ${PROJECT_NAME}
)

ament_target_dependencies(leg_model_main
  rclcpp
)

install(TARGETS leg_model_main
  DESTINATION lib/${PROJECT_NAME}
)
