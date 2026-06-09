# Executables
add_executable(walk_main walk_main.cpp)
target_link_libraries(walk_main ${PROJECT_NAME})
ament_target_dependencies(walk_main
  rclcpp
  corgi_msgs
)

add_executable(trot_main trot_main.cpp)
target_link_libraries(trot_main ${PROJECT_NAME})
ament_target_dependencies(trot_main
  rclcpp
  corgi_msgs
)

add_executable(walk_exp walk_exp.cpp)
target_link_libraries(walk_exp ${PROJECT_NAME})
ament_target_dependencies(walk_exp
  rclcpp
  corgi_msgs
)

add_executable(walk_exp_h20_v10_sl20_sh8 walk_exp_h20_v10_sl20_sh8.cpp)
target_link_libraries(walk_exp_h20_v10_sl20_sh8 ${PROJECT_NAME})
ament_target_dependencies(walk_exp_h20_v10_sl20_sh8
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  corgi_msgs
)

add_executable(walk_exp_h20_v10_sl20_sh8_impedance walk_exp_h20_v10_sl20_sh8_impedance.cpp)
target_link_libraries(walk_exp_h20_v10_sl20_sh8_impedance ${PROJECT_NAME})
ament_target_dependencies(walk_exp_h20_v10_sl20_sh8_impedance
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  corgi_msgs
)

# Install executables
install(TARGETS
  walk_main
  trot_main
  walk_exp_h20_v10_sl20_sh8
  walk_exp_h20_v10_sl20_sh8_impedance
  DESTINATION lib/${PROJECT_NAME}
)
