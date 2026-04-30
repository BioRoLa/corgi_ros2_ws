# wlw_trans node
add_executable(wlw_trans_node
  wlw_trans.cpp
)

target_link_libraries(wlw_trans_node
  ${PROJECT_NAME}
)

ament_target_dependencies(wlw_trans_node
  rclcpp
  std_msgs
  geometry_msgs
  corgi_msgs
)

# turn node
add_executable(turn
  turn.cpp
)

target_link_libraries(turn
  ${PROJECT_NAME}
)

ament_target_dependencies(turn
  rclcpp
  std_msgs
  geometry_msgs
  corgi_msgs
)

# Install
install(TARGETS
  wlw_trans_node
  turn
  DESTINATION lib/${PROJECT_NAME}
)
