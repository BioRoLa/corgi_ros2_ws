# Define the executable and link required libraries (ROS2)
add_executable(wlw_trans ./wlw_trans.cpp)
target_link_libraries(wlw_trans corgi_utils Eigen3::Eigen)
ament_target_dependencies(wlw_trans rclcpp std_msgs geometry_msgs corgi_msgs)

add_executable(turn ./turn.cpp)
target_link_libraries(turn corgi_utils Eigen3::Eigen)
ament_target_dependencies(turn rclcpp std_msgs geometry_msgs corgi_msgs)

install(TARGETS
	wlw_trans
	turn
	DESTINATION lib/${PROJECT_NAME}
)
