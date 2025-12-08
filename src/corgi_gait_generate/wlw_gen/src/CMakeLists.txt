# Define the executables with ROS2 dependencies
add_executable(wlw ./wlw.cpp)
target_link_libraries(wlw corgi_utils Eigen3::Eigen)
ament_target_dependencies(wlw rclcpp std_msgs geometry_msgs corgi_msgs)

add_executable(wlw_test ./test.cpp)
target_link_libraries(wlw_test corgi_utils Eigen3::Eigen)
ament_target_dependencies(wlw_test rclcpp std_msgs geometry_msgs corgi_msgs)

install(TARGETS
	wlw
	wlw_test
	DESTINATION lib/${PROJECT_NAME}
)