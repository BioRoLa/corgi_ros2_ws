add_executable(transform_main transform_main.cpp)
target_link_libraries(transform_main ${CMAKE_DL_LIBS} ${CORGI_UTILS} corgi_transform)
ament_target_dependencies(transform_main rclcpp corgi_msgs)
install(TARGETS transform_main
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)