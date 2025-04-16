add_executable(display display_node.cpp)
add_executable(publisher publisher.cpp)
add_executable(teleop teleop_twist_keyboard.cpp)
add_executable(wheeled_test wheeled_test.cpp wheeled_gen.cpp wheeled_cmd.cpp)

target_link_libraries(display  ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
target_link_libraries(publisher  ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
target_link_libraries(teleop  ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
target_link_libraries(wheeled_test  ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen )


