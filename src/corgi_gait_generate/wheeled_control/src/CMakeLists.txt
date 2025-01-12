add_executable(joystick_control joystick_control.cpp)
add_executable(publisher publisher.cpp)
add_executable(display display_node.cpp)

target_link_libraries(joystick_control  ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
target_link_libraries(publisher  ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
target_link_libraries(display  ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})
