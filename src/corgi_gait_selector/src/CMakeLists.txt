# add_executable(simple_fsm Simple_fsm.cpp)
# target_link_libraries(simple_fsm ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${WHEEL_CONTROL} ${CORGI_TRANSFORM} Eigen3::Eigen)

add_executable(main main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${WHEEL_CONTROL} ${CORGI_TRANSFORM} ${CORGI_FSM} Eigen3::Eigen)
