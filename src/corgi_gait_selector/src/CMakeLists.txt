add_executable(simple_fsm Simple_fsm.cpp)
target_link_libraries(simple_fsm ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${WHEEL_CONTROL} ${CORGI_TRANSFORM} Eigen3::Eigen)
