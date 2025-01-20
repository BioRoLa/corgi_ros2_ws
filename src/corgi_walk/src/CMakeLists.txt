add_executable(walk_gait walk_gait_main.cpp)
target_link_libraries(walk_gait ${catkin_LIBRARIES} ${CORGI_UTILS} corgi_walk)

add_executable(walk_test walk_test.cpp)
target_link_libraries(walk_test ${catkin_LIBRARIES} ${CORGI_UTILS} corgi_walk)