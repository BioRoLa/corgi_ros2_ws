add_executable(hybrid_test hybrid_test.cpp hybrid_gen.cpp hybrid_swing.cpp)
target_link_libraries(hybrid_test  ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen )


# Single_leg_traj.cpp

add_executable(single_leg_traj Single_leg_traj.cpp hybrid_swing.cpp)
target_link_libraries(single_leg_traj ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen )


# alpha_calculation.cpp
add_executable(alpha_calculation alpha_calculation.cpp)
target_link_libraries(alpha_calculation ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen )



# stability.cpp
add_executable(stability stability.cpp)
target_link_libraries(stability ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen )