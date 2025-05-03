add_executable(force_estimation force_estimation.cpp)
target_link_libraries(force_estimation ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} Eigen3::Eigen)

add_executable(kt_test kt_test.cpp)
target_link_libraries(kt_test ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} Eigen3::Eigen)

add_executable(exp_walk exp_walk.cpp)
target_link_libraries(exp_walk ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} Eigen3::Eigen)

add_executable(exp_wlw exp_wlw.cpp)
target_link_libraries(exp_wlw ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_GAIT_SELECTOR} ${CORGI_HYBRID} Eigen3::Eigen)


add_executable(exp_sim_leg exp_sim_leg.cpp)
target_link_libraries(exp_sim_leg ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} Eigen3::Eigen)