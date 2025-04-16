# Define the executable and link required libraries
add_executable(wlw_trans ./wlw_trans.cpp)
target_link_libraries(wlw_trans ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${NLOPT_LIBRARIES} Eigen3::Eigen )
add_executable(turn ./turn.cpp)
target_link_libraries(turn ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${NLOPT_LIBRARIES} Eigen3::Eigen )
