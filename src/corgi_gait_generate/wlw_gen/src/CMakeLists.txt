# Define the executable and link required libraries
add_executable(wlw ./wlw.cpp)
target_link_libraries(wlw ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${NLOPT_LIBRARIES} Eigen3::Eigen )
