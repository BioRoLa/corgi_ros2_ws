# Define the executable and link required libraries
add_executable(wlw ./wlw.cpp)
target_link_libraries(wlw ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} Eigen3::Eigen )

add_executable(wlw_test ./test.cpp)
target_link_libraries(wlw_test ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS}  Eigen3::Eigen )

add_executable(wlw_test2 ./test2.cpp)
target_link_libraries(wlw_test2 ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS}  Eigen3::Eigen ${CORGI_WALK} )


# ${NLOPT_LIBRARIES}