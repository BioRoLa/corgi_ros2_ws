# Define the executable and link required libraries
add_executable(force_estimation ./force_estimation.cpp)
target_link_libraries(force_estimation ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} Eigen3::Eigen)

add_executable(motor_cmd_pub ./motor_cmd_pub.cpp)
target_link_libraries(motor_cmd_pub ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} Eigen3::Eigen)
