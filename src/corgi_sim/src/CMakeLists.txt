# Define the executable and link required libraries
add_executable(corgi_sim_pos ./corgi_sim_pos.cpp)
target_link_libraries(corgi_sim_pos ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} Eigen3::Eigen)

add_executable(corgi_sim_trq ./corgi_sim_trq.cpp)
target_link_libraries(corgi_sim_trq ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} Eigen3::Eigen)