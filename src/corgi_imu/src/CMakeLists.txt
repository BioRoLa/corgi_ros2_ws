add_executable(imu_node imu.cpp)
target_link_libraries(imu_node Eigen3::Eigen mip ${catkin_LIBRARIES})

add_executable(imu_node_listener imu_listener.cpp)
target_link_libraries(imu_node_listener Eigen3::Eigen mip ${catkin_LIBRARIES})