add_executable(imu_node imu.cpp)
target_link_libraries(imu_node Eigen3::Eigen mip ${catkin_LIBRARIES})