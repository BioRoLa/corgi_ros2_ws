add_executable(Camera Receive_feedback.cpp)
target_link_libraries(Camera Eigen3::Eigen ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${X11_LIBRARIES} )

add_executable(Stair stair.cpp)
target_link_libraries(Stair Eigen3::Eigen ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
