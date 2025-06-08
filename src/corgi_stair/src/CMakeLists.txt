add_executable(stair_main stair_main.cpp)
target_link_libraries(stair_main ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(stair_main_vision stair_main_vision.cpp)
target_link_libraries(stair_main_vision ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

# add_executable(normal_vector_plane_segmentation normal_vector_plane_segmentation.cpp)
# target_link_libraries(normal_vector_plane_segmentation ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

# add_executable(normal_vector_plane_segmentation2 normal_vector_plane_segmentation2.cpp)
# target_link_libraries(normal_vector_plane_segmentation2 ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

# add_executable(stair_edge_plane_segmentation stair_edge_plane_segmentation.cpp)
# target_link_libraries(stair_edge_plane_segmentation ${catkin_LIBRARIES})

# add_executable(ransac_plane_segmentation ransac_plane_segmentation.cpp)
# target_link_libraries(ransac_plane_segmentation ${catkin_LIBRARIES})

# add_executable(flood_plane_segmentation flood_plane_segmentation.cpp)
# target_link_libraries(flood_plane_segmentation ${catkin_LIBRARIES})

# add_executable(pointcloud_map pointcloud_map.cpp)
# target_link_libraries(pointcloud_map ${catkin_LIBRARIES})

add_executable(plane_main plane_main.cpp)
target_link_libraries(plane_main ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(stair_estimator stair_estimator.cpp)
target_link_libraries(stair_estimator ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(save_motor_cmd save_motor_cmd.cpp)
target_link_libraries(save_motor_cmd ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(play_motor_cmd play_motor_cmd.cpp)
target_link_libraries(play_motor_cmd ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)