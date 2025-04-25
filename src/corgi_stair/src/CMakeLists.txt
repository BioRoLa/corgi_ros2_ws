add_executable(stair_main stair_main.cpp)
target_link_libraries(stair_main ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(normal_vector_plane_segmentation normal_vector_plane_segmentation.cpp)
target_link_libraries(normal_vector_plane_segmentation ${catkin_LIBRARIES})

add_executable(ransac_plane_segmentation ransac_plane_segmentation.cpp)
target_link_libraries(ransac_plane_segmentation ${catkin_LIBRARIES})

add_executable(flood_plane_segmentation flood_plane_segmentation.cpp)
target_link_libraries(flood_plane_segmentation ${catkin_LIBRARIES})

add_executable(pointcloud_map pointcloud_map.cpp)
target_link_libraries(pointcloud_map ${catkin_LIBRARIES})