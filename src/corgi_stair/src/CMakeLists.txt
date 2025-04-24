add_executable(stair_main stair_main.cpp)
target_link_libraries(stair_main ${catkin_LIBRARIES} ${CORGI_UTILS} ${CORGI_WALK} corgi_stair)

add_executable(plane_segmentation_node plane_segmentation_node.cpp)
target_link_libraries(plane_segmentation_node ${catkin_LIBRARIES})