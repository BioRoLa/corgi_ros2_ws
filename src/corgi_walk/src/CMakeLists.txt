add_executable(walk_gait walk_gait.cpp)
target_link_libraries(walk_gait ${catkin_LIBRARIES} ${CORGI_UTILS})

add_executable(walk_class walk_class.cpp)
target_link_libraries(walk_class ${catkin_LIBRARIES} ${CORGI_UTILS})