add_executable(algo algo.cpp)
target_link_libraries(algo ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WHEELED} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen)
