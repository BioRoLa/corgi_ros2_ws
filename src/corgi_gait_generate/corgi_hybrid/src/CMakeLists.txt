add_executable(hybrid_test hybrid_test.cpp hybrid_gen.cpp)
target_link_libraries(hybrid_test  ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen )


