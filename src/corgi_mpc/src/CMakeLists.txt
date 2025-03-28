add_executable(corgi_mpc corgi_mpc.cpp)
target_link_libraries(corgi_mpc ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)
