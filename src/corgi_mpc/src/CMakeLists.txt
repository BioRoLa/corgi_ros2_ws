add_executable(corgi_mpc_test corgi_mpc_test.cpp)
target_link_libraries(corgi_mpc_test ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)
