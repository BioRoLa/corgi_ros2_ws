# add_executable(corgi_mpc corgi_mpc.cpp)
# target_link_libraries(corgi_mpc ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(walk_closed_loop walk_closed_loop.cpp mpc.cpp)
target_link_libraries(walk_closed_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(walk_open_loop walk_open_loop.cpp mpc.cpp)
target_link_libraries(walk_open_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)
