add_executable(walk_closed_loop_ walk_closed_loop_.cpp mpc_.cpp)
target_link_libraries(walk_closed_loop_ ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

# add_executable(walk_closed_loop walk_closed_loop.cpp mpc.cpp)
# target_link_libraries(walk_closed_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

# add_executable(walk_open_loop walk_open_loop.cpp mpc.cpp)
# target_link_libraries(walk_open_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

# add_executable(wlw_open_loop wlw_open_loop.cpp mpc.cpp)
# target_link_libraries(wlw_open_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

# add_executable(wlw_closed_loop wlw_closed_loop.cpp mpc.cpp)
# target_link_libraries(wlw_closed_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)