add_executable(leg_odometry leg_odometry.cpp mpc.cpp)
target_link_libraries(leg_odometry ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(walk_closed_loop walk_closed_loop.cpp mpc.cpp)
target_link_libraries(walk_closed_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(walk_open_loop walk_open_loop.cpp mpc.cpp)
target_link_libraries(walk_open_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(wlw_open_loop wlw_open_loop.cpp mpc.cpp)
target_link_libraries(wlw_open_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(wlw_closed_loop wlw_closed_loop.cpp mpc.cpp)
target_link_libraries(wlw_closed_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(trot_open_loop trot_open_loop.cpp mpc.cpp)
target_link_libraries(trot_open_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(trot_closed_loop trot_closed_loop.cpp mpc.cpp)
target_link_libraries(trot_closed_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)