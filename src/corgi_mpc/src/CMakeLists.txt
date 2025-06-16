add_executable(leg_odometry leg_odometry.cpp mpc.cpp)
target_link_libraries(leg_odometry ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

# add_executable(walk_closed_loop walk_closed_loop.cpp mpc.cpp)
# target_link_libraries(walk_closed_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

# add_executable(walk_open_loop walk_open_loop.cpp mpc.cpp)
# target_link_libraries(walk_open_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

# add_executable(wlw_open_loop wlw_open_loop.cpp mpc.cpp)
# target_link_libraries(wlw_open_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

# add_executable(wlw_closed_loop wlw_closed_loop.cpp mpc.cpp)
# target_link_libraries(wlw_closed_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

# add_executable(trot_open_loop trot_open_loop.cpp mpc.cpp)
# target_link_libraries(trot_open_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

# add_executable(trot_closed_loop trot_closed_loop.cpp mpc.cpp)
# target_link_libraries(trot_closed_loop ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(walk_h20_v10_open walk_h20_v10_open.cpp mpc.cpp)
target_link_libraries(walk_h20_v10_open ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(walk_h20_v10_closed walk_h20_v10_closed.cpp mpc.cpp)
target_link_libraries(walk_h20_v10_closed ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(walk_h20_v15_open walk_h20_v15_open.cpp mpc.cpp)
target_link_libraries(walk_h20_v15_open ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(walk_h20_v15_closed walk_h20_v15_closed.cpp mpc.cpp)
target_link_libraries(walk_h20_v15_closed ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(walk_h25_v10_open walk_h25_v10_open.cpp mpc.cpp)
target_link_libraries(walk_h25_v10_open ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(walk_h25_v10_closed walk_h25_v10_closed.cpp mpc.cpp)
target_link_libraries(walk_h25_v10_closed ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(walk_h25_v15_open walk_h25_v15_open.cpp mpc.cpp)
target_link_libraries(walk_h25_v15_open ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(walk_h25_v15_closed walk_h25_v15_closed.cpp mpc.cpp)
target_link_libraries(walk_h25_v15_closed ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(trot_h20_v45_open trot_h20_v45_open.cpp mpc.cpp)
target_link_libraries(trot_h20_v45_open ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(trot_h20_v45_closed trot_h20_v45_closed.cpp mpc.cpp)
target_link_libraries(trot_h20_v45_closed ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(trot_h20_v60_open trot_h20_v60_open.cpp mpc.cpp)
target_link_libraries(trot_h20_v60_open ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(trot_h20_v60_closed trot_h20_v60_closed.cpp mpc.cpp)
target_link_libraries(trot_h20_v60_closed ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(trot_h25_v45_open trot_h25_v45_open.cpp mpc.cpp)
target_link_libraries(trot_h25_v45_open ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(trot_h25_v45_closed trot_h25_v45_closed.cpp mpc.cpp)
target_link_libraries(trot_h25_v45_closed ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(trot_h25_v60_open trot_h25_v60_open.cpp mpc.cpp)
target_link_libraries(trot_h25_v60_open ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)

add_executable(trot_h25_v60_closed trot_h25_v60_closed.cpp mpc.cpp)
target_link_libraries(trot_h25_v60_closed ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_WALK} ${CORGI_FORCE_EST} ${CORGI_HYBRID} ${CORGI_GAIT_SELECTOR} Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen)
