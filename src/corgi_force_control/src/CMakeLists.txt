add_executable(imp_cmd_pub imp_cmd_pub.cpp)
target_link_libraries(imp_cmd_pub ${catkin_LIBRARIES} ${CMAKE_DL_LIBS})

add_executable(admittance_control admittance_control.cpp)
target_link_libraries(admittance_control ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)

add_executable(impedance_control impedance_control.cpp)
target_link_libraries(impedance_control ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)

add_executable(force_control force_control.cpp)
target_link_libraries(force_control ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)

add_executable(csv_cmd_pub csv_cmd_pub.cpp)
target_link_libraries(csv_cmd_pub ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_FORCE_EST} Eigen3::Eigen)