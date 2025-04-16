add_executable(algo algo.cpp)
target_link_libraries(algo 
                        ${catkin_LIBRARIES} 
                        ${CMAKE_DL_LIBS} 
                        ${CORGI_UTILS} 
                        ${CORGI_GAIT_SELECTOR} 
                        ${CORGI_HYBRID} 
                        ${CORGI_WHEELED} 
                        ${CORGI_LEGGED} 
                        ${CORGI_TRANSFORMATION}
                        Eigen3::Eigen)

