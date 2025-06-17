add_executable(transform_test transform_test.cpp transform_gen.cpp WheeledToHybrid.cpp HybridToLegged.cpp )
target_link_libraries(transform_test  ${catkin_LIBRARIES} ${CMAKE_DL_LIBS} ${CORGI_UTILS} ${CORGI_GAIT_SELECTOR} ${CORGI_HYBRID} ${CORGI_WHEELED} ${CORGI_LEGGED} Eigen3::Eigen )

# HybridToWheeled.cpp 
# HybridToLegged.cpp 
# HybridToLegged.cpp
# WheeledToLegged.cpp 
# LeggedToWheeled.cpp 
# LeggedToHybrid.cpp