add_executable(leg_model leg_model_main.cpp)
target_link_libraries(leg_model Eigen3::Eigen)

add_executable(bezier bezier.cpp)
# target_link_libraries(bezier Eigen3::Eigen)