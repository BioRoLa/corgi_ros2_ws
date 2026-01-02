cmake_minimum_required(VERSION 3.8)

project(corgi_panel)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(corgi_msgs REQUIRED)

# Install Python script as an executable
install(PROGRAMS
  scripts/corgi_control_panel_dev.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
