cmake_minimum_required(VERSION 3.8)
project(corgi_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# List your message files. It is recommended to place your .msg files under the msg/ directory.
set(msg_files
  "msg/Headers.msg"
  "msg/ImpedanceCmd.msg"
  "msg/ImpedanceCmdStamped.msg"
  "msg/ForceState.msg"
  "msg/ForceStateStamped.msg"
  "msg/MotorCmd.msg"
  "msg/MotorCmdStamped.msg"
  "msg/MotorState.msg"
  "msg/MotorStateStamped.msg"
  "msg/PowerCmdStamped.msg"
  "msg/PowerStateStamped.msg"
  "msg/TriggerStamped.msg"
  "msg/SensorEnableStamped.msg"
  "msg/SteeringCmdStamped.msg"
  "msg/SteeringStateStamped.msg"
  "msg/WheelCmd.msg"
  "msg/SimContactPoint.msg"
  "msg/SimDataStamped.msg"
  "msg/FsmCmdStamped.msg"
  "msg/FsmStateStamped.msg"
  "msg/ContactState.msg"
  "msg/ContactStateStamped.msg"
  "msg/StairPlanes.msg"
)

# List your service files. It is recommended to place your .ser files under the ser/ directory.
set(srv_files
  "srv/Imu.srv"
)

# Generate the message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
