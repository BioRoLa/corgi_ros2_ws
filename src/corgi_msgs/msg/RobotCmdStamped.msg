# Robot FSM modes (values match fpga_driver RobotMode and Robot.proto ROBOTMODE)
int32 UNINITIALIZED=0  # motors off (was SYSTEM_ON)
int32 INIT=1
int32 IDLE=2
int32 ACTIVE=3         # commands accepted (was STANDBY)
int32 CONFIG=4         # motor config (was MOTORCONFIG)

Headers header
int32 request_robot_mode
