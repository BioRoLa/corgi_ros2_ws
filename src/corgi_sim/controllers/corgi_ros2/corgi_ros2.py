"""
Simple Webots ROS2 controller for Corgi robot.
Runs inside Webots and publishes motor sensor data, subscribes to motor commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from controller import Robot
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

class CorgiController(Node):
    def __init__(self, robot):
        super().__init__('corgi_webots_controller')
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        
        # Get motors and sensors
        self.motors = {
            'lf_left': robot.getDevice('lf_left_motor'),
            'lf_right': robot.getDevice('lf_right_motor'),
            'rf_left': robot.getDevice('rf_left_motor'),
            'rf_right': robot.getDevice('rf_right_motor'),
            'rh_left': robot.getDevice('rh_left_motor'),
            'rh_right': robot.getDevice('rh_right_motor'),
            'lh_left': robot.getDevice('lh_left_motor'),
            'lh_right': robot.getDevice('lh_right_motor'),
        }
        
        self.sensors = {}
        for name, motor in self.motors.items():
            sensor = motor.getPositionSensor()
            sensor.enable(self.timestep)
            self.sensors[name] = sensor
            # Initialize in position mode at current reading with a safe velocity
            try:
                current = sensor.getValue()
            except Exception:
                current = 0.0
            motor.setPosition(current)
            motor.setVelocity(3.0)
        
        # Publishers for sensor feedback
        self.sensor_pubs = {}
        for name in self.motors.keys():
            pub = self.create_publisher(Float64, f'{name}_motor_sensor/value', 10)
            self.sensor_pubs[name] = pub
        
        # Subscribers for motor commands
        self.motor_subs = {}
        for name in self.motors.keys():
            sub = self.create_subscription(
                Float64, f'{name}_motor/set_position',
                lambda msg, n=name: self.motor_cmd_callback(msg, n), 10
            )
            self.motor_subs[name] = sub
        
        # /clock publisher for sim time
        self.clock_pub = self.create_publisher(Clock, 'clock', 1000)
        
        self.get_logger().info('Corgi Webots Controller initialized')
    
    def motor_cmd_callback(self, msg, motor_name):
        """Set motor position target"""
        try:
            # Filter out NaN/inf values
            if not (isinstance(msg.data, (int, float)) and abs(msg.data) < 1e6):
                return
            motor = self.motors[motor_name]
            motor.setVelocity(3.0)
            motor.setPosition(float(msg.data))
        except Exception as e:
            self.get_logger().warn(f"Failed to set {motor_name} position: {e}")
    
    def step(self):
        """Called every simulation step"""
        # # Publish sensor values
        # for name, sensor in self.sensors.items():
        #     msg = Float64()
        #     msg.data = sensor.getValue()
        #     self.sensor_pubs[name].publish(msg)
        
        # Process ROS callbacks
        rclpy.spin_once(self, timeout_sec=0)
        # A. 發布模擬時間 /clock
        now = self.robot.getTime()
        ros_time_msg = Time()
        ros_time_msg.sec = int(now) 
        ros_time_msg.nanosec = int((now - int(now)) * 1e9)
        self.clock_pub.publish(Clock(clock=ros_time_msg))
        # Step simulation
        return self.robot.step(self.timestep) != -1

def main():
    # Initialize Webots robot
    robot = Robot()
    
    # Initialize ROS2
    rclpy.init()
    
    # Create controller node
    controller = CorgiController(robot)
    
    # Main loop
    while controller.step():
        pass
    
    # Cleanup
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
