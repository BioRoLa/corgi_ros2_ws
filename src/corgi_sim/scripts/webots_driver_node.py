#!/usr/bin/env python3
"""
External ROS2 driver for Corgi robot in Webots.
Uses webots_ros2_driver to connect to Webots as an external controller.
"""

import rclpy
from webots_ros2_driver.webots_node import WebotsNode

class CorgiDriverNode(WebotsNode):
    def __init__(self, args):
        super().__init__('corgi_driver', args=args)
        
        # Initialize robot devices
        self.robot = self.driver
        
        # Get motor devices
        self.motors = {}
        motor_names = {
            'lf_left_motor': 'AR',
            'lf_right_motor': 'AL',
            'rf_left_motor': 'BR',
            'rf_right_motor': 'BL',
            'rh_left_motor': 'CR',
            'rh_right_motor': 'CL',
            'lh_left_motor': 'DR',
            'lh_right_motor': 'DL',
        }
        
        for webots_name, alias in motor_names.items():
            motor = self.robot.getDevice(webots_name)
            if motor:
                motor.setAvailableTorque(10.0)
                self.motors[alias] = motor
                self.get_logger().info(f'Found motor: {webots_name} ({alias})')
        
        # Get position sensors
        self.position_sensors = {}
        for webots_name, alias in motor_names.items():
            sensor_name = webots_name + '_sensor'
            sensor = self.robot.getDevice(sensor_name)
            if sensor:
                sensor.enable(int(self.robot.getBasicTimeStep()))
                self.position_sensors[alias] = sensor
                self.get_logger().info(f'Found sensor: {sensor_name} ({alias})')
        
        # Get IMU devices
        self.gyro = self.robot.getDevice('gyro')
        if self.gyro:
            self.gyro.enable(int(self.robot.getBasicTimeStep()))
            self.get_logger().info('Gyro enabled')
        
        self.get_logger().info(f'Corgi driver initialized with {len(self.motors)} motors')

def main(args=None):
    rclpy.init(args=args)
    
    driver = CorgiDriverNode(args=args)
    
    rclpy.spin(driver)
    
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
