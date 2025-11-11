#!/usr/bin/env python3
"""
ROS2 controller for Corgi robot in Webots.
This controller provides the ROS2 interface that corgi_sim_trq expects.
"""

import sys
import os

# Set WEBOTS_HOME if not already set (required for controller module to load)
if 'WEBOTS_HOME' not in os.environ:
    os.environ['WEBOTS_HOME'] = '/usr/local/webots'

# Ensure we're in the default ROS domain
if 'ROS_DOMAIN_ID' not in os.environ:
    os.environ['ROS_DOMAIN_ID'] = '0'

# Force ROS_LOCALHOST_ONLY to allow discovery
os.environ['ROS_LOCALHOST_ONLY'] = '0'

# Set RMW implementation explicitly
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

# Debug: Print Python path and ROS environment
print("=== Corgi ROS2 Controller Starting ===", file=sys.stderr, flush=True)
print(f"Python executable: {sys.executable}", file=sys.stderr, flush=True)
print(f"Python path: {sys.path[:3]}", file=sys.stderr, flush=True)
print(f"ROS_DISTRO: {os.environ.get('ROS_DISTRO', 'NOT SET')}", file=sys.stderr, flush=True)
print(f"AMENT_PREFIX_PATH: {os.environ.get('AMENT_PREFIX_PATH', 'NOT SET')[:100]}", file=sys.stderr, flush=True)
print(f"WEBOTS_HOME: {os.environ.get('WEBOTS_HOME', 'NOT SET')}", file=sys.stderr, flush=True)
print(f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', 'NOT SET')}", file=sys.stderr, flush=True)
print(f"ROS_LOCALHOST_ONLY: {os.environ.get('ROS_LOCALHOST_ONLY', 'NOT SET')}", file=sys.stderr, flush=True)
print(f"RMW_IMPLEMENTATION: {os.environ.get('RMW_IMPLEMENTATION', 'NOT SET')}", file=sys.stderr, flush=True)

try:
    from controller import Robot, Motor, PositionSensor, InertialUnit, Gyro, Accelerometer, Supervisor
    print("✓ Webots controller imported", file=sys.stderr)
except ImportError as e:
    print(f"✗ Failed to import Webots controller: {e}", file=sys.stderr)
    sys.exit(1)

try:
    import rclpy
    from rclpy.node import Node
    print("✓ rclpy imported", file=sys.stderr)
except ImportError as e:
    print(f"✗ Failed to import rclpy: {e}", file=sys.stderr)
    print("Make sure ROS2 is sourced before starting Webots!", file=sys.stderr)
    sys.exit(1)

try:
    from corgi_sim.srv import SetInt, SetFloat, GetUint64, NodeGetPosition, NodeGetOrientation
    from corgi_sim.srv import NodeEnableContactPointsTracking, NodeGetContactPoints, SupervisorGetFromId
    from corgi_sim.srv import NodeGetField, FieldGetString, NodeGetDef
    from corgi_sim.msg import Float64Stamped
    print("✓ corgi_sim messages imported", file=sys.stderr)
except ImportError as e:
    print(f"✗ Failed to import corgi_sim messages: {e}", file=sys.stderr)
    sys.exit(1)

try:
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Point, Quaternion, Vector3
    from std_msgs.msg import Header
    print("✓ ROS2 standard messages imported", file=sys.stderr)
except ImportError as e:
    print(f"✗ Failed to import ROS2 messages: {e}", file=sys.stderr)
    sys.exit(1)

import math

print("=== All imports successful ===", file=sys.stderr)


class CorgiROS2Controller(Node):
    def __init__(self, robot):
        super().__init__('corgi_ros2_controller')
        print("=== Initializing CorgiROS2Controller node ===", file=sys.stderr)
        
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        print(f"Timestep: {self.timestep} ms", file=sys.stderr)
        
        # Get all motor devices
        print("=== Getting motor devices ===", file=sys.stderr)
        self.motors = {
            'AR': robot.getDevice('lf_right_motor'),
            'AL': robot.getDevice('lf_left_motor'),
            'BR': robot.getDevice('rf_right_motor'),
            'BL': robot.getDevice('rf_left_motor'),
            'CR': robot.getDevice('rh_right_motor'),
            'CL': robot.getDevice('rh_left_motor'),
            'DR': robot.getDevice('lh_right_motor'),
            'DL': robot.getDevice('lh_left_motor'),
        }
        print(f"✓ Got {len(self.motors)} motors", file=sys.stderr)
        
        # Get position sensors for encoders
        print("=== Getting position sensors ===", file=sys.stderr)
        self.position_sensors = {
            'AR': robot.getDevice('lf_right_motor_sensor'),
            'AL': robot.getDevice('lf_left_motor_sensor'),
            'BR': robot.getDevice('rf_right_motor_sensor'),
            'BL': robot.getDevice('rf_left_motor_sensor'),
            'CR': robot.getDevice('rh_right_motor_sensor'),
            'CL': robot.getDevice('rh_left_motor_sensor'),
            'DR': robot.getDevice('lh_right_motor_sensor'),
            'DL': robot.getDevice('lh_left_motor_sensor'),
        }
        
        # Enable position sensors
        for name, sensor in self.position_sensors.items():
            if sensor:
                sensor.enable(self.timestep)
        print(f"✓ Enabled {len(self.position_sensors)} position sensors", file=sys.stderr)
        
        # Get IMU devices
        print("=== Getting IMU devices ===", file=sys.stderr)
        self.gyro = robot.getDevice('gyro')
        self.accel = robot.getDevice('accelerometer')
        
        if self.gyro:
            self.gyro.enable(self.timestep)
            print("✓ Gyro enabled", file=sys.stderr)
        if self.accel:
            self.accel.enable(self.timestep)
            print("✓ Accelerometer enabled", file=sys.stderr)
        
        # Create time step service
        print("=== Creating services ===", file=sys.stderr, flush=True)
        self.time_step_srv = self.create_service(
            SetInt,
            'robot/time_step',
            self.time_step_callback
        )
        print("✓ Created robot/time_step service", file=sys.stderr, flush=True)
        
        # Give DDS time to advertise the service
        import time
        time.sleep(2)
        print("✓ Waited for DDS service advertisement", file=sys.stderr, flush=True)
        
        # Motor torque services
        self.create_service(SetFloat, 'lf_left_motor/set_torque', lambda req, res: self.set_motor_torque('AR', req, res))
        self.create_service(SetFloat, 'lf_right_motor/set_torque', lambda req, res: self.set_motor_torque('AL', req, res))
        self.create_service(SetFloat, 'rf_left_motor/set_torque', lambda req, res: self.set_motor_torque('BR', req, res))
        self.create_service(SetFloat, 'rf_right_motor/set_torque', lambda req, res: self.set_motor_torque('BL', req, res))
        self.create_service(SetFloat, 'rh_left_motor/set_torque', lambda req, res: self.set_motor_torque('CR', req, res))
        self.create_service(SetFloat, 'rh_right_motor/set_torque', lambda req, res: self.set_motor_torque('CL', req, res))
        self.create_service(SetFloat, 'lh_left_motor/set_torque', lambda req, res: self.set_motor_torque('DR', req, res))
        self.create_service(SetFloat, 'lh_right_motor/set_torque', lambda req, res: self.set_motor_torque('DL', req, res))
        
        # Motor encoders - publishers
        self.encoder_pubs = {
            'AR': self.create_publisher(Float64Stamped, 'lf_left_motor_sensor/value', 10),
            'AL': self.create_publisher(Float64Stamped, 'lf_right_motor_sensor/value', 10),
            'BR': self.create_publisher(Float64Stamped, 'rf_left_motor_sensor/value', 10),
            'BL': self.create_publisher(Float64Stamped, 'rf_right_motor_sensor/value', 10),
            'CR': self.create_publisher(Float64Stamped, 'rh_left_motor_sensor/value', 10),
            'CL': self.create_publisher(Float64Stamped, 'rh_right_motor_sensor/value', 10),
            'DR': self.create_publisher(Float64Stamped, 'lh_left_motor_sensor/value', 10),
            'DL': self.create_publisher(Float64Stamped, 'lh_right_motor_sensor/value', 10),
        }
        
        # IMU publishers
        self.gyro_pub = self.create_publisher(Imu, 'gyro/quaternion', 10)
        self.ang_vel_pub = self.create_publisher(Imu, 'ang_vel/values', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/values', 10)
        
        # Secondary device references (legacy names removed) - already acquired via explicit map above.
        self.motors = {}
        self.sensors = {}
        # Use explicit Webots names matching world file
        explicit_motor_map = {
            'AR': robot.getDevice('lf_right_motor'),
            'AL': robot.getDevice('lf_left_motor'),
            'BR': robot.getDevice('rf_right_motor'),
            'BL': robot.getDevice('rf_left_motor'),
            'CR': robot.getDevice('rh_right_motor'),
            'CL': robot.getDevice('rh_left_motor'),
            'DR': robot.getDevice('lh_right_motor'),
            'DL': robot.getDevice('lh_left_motor'),
        }
        for key, dev in explicit_motor_map.items():
            if dev:
                try:
                    dev.setAvailableTorque(10.0)
                except Exception:
                    pass
                self.motors[key] = dev
        explicit_sensor_map = {
            'AR': robot.getDevice('lf_right_motor_sensor'),
            'AL': robot.getDevice('lf_left_motor_sensor'),
            'BR': robot.getDevice('rf_right_motor_sensor'),
            'BL': robot.getDevice('rf_left_motor_sensor'),
            'CR': robot.getDevice('rh_right_motor_sensor'),
            'CL': robot.getDevice('rh_left_motor_sensor'),
            'DR': robot.getDevice('lh_right_motor_sensor'),
            'DL': robot.getDevice('lh_left_motor_sensor'),
        }
        for key, sensor in explicit_sensor_map.items():
            if sensor:
                sensor.enable(self.timestep)
                self.sensors[key] = sensor
        
        # Get IMU sensors
        self.imu_device = robot.getDevice('imu')
        self.gyro_device = robot.getDevice('gyro')
        self.accelerometer = robot.getDevice('accelerometer')
        
        if self.imu_device:
            self.imu_device.enable(self.timestep)
        if self.gyro_device:
            self.gyro_device.enable(self.timestep)
        if self.accelerometer:
            self.accelerometer.enable(self.timestep)
            
        self.get_logger().info('Corgi ROS2 Controller initialized')
        self.get_logger().info(f'Found {len(self.motors)} motors and {len(self.sensors)} sensors')
    
    def time_step_callback(self, request, response):
        """Handle timestep advancement by advancing the Webots simulation."""
        # Advance the simulation by requested number of milliseconds (or default timestep)
        step_ms = request.value if hasattr(request, 'value') and request.value > 0 else self.timestep
        result = self.robot.step(step_ms)
        response.success = (result != -1)
        # Publish sensors after each step
        try:
            self.publish_sensors()
        except Exception as e:
            self.get_logger().warn(f"publish_sensors failed: {e}")
        return response
    
    def get_self_callback(self, request, response):
        """Return the robot's node ID"""
        response.value = 1  # Robot's self ID
        return response
    
    def get_position_callback(self, request, response):
        """Get robot position"""
        if isinstance(self.robot, Supervisor):
            robot_node = self.robot.getSelf()
            if robot_node:
                pos = robot_node.getPosition()
                response.position = Point(x=pos[0], y=pos[1], z=pos[2])
        return response
    
    def get_orientation_callback(self, request, response):
        """Get robot orientation as quaternion"""
        if isinstance(self.robot, Supervisor):
            robot_node = self.robot.getSelf()
            if robot_node:
                # Get orientation matrix and convert to quaternion
                orientation_field = robot_node.getField('rotation')
                if orientation_field:
                    rotation = orientation_field.getSFRotation()
                    # rotation is [x, y, z, angle] axis-angle format
                    # Convert to quaternion
                    quat = self.axis_angle_to_quaternion(rotation)
                    response.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        return response
    
    def enable_contact_points_callback(self, request, response):
        """Enable contact point tracking"""
        # For now, just return success
        response.success = True
        return response
    
    def get_contact_points_callback(self, request, response):
        """Get contact points - return empty for now"""
        response.contact_points = []
        return response
    
    def get_from_id_callback(self, request, response):
        """Get node from ID"""
        response.node = request.id
        return response
    
    def get_field_callback(self, request, response):
        """Get field from node"""
        response.field = 0  # Dummy value
        return response
    
    def get_string_callback(self, request, response):
        """Get string from field"""
        response.value = ""
        return response
    
    def get_def_callback(self, request, response):
        """Get DEF name"""
        response.name = "CORGI"
        return response
    
    def set_motor_torque(self, motor_name, request, response):
        """Set motor torque"""
        if motor_name in self.motors:
            self.motors[motor_name].setTorque(request.value)
            response.success = True
        else:
            self.get_logger().warn(f'Motor {motor_name} not found')
            response.success = False
        return response
    
    def publish_sensors(self):
        """Publish all sensor data"""
        # Publish encoder values
        for name, sensor in self.sensors.items():
            if name in self.encoder_pubs:
                msg = Float64Stamped()
                msg.data = sensor.getValue()
                self.encoder_pubs[name].publish(msg)
        
        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Get orientation from IMU if available
        if self.imu_device:
            roll_pitch_yaw = self.imu_device.getRollPitchYaw()
            # Convert roll-pitch-yaw to quaternion
            quat = self.euler_to_quaternion(roll_pitch_yaw[0], roll_pitch_yaw[1], roll_pitch_yaw[2])
            imu_msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            self.gyro_pub.publish(imu_msg)
        
        # Get angular velocity from gyro
        if self.gyro_device:
            ang_vel = self.gyro_device.getValues()
            ang_vel_msg = Imu()
            ang_vel_msg.header = imu_msg.header
            ang_vel_msg.angular_velocity = Vector3(x=ang_vel[0], y=ang_vel[1], z=ang_vel[2])
            self.ang_vel_pub.publish(ang_vel_msg)
        
        # Get linear acceleration
        if self.accelerometer:
            accel = self.accelerometer.getValues()
            accel_msg = Imu()
            accel_msg.header = imu_msg.header
            accel_msg.linear_acceleration = Vector3(x=accel[0], y=accel[1], z=accel[2])
            self.imu_pub.publish(accel_msg)
    
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [x, y, z, w]
    
    @staticmethod
    def axis_angle_to_quaternion(rotation):
        """Convert axis-angle to quaternion"""
        x, y, z, angle = rotation
        s = math.sin(angle / 2.0)
        return [x * s, y * s, z * s, math.cos(angle / 2.0)]


def main(args=None):
    print("=== Entering main() ===", file=sys.stderr, flush=True)
    try:
        print("Creating Supervisor...", file=sys.stderr, flush=True)
        robot = Supervisor()
        timestep = int(robot.getBasicTimeStep())
        print(f"✓ Webots Supervisor initialized, timestep={timestep} ms", file=sys.stderr, flush=True)
    except Exception as e:
        print(f"✗ Failed to create Supervisor: {e}", file=sys.stderr, flush=True)
        import traceback
        traceback.print_exc(file=sys.stderr)
        sys.exit(1)

    try:
        print("Initializing rclpy...", file=sys.stderr, flush=True)
        rclpy.init(args=args)
        print("✓ rclpy initialized", file=sys.stderr, flush=True)
    except Exception as e:
        print(f"✗ Failed to initialize rclpy: {e}", file=sys.stderr, flush=True)
        import traceback
        traceback.print_exc(file=sys.stderr)
        sys.exit(1)

    try:
        print("Creating CorgiROS2Controller...", file=sys.stderr, flush=True)
        controller = CorgiROS2Controller(robot)
        print("✓ CorgiROS2Controller created", file=sys.stderr, flush=True)
    except Exception as e:
        print(f"✗ Failed to create CorgiROS2Controller: {e}", file=sys.stderr, flush=True)
        import traceback
        traceback.print_exc(file=sys.stderr)
        sys.exit(1)

    # Main loop - perform regular stepping; time_step service will just acknowledge
    try:
        print("=== Starting main loop (internal stepping) ===", file=sys.stderr, flush=True)
        step_count = 0
        while robot.step(timestep) != -1:
            rclpy.spin_once(controller, timeout_sec=0.001)
            step_count += 1
            if step_count == 1:
                print("✓ First simulation step completed", file=sys.stderr, flush=True)
            elif step_count % 1000 == 0:
                print(f"✓ {step_count} steps completed", file=sys.stderr, flush=True)
    except KeyboardInterrupt:
        print("=== Keyboard interrupt received ===", file=sys.stderr)
    except Exception as e:
        print(f"=== Exception in main loop: {e} ===", file=sys.stderr)
        import traceback
        traceback.print_exc(file=sys.stderr)

    print("=== Cleaning up ===", file=sys.stderr)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
