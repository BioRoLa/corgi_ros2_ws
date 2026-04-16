#!/usr/bin/env python3
"""
ROS Worker for Corgi UI
Manages ROS 2 communication layer with PyQt5 signal integration.
Runs in separate thread to prevent blocking the GUI.
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import threading
from typing import Optional
from PyQt5.QtCore import QObject, pyqtSignal
from rcl_interfaces.msg import Log as RosoutLog

from corgi_msgs.msg import (
    ConfigStamped,
    MotorCmdStamped,
    MotorStateStamped,
    PowerCmdStamped,
    PowerStateStamped,
    RobotCmdStamped,
    RobotStateStamped,
    TriggerStamped
)


class RosWorkerBase(QObject):
    """
    Base ROS Worker for common functionality.
    Manages ROS 2 node lifecycle and executor thread.
    """
    
    def __init__(self, node_name: str = 'corgi_ui_node'):
        """
        Initialize ROS Worker
        
        Args:
            node_name: Name for the ROS 2 node
        """
        super().__init__()
        
        self.node_name = node_name
        self.node: Optional[Node] = None
        self.executor: Optional[SingleThreadedExecutor] = None
        self._executor_thread: Optional[threading.Thread] = None
        self._running = False
    
    def start_ros(self) -> bool:
        """
        Initialize ROS 2 node and start executor thread
        
        Returns:
            True if started successfully, False otherwise
        """
        if self._running:
            return True
        
        try:
            # Initialize ROS if not already initialized
            if not rclpy.ok():
                rclpy.init(args=None)
            
            # Create node (auto-declare parameter overrides like use_sim_time)
            self.node = rclpy.create_node(
                self.node_name,
                automatically_declare_parameters_from_overrides=True
            )
            
            # Setup publishers and subscribers (implemented by subclasses)
            self._setup_publishers()
            self._setup_subscribers()
            
            # Create executor and start spinning in separate thread
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            
            self._executor_thread = threading.Thread(
                target=self.executor.spin,
                daemon=True,
                name=f'{self.node_name}_executor'
            )
            self._executor_thread.start()
            
            self._running = True
            return True
            
        except Exception as e:
            print(f"[RosWorker] Failed to start ROS: {e}")
            return False
    
    def stop_ros(self) -> None:
        """Shutdown ROS node and executor"""
        if not self._running:
            return
        
        self._running = False
        
        if self.executor:
            self.executor.shutdown()
        
        if self.node:
            self.node.destroy_node()
        
        # Note: Not calling rclpy.shutdown() as it may be shared with other nodes
    
    def _setup_publishers(self) -> None:
        """Setup ROS publishers (override in subclass)"""
        pass
    
    def _setup_subscribers(self) -> None:
        """Setup ROS subscribers (override in subclass)"""
        pass
    
    @property
    def is_running(self) -> bool:
        """Check if ROS worker is running"""
        return self._running and self.node is not None


class ControlPanelRosWorker(RosWorkerBase):
    """
    ROS Worker for Control Panel
    Handles power, robot state, motor state, and logging topics.
    """
    
    # Signals for UI updates
    power_state_updated = pyqtSignal(object)  # PowerStateStamped
    robot_state_updated = pyqtSignal(object)  # RobotStateStamped
    motor_state_updated = pyqtSignal(object)  # MotorStateStamped
    log_updated = pyqtSignal(object)  # RosoutLog
    
    def __init__(self, node_name: str = 'corgi_control_panel'):
        super().__init__(node_name)
        
        # Publishers
        self.power_cmd_pub = None
        self.robot_cmd_pub = None
        self.trigger_pub = None
        
        # Subscribers
        self.power_state_sub = None
        self.robot_state_sub = None
        self.motor_state_sub = None
        self.log_sub = None
    
    def _setup_publishers(self) -> None:
        """Setup publishers for control panel"""
        self.power_cmd_pub = self.node.create_publisher(
            PowerCmdStamped, 'power/command', 10
        )
        self.robot_cmd_pub = self.node.create_publisher(
            RobotCmdStamped, 'robot/command', 10
        )
        self.trigger_pub = self.node.create_publisher(
            TriggerStamped, 'trigger', 10
        )
    
    def _setup_subscribers(self) -> None:
        """Setup subscribers for control panel"""
        self.power_state_sub = self.node.create_subscription(
            PowerStateStamped,
            'power/state',
            self._power_state_callback,
            10
        )
        self.robot_state_sub = self.node.create_subscription(
            RobotStateStamped,
            'robot/state',
            self._robot_state_callback,
            10
        )
        self.motor_state_sub = self.node.create_subscription(
            MotorStateStamped,
            'motor/state',
            self._motor_state_callback,
            10
        )
        self.log_sub = self.node.create_subscription(
            RosoutLog,
            '/rosout',
            self._log_callback,
            10
        )
    
    # Callbacks
    def _power_state_callback(self, msg: PowerStateStamped) -> None:
        """Callback for power state messages"""
        self.power_state_updated.emit(msg)
    
    def _robot_state_callback(self, msg: RobotStateStamped) -> None:
        """Callback for robot state messages"""
        self.robot_state_updated.emit(msg)
    
    def _motor_state_callback(self, msg: MotorStateStamped) -> None:
        """Callback for motor state messages"""
        self.motor_state_updated.emit(msg)
    
    def _log_callback(self, msg: RosoutLog) -> None:
        """Callback for /rosout messages"""
        self.log_updated.emit(msg)
    
    # Publishing methods
    def send_power_command(self, msg: PowerCmdStamped) -> None:
        """
        Publish power command
        
        Args:
            msg: PowerCmdStamped message to publish
        """
        if self.power_cmd_pub and self._running:
            self.power_cmd_pub.publish(msg)
    
    def send_robot_command(self, msg: RobotCmdStamped) -> None:
        """
        Publish robot mode command
        
        Args:
            msg: RobotCmdStamped message to publish
        """
        if self.robot_cmd_pub and self._running:
            self.robot_cmd_pub.publish(msg)
    
    def send_trigger(self, msg: TriggerStamped) -> None:
        """
        Publish trigger command (for data recording)
        
        Args:
            msg: TriggerStamped message to publish
        """
        if self.trigger_pub and self._running:
            self.trigger_pub.publish(msg)


class ConfigPanelRosWorker(RosWorkerBase):
    """
    ROS Worker for Configuration Panel
    Handles motor configuration communication.
    """
    
    # Signals for UI updates
    config_state_updated = pyqtSignal(object)  # ConfigStamped
    
    def __init__(self, node_name: str = 'corgi_config_panel'):
        super().__init__(node_name)
        
        # Publishers
        self.config_cmd_pub = None
        self.robot_cmd_pub = None
        
        # Subscribers
        self.config_state_sub = None
    
    def _setup_publishers(self) -> None:
        """Setup publishers for config panel"""
        self.config_cmd_pub = self.node.create_publisher(
            ConfigStamped, 'config/command', 10
        )
        self.robot_cmd_pub = self.node.create_publisher(
            RobotCmdStamped, 'robot/command', 10
        )
    
    def _setup_subscribers(self) -> None:
        """Setup subscribers for config panel"""
        self.config_state_sub = self.node.create_subscription(
            ConfigStamped,
            'config/state',
            self._config_state_callback,
            10
        )
    
    # Callbacks
    def _config_state_callback(self, msg: ConfigStamped) -> None:
        """Callback for config state messages"""
        self.config_state_updated.emit(msg)
    
    # Publishing methods
    def send_config_command(self, msg: ConfigStamped) -> None:
        """
        Publish motor configuration command
        
        Args:
            msg: ConfigStamped message to publish
        """
        if self.config_cmd_pub and self._running:
            self.config_cmd_pub.publish(msg)
    
    def send_robot_command(self, msg: RobotCmdStamped) -> None:
        """
        Publish robot mode command
        
        Args:
            msg: RobotCmdStamped message to publish
        """
        if self.robot_cmd_pub and self._running:
            self.robot_cmd_pub.publish(msg)
