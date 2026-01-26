#!/usr/bin/env python3
"""
Configuration Panel GUI for Corgi Motor Settings
Refactored version with modular architecture
"""
import os
import sys
import logging

from datetime import datetime
from collections import deque
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel,
    QGroupBox, QLineEdit, QFormLayout, QTabWidget, QMessageBox,
    QProgressBar, QComboBox, QApplication
)

from PyQt5.QtCore import Qt, QTimer

from corgi_msgs.msg import ConfigStamped, RobotCmdStamped

# Import from corgi_ui package
from corgi_ui.core.constants import (
    ROBOTMODE, LOGLEVEL, Module, Motor, ConfigMode, ErrorCode, PATHS,
    setup_file_logger, log_to_file, close_file_logger
)
from corgi_ui.core.motor_data import MotorParameterRegistry, ConfigType
from corgi_ui.core.ros_worker import ConfigPanelRosWorker
from corgi_ui.gui.widgets.log_widget import LogWidget


class ParameterTab(QWidget):
    """
    Tab widget for displaying and editing motor parameters
    
    Organized by parameter groups (System, Control, Limits, Motor, Reserved)
    """
    
    def __init__(self, param_registry: MotorParameterRegistry, parent=None):
        super().__init__(parent)
        self.registry = param_registry
        self.inputs = {}
        self._init_ui()
    
    def _init_ui(self):
        """Initialize parameter tabs UI"""
        layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        
        # Define tab order
        order = ["System", "Control", "Limits", "Motor", "Reserved"]
        sorted_groups = sorted(
            self.registry.get_all_groups(),
            key=lambda x: order.index(x) if x in order else 99
        )
        
        for group in sorted_groups:
            page = QWidget()
            form = QFormLayout(page)
            form.setLabelAlignment(Qt.AlignRight)
            
            for param in self.registry.get_group(group):
                name = param.name
                inp = QLineEdit()
                inp.setPlaceholderText("Wait...")
                inp.setEnabled(False)
                
                if param.writable:
                    inp.returnPressed.connect(lambda n=name: self.handle_enter(n))
                else:
                    inp.setStyleSheet("background-color: #303030; color: #888;")
                
                self.inputs[name] = inp
                
                # Create label with description
                label_text = f"{param.description} ({name})" if param.description else name
                if param.unit:
                    label_text += f" {param.unit}"
                
                form.addRow(label_text, inp)
            
            self.tabs.addTab(page, group)
        
        layout.addWidget(self.tabs)
    
    def handle_enter(self, name: str):
        """Handle Enter key press on parameter field"""
        val = self.inputs[name].text()
        if hasattr(self.parent(), 'start_write_transaction'):
            self.parent().start_write_transaction(name, val)
    
    def update_field_from_motor(self, name: str, value):
        """Update field value from motor response"""
        if name in self.inputs:
            inp = self.inputs[name]
            inp.setText(str(value))
            param = self.registry.get_by_name(name)
            if param and param.writable:
                inp.setEnabled(True)
                inp.setStyleSheet("background-color: #404040; border: 1px solid #555;")
    
    def set_all_enabled(self, enabled: bool):
        """Enable or disable all writable fields"""
        for name, inp in self.inputs.items():
            param = self.registry.get_by_name(name)
            if param and param.writable:
                inp.setEnabled(enabled)
                if not enabled:
                    inp.setStyleSheet("background-color: #303030; color: #888;")
                else:
                    inp.setStyleSheet("background-color: #404040; border: 1px solid #555;")


class CorgiConfigPanel(QWidget):
    """
    Configuration Panel for Corgi Motor Parameters
    
    Features:
    - Read all motor parameters from selected motor
    - Write individual parameters with validation
    - Progress indication during bulk read operations
    - Error handling and retry logic
    - REST command to return to SYSTEM_ON mode
    """
    
    def __init__(self):
        super().__init__()
        
        # Initialize data models
        self.registry = MotorParameterRegistry()
        
        # ROS Worker
        self.ros_worker = ConfigPanelRosWorker()
        
        # File logging setup
        self.log_dir = PATHS.DEFAULT_LOG_DIR
        os.makedirs(self.log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = f'config_panel_{timestamp}.log'
        self.log_filepath = os.path.join(self.log_dir, self.log_filename)
        self.file_logger = self._setup_file_logger()
        
        # State management
        self.seq_counter = 0
        self.robot_cmd_seq = 0
        self.current_module = None
        self.current_motor = None
        
        # Read queue management
        self.load_queue = deque()
        self.pending_write = None
        self.retry_count = 0
        self.max_retries = 3
        self.successful_reads = 0
        self.total_load_items = 0
        
        # Initialize UI
        self._load_stylesheet()
        self._init_ui()
        
        # Connect ROS signals
        self._connect_ros_signals()
        
        # Start ROS worker
        if self.ros_worker.start_ros():
            self._log("ROS Worker Initialized", LOGLEVEL.INFO, "system")
        
        # Timeout timer for communication
        self.tx_timer = QTimer()
        self.tx_timer.setSingleShot(True)
        self.tx_timer.timeout.connect(self._handle_timeout)
    
    def _load_stylesheet(self):
        """
        Load QSS stylesheet from assets (DEPRECATED - now loaded by launcher)
        Kept for standalone execution compatibility
        """
        try:
            # Get path to theme.qss
            current_dir = os.path.dirname(os.path.abspath(__file__))
            assets_dir = os.path.join(os.path.dirname(current_dir), 'assets')
            theme_path = os.path.join(assets_dir, 'theme.qss')
            
            if os.path.exists(theme_path):
                with open(theme_path, 'r', encoding='utf-8') as f:
                    stylesheet = f.read()
                    app = QApplication.instance()
                    if app and not app.styleSheet():
                        app.setStyleSheet(stylesheet)
        except Exception as e:
            print(f"Warning: Failed to load theme.qss: {e}")
    
    def _init_ui(self):
        """Initialize user interface"""
        self.setWindowTitle("Corgi Motor Configurator")
        self.resize(1200, 800)
        
        main_layout = QHBoxLayout(self)
        
        # Left Panel (Selection + Actions + Log)
        main_layout.addLayout(self._create_left_panel(), 1)
        
        # Right Panel (Parameters)
        main_layout.addLayout(self._create_right_panel(), 3)
    
    def _create_left_panel(self) -> QVBoxLayout:
        """Create left panel with selection, actions, and log"""
        left_panel = QVBoxLayout()
        
        # Target Selection
        sel_group = QGroupBox("Target")
        sel_layout = QFormLayout(sel_group)
        
        self.cb_module = QComboBox()
        self.cb_module.addItems([
            "- Select -", "Module A", "Module B", "Module C", "Module D"
        ])
        self.cb_module.currentIndexChanged.connect(self._on_module_changed)
        
        self.cb_motor = QComboBox()
        self.cb_motor.addItems([
            "- Select -", "Motor R (Belt)", "Motor L (Direct)"
        ])
        self.cb_motor.setEnabled(False)
        self.cb_motor.currentIndexChanged.connect(self._on_motor_selected)
        
        sel_layout.addRow("Module:", self.cb_module)
        sel_layout.addRow("Motor:", self.cb_motor)
        left_panel.addWidget(sel_group)
        
        # Actions
        action_group = QGroupBox("Actions")
        action_layout = QVBoxLayout(action_group)
        
        self.btn_refresh = QPushButton("Refresh")
        self.btn_refresh.setEnabled(False)
        self.btn_refresh.clicked.connect(self._refresh_current_motor)
        self.btn_refresh.setStyleSheet(
            "QPushButton { background-color: #3a6ea5; color: white; border: none; } "
            "QPushButton:hover { background-color: #4a7eb5; }"
        )
        
        self.btn_rest = QPushButton("REST (System On)")
        self.btn_rest.clicked.connect(self._send_rest_command)
        self.btn_rest.setStyleSheet(
            "QPushButton { background-color: #2e7d32; color: white; border: none; } "
            "QPushButton:hover { background-color: #3e8d42; }"
        )
        
        action_layout.addWidget(self.btn_refresh)
        action_layout.addWidget(self.btn_rest)
        left_panel.addWidget(action_group)
        
        # Log Widget
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout(log_group)
        
        self.log_widget = LogWidget(default_level=LOGLEVEL.DEBUG)
        log_layout.addWidget(self.log_widget)
        log_group.setLayout(log_layout)
        
        left_panel.addWidget(log_group, 1)
        
        return left_panel
    
    def _create_right_panel(self) -> QVBoxLayout:
        """Create right panel with parameters"""
        right_panel = QVBoxLayout()
        
        # Status label
        self.status_label = QLabel("Please select a motor.")
        self.status_label.setStyleSheet(
            "color: #aaa; font-style: italic; font-size: 16px;"
        )
        
        # Progress bar
        self.progress = QProgressBar()
        self.progress.setVisible(False)
        self.progress.setStyleSheet(
            "QProgressBar { border: 1px solid #555; border-radius: 5px; text-align: center; } "
            "QProgressBar::chunk { background-color: #00e676; }"
        )
        
        # Parameter tabs
        self.param_widget = ParameterTab(self.registry, self)
        
        right_panel.addWidget(self.status_label)
        right_panel.addWidget(self.progress)
        right_panel.addWidget(self.param_widget)
        
        return right_panel
    
    def _setup_file_logger(self):
        """Setup file logger for persistent logging"""
        return setup_file_logger('CorgiConfigPanel', self.log_filepath)
    
    def _connect_ros_signals(self):
        """Connect ROS worker signals"""
        self.ros_worker.config_state_updated.connect(self._handle_config_state)
    
    def _log(self, message: str, level=LOGLEVEL.INFO, source: str = "system"):
        
        # Write to GUI log widget
        if hasattr(self, 'log_widget'):
            self.log_widget.add_log(message, level, source)
        
        # Write to file logger
        if hasattr(self, 'file_logger'):
            log_to_file(self.file_logger, level, source, message)
    
    # ========================================================================
    # Event Handlers - Selection
    # ========================================================================
    
    def _on_module_changed(self):
        """Handle module selection change"""
        idx = self.cb_module.currentIndex()
        if idx > 0:
            self.current_module = idx - 1
            self.cb_motor.setEnabled(True)
            self._log(
                f"Selected Module {chr(65 + self.current_module)}",
                LOGLEVEL.DEBUG, "user"
            )
        else:
            self.current_module = None
            self.cb_motor.setEnabled(False)
            self.cb_motor.setCurrentIndex(0)
    
    def _on_motor_selected(self):
        """Handle motor selection"""
        idx = self.cb_motor.currentIndex()
        if idx <= 0 or self.current_module is None:
            return
        
        self.current_motor = idx - 1
        motor_name = "R (Belt)" if self.current_motor == Motor.MOTOR_R else "L (Direct)"
        self._log(
            f"Selected Motor {motor_name}",
            LOGLEVEL.DEBUG, "user"
        )
        
        self._start_load_sequence()
    
    def _refresh_current_motor(self):
        """Refresh current motor parameters"""
        if self.current_module is not None and self.current_motor is not None:
            self._log(
                "Refreshing motor parameters...",
                LOGLEVEL.INFO, "user"
            )
            self._start_load_sequence()
        else:
            self._log(
                "No motor selected to refresh",
                LOGLEVEL.WARN, "user"
            )
    
    # ========================================================================
    # Read Sequence
    # ========================================================================
    
    def _start_load_sequence(self):
        """Start reading all parameters from motor"""
        self.load_queue.clear()
        self.successful_reads = 0
        
        # Disable UI during reading
        self.param_widget.set_all_enabled(False)
        self.cb_module.setEnabled(False)
        self.cb_motor.setEnabled(False)
        self.btn_refresh.setEnabled(False)
        
        # Show progress
        self.progress.setVisible(True)
        self.progress.setValue(0)
        self.status_label.setText("Reading parameters...")
        
        # Build read queue (all INT addresses, then all FLOAT addresses)
        for addr in range(8):
            self.load_queue.append((ConfigType.INT, addr))
        for addr in range(30):
            self.load_queue.append((ConfigType.FLOAT, addr))
        
        self.total_load_items = len(self.load_queue)
        self._log(
            f"Reading {self.total_load_items} parameters...",
            LOGLEVEL.INFO, "system"
        )
        
        self._process_load_queue()
    
    def _process_load_queue(self):
        """Process next item in read queue"""
        if not self.load_queue:
            self._finish_loading()
            return
        
        c_type, addr = self.load_queue[0]
        self.retry_count = 0
        
        # Update progress
        done = self.total_load_items - len(self.load_queue)
        self.progress.setValue(int((done / self.total_load_items) * 100))
        
        # Send read command
        self._send_config_cmd(ConfigMode.READ, c_type, addr, 0)
        self.tx_timer.start(3000)  # 3 second timeout
    
    def _finish_loading(self):
        """Finish loading sequence and update UI"""
        if self.successful_reads == 0:
            self.status_label.setText("Connection Failed")
            self.status_label.setStyleSheet("color: #ff5252; font-weight: bold;")
            self._log(
                "Failed to read parameters.",
                LOGLEVEL.ERROR, "system"
            )
        else:
            total_params = self.total_load_items
            if self.successful_reads < total_params:
                self.status_label.setText(f"Partial: {self.successful_reads}/{total_params}")
                self.status_label.setStyleSheet("color: #ffea00; font-weight: bold;")
                self._log(
                    f"Partial success ({self.successful_reads}/{total_params})",
                    LOGLEVEL.WARN, "system"
                )
            else:
                self.status_label.setText(
                    f"Connected: M{self.current_module}-M{self.current_motor}"
                )
                self.status_label.setStyleSheet("color: #00e676; font-weight: bold;")
                self._log("Read complete.", LOGLEVEL.INFO, "system")
        
        # Re-enable UI
        self.progress.setVisible(False)
        self.cb_module.setEnabled(True)
        self.cb_motor.setEnabled(True)
        self.btn_refresh.setEnabled(True)
    
    # ========================================================================
    # Write Transaction
    # ========================================================================
    
    def start_write_transaction(self, name: str, value_str: str):
        """Start write transaction for a parameter"""
        param = self.registry.get_by_name(name)
        if not param:
            return
        
        # Validate and parse value
        try:
            if param.data_type == ConfigType.INT:
                val = int(float(value_str))
            else:
                val = float(value_str)
            
            # Check limits
            if val < param.min_value or val > param.max_value:
                QMessageBox.warning(
                    self,
                    "Limit Error",
                    f"Value out of range ({param.min_value} ~ {param.max_value})"
                )
                return
        except ValueError:
            QMessageBox.warning(self, "Format Error", "Invalid number format.")
            return
        
        # Disable UI during write
        self.param_widget.set_all_enabled(False)
        self.status_label.setText(f"Writing {name}...")
        
        # Store pending write info
        self.pending_write = {
            'name': name,
            'target_val': val,
            'param': param,
            'seq_sent': 0
        }
        
        self._log(
            f"Writing {name} = {val}",
            LOGLEVEL.INFO, "orin"
        )
        
        # Send write command
        sent_seq = self._send_config_cmd(
            ConfigMode.WRITE,
            param.data_type,
            param.address,
            val
        )
        self.pending_write['seq_sent'] = sent_seq
        self.tx_timer.start(3000)
    
    # ========================================================================
    # ROS Communication
    # ========================================================================
    
    def _send_config_cmd(
        self,
        mode: ConfigMode,
        c_type: ConfigType,
        addr: int,
        val
    ) -> int:
        """Send configuration command to motor"""
        if not self.ros_worker.is_running:
            return 0
        
        self.seq_counter = (self.seq_counter + 1) % 65535
        
        msg = ConfigStamped()
        msg.header.seq = self.seq_counter
        msg.header.stamp = self.ros_worker.node.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        msg.transmit = True
        msg.module = int(self.current_module)
        msg.motor = int(self.current_motor)
        msg.mode = int(mode)
        msg.type = int(c_type)
        msg.address = int(addr)
        msg.error_code = 0
        
        if c_type == ConfigType.INT:
            msg.value_i = int(val)
            msg.value_f = 0.0
        else:
            msg.value_i = 0
            msg.value_f = float(val)
        
        self.ros_worker.send_config_command(msg)
        
        # Debug log
        mode_str = "READ" if mode == ConfigMode.READ else "WRITE"
        self._log(
            f"CMD {mode_str} T:{c_type.name} A:{addr} SEQ:{self.seq_counter}",
            LOGLEVEL.DEBUG, "orin"
        )
        
        return self.seq_counter
    
    def _handle_config_state(self, msg: ConfigStamped):
        """Handle configuration state message from ROS"""
        # Filter mismatching sequence IDs
        if msg.header.seq != self.seq_counter:
            return
        
        error_code = msg.error_code
        error_name = (
            ErrorCode(error_code).name
            if error_code in ErrorCode.__members__.values()
            else f"ERR_{error_code}"
        )
        
        # Debug log
        self._log(
            f"REPLY SEQ:{msg.header.seq} ERR:{error_code}({error_name}) V:{msg.value_f:.2f}",
            LOGLEVEL.DEBUG, "fpga_driver"
        )
        
        # Priority 1: Handle Read Queue
        if self.load_queue:
            self._handle_read_response(msg, error_code)
            return
        
        # Priority 2: Handle Write Transaction
        if self.pending_write:
            self._handle_write_response(msg, error_code)
    
    def _handle_read_response(self, msg: ConfigStamped, error_code: int):
        """Handle response during read sequence"""
        req_type, req_addr = self.load_queue[0]
        
        # Check if response matches request
        if msg.type == int(req_type) and msg.address == req_addr:
            self.tx_timer.stop()
            self.load_queue.popleft()
            
            if error_code == ErrorCode.CODE_CONFIG_SUCCESS:
                # Success - update UI
                param = self.registry.get_by_addr(ConfigType(msg.type), msg.address)
                if param:
                    value = msg.value_i if msg.type == ConfigType.INT else msg.value_f
                    self.param_widget.update_field_from_motor(param.name, value)
                    self.successful_reads += 1
            else:
                # Error - log and skip
                self._log(
                    f"Read error at {req_type.name}:{req_addr} - {error_code}",
                    LOGLEVEL.WARN, "fpga_driver"
                )
            
            # Continue with next item
            self._process_load_queue()
    
    def _handle_write_response(self, msg: ConfigStamped, error_code: int):
        """Handle response during write transaction"""
        tx = self.pending_write
        param = tx['param']
        
        # Check if response targets current parameter
        if ConfigType(msg.type) != param.data_type or msg.address != param.address:
            return
        
        # Check error code
        if error_code != ErrorCode.CODE_CONFIG_SUCCESS:
            error_name = (
                ErrorCode(error_code).name
                if error_code in ErrorCode.__members__.values()
                else f"ERR_{error_code}"
            )
            self._log(
                f"Write Failed: {error_name}",
                LOGLEVEL.ERROR, "fpga_driver"
            )
            self._end_transaction(success=False, msg=f"Write Error: {error_name}")
            return
        
        # Check if echoed value matches target
        received_val = msg.value_i if param.data_type == ConfigType.INT else msg.value_f
        target_val = tx['target_val']
        
        is_match = False
        if param.data_type == ConfigType.INT:
            is_match = (received_val == int(target_val))
        else:
            is_match = abs(received_val - float(target_val)) < 0.001
        
        if is_match:
            self._log(
                f"Write Success (Ack): {param.name} -> {received_val}",
                LOGLEVEL.INFO, "system"
            )
            self.param_widget.update_field_from_motor(param.name, received_val)
            self._end_transaction(success=True)
        else:
            self._log(
                f"Write Mismatch! Sent: {target_val}, Echo: {received_val}",
                LOGLEVEL.ERROR, "system"
            )
            self._end_transaction(
                success=False,
                msg=f"Mismatch: Sent {target_val} != Got {received_val}"
            )
    
    def _end_transaction(self, success: bool, msg: str = ""):
        """End write transaction"""
        self.pending_write = None
        self.tx_timer.stop()
        
        if not success:
            QMessageBox.critical(self, "Write Failed", msg)
            self.status_label.setText("Write Failed")
            self.status_label.setStyleSheet("color: #ff5252; font-weight: bold;")
        else:
            self.status_label.setText("Write Successful")
            self.status_label.setStyleSheet("color: #00e676; font-weight: bold;")
        
        self.param_widget.set_all_enabled(True)
    
    def _handle_timeout(self):
        """Handle communication timeout"""
        if self.load_queue:
            # Timeout during read - skip parameter
            c_type, addr = self.load_queue[0]
            self._log(
                f"Timeout reading {c_type.name}:{addr}. Skipping.",
                LOGLEVEL.WARN, "system"
            )
            self.load_queue.popleft()
            self._process_load_queue()
        
        elif self.pending_write:
            # Timeout during write
            self._log(
                "Write operation timed out.",
                LOGLEVEL.ERROR, "system"
            )
            self._end_transaction(
                success=False,
                msg="Motor did not respond (Timeout)."
            )
    
    # ========================================================================
    # REST Command
    # ========================================================================
    
    def _send_rest_command(self):
        """Send REST command to return to SYSTEM_ON mode"""
        if not self.ros_worker.is_running:
            self._log(
                "ROS Worker not running",
                LOGLEVEL.ERROR, "system"
            )
            return
        
        self.robot_cmd_seq += 1
        
        robot_cmd = RobotCmdStamped()
        robot_cmd.header.seq = self.robot_cmd_seq
        robot_cmd.header.stamp = self.ros_worker.node.get_clock().now().to_msg()
        robot_cmd.header.frame_id = ''
        robot_cmd.request_robot_mode = int(ROBOTMODE.SYSTEM_ON)
        
        self.ros_worker.send_robot_command(robot_cmd)
        
        self._log("Sent REST (SYSTEM_ON)", LOGLEVEL.INFO, "orin")
        
        # Close window
        self.close()
    
    # ========================================================================
    # Cleanup
    # ========================================================================
    
    def closeEvent(self, event):
        """Handle window close event"""
        close_file_logger(self.file_logger, self.log_filepath, "Config Panel")
        
        # Stop timers
        self.tx_timer.stop()
        
        # Stop ROS worker
        self.ros_worker.stop_ros()
        
        event.accept()


# Entry point for standalone execution
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CorgiConfigPanel()
    window.show()
    sys.exit(app.exec_())
