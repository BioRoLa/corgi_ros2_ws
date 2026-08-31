#!/usr/bin/env python3
"""
Control Panel GUI for Corgi Robot
Refactored version with modular architecture (MVC pattern)
"""
import os
import sys
import logging
import yaml
import numpy as np
from datetime import datetime
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, 
    QGroupBox, QLineEdit, QGridLayout, QFrame, QFileDialog, QApplication
)
from PyQt5.QtCore import Qt, QTimer

from corgi_msgs.msg import RobotCmdStamped, TriggerStamped

# Import from corgi_ui package
from corgi_ui.core.constants import (
    ROBOTMODE, LOGLEVEL, COLORS, PATHS,
    LOGLEVEL_TO_LOGGING_MAP,
    setup_file_logger, log_to_file, close_file_logger
)
from corgi_ui.core.ros_worker import ControlPanelRosWorker
from corgi_ui.core.process_manager import ProcessManager
from corgi_ui.gui.widgets.log_widget import LogWidget

# GPIO Support (optional)
GPIO_defined = True
try:
    import Jetson.GPIO as GPIO
except ImportError:
    GPIO_defined = False

class CorgiControlPanel(QWidget):
    """
    Main Control Panel for Corgi Robot
    
    Features:
    - ROS Bridge control
    - Robot FSM management
    - Process management (IMU, CSV control, homing)
    - Data recording trigger
    - Real-time monitoring (power, motor states)
    - Logging system with file output
    """
    
    def __init__(self):
        super().__init__()
        
        # Initialize GPIO if available
        if GPIO_defined:
            self.trigger_pin = 11
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.output(self.trigger_pin, GPIO.HIGH)
        
        # Initialize instance variables
        self._robot_cmd_seq = 0
        self._pending_robot_mode = None
        self._last_confirmed_mode = None
        
        # Check if running in simulation mode
        self.use_sim_time = self._check_use_sim_time()
        
        # ROS Worker (thread-safe communication layer)
        self.ros_worker = ControlPanelRosWorker()
        
        # Process Manager (subprocess lifecycle management)
        self.process_manager = ProcessManager()
        
        # File logging setup
        self.log_dir = PATHS.DEFAULT_LOG_DIR
        os.makedirs(self.log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = f'control_panel_{timestamp}.log'
        self.log_filepath = os.path.join(self.log_dir, self.log_filename)
        self.file_logger = self._setup_file_logger()
        
        # Initialize UI
        # Note: Stylesheet loaded by run_control.py at application level
        # For standalone execution, use: python -m corgi_ui.gui.control_panel
        self._load_stylesheet()  # Fallback for standalone mode
        self._init_ui()
        
        # Connect ROS signals
        self._connect_ros_signals()

        # Start ROS worker in simulation mode (no bridge)
        if self.use_sim_time:
            if self.ros_worker.start_ros():
                self._log('ROS Worker Initialized (simulation mode)', LOGLEVEL.INFO, 'system')
        
        # Initialize state
        self.reset()

        # Update button states after init
        self._update_button_states()
        
        # Sim time tracking for clock jump detection
        self._last_sim_time_sec = 0.0

        # Auto-start data recorder
        self._start_data_recorder()

        # Auto-start the IMU alongside it: every capture should carry
        # IMU data, and the roll channel is what the hardware roll
        # prediction is scored on. Real hardware only -- the simulator
        # publishes its own IMU.
        self._start_imu()
    
    def _check_use_sim_time(self) -> bool:
        """Check if use_sim_time parameter is set to true"""
        try:
            import rclpy
            if not rclpy.ok():
                rclpy.init()
            
            # Create a temporary node with the same name as the panel node
            temp_node = rclpy.create_node(
                'corgi_control_panel',
                automatically_declare_parameters_from_overrides=True
            )
            
            if not temp_node.has_parameter('use_sim_time'):
                temp_node.declare_parameter('use_sim_time', False)
            
            result = temp_node.get_parameter('use_sim_time').value
            temp_node.destroy_node()
            
            return bool(result)
        except Exception as e:
            print(f"Warning: Could not check use_sim_time parameter: {e}")
            return False
    
    def _setup_file_logger(self) -> logging.Logger:
        """Setup file logger for persistent logging"""
        return setup_file_logger('CorgiControlPanel', self.log_filepath)
    
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
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(15, 15, 15, 15)
        
        # Top Bar (Power + E-Stop)
        main_layout.addLayout(self._create_top_bar())
        
        # Middle Area (Controls + Monitor)
        main_layout.addLayout(self._create_middle_area())
        
        # Log Area (using LogWidget)
        main_layout.addWidget(self._create_log_area())
        
        self.setLayout(main_layout)
        title = 'Corgi Control Panel'
        if self.use_sim_time:
            title += ' [SIMULATION MODE]'
        self.setWindowTitle(title)
        self.resize(1024, 768)
        
        # Setup timer for periodic updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._timer_update)
        self.timer.start(100)
        
        self.show()
    
    def _create_top_bar(self) -> QHBoxLayout:
        """Create top bar with power display and E-Stop button"""
        top_bar = QHBoxLayout()
        
        # Power summary badges
        power_box = QHBoxLayout()
        power_box.setSpacing(8)

        pb1_container = QVBoxLayout()
        pb1_container.setSpacing(3)
        pb1_title = QLabel("PB1")
        pb1_title.setObjectName("PowerBoardTitle")
        pb1_title.setAlignment(Qt.AlignCenter)
        pb1_frame = QFrame()
        pb1_frame.setObjectName("PowerBoardFrame")
        pb1_frame_layout = QHBoxLayout()
        pb1_frame_layout.setSpacing(10)
        pb1_frame_layout.setContentsMargins(8, 8, 8, 8)

        pb2_container = QVBoxLayout()
        pb2_container.setSpacing(3)
        pb2_title = QLabel("PB2")
        pb2_title.setObjectName("PowerBoardTitle")
        pb2_title.setAlignment(Qt.AlignCenter)
        pb2_frame = QFrame()
        pb2_frame.setObjectName("PowerBoardFrame")
        pb2_frame_layout = QHBoxLayout()
        pb2_frame_layout.setSpacing(10)
        pb2_frame_layout.setContentsMargins(8, 8, 8, 8)
        
        self.lbl_pb1_voltage = QLabel('--.- V')
        self.lbl_pb1_voltage.setObjectName('PowerBadge')
        self.lbl_pb1_soc = QLabel('-- %')
        self.lbl_pb1_soc.setObjectName('PowerBadge')
        self.lbl_pb1_current = QLabel('-.-- A')
        self.lbl_pb1_current.setObjectName('PowerBadge')
        self.lbl_pb1_power = QLabel('--.- W')
        self.lbl_pb1_power.setObjectName('PowerBadge')

        self.lbl_pb2_voltage = QLabel('--.- V')
        self.lbl_pb2_voltage.setObjectName('PowerBadge')
        self.lbl_pb2_soc = QLabel('-- %')
        self.lbl_pb2_soc.setObjectName('PowerBadge')
        self.lbl_pb2_current = QLabel('-.-- A')
        self.lbl_pb2_current.setObjectName('PowerBadge')
        self.lbl_pb2_power = QLabel('--.- W')
        self.lbl_pb2_power.setObjectName('PowerBadge')
        
        pb1_frame_layout.addWidget(self.lbl_pb1_voltage)
        pb1_frame_layout.addWidget(self.lbl_pb1_soc)
        pb1_frame_layout.addWidget(self.lbl_pb1_current)
        pb1_frame_layout.addWidget(self.lbl_pb1_power)
        pb1_frame.setLayout(pb1_frame_layout)
        pb1_container.addWidget(pb1_title)
        pb1_container.addWidget(pb1_frame)

        pb2_frame_layout.addWidget(self.lbl_pb2_voltage)
        pb2_frame_layout.addWidget(self.lbl_pb2_soc)
        pb2_frame_layout.addWidget(self.lbl_pb2_current)
        pb2_frame_layout.addWidget(self.lbl_pb2_power)
        pb2_frame.setLayout(pb2_frame_layout)
        pb2_container.addWidget(pb2_title)
        pb2_container.addWidget(pb2_frame)

        power_box.addLayout(pb1_container)
        power_box.addLayout(pb2_container)
        top_bar.addLayout(power_box)
        
        # E-Stop Button
        self.btn_estop = QPushButton('< ! >')
        self.btn_estop.setObjectName("EstopBtn")
        self.btn_estop.setMinimumWidth(100)
        self.btn_estop.setMinimumHeight(50)
        self.btn_estop.clicked.connect(self._on_estop_clicked)
        self.btn_estop.setEnabled(False)
        
        top_bar.addStretch(1)
        top_bar.addWidget(self.btn_estop)
        
        return top_bar
    
    def _create_middle_area(self) -> QHBoxLayout:
        """Create middle area with controls and monitor"""
        middle_layout = QHBoxLayout()
        
        # Left sidebar - FSM Control
        middle_layout.addLayout(self._create_fsm_sidebar(), 1)
        
        # Middle sidebar - CSV, Recorder, IMU
        middle_layout.addLayout(self._create_tools_sidebar(), 1)
        
        # Right monitor - Motor status
        middle_layout.addLayout(self._create_motor_monitor(), 3)
        
        return middle_layout
    
    def _create_fsm_sidebar(self) -> QVBoxLayout:
        """Create FSM control sidebar"""
        sidebar = QVBoxLayout()
        sidebar.setSpacing(15)
        
        # ROS Bridge button
        self.btn_ros_bridge = QPushButton('Run ROS Bridge')
        self.btn_ros_bridge.setCheckable(True)
        self.btn_ros_bridge.clicked.connect(self._on_ros_bridge_clicked)
        # Disable ROS bridge in simulation mode
        if hasattr(self, 'use_sim_time') and self.use_sim_time:
            self.btn_ros_bridge.setEnabled(False)
            self.btn_ros_bridge.setToolTip('ROS Bridge disabled in simulation mode')
        sidebar.addWidget(self.btn_ros_bridge)
        
        # FSM Group
        grp_fsm = QGroupBox("FSM")
        grp_fsm_layout = QVBoxLayout()
        
        # Current Mode Display
        mode_container = QFrame()
        mode_container.setStyleSheet(
            "background-color: #222; border-radius: 5px; margin-bottom: 5px;"
        )
        mode_h_layout = QHBoxLayout(mode_container)
        mode_h_layout.setContentsMargins(5, 5, 5, 5)
        
        lbl_mode_title = QLabel("Current mode:")
        lbl_mode_title.setStyleSheet("color: #888; font-size: 12px;")
        
        self.label_robot_mode_value = QLabel("---")
        self.label_robot_mode_value.setObjectName("StatusLabel")
        self.label_robot_mode_value.setAlignment(Qt.AlignCenter)
        self.label_robot_mode_value.setStyleSheet(
            "color: #bdbdbd; font-weight: bold; font-size: 20px;"
        )
        
        mode_h_layout.addWidget(lbl_mode_title)
        mode_h_layout.addWidget(self.label_robot_mode_value)
        grp_fsm_layout.addWidget(mode_container)
        
        # FSM Buttons
        self.btn_systemon = QPushButton('Set to System ON')
        self.btn_systemon.setObjectName("SystemOnBtn")
        self.btn_systemon.setCheckable(True)
        self.btn_systemon.clicked.connect(lambda: self._request_robot_mode(ROBOTMODE.SYSTEM_ON))
        self.btn_systemon.setEnabled(False)
        
        self.btn_idle = QPushButton('Set to IDLE')
        self.btn_idle.setCheckable(True)
        self.btn_idle.clicked.connect(lambda: self._request_robot_mode(ROBOTMODE.IDLE))
        self.btn_idle.setEnabled(False)
        
        self.btn_standby = QPushButton('Set STANDBY')
        self.btn_standby.setCheckable(True)
        self.btn_standby.clicked.connect(lambda: self._request_robot_mode(ROBOTMODE.STANDBY))
        self.btn_standby.setEnabled(False)
        
        self.btn_motorconfig = QPushButton('Set to CONFIG')
        self.btn_motorconfig.setObjectName("ConfigBtn")
        self.btn_motorconfig.setCheckable(True)
        self.btn_motorconfig.clicked.connect(lambda: self._request_robot_mode(ROBOTMODE.MOTORCONFIG))
        self.btn_motorconfig.setEnabled(False)
        
        grp_fsm_layout.addWidget(self.btn_systemon)
        grp_fsm_layout.addWidget(self.btn_idle)
        grp_fsm_layout.addWidget(self.btn_standby)
        grp_fsm_layout.addWidget(self.btn_motorconfig)
        grp_fsm.setLayout(grp_fsm_layout)
        sidebar.addWidget(grp_fsm)
        
        # Homing button
        self.btn_home = QPushButton('Homing')
        self.btn_home.clicked.connect(self._on_home_clicked)
        self.btn_home.setEnabled(False)
        sidebar.addWidget(self.btn_home)
        
        sidebar.addStretch(1)
        return sidebar
    
    def _create_tools_sidebar(self) -> QVBoxLayout:
        """Create tools sidebar (CSV, Recorder, IMU)"""
        sidebar = QVBoxLayout()
        sidebar.setSpacing(15)
        
        # CSV Control Group
        grp_csv = QGroupBox("CSV Control")
        grp_csv_layout = QVBoxLayout()
        
        self.label_csv = QLabel('Input File Name (.csv):')
        self.label_csv.setStyleSheet('color: #aaa; font-size: 12px;')
        
        self.edit_csv = QLineEdit()
        self.edit_csv.setPlaceholderText("Select or enter CSV file path")
        
        csv_btn_layout = QHBoxLayout()
        self.btn_csv_select = QPushButton('Select')
        self.btn_csv_select.clicked.connect(self._on_select_csv_clicked)
        self.btn_csv_select.setEnabled(False)
        
        self.btn_csv_run = QPushButton('Run')
        self.btn_csv_run.setCheckable(True)
        self.btn_csv_run.clicked.connect(self._on_csv_run_clicked)
        self.btn_csv_run.setEnabled(False)
        
        csv_btn_layout.addWidget(self.btn_csv_select)
        csv_btn_layout.addWidget(self.btn_csv_run)
        
        grp_csv_layout.addWidget(self.label_csv)
        grp_csv_layout.addWidget(self.edit_csv)
        grp_csv_layout.addLayout(csv_btn_layout)
        grp_csv.setLayout(grp_csv_layout)
        sidebar.addWidget(grp_csv)
        
        # Recorder Group
        grp_rec = QGroupBox("Recorder")
        grp_rec_layout = QVBoxLayout()
        
        self.edit_output = QLineEdit()
        self.edit_output.setPlaceholderText("Text File Name (.csv)")
        self.edit_output.returnPressed.connect(self._on_recording_input_entered)
        
        self.btn_trigger = QPushButton('Start Trigger')
        self.btn_trigger.setCheckable(True)
        self.btn_trigger.clicked.connect(self._on_trigger_clicked)
        self.btn_trigger.setEnabled(False)
        
        grp_rec_layout.addWidget(self.edit_output)
        grp_rec_layout.addWidget(self.btn_trigger)
        grp_rec.setLayout(grp_rec_layout)
        sidebar.addWidget(grp_rec)
        
        # IMU button
        self.btn_imu = QPushButton('IMU')
        self.btn_imu.setCheckable(True)
        self.btn_imu.clicked.connect(self._on_imu_clicked)
        self.btn_imu.setEnabled(False)
        sidebar.addWidget(self.btn_imu)
        
        sidebar.addStretch(1)
        return sidebar
    
    def _create_motor_monitor(self) -> QVBoxLayout:
        """Create motor status monitor"""
        monitor_layout = QVBoxLayout()

        grid_motors = QGridLayout()
        self.motor_labels = {}

        # Define legs: (name, row, col, module state fields)
        legs = [
            ('LF', 0, 0, [('theta', 'θ'), ('beta', 'β'), ('gamma', 'γ')]),
            ('RF', 0, 1, [('theta', 'θ'), ('beta', 'β'), ('gamma', 'γ')]),
            ('LH', 1, 0, [('theta', 'θ'), ('beta', 'β'), ('gamma', 'γ')]),
            ('RH', 1, 1, [('theta', 'θ'), ('beta', 'β'), ('gamma', 'γ')])
        ]

        for leg_name, r, c, fields in legs:
            leg_group = QGroupBox(leg_name)
            leg_layout = QVBoxLayout()

            for field_name, display_name in fields:
                label_key = f"{leg_name}_{field_name}"
                lbl = QLabel(f"{display_name}: --")
                lbl.setObjectName("MotorLabel")
                leg_layout.addWidget(lbl)
                self.motor_labels[label_key] = lbl

            leg_group.setLayout(leg_layout)
            grid_motors.addWidget(leg_group, r, c)

        monitor_layout.addLayout(grid_motors)
        monitor_layout.addWidget(self._create_motor_config_display())
        monitor_layout.addStretch(1)

        return monitor_layout

    def _load_motor_config(self) -> dict:
        """Load motor_config.yaml from the corgi_driver_pkg directory.

        Uses importlib to locate the installed (or symlinked) package so the
        YAML is read directly from source — no rebuild needed.
        """
        import importlib.util
        # Primary: find via the installed Python package (works with --symlink-install)
        try:
            spec = importlib.util.find_spec('corgi_driver_pkg')
            if spec and spec.origin:
                pkg_dir = os.path.dirname(spec.origin)
                yaml_path = os.path.join(pkg_dir, 'motor_config.yaml')
                with open(yaml_path, 'r') as f:
                    return yaml.safe_load(f)
        except Exception as e:
            print(f"Warning: importlib path failed: {e}")

        # Fallback: ament share directory (requires colcon build)
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('corgi_sim')
            yaml_path = os.path.join(share, 'motor_config.yaml')
            with open(yaml_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"Warning: could not load motor_config.yaml from share: {e}")

        return {}

    def _create_motor_config_display(self) -> QGroupBox:
        """Create a read-only display of the motor direction config (motor_config.yaml)."""
        cfg = self._load_motor_config()

        grp = QGroupBox("Motor Direction Config  (motor_config.yaml)")
        outer = QVBoxLayout()

        # Reload button
        self.btn_reload_config = QPushButton("↺  Reload Config")
        self.btn_reload_config.setMaximumWidth(160)
        self.btn_reload_config.clicked.connect(self._reload_motor_config_display)
        outer.addWidget(self.btn_reload_config, alignment=Qt.AlignLeft)

        # Grid: one column per module (A B C D), rows = each direction field
        self._config_grid = QGridLayout()
        self._config_grid.setSpacing(6)

        # Map module id → friendly leg label
        MODULE_LABEL = {'A': 'A (FL)', 'B': 'B (FR)', 'C': 'C (RR)', 'D': 'D (RL)'}
        FIELDS = [
            ('joint_dir',  'theta',        'θ  joint dir'),
            ('joint_dir',  'beta',         'β  joint dir'),
            ('joint_dir',  'g_joint_beta', 'G-joint β dir'),
            ('motor_dir',  'L',            'Motor L dir'),
            ('motor_dir',  'R',            'Motor R dir'),
            ('motor_dir',  'ABAD',         'ABAD dir'),
        ]

        # Header row
        self._config_grid.addWidget(QLabel(""), 0, 0)
        for col, mod_id in enumerate(('A', 'B', 'C', 'D'), start=1):
            hdr = QLabel(MODULE_LABEL[mod_id])
            hdr.setAlignment(Qt.AlignCenter)
            hdr.setStyleSheet("font-weight: bold; color: #ccc;")
            self._config_grid.addWidget(hdr, 0, col)

        # Data rows
        self._config_value_labels = {}
        for row, (section, key, label_text) in enumerate(FIELDS, start=1):
            row_lbl = QLabel(label_text)
            row_lbl.setStyleSheet("color: #aaa;")
            self._config_grid.addWidget(row_lbl, row, 0)

            for col, mod_id in enumerate(('A', 'B', 'C', 'D'), start=1):
                val = cfg.get(mod_id, {}).get(section, {}).get(key, '?')
                lbl = QLabel(self._fmt_dir(val))
                lbl.setAlignment(Qt.AlignCenter)
                lbl.setStyleSheet(self._dir_style(val))
                self._config_grid.addWidget(lbl, row, col)
                self._config_value_labels[(mod_id, section, key)] = lbl

        outer.addLayout(self._config_grid)
        grp.setLayout(outer)
        return grp

    @staticmethod
    def _fmt_dir(val) -> str:
        """Format a direction value as +1 / -1 / ?"""
        try:
            v = float(val)
            return '+1' if v > 0 else '-1'
        except (TypeError, ValueError):
            return '?'

    @staticmethod
    def _dir_style(val) -> str:
        """Return colour style based on direction value."""
        try:
            v = float(val)
            colour = '#4caf50' if v > 0 else '#ef5350'   # green / red
        except (TypeError, ValueError):
            colour = '#888'
        return f"color: {colour}; font-weight: bold; font-size: 14px;"

    def _reload_motor_config_display(self):
        """Reload motor_config.yaml and refresh every label in the grid."""
        cfg = self._load_motor_config()
        FIELDS = [
            ('joint_dir',  'theta',        ),
            ('joint_dir',  'beta',         ),
            ('joint_dir',  'g_joint_beta', ),
            ('motor_dir',  'L',            ),
            ('motor_dir',  'R',            ),
            ('motor_dir',  'ABAD',         ),
        ]
        for mod_id in ('A', 'B', 'C', 'D'):
            for section, key in FIELDS:
                lbl = self._config_value_labels.get((mod_id, section, key))
                if lbl is None:
                    continue
                val = cfg.get(mod_id, {}).get(section, {}).get(key, '?')
                lbl.setText(self._fmt_dir(val))
                lbl.setStyleSheet(self._dir_style(val))
        self._log("Motor config reloaded from YAML", LOGLEVEL.INFO, "system")

    
    def _create_log_area(self) -> QGroupBox:
        """Create log area using LogWidget"""
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout()
        
        # Use the reusable LogWidget
        self.log_widget = LogWidget(default_level=LOGLEVEL.DEBUG)
        self.log_widget.set_max_height(400)
        self.log_widget.log_level_changed.connect(self._on_log_level_changed)
        
        log_layout.addWidget(self.log_widget)
        log_group.setLayout(log_layout)
        
        return log_group
    
    def _connect_ros_signals(self):
        """Connect ROS worker signals to handlers"""
        self.ros_worker.power_state_updated.connect(self._handle_power_state_update)
        self.ros_worker.robot_state_updated.connect(self._handle_robot_state_update)
        self.ros_worker.motor_state_updated.connect(self._handle_motor_state_update)
        self.ros_worker.log_updated.connect(self._handle_log_update)
    
    def _start_data_recorder(self):
        """Auto-start data recorder when panel opens"""
        if self.process_manager.is_running('data_recorder'):
            self._log('Data recorder already running', LOGLEVEL.INFO, 'system')
            return
        
        # Build command with use_sim_time parameter if needed
        cmd = ['ros2', 'run', 'corgi_data_recorder', 'corgi_data_recorder']
        if self.use_sim_time:
            cmd.extend(['--ros-args', '-p', 'use_sim_time:=true'])
        
        success = self.process_manager.start_process(
            'data_recorder',
            cmd,
            capture_output=False
        )
        
        if success:
            mode_str = 'simulation' if self.use_sim_time else 'real'
            self._log(f'Data recorder started ({mode_str} mode)', LOGLEVEL.INFO, 'system')
        else:
            self._log('Failed to start data recorder', LOGLEVEL.ERROR, 'system')

    def _start_imu(self):
        """Auto-start the IMU node so recordings always contain IMU data.

        Mirrors _start_data_recorder. Skipped under use_sim_time (the
        simulator publishes its own IMU). Reflects the state on the IMU
        button so the operator sees it is live; the button still stops
        and restarts it by hand.
        """
        if self.use_sim_time:
            return

        if self.process_manager.is_running('imu'):
            self.btn_imu.setChecked(True)
            self.btn_imu.setText('Stop IMU')
            self._log('IMU already running', LOGLEVEL.INFO, 'system')
            return

        success = self.process_manager.start_process(
            'imu',
            ['ros2', 'run', 'corgi_imu', 'imu_node'],
            capture_output=False
        )

        if success:
            self.btn_imu.setChecked(True)
            self.btn_imu.setText('Stop IMU')
            self._log('IMU started (auto, with recorder)', LOGLEVEL.INFO, 'system')
        else:
            self.btn_imu.setChecked(False)
            self.btn_imu.setText('IMU')
            self._log('Failed to auto-start IMU -- start it by hand before '
                      'recording, or the IMU columns will be empty',
                      LOGLEVEL.ERROR, 'system')
    
    def _log(self, message: str, level=LOGLEVEL.INFO, source: str = "system"):
        """Unified logging method that writes to both GUI and file"""
        
        # Write to GUI log widget
        self.log_widget.add_log(message, level, source)
        
        # Write to file logger
        log_to_file(self.file_logger, level, source, message)

    def _map_external_log_level(self, level: int) -> LOGLEVEL:
        """Map external ROS log levels to panel LOGLEVEL."""
        if level in LOGLEVEL._value2member_map_:
            return LOGLEVEL(level)

        if level >= 50:
            return LOGLEVEL.FATAL
        if level >= 40:
            return LOGLEVEL.ERROR
        if level >= 30:
            return LOGLEVEL.WARN
        if level >= 20:
            return LOGLEVEL.INFO
        return LOGLEVEL.DEBUG
    
    # ========================================================================
    # Event Handlers - ROS Bridge
    # ========================================================================
    
    def _on_ros_bridge_clicked(self):
        """Handle ROS Bridge button click"""
        # Disable ROS bridge in simulation mode
        if self.use_sim_time and self.btn_ros_bridge.isChecked():
            self.log_widget.add_log(
                'ROS Bridge disabled in simulation mode',
                LOGLEVEL.WARN, 'system'
            )
            self.btn_ros_bridge.setChecked(False)
            return
        
        if self.btn_ros_bridge.isChecked():
            # Start ROS Bridge
            self.btn_ros_bridge.setText('Stop ROS Bridge')
            
            if self.process_manager.is_running('ros_bridge'):
                self.log_widget.add_log(
                    'Bridge already running; skip start',
                    LOGLEVEL.WARN, 'system'
                )
                return
            
            # Start ROS worker
            if self.ros_worker.start_ros():
                self.log_widget.add_log(
                    'ROS Worker Initialized',
                    LOGLEVEL.INFO, 'system'
                )
            
            # Start ROS bridge process
            success = self.process_manager.start_process(
                'ros_bridge',
                ['ros2', 'run', 'corgi_ros_bridge', 'corgi_ros_bridge'],
                capture_output=False
            )
            
            if success:
                self.log_widget.add_log(
                    'ROS Bridge Started',
                    LOGLEVEL.INFO, 'system'
                )
            else:
                self.log_widget.add_log(
                    'Failed to start ROS Bridge',
                    LOGLEVEL.ERROR, 'system'
                )
                self.btn_ros_bridge.setChecked(False)
                self.btn_ros_bridge.setText('Run ROS Bridge')
        else:
            # Stop ROS Bridge
            self.btn_ros_bridge.setText('Run ROS Bridge')
            
            if self.process_manager.stop_process('ros_bridge', timeout=3.0):
                self.log_widget.add_log(
                    'ROS Bridge Stopped',
                    LOGLEVEL.WARN, 'system'
                )
            
            self.ros_worker.stop_ros()
        
        self._update_button_states()
    
    # ========================================================================
    # Event Handlers - Robot FSM
    # ========================================================================
    
    def _request_robot_mode(self, mode: ROBOTMODE):
        """Request robot mode change"""
        if not self.ros_worker.is_running:
            return
        
        robot_cmd = RobotCmdStamped()
        robot_cmd.header.seq = self._robot_cmd_seq + 1
        robot_cmd.header.stamp = self.ros_worker.node.get_clock().now().to_msg()
        robot_cmd.header.frame_id = ''
        robot_cmd.request_robot_mode = int(mode)
        
        self.ros_worker.send_robot_command(robot_cmd)
        
        self._robot_cmd_seq += 1
        self._pending_robot_mode = int(mode)
        
        self.log_widget.add_log(
            f'Sent Robot Mode Command: {mode.name} ({mode.value}), seq={self._robot_cmd_seq}',
            LOGLEVEL.INFO, 'orin'
        )
        
        self._update_button_states()
    
    def _on_estop_clicked(self):
        """Handle E-Stop button click"""
        if not self.ros_worker.is_running:
            return
        
        self.log_widget.add_log('EMERGENCY STOP ACTIVATED!', LOGLEVEL.FATAL, 'orin')
        
        robot_cmd = RobotCmdStamped()
        robot_cmd.header.seq = self._robot_cmd_seq + 1
        robot_cmd.header.stamp = self.ros_worker.node.get_clock().now().to_msg()
        robot_cmd.header.frame_id = ''
        
        # E-stop logic based on current robot state
        if hasattr(self, 'robot_state') and hasattr(self.robot_state, 'robot_mode'):
            current = self.robot_state.robot_mode
        else:
            current = -1
        
        if current == ROBOTMODE.STANDBY:
            robot_cmd.request_robot_mode = int(ROBOTMODE.IDLE)
            self._pending_robot_mode = int(ROBOTMODE.IDLE)
            self.log_widget.add_log('E-Stop: STANDBY -> IDLE', LOGLEVEL.WARN, 'orin')
        else:
            robot_cmd.request_robot_mode = int(ROBOTMODE.SYSTEM_ON)
            self._pending_robot_mode = int(ROBOTMODE.SYSTEM_ON)
            self.log_widget.add_log('E-Stop: -> SYSTEM_ON', LOGLEVEL.WARN, 'orin')
        
        self.ros_worker.send_robot_command(robot_cmd)
        self._robot_cmd_seq += 1
        self._update_button_states()
    
    # ========================================================================
    # Event Handlers - Tools
    # ========================================================================
    
    def _on_imu_clicked(self):
        """Handle IMU button click"""
        if self.btn_imu.isChecked():
            self.btn_imu.setText('Stop IMU')
            
            if self.process_manager.is_running('imu'):
                self.log_widget.add_log('IMU already running; skip start', LOGLEVEL.WARN, 'system')
                return
            
            success = self.process_manager.start_process(
                'imu',
                ['ros2', 'run', 'corgi_imu', 'imu_node'],
                capture_output=False
            )
            
            if success:
                self.log_widget.add_log('IMU Started', LOGLEVEL.INFO, 'system')
            else:
                self.log_widget.add_log('Failed to start IMU', LOGLEVEL.ERROR, 'system')
                self.btn_imu.setChecked(False)
                self.btn_imu.setText('IMU')
        else:
            self.btn_imu.setText('IMU')
            
            if self.process_manager.stop_process('imu', timeout=3.0):
                self.log_widget.add_log('IMU Stopped', LOGLEVEL.WARN, 'system')
    
    def _on_home_clicked(self):
        """Handle Home button click"""
        self.btn_home.setEnabled(False)
        self.btn_home.setText('Homing...')
        
        success = self.process_manager.start_process(
            'homing',
            ['ros2', 'run', 'corgi_homing', 'homing'],
            capture_output=False
        )
        
        if success:
            self.log_widget.add_log('Homing Started', LOGLEVEL.INFO, 'system')
        else:
            self.log_widget.add_log('Failed to start homing', LOGLEVEL.ERROR, 'system')
            self.btn_home.setEnabled(True)
            self.btn_home.setText('Homing')
    
    def _on_select_csv_clicked(self):
        """Handle CSV file selection"""
        file_name, _ = QFileDialog.getOpenFileName(
            self,
            "Select CSV File",
            PATHS.DEFAULT_CSV_DIR,
            "CSV Files (*.csv);;All Files (*)"
        )
        
        if file_name:
            self.edit_csv.setText(file_name)
            self.log_widget.add_log(f'Selected CSV file: {file_name}', LOGLEVEL.INFO, 'orin')
    
    def _on_csv_run_clicked(self):
        """Handle CSV control run button click"""
        if self.btn_csv_run.isChecked():
            self.btn_csv_run.setText('Stop')
            
            csv_file = self.edit_csv.text().strip()
            if not csv_file:
                self.log_widget.add_log('No CSV file selected', LOGLEVEL.WARN, 'system')
                self.btn_csv_run.setChecked(False)
                self.btn_csv_run.setText('Run')
                return
            
            # Determine argument to pass (full path if provided)
            csv_filename = os.path.basename(csv_file)
            if csv_filename.endswith('.csv'):
                csv_filename = csv_filename[:-4]
            csv_arg = csv_file
            if not os.path.isabs(csv_file) and '/' not in csv_file:
                csv_arg = csv_filename
            
            if self.process_manager.is_running('csv_control'):
                self.log_widget.add_log('CSV Control already running; skip start', LOGLEVEL.WARN, 'system')
                return
            
            # Build command with use_sim_time parameter if needed
            cmd = ['ros2', 'run', 'corgi_csv_control', 'corgi_csv_control', csv_arg]
            if self.use_sim_time:
                cmd = ['ros2', 'run', 'corgi_csv_control', 'corgi_csv_control', csv_arg, '--ros-args', '-p', 'use_sim_time:=true']
            
            success = self.process_manager.start_process(
                'csv_control',
                cmd,
                capture_output=False
            )
            
            if success:
                mode_str = ' (sim)' if self.use_sim_time else ''
                self.log_widget.add_log(f'CSV Control Started with file: {csv_filename}{mode_str}', LOGLEVEL.INFO, 'system')
            else:
                self.log_widget.add_log('Failed to start CSV Control', LOGLEVEL.ERROR, 'system')
                self.btn_csv_run.setChecked(False)
                self.btn_csv_run.setText('Run')
        else:
            self.btn_csv_run.setText('Run')
            
            if self.process_manager.stop_process('csv_control', timeout=3.0):
                self.log_widget.add_log('CSV Control Stopped', LOGLEVEL.WARN, 'system')
    
    def _on_recording_input_entered(self):
        """Handle recording filename input (Enter key pressed)"""
        filename = self.edit_output.text().strip()
        if filename and not self.btn_trigger.isChecked():
            self.btn_trigger.setChecked(True)
            self._on_trigger_clicked()
        elif not filename:
            self.log_widget.add_log('No filename entered', LOGLEVEL.WARN, 'orin')
        else:
            self.log_widget.add_log('Recording already in progress', LOGLEVEL.WARN, 'orin')
    
    def _on_trigger_clicked(self):
        """Handle trigger button click"""
        if not self.ros_worker.is_running:
            return
        
        trigger_cmd = TriggerStamped()
        trigger_cmd.header.stamp = self.ros_worker.node.get_clock().now().to_msg()
        trigger_cmd.enable = self.btn_trigger.isChecked()
        trigger_cmd.output_filename = self.edit_output.text().strip()
        
        self.ros_worker.send_trigger(trigger_cmd)
        
        # Control GPIO if available
        if GPIO_defined:
            GPIO.output(
                self.trigger_pin,
                GPIO.LOW if self.btn_trigger.isChecked() else GPIO.HIGH
            )
        
        if self.btn_trigger.isChecked():
            filename = trigger_cmd.output_filename if trigger_cmd.output_filename else "no filename"
            self.log_widget.add_log(f'Trigger enabled: {filename}', LOGLEVEL.INFO, 'orin')
        else:
            self.log_widget.add_log('Trigger stopped', LOGLEVEL.INFO, 'orin')
    
    # ========================================================================
    # ROS Message Handlers
    # ========================================================================
    
    def _handle_power_state_update(self, state):
        """Handle power state update from ROS"""
        self.power_state = state
        
        pb1_voltage = self._get_float_field(state, 'pb1_v_0')
        pb1_current = self._sum_powerboard_current(state, 'pb1')
        pb2_voltage = self._get_float_field(state, 'pb2_v_0')
        pb2_current = self._sum_powerboard_current(state, 'pb2')

        self._update_power_badges(
            pb1_voltage,
            pb1_current,
            self.lbl_pb1_voltage,
            self.lbl_pb1_soc,
            self.lbl_pb1_current,
            self.lbl_pb1_power,
        )
        self._update_power_badges(
            pb2_voltage,
            pb2_current,
            self.lbl_pb2_voltage,
            self.lbl_pb2_soc,
            self.lbl_pb2_current,
            self.lbl_pb2_power,
        )
        
        self._update_button_states()
    
    def _handle_robot_state_update(self, state):
        """Handle robot state update from ROS"""
        self.robot_state = state
        current_mode = int(state.robot_mode)
        
        # Check if pending mode transition completed
        if self._pending_robot_mode is not None and current_mode == int(self._pending_robot_mode):
            self.log_widget.add_log(
                f'Robot mode reached: {ROBOTMODE(self._pending_robot_mode).name} ({self._pending_robot_mode})',
                LOGLEVEL.INFO, 'system'
            )
            self._pending_robot_mode = None
            
            # Launch config panel if entering CONFIG mode
            if current_mode == ROBOTMODE.MOTORCONFIG:
                self._launch_config_panel()
        
        self._last_confirmed_mode = current_mode
        
        # Update mode display
        try:
            mode_enum = ROBOTMODE(state.robot_mode)
            mode_text = mode_enum.name
        except ValueError:
            mode_text = "---"
        
        self.label_robot_mode_value.setText(mode_text)
        
        # Update mode display color
        if state.robot_mode == ROBOTMODE.SYSTEM_ON:
            color = COLORS.STATUS_SUCCESS
        elif state.robot_mode == ROBOTMODE.IDLE:
            color = "#2979ff"
        elif state.robot_mode == ROBOTMODE.MOTORCONFIG:
            color = COLORS.STATUS_WARNING
        else:
            color = COLORS.STATUS_NEUTRAL
        
        self.label_robot_mode_value.setStyleSheet(
            f"color: {color}; font-weight: bold; font-size: 20px;"
        )
        
        # Update button checked states to reflect current mode
        self.btn_systemon.setChecked(state.robot_mode == ROBOTMODE.SYSTEM_ON)
        self.btn_idle.setChecked(state.robot_mode == ROBOTMODE.IDLE)
        self.btn_standby.setChecked(state.robot_mode == ROBOTMODE.STANDBY)
        self.btn_motorconfig.setChecked(state.robot_mode == ROBOTMODE.MOTORCONFIG)
        
        self._update_button_states()
    
    def _handle_motor_state_update(self, state):
        """Handle motor state update from ROS"""
        self.motor_state = state
        
        if not hasattr(state, 'module_a'):
            return
        
        modules = [
            ('LF', state.module_a),
            ('RF', state.module_b),
            ('LH', state.module_d),
            ('RH', state.module_c),
        ]
        state_fields = [
            ('theta', 'θ'),
            ('beta', 'β'),
            ('gamma', 'γ'),
        ]
        
        for leg_name, module in modules:
            if module is None:
                continue
            
            for field_name, display_name in state_fields:
                key = f"{leg_name}_{field_name}"
                if key not in self.motor_labels:
                    continue
                
                value = np.degrees(self._get_float_field(module, field_name))
                self.motor_labels[key].setText(f"{display_name}: {value:.1f}°")
                self.motor_labels[key].setStyleSheet("color: #aaa;")
    
    def _handle_log_update(self, log_msg):
        """Handle log message from ROS"""
        raw_level = int(log_msg.level) if hasattr(log_msg, 'level') else int(LOGLEVEL.INFO)
        level = self._map_external_log_level(raw_level)
        node_name = log_msg.name if hasattr(log_msg, 'name') else 'unknown'
        message = log_msg.msg if hasattr(log_msg, 'msg') else ''

        if not message:
            return
        
        # Always log to file
        file_log_level = LOGLEVEL_TO_LOGGING_MAP.get(level, logging.INFO)
        self.file_logger.log(file_log_level, f'[{node_name}] {message}')
        
        # Display in log widget
        self.log_widget.add_log(message, LOGLEVEL(level), node_name)
        
        # Handle special messages - check for homing completion
        if 'homing' in node_name.lower():
            if 'completed' in message.lower() or 'complete' in message.lower():
                self.log_widget.add_log(
                    'Homing operation completed successfully',
                    LOGLEVEL.INFO, 'system'
                )
                self._on_homing_completed()
        
        # Handle error recovery
        if level in [LOGLEVEL.ERROR, LOGLEVEL.FATAL]:
            if self._pending_robot_mode is not None:
                reverted_mode = ROBOTMODE(self._pending_robot_mode).name if self._pending_robot_mode in ROBOTMODE.__members__.values() else str(self._pending_robot_mode)
                self.log_widget.add_log(
                    f'Command to {reverted_mode} failed - system reverted',
                    LOGLEVEL.WARN, 'system'
                )
                self._pending_robot_mode = None
                self._update_button_states()
    
    # ========================================================================
    # Helper Methods
    # ========================================================================

    def _get_float_field(self, state, field_name: str) -> float:
        """Read a numeric field from a ROS message without breaking UI updates."""
        try:
            return float(getattr(state, field_name, 0.0))
        except Exception:
            return 0.0

    def _sum_powerboard_current(self, state, board_prefix: str) -> float:
        """Sum i_0 through i_7 for a powerboard."""
        return sum(
            self._get_float_field(state, f'{board_prefix}_i_{index}')
            for index in range(8)
        )

    def _update_power_badges(
        self,
        voltage: float,
        current: float,
        voltage_label: QLabel,
        soc_label: QLabel,
        current_label: QLabel,
        power_label: QLabel,
    ):
        """Update one powerboard summary row using v_0 and summed current."""
        soc = self._calculate_soc(voltage)
        power = voltage * current

        voltage_label.setText(f"{voltage:.1f} V")
        soc_label.setText(f"{soc:.0f} %")
        current_label.setText(f"{current:.2f} A")
        power_label.setText(f"{power:.1f} W")
    
    def _calculate_soc(self, v_total: float) -> float:
        """Calculate state of charge from voltage"""
        V_MIN = 42.0  # 3.5V * 12
        V_MAX = 50.4  # 4.2V * 12
        
        if V_MAX <= V_MIN:
            return 0.0
        
        soc = (v_total - V_MIN) / (V_MAX - V_MIN) * 100.0
        return max(0.0, min(100.0, soc))
    
    def _update_button_states(self):
        """Update button enabled/disabled states based on current state"""
        bridge_on = self.btn_ros_bridge.isChecked()
        
        # In simulation mode, enable basic buttons even without bridge
        # In real mode, require bridge for all operations
        enable_basic = bridge_on or self.use_sim_time
        
        # Basic buttons
        self.btn_estop.setEnabled(bridge_on)  # E-stop always requires bridge (safety critical)
        self.btn_imu.setEnabled(enable_basic)
        self.btn_trigger.setEnabled(enable_basic)
        self.btn_csv_select.setEnabled(enable_basic)
        self.btn_csv_run.setEnabled(enable_basic)
        
        # Set zero button (only in STANDBY mode)
        if hasattr(self, 'robot_state') and hasattr(self.robot_state, 'robot_mode'):
            current = self.robot_state.robot_mode
        else:
            current = -1
        
        self.btn_home.setEnabled(bridge_on and current == ROBOTMODE.STANDBY)
        
        # FSM buttons - disable in simulation mode or when bridge is off
        if not bridge_on or self.use_sim_time:
            self.btn_systemon.setEnabled(False)
            self.btn_idle.setEnabled(False)
            self.btn_standby.setEnabled(False)
            self.btn_motorconfig.setEnabled(False)
        else:
            if current == -1:
                # No state yet
                self.btn_systemon.setEnabled(False)
                self.btn_idle.setEnabled(True)
                self.btn_standby.setEnabled(False)
                self.btn_motorconfig.setEnabled(True)
            else:
                # FSM Transition Logic:
                # SYSTEM_ON (0) <=> IDLE (2)
                # SYSTEM_ON (0) <=> MOTORCONFIG (4)
                # IDLE (2) <=> STANDBY (3)
                # IDLE (2) -> MOTORCONFIG (4)
                
                self.btn_systemon.setEnabled(
                    current in [ROBOTMODE.INIT, ROBOTMODE.IDLE, ROBOTMODE.MOTORCONFIG]
                )
                self.btn_idle.setEnabled(
                    current in [ROBOTMODE.SYSTEM_ON, ROBOTMODE.STANDBY]
                )
                self.btn_standby.setEnabled(current == ROBOTMODE.IDLE)
                self.btn_motorconfig.setEnabled(
                    current in [ROBOTMODE.SYSTEM_ON, ROBOTMODE.IDLE, ROBOTMODE.MOTORCONFIG]
                )
    
    def _on_homing_completed(self):
        """Handle homing completion"""
        if self.process_manager.is_running('homing'):
            self.process_manager.stop_process('homing', timeout=1.0)
        
        self.btn_home.setEnabled(True)
        self.btn_home.setText('Homing')
        self.log_widget.add_log('Motor home position set successfully', LOGLEVEL.INFO, 'system')
    
    def _launch_config_panel(self):
        """Launch configuration panel"""
        if self.process_manager.is_running('config_panel'):
            self.log_widget.add_log('Config Panel already running', LOGLEVEL.WARN, 'system')
            return
        
        try:
            success = self.process_manager.start_process(
                'config_panel',
                ['ros2', 'run', 'corgi_panel', 'corgi_config_panel'],
                capture_output=False
            )
            
            if success:
                self.log_widget.add_log('Config Panel launched', LOGLEVEL.INFO, 'system')
            else:
                self.log_widget.add_log('Failed to launch Config Panel', LOGLEVEL.ERROR, 'system')
        except Exception as e:
            self.log_widget.add_log(f'Failed to launch Config Panel: {e}', LOGLEVEL.ERROR, 'system')
    
    def _on_log_level_changed(self, index: int):
        """Handle log level filter change"""
        level_name = ['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'][index]
        self.log_widget.add_log(
            f'Log filter set to {level_name} and above',
            LOGLEVEL.INFO, 'system'
        )
    
    def _timer_update(self):
        """Periodic timer update"""
        # Check if homing process has completed
        if self.btn_home.text() == 'Homing...':
            if not self.process_manager.is_running('homing'):
                # Process finished but callback wasn't triggered
                self._on_homing_completed()

        # Detect sim-time clock jump (Webots reset) and restart data recorder
        if self.use_sim_time and self.ros_worker.is_running:
            try:
                current = self.ros_worker.node.get_clock().now().nanoseconds / 1e9
                if self._last_sim_time_sec > 1.0 and current < self._last_sim_time_sec - 1.0:
                    self._log(
                        f'Clock jumped backward ({self._last_sim_time_sec:.1f}s -> {current:.1f}s). '
                        'Restarting data recorder...',
                        LOGLEVEL.WARN, 'system'
                    )
                    # Reset trigger button
                    self.btn_trigger.setChecked(False)
                    # Kill and restart data recorder
                    if self.process_manager.is_running('data_recorder'):
                        self.process_manager.stop_process('data_recorder', timeout=2.0)
                    self._start_data_recorder()
                self._last_sim_time_sec = current
            except Exception:
                pass
    
    def reset(self):
        """Reset panel to initial state"""
        self.btn_systemon.setChecked(False)
        self.btn_trigger.setChecked(False)
        
        if self.ros_worker.is_running:
            trigger_cmd = TriggerStamped()
            trigger_cmd.header.stamp = self.ros_worker.node.get_clock().now().to_msg()
            trigger_cmd.enable = False
            trigger_cmd.output_filename = ''
            self.ros_worker.send_trigger(trigger_cmd)
    
    # ========================================================================
    # Cleanup
    # ========================================================================
    
    def closeEvent(self, event):
        """Handle window close event"""
        # Close file handler and save log
        close_file_logger(self.file_logger, self.log_filepath, "Control Panel")
        
        # Reset and cleanup
        self.reset()
        
        # Stop data recorder
        if self.process_manager.is_running('data_recorder'):
            self.process_manager.stop_process('data_recorder', timeout=2.0)
        
        # Stop all processes
        self.process_manager.cleanup_all(timeout=2.0)
        
        # Stop ROS worker
        self.ros_worker.stop_ros()
        
        # Accept close event
        super().closeEvent(event)


# Entry point for standalone execution
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CorgiControlPanel()
    sys.exit(app.exec_())
