#!/usr/bin/env python3
"""
Control Panel GUI for Corgi Robot
Refactored version with modular architecture (MVC pattern)
"""
import os
import sys
import logging
import time
import csv
import yaml
import numpy as np
from datetime import datetime
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, 
    QGroupBox, QLineEdit, QGridLayout, QFrame, QFileDialog, QApplication,
    QDoubleSpinBox, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer

from corgi_msgs.msg import MotorCmdStamped, RobotCmdStamped, TriggerStamped

# Import from corgi_ui package
from corgi_ui.core.constants import (
    ROBOTMODE, LOGLEVEL, COLORS, PATHS,
    LOGLEVEL_TO_LOGGING_MAP,
    setup_file_logger, log_to_file, close_file_logger
)
from corgi_ui.core.ros_worker import ControlPanelRosWorker
from corgi_ui.core.process_manager import ProcessManager
from corgi_ui.gui.widgets.log_widget import LogWidget
from corgi_ui.core.sequence_model import (
    ExecutionState, NodeExecutionRecord,
    check_sequence_reachability, save_execution_record,
    LEGS, JOINTS,
)
from corgi_ui.gui.custom_sequence_window import CustomSequenceWindow

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
    - Process management (IMU, CSV control, set_zero)
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

        # Check if custom sequence feature is enabled
        self.enable_custom_sequence = self._check_bool_param('enable_custom_sequence', False)
        
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

        # Manual joint command state
        self._manual_cmd_active = False
        self._last_manual_cmd_ts = None
        self._manual_max_rpm = 300.0
        self._manual_current_deg = {
            leg: {'theta': 0.0, 'beta': 0.0, 'gamma': 0.0}
            for leg in ('A', 'B', 'C', 'D')
        }
        self._manual_target_deg = {
            leg: {'theta': 0.0, 'beta': 0.0, 'gamma': 0.0}
            for leg in ('A', 'B', 'C', 'D')
        }
        self._manual_session_started = None
        self._pid_tune_samples = {axis: [] for axis in ('theta', 'beta', 'gamma')}
        self._pid_tune_initial_avg = {axis: 0.0 for axis in ('theta', 'beta', 'gamma')}
        self._pid_tune_target_avg = {axis: 0.0 for axis in ('theta', 'beta', 'gamma')}
        self._csv_tune_active = False
        self._csv_tune_start_ts = None
        self._csv_tune_samples = {'t': [], 'theta': [], 'beta': []}

        # Custom sequence executor state
        self._seq_state: ExecutionState = ExecutionState.IDLE
        self._seq_nodes: list = []
        self._seq_node_idx: int = 0
        self._seq_limit_profile = None
        self._seq_sequence_name: str = ""
        self._seq_dry_run: bool = False
        self._seq_node_start_ts: float = 0.0
        self._seq_last_tick_ts: float = 0.0
        self._seq_node_start_pos: dict = {
            leg: {'theta': 0.0, 'beta': 0.0, 'gamma': 0.0} for leg in LEGS
        }
        self._seq_node_records: list = []
        self._custom_seq_window: CustomSequenceWindow | None = None

        # Auto-start data recorder
        self._start_data_recorder()
    
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
    
    def _check_bool_param(self, param_name: str, default: bool = False) -> bool:
        """Check a boolean ROS parameter via a temporary node."""
        try:
            import rclpy
            if not rclpy.ok():
                rclpy.init()
            temp_node = rclpy.create_node(
                'corgi_control_panel',
                automatically_declare_parameters_from_overrides=True
            )
            if not temp_node.has_parameter(param_name):
                temp_node.declare_parameter(param_name, default)
            result = temp_node.get_parameter(param_name).value
            temp_node.destroy_node()
            return bool(result)
        except Exception as e:
            print(f"Warning: Could not check parameter '{param_name}': {e}")
            return default

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
        self.setMinimumSize(900, 650)
        self.resize(1280, 860)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setWindowFlag(Qt.WindowMinMaxButtonsHint, True)
        
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
        
        self.lbl_voltage = QLabel('--.- V')
        self.lbl_voltage.setObjectName('PowerBadge')
        self.lbl_soc = QLabel('-- %')
        self.lbl_soc.setObjectName('PowerBadge')
        self.lbl_current = QLabel('-.-- A')
        self.lbl_current.setObjectName('PowerBadge')
        self.lbl_power = QLabel('--.- W')
        self.lbl_power.setObjectName('PowerBadge')
        
        power_box.addWidget(self.lbl_voltage)
        power_box.addWidget(self.lbl_soc)
        power_box.addWidget(self.lbl_current)
        power_box.addWidget(self.lbl_power)
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
        
        # Set Zero button
        self.btn_set_zero = QPushButton('Set Zero')
        self.btn_set_zero.clicked.connect(self._on_set_zero_clicked)
        self.btn_set_zero.setEnabled(False)
        sidebar.addWidget(self.btn_set_zero)
        
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

        self.btn_csv_pid_tune = QPushButton('Tune PID from Last CSV Run')
        self.btn_csv_pid_tune.clicked.connect(self._on_tune_pid_from_csv_clicked)
        self.btn_csv_pid_tune.setEnabled(False)
        
        csv_btn_layout.addWidget(self.btn_csv_select)
        csv_btn_layout.addWidget(self.btn_csv_run)
        
        grp_csv_layout.addWidget(self.label_csv)
        grp_csv_layout.addWidget(self.edit_csv)
        grp_csv_layout.addLayout(csv_btn_layout)
        grp_csv_layout.addWidget(self.btn_csv_pid_tune)
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

        # Custom Command Sequence button (only shown when enable_custom_sequence=True)
        self.btn_custom_cmd = QPushButton('Custom Command Sequence')
        self.btn_custom_cmd.setStyleSheet(
            'background: #1a3a5c; font-weight: bold;'
        )
        self.btn_custom_cmd.clicked.connect(self._on_custom_cmd_clicked)
        self.btn_custom_cmd.setVisible(self.enable_custom_sequence)
        sidebar.addWidget(self.btn_custom_cmd)

        sidebar.addWidget(self._create_motor_config_display())
        sidebar.addStretch(1)
        return sidebar
    
    def _create_motor_monitor(self) -> QVBoxLayout:
        """Create motor status monitor"""
        monitor_layout = QVBoxLayout()

        grid_motors = QGridLayout()
        self.angle_feedback_labels = {}
        self.joint_cmd_inputs = {}

        legs = [
            ('A', 0, 0),
            ('B', 0, 1),
            ('C', 1, 0),
            ('D', 1, 1),
        ]

        for leg_name, r, c in legs:
            leg_group = QGroupBox(leg_name)
            leg_layout = QVBoxLayout()

            target_layout = QGridLayout()
            target_layout.addWidget(QLabel('Target θ'), 0, 0)
            target_layout.addWidget(QLabel('Target β'), 0, 1)
            target_layout.addWidget(QLabel('Target γ'), 0, 2)

            for col, key in (
                (0, 'theta'),
                (1, 'beta'),
                (2, 'gamma'),
            ):
                spin = QDoubleSpinBox()
                spin.setDecimals(2)
                spin.setSingleStep(0.5)
                spin.setRange(-1e9, 1e9)
                spin.setValue(0.0)
                self.joint_cmd_inputs[(leg_name, key)] = spin
                target_layout.addWidget(spin, 1, col)

            leg_layout.addLayout(target_layout)

            for key, title in (
                ('current', 'Current θ/β/γ (deg)'),
                ('target', 'Target  θ/β/γ (deg)'),
                ('error', 'Error    θ/β/γ (deg)'),
                ('vel', 'Gamma velocity (deg/s)'),
            ):
                lbl = QLabel(f"{title}: -- / -- / --" if key != 'vel' else f"{title}: --")
                lbl.setObjectName('MotorLabel')
                leg_layout.addWidget(lbl)
                self.angle_feedback_labels[(leg_name, key)] = lbl

            leg_group.setLayout(leg_layout)
            grid_motors.addWidget(leg_group, r, c)

        monitor_layout.addWidget(self._create_joint_command_group())
        monitor_layout.addLayout(grid_motors)
        monitor_layout.addStretch(1)

        return monitor_layout

    def _create_joint_command_group(self) -> QGroupBox:
        """Create combined joint command and PID tuning group."""
        grp_joint = QGroupBox("Integrated Command + PID")
        grp_joint_layout = QVBoxLayout()

        joint_hint = QLabel('Use target boxes inside A/B/C/D cards, then send and tune by cmd/state response')
        joint_hint.setStyleSheet('color: #aaa; font-size: 12px;')
        grp_joint_layout.addWidget(joint_hint)

        joint_btn_layout = QHBoxLayout()
        self.btn_joint_load_state = QPushButton('Load Current θ/β/γ')
        self.btn_joint_load_state.clicked.connect(self._on_joint_load_state_clicked)
        self.btn_joint_load_state.setEnabled(False)
        self.btn_joint_send = QPushButton('Send Joint Command')
        self.btn_joint_send.clicked.connect(self._on_joint_send_clicked)
        self.btn_joint_send.setEnabled(False)
        joint_btn_layout.addWidget(self.btn_joint_load_state)
        joint_btn_layout.addWidget(self.btn_joint_send)
        grp_joint_layout.addLayout(joint_btn_layout)

        pid_grid = QGridLayout()
        pid_grid.addWidget(QLabel('θ/β Kp:'), 0, 0)
        self.spin_leg_kp = QDoubleSpinBox()
        self.spin_leg_kp.setDecimals(2)
        self.spin_leg_kp.setRange(-1e9, 1e9)
        self.spin_leg_kp.setSingleStep(1.0)
        self.spin_leg_kp.setValue(120.0)
        pid_grid.addWidget(self.spin_leg_kp, 0, 1)

        pid_grid.addWidget(QLabel('θ/β Kd:'), 0, 2)
        self.spin_leg_kd = QDoubleSpinBox()
        self.spin_leg_kd.setDecimals(2)
        self.spin_leg_kd.setRange(-1e9, 1e9)
        self.spin_leg_kd.setSingleStep(0.1)
        self.spin_leg_kd.setValue(0.25)
        pid_grid.addWidget(self.spin_leg_kd, 0, 3)

        pid_grid.addWidget(QLabel('γ Kp:'), 1, 0)
        self.spin_gamma_kp = QDoubleSpinBox()
        self.spin_gamma_kp.setDecimals(2)
        self.spin_gamma_kp.setRange(-1e9, 1e9)
        self.spin_gamma_kp.setSingleStep(0.5)
        self.spin_gamma_kp.setValue(150.0)
        pid_grid.addWidget(self.spin_gamma_kp, 1, 1)

        pid_grid.addWidget(QLabel('γ Kd:'), 1, 2)
        self.spin_gamma_kd = QDoubleSpinBox()
        self.spin_gamma_kd.setDecimals(2)
        self.spin_gamma_kd.setRange(-1e9, 1e9)
        self.spin_gamma_kd.setSingleStep(0.1)
        self.spin_gamma_kd.setValue(1.75)
        pid_grid.addWidget(self.spin_gamma_kd, 1, 3)
        grp_joint_layout.addLayout(pid_grid)

        tune_btn_layout = QHBoxLayout()
        self.btn_pid_auto_tune = QPushButton('Auto Tune PID')
        self.btn_pid_auto_tune.clicked.connect(self._on_pid_auto_tune_clicked)
        self.btn_pid_auto_tune.setEnabled(False)
        self.btn_gamma_auto_tune = self.btn_pid_auto_tune
        tune_btn_layout.addWidget(self.btn_pid_auto_tune)

        self.btn_gamma_live_tune = QPushButton('Live Tune PID')
        self.btn_gamma_live_tune.setCheckable(True)
        self.btn_gamma_live_tune.setEnabled(False)
        tune_btn_layout.addWidget(self.btn_gamma_live_tune)
        grp_joint_layout.addLayout(tune_btn_layout)

        rpm_layout = QHBoxLayout()
        rpm_layout.addWidget(QLabel('Speed limit (rpm):'))
        self.spin_joint_rpm_limit = QDoubleSpinBox()
        self.spin_joint_rpm_limit.setDecimals(1)
        self.spin_joint_rpm_limit.setRange(0.0, 1e9)
        self.spin_joint_rpm_limit.setSingleStep(10.0)
        self.spin_joint_rpm_limit.setValue(300.0)
        self.spin_joint_rpm_limit.valueChanged.connect(self._on_joint_rpm_changed)
        rpm_layout.addWidget(self.spin_joint_rpm_limit)
        grp_joint_layout.addLayout(rpm_layout)

        self.label_pid_tune_status = QLabel('PID tune status: waiting for a manual move')
        self.label_pid_tune_status.setStyleSheet('color: #888; font-size: 11px;')
        grp_joint_layout.addWidget(self.label_pid_tune_status)

        grp_joint.setLayout(grp_joint_layout)
        return grp_joint

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

    def _create_motor_config_display(self) -> QWidget:
        """Create a collapsible read-only display of the motor direction config (motor_config.yaml)."""
        cfg = self._load_motor_config()

        # Outer container (not a QGroupBox – we build a custom header + body)
        wrapper = QWidget()
        wrapper_layout = QVBoxLayout(wrapper)
        wrapper_layout.setContentsMargins(0, 0, 0, 0)
        wrapper_layout.setSpacing(2)

        # ── Header row: toggle button ──────────────────────────────────────
        self._btn_config_toggle = QPushButton("▶  Motor Direction Config  (motor_config.yaml)")
        self._btn_config_toggle.setCheckable(True)
        self._btn_config_toggle.setChecked(False)   # collapsed by default
        self._btn_config_toggle.setStyleSheet(
            "QPushButton { text-align: left; padding: 4px 8px; "
            "background: #3a3a3a; border: 1px solid #555; border-radius: 4px; }"
            "QPushButton:checked { background: #2d4a6a; }"
        )
        wrapper_layout.addWidget(self._btn_config_toggle)

        # ── Body widget (hidden by default) ───────────────────────────────
        self._config_body = QWidget()
        body_layout = QVBoxLayout(self._config_body)
        body_layout.setContentsMargins(4, 4, 4, 4)
        body_layout.setSpacing(4)
        self._config_body.setVisible(False)

        def _toggle_config(checked):
            self._config_body.setVisible(checked)
            arrow = '▼' if checked else '▶'
            self._btn_config_toggle.setText(
                f"{arrow}  Motor Direction Config  (motor_config.yaml)")

        self._btn_config_toggle.toggled.connect(_toggle_config)

        # Reload button
        self.btn_reload_config = QPushButton("↺  Reload Config")
        self.btn_reload_config.setMaximumWidth(160)
        self.btn_reload_config.clicked.connect(self._reload_motor_config_display)
        body_layout.addWidget(self.btn_reload_config, alignment=Qt.AlignLeft)

        outer = body_layout   # alias so the rest of the method works unchanged

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
        wrapper_layout.addWidget(self._config_body)
        return wrapper

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
    
    def _log(self, message: str, level=LOGLEVEL.INFO, source: str = "system"):
        """Unified logging method that writes to both GUI and file"""
        
        # Write to GUI log widget
        self.log_widget.add_log(message, level, source)
        
        # Write to file logger
        log_to_file(self.file_logger, level, source, message)
    
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
    
    def _on_set_zero_clicked(self):
        """Handle Set Zero button click"""
        self.btn_set_zero.setEnabled(False)
        self.btn_set_zero.setText('Setting Zero...')
        
        success = self.process_manager.start_process(
            'set_zero',
            ['ros2', 'run', 'corgi_set_zero', 'set_zero'],
            capture_output=False
        )
        
        if success:
            self.log_widget.add_log('Set Zero Started', LOGLEVEL.INFO, 'system')
        else:
            self.log_widget.add_log('Failed to start set_zero', LOGLEVEL.ERROR, 'system')
            self.btn_set_zero.setEnabled(True)
            self.btn_set_zero.setText('Set Zero')
    
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

            self._csv_tune_active = True
            self._csv_tune_start_ts = time.monotonic()
            self._csv_tune_samples = {'t': [], 'theta': [], 'beta': [], 'gamma': []}
            
            # Build command with PID and sim_time parameters
            leg_kp = self.spin_leg_kp.value()
            leg_kd = self.spin_leg_kd.value()
            gamma_kp = self.spin_gamma_kp.value()
            gamma_kd = self.spin_gamma_kd.value()
            pid_args = [
                '--ros-args',
                '-p', f'leg_kp:={leg_kp}',
                '-p', f'leg_kd:={leg_kd}',
                '-p', f'gamma_kp:={gamma_kp}',
                '-p', f'gamma_kd:={gamma_kd}',
            ]
            if self.use_sim_time:
                pid_args += ['-p', 'use_sim_time:=true']
            cmd = ['ros2', 'run', 'corgi_csv_control', 'corgi_csv_control', csv_arg] + pid_args
            
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
            self._csv_tune_active = False
            
            if self.process_manager.stop_process('csv_control', timeout=3.0):
                self.log_widget.add_log('CSV Control Stopped', LOGLEVEL.WARN, 'system')

    def _resolve_csv_file_path(self) -> str:
        """Resolve CSV file path from input field to an absolute path."""
        csv_file = self.edit_csv.text().strip()
        if not csv_file:
            return ''

        if os.path.isabs(csv_file) and os.path.exists(csv_file):
            return csv_file

        if os.path.exists(csv_file):
            return os.path.abspath(csv_file)

        base_name = os.path.basename(csv_file)
        candidate = os.path.join(PATHS.DEFAULT_CSV_DIR, base_name)
        if os.path.exists(candidate):
            return candidate

        if not base_name.endswith('.csv'):
            candidate_csv = os.path.join(PATHS.DEFAULT_CSV_DIR, f'{base_name}.csv')
            if os.path.exists(candidate_csv):
                return candidate_csv

        return ''

    def _on_tune_pid_from_csv_clicked(self):
        """Tune theta/beta/gamma PID by comparing recorded state against selected CSV trajectory."""
        csv_path = self._resolve_csv_file_path()
        if not csv_path:
            self.log_widget.add_log('CSV PID tune failed: invalid CSV path', LOGLEVEL.WARN, 'system')
            return

        if len(self._csv_tune_samples['theta']) < 30:
            self.log_widget.add_log('CSV PID tune failed: not enough state samples, run CSV first', LOGLEVEL.WARN, 'system')
            return

        cmd_theta = []
        cmd_beta = []
        cmd_gamma = []
        try:
            with open(csv_path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) < 8:
                        continue
                    try:
                        vals = [float(v) for v in row[:12]]
                    except ValueError:
                        continue
                    cmd_theta.append(np.degrees(np.mean([vals[0], vals[2], vals[4], vals[6]])))
                    cmd_beta.append(np.degrees(np.mean([vals[1], vals[3], vals[5], vals[7]])))
                    if len(vals) >= 12:
                        cmd_gamma.append(np.degrees(np.mean([vals[8], vals[9], vals[10], vals[11]])))
        except Exception as e:
            self.log_widget.add_log(f'CSV PID tune failed: {e}', LOGLEVEL.ERROR, 'system')
            return

        if len(cmd_theta) < 10:
            self.log_widget.add_log('CSV PID tune failed: CSV format unsupported', LOGLEVEL.WARN, 'system')
            return

        state_theta = np.array(self._csv_tune_samples['theta'], dtype=np.float64)
        state_beta = np.array(self._csv_tune_samples['beta'], dtype=np.float64)
        cmd_theta = np.array(cmd_theta, dtype=np.float64)
        cmd_beta = np.array(cmd_beta, dtype=np.float64)

        idx = np.linspace(0, len(cmd_theta) - 1, len(state_theta)).astype(np.int64)
        cmd_theta_rs = cmd_theta[idx]
        cmd_beta_rs = cmd_beta[idx]

        err_theta = cmd_theta_rs - state_theta
        err_beta = cmd_beta_rs - state_beta

        rmse_theta = float(np.sqrt(np.mean(err_theta ** 2)))
        rmse_beta = float(np.sqrt(np.mean(err_beta ** 2)))
        cmd_span_theta = float(max(np.max(cmd_theta_rs) - np.min(cmd_theta_rs), 1.0))
        cmd_span_beta = float(max(np.max(cmd_beta_rs) - np.min(cmd_beta_rs), 1.0))

        kp = float(self.spin_leg_kp.value())
        kd = float(self.spin_leg_kd.value())

        scale_theta = min(1.0, rmse_theta / cmd_span_theta)
        scale_beta = min(1.0, rmse_beta / cmd_span_beta)
        err_scale = max(scale_theta, scale_beta)

        de_theta = np.diff(err_theta)
        de_beta = np.diff(err_beta)
        cmd_d_theta = np.diff(cmd_theta_rs)
        cmd_d_beta = np.diff(cmd_beta_rs)
        jitter_theta = float(np.std(de_theta) / (np.std(cmd_d_theta) + 1e-6))
        jitter_beta = float(np.std(de_beta) / (np.std(cmd_d_beta) + 1e-6))
        jitter_scale = min(1.0, max(jitter_theta, jitter_beta))

        kp_new = float(kp * (1.0 + 0.6 * err_scale))
        kd_new = float(kd * (1.0 + 0.35 * jitter_scale))

        self.spin_leg_kp.setValue(kp_new)
        self.spin_leg_kd.setValue(kd_new)

        # ── Gamma PID tune ────────────────────────────────────────────────
        gamma_tune_msg = ''
        if cmd_gamma and len(self._csv_tune_samples['gamma']) >= 30:
            state_gamma = np.array(self._csv_tune_samples['gamma'], dtype=np.float64)
            cmd_gamma_arr = np.array(cmd_gamma, dtype=np.float64)
            idx_g = np.linspace(0, len(cmd_gamma_arr) - 1, len(state_gamma)).astype(np.int64)
            cmd_gamma_rs = cmd_gamma_arr[idx_g]
            err_gamma = cmd_gamma_rs - state_gamma
            rmse_gamma = float(np.sqrt(np.mean(err_gamma ** 2)))
            cmd_span_gamma = float(max(np.max(cmd_gamma_rs) - np.min(cmd_gamma_rs), 1.0))

            gkp = float(self.spin_gamma_kp.value())
            gkd = float(self.spin_gamma_kd.value())
            scale_gamma = min(1.0, rmse_gamma / cmd_span_gamma)
            de_gamma = np.diff(err_gamma)
            cmd_d_gamma = np.diff(cmd_gamma_rs)
            jitter_gamma = min(1.0, float(np.std(de_gamma) / (np.std(cmd_d_gamma) + 1e-6)))
            gkp_new = float(gkp * (1.0 + 0.6 * scale_gamma))
            gkd_new = float(gkd * (1.0 + 0.35 * jitter_gamma))
            self.spin_gamma_kp.setValue(gkp_new)
            self.spin_gamma_kd.setValue(gkd_new)
            gamma_tune_msg = f', γ RMSE={rmse_gamma:.2f}° -> γ Kp={gkp_new:.2f}, Kd={gkd_new:.2f}'

        self.label_pid_tune_status.setText(
            f'PID tune: CSV RMSE θ={rmse_theta:.2f}°, β={rmse_beta:.2f}° -> θ/β Kp={kp_new:.2f}, Kd={kd_new:.2f}'
            + gamma_tune_msg
        )
        self.log_widget.add_log(
            f'CSV PID tune done: RMSE θ={rmse_theta:.2f}°, β={rmse_beta:.2f}°; new θ/β Kp={kp_new:.2f}, Kd={kd_new:.2f}'
            + gamma_tune_msg,
            LOGLEVEL.INFO,
            'system'
        )

    def _on_joint_load_state_clicked(self):
        """Load current motor state into joint command boxes (degree)."""
        if not hasattr(self, 'motor_state'):
            self.log_widget.add_log('No motor state yet, cannot load current pose', LOGLEVEL.WARN, 'system')
            return

        modules = [
            getattr(self.motor_state, 'module_a', None),
            getattr(self.motor_state, 'module_b', None),
            getattr(self.motor_state, 'module_c', None),
            getattr(self.motor_state, 'module_d', None),
        ]

        loaded_count = 0
        for leg_id, module in zip(('A', 'B', 'C', 'D'), modules):
            if module is None:
                continue

            if hasattr(module, 'theta') and hasattr(module, 'beta'):
                theta_deg = float(np.degrees(module.theta))
                beta_deg = float(np.degrees(module.beta))
                gamma_deg = float(np.degrees(module.gamma)) if hasattr(module, 'gamma') else 0.0
            elif hasattr(module, 'position') and len(module.position) >= 2:
                theta_deg = float(np.degrees(module.position[0]))
                beta_deg = float(np.degrees(module.position[1]))
                gamma_deg = 0.0
            else:
                continue

            self.joint_cmd_inputs[(leg_id, 'theta')].setValue(theta_deg)
            self.joint_cmd_inputs[(leg_id, 'beta')].setValue(beta_deg)
            self.joint_cmd_inputs[(leg_id, 'gamma')].setValue(gamma_deg)
            loaded_count += 1

        if loaded_count > 0:
            self.log_widget.add_log(f'Loaded current pose for {loaded_count} leg modules', LOGLEVEL.INFO, 'system')
        else:
            self.log_widget.add_log('Motor state format is unsupported for loading', LOGLEVEL.WARN, 'system')

    def _on_joint_send_clicked(self):
        """Set manual target and start speed-limited publish to /motor/command."""
        if self._seq_state == ExecutionState.RUNNING_NODE:
            self.log_widget.add_log(
                'Cannot send manual command while sequence is running – stop the sequence first.',
                LOGLEVEL.WARN, 'system'
            )
            return
        if not self.ros_worker.is_running:
            self.log_widget.add_log('ROS worker is not running', LOGLEVEL.WARN, 'system')
            return

        self._manual_max_rpm = self.spin_joint_rpm_limit.value()
        for leg_id in ('A', 'B', 'C', 'D'):
            for key in ('theta', 'beta', 'gamma'):
                self._manual_target_deg[leg_id][key] = float(
                    self.joint_cmd_inputs[(leg_id, key)].value()
                )

        # Initialize current command from measured state if available.
        if hasattr(self, 'motor_state') and hasattr(self.motor_state, 'module_a'):
            for leg_id, module in zip(
                ('A', 'B', 'C', 'D'),
                [self.motor_state.module_a, self.motor_state.module_b, self.motor_state.module_c, self.motor_state.module_d],
            ):
                if hasattr(module, 'theta') and hasattr(module, 'beta'):
                    self._manual_current_deg[leg_id]['theta'] = float(np.degrees(module.theta))
                    self._manual_current_deg[leg_id]['beta'] = float(np.degrees(module.beta))
                    self._manual_current_deg[leg_id]['gamma'] = float(np.degrees(module.gamma))

        self._manual_cmd_active = True
        self._last_manual_cmd_ts = None
        self._manual_session_started = time.monotonic()
        self._reset_pid_tune_session()
        self._publish_manual_joint_command_step(force_first=True)
        self.log_widget.add_log(
            f'Manual target accepted with speed limit {self._manual_max_rpm:.1f} rpm',
            LOGLEVEL.INFO,
            'orin'
        )

    def _on_joint_rpm_changed(self, value: float):
        """Update speed limit for manual joint ramp command."""
        self._manual_max_rpm = float(value)

    def _reset_pid_tune_session(self):
        """Reset command/response sample buffer for PID tuning."""
        self._pid_tune_samples = {axis: [] for axis in ('theta', 'beta', 'gamma')}
        if hasattr(self, 'motor_state') and hasattr(self.motor_state, 'module_a'):
            modules = [self.motor_state.module_a, self.motor_state.module_b, self.motor_state.module_c, self.motor_state.module_d]
            for axis in ('theta', 'beta', 'gamma'):
                values = [float(np.degrees(getattr(module, axis))) for module in modules if hasattr(module, axis)]
                self._pid_tune_initial_avg[axis] = float(np.mean(values)) if values else 0.0
                self._pid_tune_target_avg[axis] = float(np.mean([self._manual_target_deg[leg][axis] for leg in ('A', 'B', 'C', 'D')]))
        self.label_pid_tune_status.setText('PID tune status: sampling command/state response...')

    def _record_pid_tune_sample(self, modules):
        """Record averaged target/current response for each axis."""
        if self._manual_session_started is None:
            return
        elapsed = time.monotonic() - self._manual_session_started
        for axis in ('theta', 'beta', 'gamma'):
            current_values = [float(np.degrees(getattr(module, axis))) for module in modules if hasattr(module, axis)]
            if not current_values:
                continue
            current_avg = float(np.mean(current_values))
            target_avg = float(np.mean([self._manual_target_deg[leg][axis] for leg in ('A', 'B', 'C', 'D')]))
            error = target_avg - current_avg
            self._pid_tune_samples[axis].append({
                't': elapsed,
                'current': current_avg,
                'target': target_avg,
                'error': error,
            })

    def _suggest_pid_from_response(self, axis: str, current_kp: float, current_kd: float) -> tuple[float, float, dict]:
        """Suggest PID gains from recorded command/state response."""
        samples = self._pid_tune_samples.get(axis, [])
        if len(samples) < 4:
            return current_kp, current_kd, {'reason': 'not enough samples'}

        initial = self._pid_tune_initial_avg[axis]
        target = self._pid_tune_target_avg[axis]
        step = target - initial
        step_mag = max(abs(step), 1e-3)
        sign = 1.0 if step >= 0.0 else -1.0
        abs_errors = [abs(sample['error']) for sample in samples]
        mean_abs_error = float(np.mean(abs_errors))
        final_abs_error = abs(samples[-1]['error'])

        overshoot = 0.0
        for sample in samples:
            signed_progress = sign * (sample['current'] - target)
            overshoot = max(overshoot, signed_progress)
        overshoot_ratio = max(0.0, overshoot) / step_mag
        error_ratio = mean_abs_error / step_mag
        final_error_ratio = final_abs_error / step_mag

        kp_scale = 1.0 + 0.45 * min(error_ratio, 1.0)
        if final_error_ratio < 0.03:
            kp_scale *= 0.98

        kd_scale = 1.0 + 1.2 * min(overshoot_ratio, 0.8)
        if overshoot_ratio < 0.02 and final_error_ratio > 0.08:
            kd_scale *= 0.95

        kp_new = float(current_kp * kp_scale)
        kd_new = float(current_kd * kd_scale)
        return kp_new, kd_new, {
            'step_mag': step_mag,
            'mean_abs_error': mean_abs_error,
            'final_abs_error': final_abs_error,
            'overshoot_ratio': overshoot_ratio,
        }

    def _on_pid_auto_tune_clicked(self, log_result: bool = True):
        """Tune θ/β and γ PID from recorded command/state response."""
        leg_kp, leg_kd, leg_stats = self._suggest_pid_from_response(
            'theta',
            float(self.spin_leg_kp.value()),
            float(self.spin_leg_kd.value()),
        )
        gamma_kp, gamma_kd, gamma_stats = self._suggest_pid_from_response(
            'gamma',
            float(self.spin_gamma_kp.value()),
            float(self.spin_gamma_kd.value()),
        )

        self.spin_leg_kp.setValue(leg_kp)
        self.spin_leg_kd.setValue(leg_kd)
        self.spin_gamma_kp.setValue(gamma_kp)
        self.spin_gamma_kd.setValue(gamma_kd)

        status = (
            f"PID tune status: θ/β avg err={leg_stats.get('mean_abs_error', 0.0):.2f} deg, "
            f"γ avg err={gamma_stats.get('mean_abs_error', 0.0):.2f} deg"
        )
        self.label_pid_tune_status.setText(status)
        if log_result:
            self.log_widget.add_log(
                f"Auto tuned PID from cmd/state response -> θ/β Kp={leg_kp:.2f}, Kd={leg_kd:.2f}; γ Kp={gamma_kp:.2f}, Kd={gamma_kd:.2f}",
                LOGLEVEL.INFO,
                'system'
            )

    def _publish_manual_joint_command_step(self, force_first: bool = False):
        """Publish one speed-limited interpolation step toward manual target."""
        if not self.ros_worker.is_running:
            self._manual_cmd_active = False
            return

        now_ts = time.monotonic()
        if self._last_manual_cmd_ts is None:
            dt = 0.02 if force_first else 0.1
        else:
            dt = max(0.005, min(0.2, now_ts - self._last_manual_cmd_ts))
        self._last_manual_cmd_ts = now_ts

        # rpm -> deg/s = rpm * 6
        max_delta_deg = self._manual_max_rpm * 6.0 * dt
        reached = True

        for leg_id in ('A', 'B', 'C', 'D'):
            for key in ('theta', 'beta', 'gamma'):
                current = self._manual_current_deg[leg_id][key]
                target = self._manual_target_deg[leg_id][key]
                delta = target - current
                if abs(delta) > max_delta_deg:
                    step = np.sign(delta) * max_delta_deg
                    self._manual_current_deg[leg_id][key] = current + step
                    reached = False
                else:
                    self._manual_current_deg[leg_id][key] = target

        msg = MotorCmdStamped()
        msg.header.seq = self._robot_cmd_seq + 1
        msg.header.stamp = self.ros_worker.node.get_clock().now().to_msg()
        msg.header.frame_id = ''

        leg_kp = float(self.spin_leg_kp.value())
        leg_kd = float(self.spin_leg_kd.value())
        gamma_kp = float(self.spin_gamma_kp.value())
        gamma_kd = float(self.spin_gamma_kd.value())

        for module_name, leg_id in (
            ('module_a', 'A'),
            ('module_b', 'B'),
            ('module_c', 'C'),
            ('module_d', 'D'),
        ):
            module = getattr(msg, module_name)
            module.theta = np.radians(self._manual_current_deg[leg_id]['theta'])
            module.beta = np.radians(self._manual_current_deg[leg_id]['beta'])
            module.gamma = np.radians(self._manual_current_deg[leg_id]['gamma'])

            module.kp_r = leg_kp
            module.kp_l = leg_kp
            module.kp_h = gamma_kp
            module.ki_r = 0.0
            module.ki_l = 0.0
            module.ki_h = 0.0
            module.kd_r = leg_kd
            module.kd_l = leg_kd
            module.kd_h = gamma_kd
            module.torque_r = 0.0
            module.torque_l = 0.0
            module.torque_h = 0.0

        self.ros_worker.send_motor_command(msg)
        self._robot_cmd_seq += 1

        if reached:
            self._manual_cmd_active = False
            self.log_widget.add_log('Reached manual joint target', LOGLEVEL.INFO, 'system')
    
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
        
        try:
            v_total = float(getattr(state, 'v_0', 0.0))
        except Exception:
            v_total = 0.0
        
        try:
            i_total = float(getattr(state, 'i_1', 0.0))
        except Exception:
            i_total = 0.0
        
        soc = self._calculate_soc(v_total)
        power = v_total * i_total
        
        self.lbl_voltage.setText(f"{v_total:.1f} V")
        self.lbl_soc.setText(f"{soc:.0f} %")
        self.lbl_current.setText(f"{i_total:.2f} A")
        self.lbl_power.setText(f"{power:.1f} W")
        
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
        
        modules = [state.module_a, state.module_b, state.module_c, state.module_d]
        self._record_pid_tune_sample(modules)

        if self._csv_tune_active and self._csv_tune_start_ts is not None:
            theta_avg = float(np.degrees(np.mean([module.theta for module in modules if hasattr(module, 'theta')])))
            beta_avg = float(np.degrees(np.mean([module.beta for module in modules if hasattr(module, 'beta')])))
            gamma_avg = float(np.degrees(np.mean([module.gamma for module in modules if hasattr(module, 'gamma')]))) if any(hasattr(m, 'gamma') for m in modules) else 0.0
            self._csv_tune_samples['t'].append(time.monotonic() - self._csv_tune_start_ts)
            self._csv_tune_samples['theta'].append(theta_avg)
            self._csv_tune_samples['beta'].append(beta_avg)
            self._csv_tune_samples['gamma'].append(gamma_avg)

        for leg_id, module in zip(('A', 'B', 'C', 'D'), modules):
            theta = float(np.degrees(module.theta)) if hasattr(module, 'theta') else 0.0
            beta = float(np.degrees(module.beta)) if hasattr(module, 'beta') else 0.0
            gamma = float(np.degrees(module.gamma)) if hasattr(module, 'gamma') else 0.0
            gamma_vel = float(np.degrees(module.velocity_h)) if hasattr(module, 'velocity_h') else 0.0

            target = self._manual_target_deg.get(leg_id, {'theta': 0.0, 'beta': 0.0, 'gamma': 0.0})
            err_theta = target['theta'] - theta
            err_beta = target['beta'] - beta
            err_gamma = target['gamma'] - gamma

            self.angle_feedback_labels[(leg_id, 'current')].setText(
                f"Current θ/β/γ (deg): {theta:6.2f} / {beta:6.2f} / {gamma:6.2f}"
            )
            self.angle_feedback_labels[(leg_id, 'target')].setText(
                f"Target  θ/β/γ (deg): {target['theta']:6.2f} / {target['beta']:6.2f} / {target['gamma']:6.2f}"
            )
            self.angle_feedback_labels[(leg_id, 'error')].setText(
                f"Error    θ/β/γ (deg): {err_theta:6.2f} / {err_beta:6.2f} / {err_gamma:6.2f}"
            )
            self.angle_feedback_labels[(leg_id, 'vel')].setText(
                f"Gamma velocity (deg/s): {gamma_vel:6.2f}"
            )
    
    def _handle_log_update(self, log_msg):
        """Handle log message from ROS"""
        level = log_msg.level
        node_name = log_msg.node_name if hasattr(log_msg, 'node_name') else 'unknown'
        message = log_msg.message if hasattr(log_msg, 'message') else ''
        
        # Always log to file
        file_log_level = LOGLEVEL_TO_LOGGING_MAP.get(level, logging.INFO)
        self.file_logger.log(file_log_level, f'[{node_name}] {message}')
        
        # Display in log widget
        self.log_widget.add_log(message, LOGLEVEL(level), node_name)
        
        # Handle special messages - check for set_zero completion
        if 'set_zero' in node_name.lower():
            if 'completed' in message.lower() or 'complete' in message.lower():
                self.log_widget.add_log(
                    'Set Zero operation completed successfully',
                    LOGLEVEL.INFO, 'system'
                )
                self._on_set_zero_completed()
        
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
        self.btn_csv_pid_tune.setEnabled(enable_basic)
        self.btn_joint_load_state.setEnabled(enable_basic)
        self.btn_joint_send.setEnabled(enable_basic)
        self.btn_pid_auto_tune.setEnabled(enable_basic)
        self.btn_gamma_live_tune.setEnabled(enable_basic)
        
        # Set zero button (only in STANDBY mode)
        if hasattr(self, 'robot_state') and hasattr(self.robot_state, 'robot_mode'):
            current = self.robot_state.robot_mode
        else:
            current = -1
        
        self.btn_set_zero.setEnabled(bridge_on and current == ROBOTMODE.STANDBY)
        
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
    
    def _on_set_zero_completed(self):
        """Handle set zero completion"""
        if self.process_manager.is_running('set_zero'):
            self.process_manager.stop_process('set_zero', timeout=1.0)
        
        self.btn_set_zero.setEnabled(True)
        self.btn_set_zero.setText('Set Zero')
        self.log_widget.add_log('Motor zero points set successfully', LOGLEVEL.INFO, 'system')
    
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
        # Check if set_zero process has completed
        if self.btn_set_zero.text() == 'Setting Zero...':
            if not self.process_manager.is_running('set_zero'):
                # Process finished but callback wasn't triggered
                self._on_set_zero_completed()

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

        if self._manual_cmd_active:
            if self.btn_gamma_live_tune.isChecked():
                self._on_pid_auto_tune_clicked(log_result=False)
            self._publish_manual_joint_command_step()
        elif self._seq_state == ExecutionState.RUNNING_NODE:
            self._seq_tick()
    
    # ========================================================================
    # Custom Sequence – window
    # ========================================================================

    def _on_custom_cmd_clicked(self) -> None:
        """Open (or raise) the Custom Command Sequence window."""
        if self._custom_seq_window is None:
            self._custom_seq_window = CustomSequenceWindow(
                get_current_pose_fn=self._seq_get_current_pose,
                output_dir=PATHS.DEFAULT_OUTPUT_DIR,
            )
            self._custom_seq_window.run_requested.connect(self._seq_run)
            self._custom_seq_window.stop_requested.connect(self._seq_stop)
        self._custom_seq_window.show()
        self._custom_seq_window.raise_()
        self._custom_seq_window.activateWindow()

    def _seq_get_current_pose(self) -> dict | None:
        """Return current motor positions as dict[leg][joint] in degrees."""
        if not hasattr(self, 'motor_state') or not hasattr(self.motor_state, 'module_a'):
            return None
        pose: dict = {}
        for leg_id, module in zip(
            LEGS,
            [self.motor_state.module_a, self.motor_state.module_b,
             self.motor_state.module_c, self.motor_state.module_d],
        ):
            pose[leg_id] = {
                'theta': float(np.degrees(module.theta)),
                'beta':  float(np.degrees(module.beta)),
                'gamma': float(np.degrees(module.gamma)),
            }
        return pose

    # ========================================================================
    # Custom Sequence – executor
    # ========================================================================

    def _seq_run(self, sequence, limit_profile, dry_run: bool) -> None:
        """
        Called when CustomSequenceWindow emits run_requested.
        Validates and either simulates (dry_run) or starts execution.
        """
        if not sequence.nodes:
            self.log_widget.add_log('Sequence is empty – nothing to run.', LOGLEVEL.WARN, 'seq')
            return

        # Warn if CSV control is running
        if self.process_manager.is_running('csv_control'):
            self.log_widget.add_log(
                'WARNING: csv_control is running while starting sequence. '
                'Stop CSV control to avoid conflicting commands.',
                LOGLEVEL.WARN, 'seq'
            )

        # Validate reachability
        pose = self._seq_get_current_pose()
        errors = check_sequence_reachability(sequence, limit_profile, pose)
        if errors:
            self.log_widget.add_log(
                f'Sequence validation failed with {len(errors)} error(s):', LOGLEVEL.ERROR, 'seq'
            )
            for err in errors:
                self.log_widget.add_log(
                    f'  Node[{err.node_idx}] "{err.node_name}" '
                    f'leg={err.leg} joint={err.joint}: {err.message}',
                    LOGLEVEL.ERROR, 'seq'
                )
            if self._custom_seq_window:
                self._custom_seq_window.update_execution_state(
                    ExecutionState.FAILED, 0, len(sequence.nodes),
                    0.0, 0.0, 'Validation failed'
                )
            return

        total_time = sum(n.duration_sec for n in sequence.nodes)

        # ---- dry-run: simulate and report without sending commands ----------
        if dry_run:
            self.log_widget.add_log(
                f'[DRY-RUN] Sequence "{sequence.name}" – '
                f'{len(sequence.nodes)} nodes, total {total_time:.2f}s',
                LOGLEVEL.INFO, 'seq'
            )
            t_accum = 0.0
            for i, node in enumerate(sequence.nodes):
                self.log_widget.add_log(
                    f'  Node[{i}] "{node.name}"  duration={node.duration_sec:.2f}s  '
                    f'starts_at={t_accum:.2f}s',
                    LOGLEVEL.INFO, 'seq'
                )
                t_accum += node.duration_sec
            self.log_widget.add_log('[DRY-RUN] Complete (no commands sent)', LOGLEVEL.INFO, 'seq')
            if self._custom_seq_window:
                self._custom_seq_window.log_node_event(
                    f'[DRY-RUN] Validation OK – {len(sequence.nodes)} nodes, '
                    f'{total_time:.2f}s total', LOGLEVEL.INFO
                )
            return

        # ---- real run -------------------------------------------------------
        if not self.ros_worker.is_running:
            self.log_widget.add_log('ROS worker is not running.', LOGLEVEL.ERROR, 'seq')
            return

        # Stop any ongoing manual command
        self._manual_cmd_active = False

        # Store sequence metadata
        self._seq_nodes        = list(sequence.nodes)
        self._seq_limit_profile = limit_profile
        self._seq_sequence_name = sequence.name
        self._seq_dry_run       = False
        self._seq_node_records  = []

        self._seq_state = ExecutionState.RUNNING_NODE
        self.log_widget.add_log(
            f'Starting sequence "{sequence.name}"  '
            f'({len(sequence.nodes)} nodes, {total_time:.2f}s total)',
            LOGLEVEL.INFO, 'seq'
        )
        self._seq_start_node(0)

    def _seq_start_node(self, idx: int) -> None:
        """Begin execution of node *idx*."""
        self._seq_node_idx    = idx
        self._seq_node_start_ts = self.ros_worker.node.get_clock().now().nanoseconds / 1e9
        self._seq_last_tick_ts = self._seq_node_start_ts

        # Record start position from measured state to avoid startup jump.
        pose = self._seq_get_current_pose()
        if pose is not None:
            for leg in LEGS:
                for joint in JOINTS:
                    val = float(pose[leg][joint])
                    self._seq_node_start_pos[leg][joint] = val
                    self._manual_current_deg[leg][joint] = val
        else:
            for leg in LEGS:
                for joint in JOINTS:
                    self._seq_node_start_pos[leg][joint] = self._manual_current_deg[leg][joint]

        node = self._seq_nodes[idx]
        record = NodeExecutionRecord(
            node_idx=idx,
            node_name=node.name,
            started_at_wall=datetime.now().isoformat(timespec='milliseconds'),
            duration_planned_sec=node.duration_sec,
        )
        self._seq_node_records.append(record)

        msg = f'[{idx + 1}/{len(self._seq_nodes)}] Starting node "{node.name}"  ({node.duration_sec:.2f}s)'
        self.log_widget.add_log(msg, LOGLEVEL.INFO, 'seq')
        if self._custom_seq_window:
            self._custom_seq_window.log_node_event(msg, LOGLEVEL.INFO)
            # Highlight node in list
            self._custom_seq_window.list_nodes.setCurrentRow(idx)

    def _seq_tick(self) -> None:
        """Called every timer tick while RUNNING_NODE.  Publishes interpolated command."""
        if self._seq_state != ExecutionState.RUNNING_NODE:
            return
        if not self.ros_worker.is_running:
            self._seq_fail('ROS worker stopped unexpectedly')
            return

        node     = self._seq_nodes[self._seq_node_idx]
        now      = self.ros_worker.node.get_clock().now().nanoseconds / 1e9
        elapsed  = now - self._seq_node_start_ts
        t        = min(1.0, elapsed / max(node.duration_sec, 1e-6))
        dt       = max(0.005, min(0.2, now - self._seq_last_tick_ts))
        self._seq_last_tick_ts = now

        # Build interpolated MotorCmdStamped
        msg = MotorCmdStamped()
        msg.header.seq   = self._robot_cmd_seq + 1
        msg.header.stamp = self.ros_worker.node.get_clock().now().to_msg()

        gains = getattr(node, 'gains', {}) if isinstance(getattr(node, 'gains', {}), dict) else {}
        leg_kp   = float(gains.get('leg_kp', self.spin_leg_kp.value()))
        leg_kd   = float(gains.get('leg_kd', self.spin_leg_kd.value()))
        gamma_kp = float(gains.get('gamma_kp', self.spin_gamma_kp.value()))
        gamma_kd = float(gains.get('gamma_kd', self.spin_gamma_kd.value()))

        for module_name, leg_id in (
            ('module_a', 'A'), ('module_b', 'B'),
            ('module_c', 'C'), ('module_d', 'D'),
        ):
            module = getattr(msg, module_name)
            for joint in JOINTS:
                start  = self._seq_node_start_pos[leg_id][joint]
                target = getattr(node.targets[leg_id], joint)
                desired_deg = start + t * (target - start)
                current_deg = self._manual_current_deg[leg_id][joint]
                speed_limit = self._seq_joint_speed_limit(leg_id, joint)
                max_step = speed_limit * dt
                delta = desired_deg - current_deg
                if abs(delta) > max_step:
                    current_deg += np.sign(delta) * max_step
                    all_reached = False
                else:
                    current_deg = desired_deg
                self._manual_current_deg[leg_id][joint] = current_deg
                setattr(module, joint, np.radians(current_deg))

            module.kp_r = leg_kp
            module.kp_l = leg_kp
            module.kp_h = gamma_kp
            module.kd_r = leg_kd
            module.kd_l = leg_kd
            module.kd_h = gamma_kd
            module.ki_r = module.ki_l = module.ki_h = 0.0
            module.torque_r = module.torque_l = module.torque_h = 0.0

        self.ros_worker.send_motor_command(msg)
        self._robot_cmd_seq += 1

        # Update window progress
        if self._custom_seq_window:
            remaining_nodes_time = sum(
                n.duration_sec for n in self._seq_nodes[self._seq_node_idx + 1:]
            )
            eta = node.duration_sec * (1.0 - t) + remaining_nodes_time
            self._custom_seq_window.update_execution_state(
                ExecutionState.RUNNING_NODE,
                self._seq_node_idx,
                len(self._seq_nodes),
                t, eta,
            )

        # Advance to next node when this one completes
        if t >= 1.0:
            reached = True
            for leg in LEGS:
                for joint in JOINTS:
                    target = getattr(node.targets[leg], joint)
                    if abs(self._manual_current_deg[leg][joint] - target) > 0.5:
                        reached = False
                        break
                if not reached:
                    break
            if reached or elapsed > node.duration_sec + 0.5:
                # Snap final value if timeout grace elapsed.
                for leg in LEGS:
                    for joint in JOINTS:
                        self._manual_current_deg[leg][joint] = float(getattr(node.targets[leg], joint))
                self._seq_advance_node(now)

    def _seq_joint_speed_limit(self, leg_id: str, joint: str) -> float:
        """Return deg/s speed limit from profile, or a very large default."""
        if self._seq_limit_profile is None:
            return 1e6
        try:
            entry = self._seq_limit_profile.limits.get((leg_id, joint))
            if entry is None:
                return 1e6
            return float(max(0.0, entry.max_speed_deg_per_sec))
        except Exception:
            return 1e6

    def _seq_advance_node(self, now: float) -> None:
        """Finalise current node and move to the next (or complete)."""
        idx  = self._seq_node_idx
        node = self._seq_nodes[idx]
        rec  = self._seq_node_records[idx]
        rec.finished_at_wall   = datetime.now().isoformat(timespec='milliseconds')
        rec.duration_actual_sec = now - self._seq_node_start_ts
        rec.result             = 'success'

        self.log_widget.add_log(
            f'  Node "{node.name}" done  (actual {rec.duration_actual_sec:.2f}s)',
            LOGLEVEL.INFO, 'seq'
        )

        next_idx = idx + 1
        if next_idx < len(self._seq_nodes):
            self._seq_start_node(next_idx)
        else:
            self._seq_complete()

    def _seq_complete(self) -> None:
        self._seq_state = ExecutionState.COMPLETED
        self.log_widget.add_log(
            f'Sequence "{self._seq_sequence_name}" completed successfully.',
            LOGLEVEL.INFO, 'seq'
        )
        self._seq_finalise_records('success')
        if self._custom_seq_window:
            self._custom_seq_window.update_execution_state(
                ExecutionState.COMPLETED, len(self._seq_nodes) - 1,
                len(self._seq_nodes), 1.0, 0.0
            )

    def _seq_stop(self) -> None:
        """Stop sequence execution (user-initiated or window close)."""
        if self._seq_state != ExecutionState.RUNNING_NODE:
            return
        self._seq_state = ExecutionState.STOPPED
        idx = self._seq_node_idx
        self.log_widget.add_log(
            f'Sequence stopped at node [{idx}] "{self._seq_nodes[idx].name}".',
            LOGLEVEL.WARN, 'seq'
        )
        # Finalise current node record
        if idx < len(self._seq_node_records):
            rec = self._seq_node_records[idx]
            if rec.finished_at_wall is None:
                rec.finished_at_wall   = datetime.now().isoformat(timespec='milliseconds')
                rec.duration_actual_sec = self.ros_worker.node.get_clock().now().nanoseconds / 1e9 - self._seq_node_start_ts
                rec.result             = 'stopped'
        self._seq_finalise_records('stopped')
        if self._custom_seq_window:
            self._custom_seq_window.update_execution_state(
                ExecutionState.STOPPED, idx, len(self._seq_nodes), 0.0, 0.0
            )

    def _seq_fail(self, message: str) -> None:
        """Abort sequence due to an error."""
        self._seq_state = ExecutionState.FAILED
        idx = self._seq_node_idx
        self.log_widget.add_log(
            f'Sequence FAILED at node [{idx}]: {message}', LOGLEVEL.ERROR, 'seq'
        )
        if idx < len(self._seq_node_records):
            rec = self._seq_node_records[idx]
            if rec.finished_at_wall is None:
                rec.finished_at_wall   = datetime.now().isoformat(timespec='milliseconds')
                rec.duration_actual_sec = self.ros_worker.node.get_clock().now().nanoseconds / 1e9 - self._seq_node_start_ts
                rec.result             = 'failed'
                rec.message            = message
        self._seq_finalise_records('failed')
        if self._custom_seq_window:
            self._custom_seq_window.update_execution_state(
                ExecutionState.FAILED, idx, len(self._seq_nodes),
                0.0, 0.0, message
            )

    def _seq_finalise_records(self, overall_result: str) -> None:
        """Mark any still-running records as *overall_result* and save to disk."""
        now_str = datetime.now().isoformat(timespec='milliseconds')
        for rec in self._seq_node_records:
            if rec.finished_at_wall is None:
                rec.finished_at_wall   = now_str
                rec.duration_actual_sec = 0.0
                rec.result             = overall_result

        # Auto-save execution record
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_name = self._seq_sequence_name.replace(' ', '_') or 'sequence'
        rec_path  = os.path.join(
            PATHS.DEFAULT_OUTPUT_DIR,
            f'{safe_name}_{timestamp}.json'
        )
        try:
            save_execution_record(rec_path, self._seq_sequence_name, self._seq_node_records)
            self.log_widget.add_log(f'Execution record saved → {rec_path}', LOGLEVEL.INFO, 'seq')
        except Exception as e:
            self.log_widget.add_log(f'Could not save execution record: {e}', LOGLEVEL.WARN, 'seq')

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

        # Close sequence window
        if self._custom_seq_window is not None:
            self._custom_seq_window.close()
            self._custom_seq_window = None

        # Stop ROS worker
        self.ros_worker.stop_ros()
        
        # Accept close event
        super().closeEvent(event)


# Entry point for standalone execution
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CorgiControlPanel()
    sys.exit(app.exec_())
