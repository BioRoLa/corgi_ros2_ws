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
    QGroupBox, QLineEdit, QGridLayout, QFrame, QFileDialog, QApplication,
    QMessageBox
)
from PyQt5.QtCore import Qt, QTimer

from corgi_msgs.msg import RobotCmdStamped, TriggerStamped

# Import from corgi_ui package
from corgi_ui.core.constants import (
    ROBOTMODE, LOGLEVEL, COLORS, PATHS,
    LOGLEVEL_TO_LOGGING_MAP,
    setup_file_logger, log_to_file, close_file_logger
)
from corgi_ui.core import sbrio
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

    # The FSM's own names read as a flat list of synonyms -- "System On",
    # "Idle" and "Standby" all sound like the robot is doing nothing, and
    # "Idle" in particular sounds like the BOTTOM of the ladder when it is
    # the middle rung and the only door to the state where the robot runs.
    # These are the operator-facing names, describing position in the
    # workflow and what entering each state actually does. The BUTTONS keep
    # them IMPERATIVE ("Set Powered", "Go Live") -- a bare state name on a
    # greyed-out button reads as a status readout claiming the robot is in
    # that state, which is exactly what the first cut of this rename did.
    #
    #     Powered (SYSTEM_ON 0) <-> Arm (IDLE 2) <-> Live (STANDBY 3)
    #     Motor Config (MOTORCONFIG 4) hangs off Powered / Arm.
    #
    # IDLE is emphatically NOT idle: entering it runs the motors' HALL
    # CALIBRATION (Alex, 2026-08-31 -- the FSM lives in sbRIO firmware this
    # repo cannot see, so this is not derivable from the source here). That
    # is why it is the only door to Live, and why calling it 'Idle' -- or
    # 'Ready' -- misleads in the direction that matters: it sounds like a
    # state where nothing happens, and it is a state where the motors move.
    #
    # The canonical name is never hidden: it rides on every button's tooltip
    # and in the mode readout, because the firmware, the logs and the gRPC
    # enum all still say STANDBY.
    MODE_LABEL = {
        ROBOTMODE.SYSTEM_ON:   'Powered',
        ROBOTMODE.INIT:        'Init',
        ROBOTMODE.IDLE:        'Armed',
        ROBOTMODE.STANDBY:     'Live',
        ROBOTMODE.MOTORCONFIG: 'Motor Config',
    }

    @classmethod
    def _mode_text(cls, mode) -> str:
        """'Live \u00b7 STANDBY' -- friendly first, canonical alongside, so the
        panel and the robot's own log lines can be matched up at a glance."""
        try:
            enum = ROBOTMODE(mode)
        except ValueError:
            return '---'
        return '%s \u00b7 %s' % (cls.MODE_LABEL.get(enum, enum.name), enum.name)

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
        # None = not yet known (the panel does not ssh at startup just to
        # find out); True/False only ever set from what the sbRIO reported.
        self._fpga_running = None
        
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

        # Homing in progress. Tracked as REAL STATE, not inferred from
        # the button's text: the completion poller used to key on
        # text == 'Homing...', so any handler that reset the text (a
        # failed second press did exactly that) silently disabled
        # completion detection for the rest of the session.
        self._homing_active = False
        # Did the homing node actually report completion? It has abort
        # paths (theta below the 17 deg target, or no motor/state), and
        # the panel used to announce 'home position set successfully'
        # whenever the PROCESS EXITED -- abort included. Believing that
        # leaves the zero unset while you think it is set, corrupting
        # every theta/beta reference downstream.
        self._homing_saw_complete = False

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
        
        # sbRIO FPGA driver -- the bottom of the stack, and the one step
        # that used to be a hand-typed ssh session left open in a terminal.
        # It sits above the bridge button because that is the order: driver,
        # then bridge, then everything else.
        # One driver, one button. Not setCheckable: the checked state would
        # flip on the click, before the sbRIO has said anything, so a failed
        # start would sit there looking started.
        self.btn_fpga_driver = QPushButton()
        self.btn_fpga_driver.clicked.connect(self._on_fpga_driver_clicked)
        self._refresh_fpga_button()

        if hasattr(self, 'use_sim_time') and self.use_sim_time:
            self.btn_fpga_driver.setEnabled(False)
            self.btn_fpga_driver.setToolTip('No sbRIO in simulation mode')

        sidebar.addWidget(self.btn_fpga_driver)

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
        # Friendly name first, canonical second -- see MODE_LABEL. The
        # readout is the one place both have to be visible, because the
        # robot's own log lines say STANDBY while this panel says Live.
        lbl_mode_title.setStyleSheet("color: #888; font-size: 12px;")
        
        self.label_robot_mode_value = QLabel("no robot state")
        self.label_robot_mode_value.setObjectName("StatusLabel")
        self.label_robot_mode_value.setAlignment(Qt.AlignCenter)
        self.label_robot_mode_value.setStyleSheet(
            "color: #bdbdbd; font-weight: bold; font-size: 16px;"
        )
        
        # Every FSM button is gated on a mode arriving on /robot/state. When
        # none ever does, all four sit greyed out with nothing saying why --
        # and the cause is upstream of the panel every time.
        self.label_robot_mode_value.setToolTip(
            'The mode comes from /robot/state, via the bridge, from the '
            'sbRIO.\nWhile this reads "no robot state" the FSM buttons stay '
            'disabled and the fault is upstream of the panel:\n'
            '  - is the FPGA driver up AND STAYING up on the sbRIO?\n'
            '  - is the ROS bridge running?\n'
            'Mode commands are still sent -- they are simply never confirmed.')
        mode_h_layout.addWidget(lbl_mode_title)
        mode_h_layout.addWidget(self.label_robot_mode_value)
        grp_fsm_layout.addWidget(mode_container)
        
        # FSM Buttons
        self.btn_systemon = QPushButton('1 \u00b7 Set Powered')
        self.btn_systemon.setToolTip(
            'SYSTEM_ON (0). The state the robot boots into, and where the '
            'e-stop retreats to from anywhere but Live.\nUp: Arm.')
        self.btn_systemon.setObjectName("SystemOnBtn")
        self.btn_systemon.setCheckable(True)
        self.btn_systemon.clicked.connect(lambda: self._request_robot_mode(ROBOTMODE.SYSTEM_ON))
        self.btn_systemon.setEnabled(False)
        
        self.btn_idle = QPushButton('2 \u00b7 Arm  \u2014  hall calib')
        self.btn_idle.setToolTip(
            "IDLE (2). Entering this runs the motors' HALL CALIBRATION and "
            'leaves them armed \u2014 the motors MOVE here; this is not a '
            'rest state.\nAlso where you sit between runs, and where the '
            'e-stop drops you FROM Live.\nDown: Powered.  Up: Live.')
        self.btn_idle.setCheckable(True)
        self.btn_idle.clicked.connect(lambda: self._request_robot_mode(ROBOTMODE.IDLE))
        self.btn_idle.setEnabled(False)
        
        self.btn_standby = QPushButton('3 \u00b7 Go Live  \u2014  home && run')
        self.btn_standby.setToolTip(
            'STANDBY (3). Commands are accepted here: homing and gaits run '
            'in this state, and nowhere else.\nDown: Arm.')
        self.btn_standby.setCheckable(True)
        self.btn_standby.clicked.connect(lambda: self._request_robot_mode(ROBOTMODE.STANDBY))
        self.btn_standby.setEnabled(False)
        
        self.btn_motorconfig = QPushButton('Enter Motor Config')
        self.btn_motorconfig.setToolTip(
            'MOTORCONFIG (4). Side branch off Powered / Arm, not a rung '
            'on the ladder. Opens the config panel on entry.')
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
        self.btn_home = QPushButton('Set Home')
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

    # Direction-table rows. Each row lists the key under BOTH schemas, because
    # there are two of these files and they do not agree on names:
    #
    #   hardware  corgi_ros_bridge/config/real_motor_config.yaml
    #             joint_dir{theta, beta, gamma}   motor_dir{R, L, H}
    #   sim       corgi_driver_pkg/motor_config.yaml
    #             joint_dir{theta, beta, g_joint_beta}  motor_dir{L, R, ABAD}
    CONFIG_FIELDS = [
        ('joint_dir', ('theta',),                'θ  joint dir'),
        ('joint_dir', ('beta',),                 'β  joint dir'),
        ('joint_dir', ('gamma', 'g_joint_beta'), 'γ / G-joint dir'),
        ('motor_dir', ('L',),                    'Motor L dir'),
        ('motor_dir', ('R',),                    'Motor R dir'),
        ('motor_dir', ('H', 'ABAD'),             'ABAD motor dir'),
    ]

    def _load_motor_config(self) -> dict:
        """Load the direction config that is ACTUALLY IN EFFECT.

        The hardware file is read from the SOURCE tree, because that is the
        path corgi_ros_bridge defaults to; override with
        CORGI_REAL_MOTOR_CONFIG if the bridge was launched with a
        real_motor_config_path parameter pointing elsewhere.

        This used to look only for the simulator's motor_config.yaml inside
        corgi_driver_pkg, a package that does not exist on the robot — so on
        hardware every cell rendered '?'. Had it resolved, it would have been
        worse: the panel would have shown the SIMULATOR's mirror signs while
        the bridge was applying the real robot's. Load the file matching the
        current mode first, and remember which one won so the group box can
        name it.
        """
        import importlib.util
        self._config_source = None
        self._config_source_kind = None

        def _real_source():
            # What corgi_ros_bridge ACTUALLY reads: its
            # default_real_motor_config_path() points into the source tree, not
            # the installed share, so the bridge picks up an edit without a
            # rebuild. The panel has to read the same file or it can show
            # yesterday's signs while the robot uses today's.
            override = os.environ.get('CORGI_REAL_MOTOR_CONFIG')
            if override:
                return override, 'CORGI_REAL_MOTOR_CONFIG'
            return (os.path.join(
                os.path.expanduser('~'),
                'corgi_ws/corgi_ros2_ws/src/corgi_ros_bridge/config',
                'real_motor_config.yaml'), 'bridge source')

        def _real_share():
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('corgi_ros_bridge')
            return (os.path.join(share, 'config', 'real_motor_config.yaml'),
                    'installed share — the bridge may be reading a newer source copy')

        def _sim():
            spec = importlib.util.find_spec('corgi_driver_pkg')
            if spec and spec.origin:
                return (os.path.join(os.path.dirname(spec.origin),
                                     'motor_config.yaml'), 'simulator')
            from ament_index_python.packages import get_package_share_directory
            return (os.path.join(get_package_share_directory('corgi_sim'),
                                 'motor_config.yaml'), 'simulator')

        order = ([_sim, _real_source, _real_share] if self.use_sim_time
                 else [_real_source, _real_share, _sim])
        for getter in order:
            try:
                path, kind = getter()
                with open(path, 'r') as f:
                    cfg = yaml.safe_load(f) or {}
                if cfg:
                    self._config_source = path
                    self._config_source_kind = kind
                    return cfg
            except Exception as e:
                print(f"Warning: direction config not readable ({e})")

        return {}

    @staticmethod
    def _config_value(cfg, mod_id, section, keys):
        """First key present under this schema, or None when the row simply
        does not exist in the file that was loaded (rendered as a dash, which
        is a different statement from '?')."""
        sect = (cfg or {}).get(mod_id, {}).get(section, {}) or {}
        for k in keys:
            if k in sect:
                return sect[k]
        return None

    def _set_config_title(self):
        """Name the loaded file on the group box. Which file it is matters
        more than the signs it carries: an unlabelled direction table invites
        reading the wrong robot's mirror convention."""
        src = getattr(self, '_config_source', None)
        kind = getattr(self, '_config_source_kind', None)
        if src:
            self._config_grp.setTitle(
                "Motor Direction Config  —  %s  [%s]"
                % (os.path.basename(src), kind or '?'))
            self._config_grp.setToolTip(src)
        else:
            self._config_grp.setTitle(
                "Motor Direction Config  —  NOT LOADED (no yaml found)")
            self._config_grp.setToolTip(
                'Looked for the bridge config in the source tree, then the '
                'installed share, then the simulator package.')

    def _create_motor_config_display(self) -> QGroupBox:
        """Create a read-only display of the motor direction config (motor_config.yaml)."""
        cfg = self._load_motor_config()

        self._config_grp = QGroupBox()
        grp = self._config_grp
        self._set_config_title()
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
        FIELDS = self.CONFIG_FIELDS

        # Header row
        self._config_grid.addWidget(QLabel(""), 0, 0)
        for col, mod_id in enumerate(('A', 'B', 'C', 'D'), start=1):
            hdr = QLabel(MODULE_LABEL[mod_id])
            hdr.setAlignment(Qt.AlignCenter)
            hdr.setStyleSheet("font-weight: bold; color: #ccc;")
            self._config_grid.addWidget(hdr, 0, col)

        # Data rows
        self._config_value_labels = {}
        for row, (section, keys, label_text) in enumerate(FIELDS, start=1):
            row_lbl = QLabel(label_text)
            row_lbl.setStyleSheet("color: #aaa;")
            self._config_grid.addWidget(row_lbl, row, 0)

            for col, mod_id in enumerate(('A', 'B', 'C', 'D'), start=1):
                val = self._config_value(cfg, mod_id, section, keys)
                lbl = QLabel(self._fmt_dir(val))
                lbl.setAlignment(Qt.AlignCenter)
                lbl.setStyleSheet(self._dir_style(val))
                self._config_grid.addWidget(lbl, row, col)
                self._config_value_labels[(mod_id, section, keys)] = lbl

        outer.addLayout(self._config_grid)
        grp.setLayout(outer)
        return grp

    @staticmethod
    def _fmt_dir(val) -> str:
        """+1 / -1; an em dash when the row does not exist in the loaded
        schema; '?' only when the value is there but unreadable."""
        if val is None:
            return '—'
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
        """Reload the direction config and refresh every label in the grid."""
        cfg = self._load_motor_config()
        self._set_config_title()
        for mod_id in ('A', 'B', 'C', 'D'):
            for section, keys, _label in self.CONFIG_FIELDS:
                lbl = self._config_value_labels.get((mod_id, section, keys))
                if lbl is None:
                    continue
                val = self._config_value(cfg, mod_id, section, keys)
                lbl.setText(self._fmt_dir(val))
                lbl.setStyleSheet(self._dir_style(val))
        src = getattr(self, '_config_source', None)
        if src:
            self._log(f"Direction config reloaded from {src}",
                      LOGLEVEL.INFO, "system")
        else:
            self._log("No direction config found — table shows nothing",
                      LOGLEVEL.WARN, "system")

    
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

        # The CV7 hangs off /dev/ttyTHS1 and the packaged imu_node.sh opens
        # with `sudo chmod 777` on it -- so the port is NOT reliably writable
        # after a boot. ProcessManager only reports whether the spawn
        # succeeded; a node that starts and then dies on the port looks
        # identical to a healthy one from here, and the result is a capture
        # with silently empty IMU columns. Check the port first and say what
        # to do, rather than discovering it when P-HW-rho turns out unscoreable.
        imu_port = os.environ.get('CORGI_IMU_PORT', '/dev/ttyTHS1')
        if not os.path.exists(imu_port):
            self._log('IMU port %s not present -- IMU columns will be empty'
                      % imu_port, LOGLEVEL.WARN, 'system')
        elif not os.access(imu_port, os.R_OK | os.W_OK):
            self._log('IMU port %s is not writable; run  sudo chmod 777 %s  '
                      '(once per boot) or the IMU node will die on startup'
                      % (imu_port, imu_port), LOGLEVEL.WARN, 'system')

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
    # Event Handlers - sbRIO FPGA driver
    # ========================================================================

    def _refresh_fpga_button(self):
        """Label and tooltip follow _fpga_running, which follows the sbRIO."""
        if self._fpga_running:
            self.btn_fpga_driver.setText('Stop FPGA Driver (sbRIO)')
            self.btn_fpga_driver.setToolTip(
                'Stop grpccore + fpga_driver on %s.\n'
                'SIGTERM first so the FPGA session is released cleanly.\n'
                'Refused while the robot is Live or the trigger is on.'
                % sbrio.SBRIO_HOST)
        else:
            self.btn_fpga_driver.setText('Start FPGA Driver (sbRIO)')
            tip = ('ssh to %s and run the FPGA driver.\n'
                   'Needs key-based ssh; see corgi_ui/core/sbrio.py for the '
                   'one-time setup.' % sbrio.target_description())
            if self._fpga_running is None:
                tip += ('\nState not yet known — the panel does not ssh at '
                        'startup. Pressing this is safe either way: a driver '
                        'that is already up reports so and is left alone.')
            self.btn_fpga_driver.setToolTip(tip)

    def _on_fpga_driver_clicked(self):
        """One driver, one button: start it, or stop it."""
        if self.use_sim_time:
            self._log('No sbRIO in simulation mode', LOGLEVEL.WARN, 'fpga_driver')
            return
        if self._fpga_running:
            self._request_fpga_stop()
        else:
            self._start_fpga_driver(manual=True)

    def _request_fpga_stop(self):
        """Stop grpccore + fpga_driver on the sbRIO."""

        # Refuse under a live gait. Taking the FPGA layer away from a robot
        # that is running is not a stop, it is a fall: the motors lose their
        # command source with the legs loaded. The e-stop is the control for
        # that situation; this button is for between runs.
        mode = getattr(getattr(self, 'robot_state', None), 'robot_mode', -1)
        if self.btn_trigger.isChecked() or mode == ROBOTMODE.STANDBY:
            why = []
            if self.btn_trigger.isChecked():
                why.append('the trigger is ON')
            if mode == ROBOTMODE.STANDBY:
                why.append('the robot is Live (STANDBY)')
            self._log('REFUSED to stop the FPGA driver: %s. Drop to Armed '
                      '(IDLE) and stop the trigger first -- pulling the command '
                      'source out from under a running gait is not a stop. Use '
                      'E-STOP if you need the robot to stop NOW.'
                      % ' and '.join(why), LOGLEVEL.ERROR, 'fpga_driver')
            return

        answer = QMessageBox.question(
            self, 'Stop the FPGA driver?',
            'This stops grpccore and fpga_driver on the sbRIO (%s).\n\n'
            'The ROS bridge will lose its gRPC server, and nothing can '
            'command the motors until the driver is started again.\n\n'
            'Continue?' % sbrio.SBRIO_HOST,
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            self._log('FPGA driver stop cancelled', LOGLEVEL.INFO, 'fpga_driver')
            return

        self._stop_fpga_driver()

    def _stop_fpga_driver(self) -> bool:
        """SIGTERM on the sbRIO, escalating only if it will not go."""
        self.btn_fpga_driver.setEnabled(False)
        self.btn_fpga_driver.setText('Stopping FPGA driver…')
        QApplication.setOverrideCursor(Qt.WaitCursor)
        QApplication.processEvents()
        try:
            token, lines = sbrio.stop_fpga_driver()
        finally:
            QApplication.restoreOverrideCursor()
            self.btn_fpga_driver.setEnabled(True)

        ok = token in ('STOPPED', 'STOPPED_HARD', 'NOT_RUNNING')
        headline = {
            'STOPPED':      'FPGA driver stopped on %s' % sbrio.SBRIO_HOST,
            'STOPPED_HARD': 'FPGA driver KILLED on %s -- it ignored SIGTERM; if '
                            'the next start fails, reboot the sbRIO'
                            % sbrio.SBRIO_HOST,
            'NOT_RUNNING':  'FPGA driver was not running on %s' % sbrio.SBRIO_HOST,
            'NO_KEY':       'sbRIO refused key-based ssh',
            'UNREACHABLE':  'sbRIO %s unreachable' % sbrio.SBRIO_HOST,
            'TIMEOUT':      'sbRIO %s did not answer' % sbrio.SBRIO_HOST,
            'NO_SSH':       'no ssh client on this machine',
        }.get(token, 'FPGA driver did not stop')

        level = LOGLEVEL.INFO if token in ('STOPPED', 'NOT_RUNNING') else (
            LOGLEVEL.WARN if token == 'STOPPED_HARD' else LOGLEVEL.ERROR)
        self._log(headline, level, 'fpga_driver')
        for line in lines[:12]:
            self._log('  ' + line,
                      LOGLEVEL.INFO if ok else LOGLEVEL.WARN, 'fpga_driver')
        if ok:
            self._fpga_running = False
        self._refresh_fpga_button()
        return ok

    def _stray_bridges(self):
        """PIDs of corgi_ros_bridge processes this panel did not start.

        A bridge outliving the panel that spawned it is easy to produce --
        close the panel uncleanly and ProcessManager never gets to clean up
        -- and it keeps trying to reach the sbRIO forever.
        """
        import subprocess as sp
        try:
            out = sp.run(['pgrep', '-f', 'corgi_ros_bridge'],
                         capture_output=True, text=True, timeout=5)
        except Exception:
            return []
        mine = {self.process_manager.get_pid('ros_bridge'), os.getpid()}
        pids = []
        for tok in out.stdout.split():
            if tok.isdigit() and int(tok) not in mine:
                pids.append(tok)
        return pids

    def _launch_fpga(self, allow_prompt: bool = True):
        """Open the driver's UI in a terminal window, then verify over ssh.

        The headless launches all aborted -- three different theories about
        why, all wrong -- while running it in a terminal works every time.
        So use the working path and verify separately, rather than guess a
        fourth time at which property of a terminal session matters.

        Falls back to the headless launch when there is no display or no
        terminal emulator, so a panel running over plain ssh still has the
        old behaviour rather than nothing.
        """
        # Is one already up? The remote start script guards against this,
        # but the terminal launcher does not run that script -- it runs
        # run_fpga_driver.sh directly -- so without this check pressing Run
        # ROS Bridge on a working driver opens a second one.
        st, st_lines = sbrio.fpga_driver_status()
        if st == 'ALREADY_RUNNING':
            return 'ALREADY_RUNNING', (
                ['already up on the sbRIO — not opening another window']
                + st_lines)

        if st not in ('NOT_RUNNING',):
            # Cannot tell. A duplicate driver is worse than a refused start,
            # so do not guess -- ask, and only when a human pressed the button.
            if not allow_prompt:
                return 'UNVERIFIED_SKIPPED', [
                    'cannot check whether the driver is already running (%s), '
                    'so not starting one automatically.' % st,
                    'Press "Start FPGA Driver" yourself, or install the ssh '
                    'key so the panel can check (see corgi_ui/core/sbrio.py).']
            answer = QMessageBox.question(
                self, 'Cannot check the FPGA driver',
                'The panel cannot tell whether the driver is already running '
                'on %s (%s).\n\nOpening a second one would leave two '
                'drivers contending for the same FPGA.\n\n'
                'Open a terminal and start it anyway?' % (sbrio.SBRIO_HOST, st),
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer != QMessageBox.Yes:
                return 'UNVERIFIED_SKIPPED', ['cancelled — nothing started']

        # grpccore aborts while SERVICING a registration, not while
        # starting (its own log: binds, "receive publisher", then abort). Its
        # only client is the bridge, so a bridge already running when the
        # driver comes up is the first thing to rule out.
        strays = self._stray_bridges()
        if strays:
            self._log('WARNING: corgi_ros_bridge is ALREADY RUNNING (pid %s) '
                      'and this panel did not start it. It will connect to '
                      'grpccore the moment the driver comes up, and grpccore '
                      'has been aborting while servicing exactly that. Stop it '
                      'first:  kill %s'
                      % (', '.join(strays), ' '.join(strays)),
                      LOGLEVEL.ERROR, 'fpga_driver')

        token, lines = sbrio.launch_in_terminal()

        if token != 'LAUNCHED_TERMINAL':
            self._log('cannot open a terminal window (%s) — falling back to '
                      'the headless launch, which is known to abort on this '
                      'driver' % token, LOGLEVEL.WARN, 'fpga_driver')
            for line in lines[:4]:
                self._log('  ' + line, LOGLEVEL.WARN, 'fpga_driver')
            return sbrio.start_fpga_driver()

        self._log(lines[0] if lines else 'terminal opened',
                  LOGLEVEL.INFO, 'fpga_driver')
        self._log('  watch that window: it shows the driver\'s dashboard, and '
                  'holds the reason on screen if it exits',
                  LOGLEVEL.INFO, 'fpga_driver')

        # Launching a window is not starting a driver. Ask the sbRIO.
        self.btn_fpga_driver.setText('Verifying FPGA driver…')
        for attempt in range(8):
            QApplication.processEvents()
            st, st_lines = sbrio.fpga_driver_status()
            if st == 'ALREADY_RUNNING':
                return 'STARTED', ['confirmed running on the sbRIO'] + st_lines
            if st in ('NO_KEY', 'NO_SSH', 'UNREACHABLE'):
                # The terminal window does not need a key; this check does.
                return 'UNVERIFIED', [
                    'the terminal window is open and may well be running the '
                    'driver, but this panel cannot confirm it (%s).' % st,
                    'Look at the window. To let the panel check and stop it, '
                    'install the ssh key — see corgi_ui/core/sbrio.py.'] + st_lines
        return 'FAILED', [
            'a terminal opened but no driver was running on the sbRIO 8 '
            'checks later — read that window, it holds the reason'] + st_lines

    def _start_fpga_driver(self, manual: bool) -> bool:
        """Bring the driver up over ssh and report what happened.

        This blocks the GUI, deliberately: it is bounded (5 s to connect, 30 s
        overall), it only ever runs at bring-up, and nothing else in the panel
        is useful until it finishes. A worker thread here would buy an
        unresponsive-looking wait instead of an honest one.
        """
        self.btn_fpga_driver.setEnabled(False)
        self.btn_fpga_driver.setText('Starting FPGA driver…')
        QApplication.setOverrideCursor(Qt.WaitCursor)
        QApplication.processEvents()
        try:
            token, lines = self._launch_fpga(allow_prompt=manual)
        finally:
            QApplication.restoreOverrideCursor()
            self.btn_fpga_driver.setEnabled(True)

        ok = token in ('ALREADY_RUNNING', 'STARTED', 'UNVERIFIED')
        # UNVERIFIED_SKIPPED is not a failure of the driver, just of the
        # check -- log it as a warning rather than an error.
        if token == 'UNVERIFIED_SKIPPED':
            self._log('FPGA driver not started (could not verify whether one '
                      'was already running)', LOGLEVEL.WARN, 'fpga_driver')
            for line in lines[:4]:
                self._log('  ' + line, LOGLEVEL.WARN, 'fpga_driver')
            self._refresh_fpga_button()
            return False
        headline = {
            'STARTED':         'FPGA driver started on %s' % sbrio.SBRIO_HOST,
            'ALREADY_RUNNING': 'FPGA driver already running on %s' % sbrio.SBRIO_HOST,
            'FAILED':          'FPGA driver did not come up on %s -- its own '
                               'log follows' % sbrio.SBRIO_HOST,
            'NOT_RUNNING':     'FPGA driver is not running on %s' % sbrio.SBRIO_HOST,
            'NO_SCRIPT':       'FPGA start script not found (%s)' % sbrio.target_description(),
            'NO_KEY':          'sbRIO refused key-based ssh',
            'UNREACHABLE':     'sbRIO %s unreachable' % sbrio.SBRIO_HOST,
            'TIMEOUT':         'sbRIO %s did not answer' % sbrio.SBRIO_HOST,
            'NO_SSH':          'no ssh client on this machine',
            'UNVERIFIED':      'FPGA driver launched in a terminal — NOT '
                               'confirmed by this panel',
            'UNVERIFIED_SKIPPED': 'FPGA driver NOT started — could not check '
                                  'whether one is already running',
        }.get(token, 'FPGA driver: unrecognised result %r' % token)

        self._log(headline, LOGLEVEL.INFO if ok else LOGLEVEL.ERROR, 'fpga_driver')
        for line in lines[:12]:
            self._log('  ' + line,
                      LOGLEVEL.INFO if ok else LOGLEVEL.WARN, 'fpga_driver')
        if not ok and not manual:
            self._log('  starting the bridge anyway — start the driver by hand '
                      'if it is really down', LOGLEVEL.WARN, 'fpga_driver')
        if ok:
            self._fpga_running = True
        self._refresh_fpga_button()
        return ok

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
            
            # The bridge has nothing to talk to until the sbRIO's FPGA driver
            # is up, so bring it up here instead of leaving it as a step to
            # remember. Best-effort by design: a failure here is logged and the
            # bridge still starts, because the driver may have been started by
            # hand, or from another machine, or ssh keys may not be installed.
            self._start_fpga_driver(manual=False)

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

        # STEP 1 -- CUT THE COMMAND SOURCE FIRST.
        #
        # Requesting a robot-mode change on its own does NOT stop anything
        # publishing motor commands: gslip_pronk keeps streaming a moving
        # gait trajectory at 1 kHz while the firmware changes mode under
        # it, and the two fight every control cycle -- that is the loud
        # buzzing after an e-stop (observed 2026-08-31, motors energised
        # and conflicted, which is not the same as stopped).
        #
        # The controller's gait loop is `while (rclcpp::ok() && trigger_)`
        # (gslip_pronk.cpp:3567), so dropping the trigger is its designed
        # abort: the loop exits, run() returns, the node terminates, and
        # nothing publishes /motor/command afterwards. It also closes the
        # recording cleanly, so the data up to the abort survives.
        #
        # Reuses the trigger handler wholesale (same idiom as
        # _on_recording_input_entered) so the message, the GPIO line and
        # the logging all follow their tested path.
        if self.btn_trigger.isChecked():
            self.btn_trigger.setChecked(False)
            self._on_trigger_clicked()
            self.log_widget.add_log(
                'E-Stop: trigger dropped -- controller gait loop exits, '
                'motor command stream stops', LOGLEVEL.WARN, 'orin')

        # STEP 2 -- then the mode request.
        #
        # NOTE this remains a REQUEST forwarded over gRPC to the robot's
        # own state machine: there is no ack, no timeout and no retry, and
        # it does not de-energise the motors. The bench supply output-off
        # (or a physical e-stop inline) is the real emergency stop; this
        # button stops the GAIT.
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
            self.log_widget.add_log('E-Stop: Live (STANDBY) -> Armed (IDLE)',
                                    LOGLEVEL.WARN, 'orin')
        else:
            robot_cmd.request_robot_mode = int(ROBOTMODE.SYSTEM_ON)
            self._pending_robot_mode = int(ROBOTMODE.SYSTEM_ON)
            self.log_widget.add_log('E-Stop: -> Powered (SYSTEM_ON)',
                                    LOGLEVEL.WARN, 'orin')
        
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
        # Refuse re-entry while a homing run is live. Without this, a second
        # press reached start_process(), which returns False because the
        # process is already running -- and the old failure branch then reset
        # the button text, breaking completion detection. Same guard the IMU
        # handler already had.
        if self._homing_active or self.process_manager.is_running('homing'):
            self.log_widget.add_log('Homing already in progress; ignoring press',
                                    LOGLEVEL.WARN, 'system')
            return

        self._homing_active = True
        self._homing_saw_complete = False
        self.btn_home.setEnabled(False)
        self.btn_home.setText('Homing…')
        
        success = self.process_manager.start_process(
            'homing',
            ['ros2', 'run', 'corgi_homing', 'homing'],
            capture_output=False
        )
        
        if success:
            self.log_widget.add_log('Homing Started', LOGLEVEL.INFO, 'system')
        else:
            self.log_widget.add_log('Failed to start homing', LOGLEVEL.ERROR, 'system')
            self._homing_active = False
            self.btn_home.setEnabled(True)
            self.btn_home.setText('Set Home')
    
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
                f'Robot mode reached: {self._mode_text(self._pending_robot_mode)} ({self._pending_robot_mode})',
                LOGLEVEL.INFO, 'system'
            )
            self._pending_robot_mode = None
            
            # Launch config panel if entering CONFIG mode
            if current_mode == ROBOTMODE.MOTORCONFIG:
                self._launch_config_panel()
        
        self._last_confirmed_mode = current_mode
        
        # Update mode display
        try:
            mode_text = self._mode_text(state.robot_mode)
        except ValueError:
            mode_text = "no robot state"
        
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
            if 'abort' in message.lower():
                self._homing_saw_complete = False
            if 'completed' in message.lower() or 'complete' in message.lower():
                self._homing_saw_complete = True
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

    # Channel 0 is NOT load current, and including it made the panel read
    # ~14.4 A per board (~1.4 kW total) while the bench supply showed 48 V
    # 2 A (~96 W). Measured across the three hardware air runs of
    # 2026-08-31 (n = 348279 / 31953 / 61590 rows):
    #
    #   i_0        median 13.2-13.9 A, sd as low as 0.03, and it does not
    #              move with the gait -- it correlates NEGATIVELY with total
    #              |torque| (r = -0.29, -0.11). A real supply current rises
    #              with torque.
    #   i_1..i_7   median 0.0-0.3 A each, spiking to 2-6 A during the pronk,
    #              and their sum correlates +0.67 with total |torque|.
    #
    # Summed over 1..7 both boards come to ~1.7 A, which is what the supply
    # actually shows. What i_0 IS remains unidentified -- that lives in the
    # power-board firmware -- so it is excluded rather than reinterpreted.
    POWER_LOAD_CHANNELS = tuple(range(1, 8))

    def _sum_powerboard_current(self, state, board_prefix: str) -> float:
        """Load current for one board: channels 1..7. See POWER_LOAD_CHANNELS."""
        return sum(
            self._get_float_field(state, f'{board_prefix}_i_{index}')
            for index in self.POWER_LOAD_CHANNELS
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

        note = ('Sum of i_1..i_7. Channel 0 is excluded: it sits at a near '
                'constant ~13.5 A, does not move with the gait, and '
                'correlates negatively with torque — it is not load current. '
                'Including it read ~1.4 kW against a supply showing ~96 W.')
        current_label.setToolTip(note)
        power_label.setToolTip(note)
    
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
        
        # STANDBY gate kept (homing outside STANDBY is unsafe), but a
        # live homing run also holds the button down: this is called on
        # every robot-state update and used to re-enable it mid-run.
        self.btn_home.setEnabled(bridge_on
                                 and current == ROBOTMODE.STANDBY
                                 and not self._homing_active)
        
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
        self._homing_active = False
        if self.process_manager.is_running('homing'):
            self.process_manager.stop_process('homing', timeout=1.0)
        
        self.btn_home.setEnabled(True)
        self.btn_home.setText('Set Home')
        if self._homing_saw_complete:
            self.log_widget.add_log('Motor home position set successfully',
                                    LOGLEVEL.INFO, 'system')
        else:
            self.log_widget.add_log(
                'HOMING DID NOT COMPLETE -- the node exited without reporting '
                'completion (see its warning above). The zero was NOT set; do '
                'not run a gait on this. Fix the cause and home again.',
                LOGLEVEL.ERROR, 'system')
    
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
        # Check if homing process has completed. Keyed on the state flag,
        # not the button text -- the text is presentation and anything that
        # rewrites it used to switch this detector off permanently.
        if self._homing_active:
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
