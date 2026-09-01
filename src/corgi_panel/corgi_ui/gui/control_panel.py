#!/usr/bin/env python3
"""
Control Panel GUI for Corgi Robot
Refactored version with modular architecture (MVC pattern)
"""
import os
import sys
import time
import logging
import threading
import yaml
import numpy as np
from datetime import datetime
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, 
    QGroupBox, QLineEdit, QGridLayout, QFrame, QFileDialog, QApplication,
    QMessageBox, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFontMetrics

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

# GPIO Support (optional) -- used only to mirror the software trigger onto a
# hardware pin, for syncing external instruments. Everything else works
# without it, because the trigger is published on ROS regardless.
#
# On the Orin, importing this prints
#     UserWarning: Could not open /dev/mem for pinmux check ... Permission
#     denied: '/dev/mem'
# That is Jetson.GPIO skipping an OPTIONAL sanity check (is this pin actually
# muxed as GPIO?) which needs root to read physical memory. It is a warning,
# not a failure: the library drives the pin through /dev/gpiochip* instead,
# and that path does not need /dev/mem. Nothing is disabled by it.
GPIO_defined = True
try:
    import Jetson.GPIO as GPIO
except ImportError:
    GPIO_defined = False

class CorgiControlPanel(QWidget):

    # Emitted from the status worker thread; Qt delivers it to the GUI
    # thread, which is the only thread allowed to touch widgets.
    fpga_status_ready = pyqtSignal(str, object)

    # Operator-facing names, taken from the FIRMWARE rather than inferred:
    # fpga_driver/src/robot_fsm.cpp and motor_fsm.cpp (read 2026-09-01).
    #
    #   SYSTEM_ON (0)   switchMode(REST) + power switches OFF. REST zeroes
    #                   every motor command, so the motors are DE-ENERGISED
    #                   and the robot goes LIMP. The firmware's own name is
    #                   the most misleading label in the stack.
    #   INIT (1)        digital -> signal -> power, SET_ZERO, HALL_CALIBRATE
    #                   (a driven velocity sweep -- THE LEGS MOVE), MOTOR,
    #                   then auto-enters IDLE. Not a button: SystemOn -> IDLE
    #                   is routed through it.
    #   IDLE (2)        MOTOR mode, does NOT accept gRPC commands. Holds
    #                   position only SOFTLY -- observed on the robot, and it
    #                   follows: the gains still in force are HALL_CALIBRATE's
    #                   (kp 50, kd 1.5). Firm torque arrives after Set Home.
    #   STANDBY (3)     MOTOR mode, commands accepted -- nothing moves on
    #                   entry, it only opens the door. Gaits run here; homing
    #                   is permitted here and is what produces firm torque.
    #   MOTORCONFIG (4) switchMode(CONFIG), which sets REST: motors LIMP
    #                   again while parameters are read/written.
    #
    # Three facts the old labels hid, the first two safety-relevant:
    # "Set Powered" DROPPED the legs, "Arm" MOVES them, and "Go Live - home
    # && run" neither homed nor produced holding torque. The buttons now say
    # what each one does.
    MODE_LABEL = {
        ROBOTMODE.SYSTEM_ON:   'Motors Off',
        ROBOTMODE.INIT:        'Initialising',
        ROBOTMODE.IDLE:        'Energised (soft)',
        ROBOTMODE.STANDBY:     'Commands Enabled',
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
        # When the last e-stop press was, so a rapid second one can be
        # recognised as a double-tap rather than a decision.
        self._last_estop_at = 0.0
        # Power badges: accumulate at 1 kHz, paint the mean at 5 Hz. Defined
        # here, before _connect_ros_signals, so a message arriving early
        # cannot hit an attribute that does not exist yet.
        self._power_accum = {'n': 0, 'pb1_v': 0.0, 'pb1_i': 0.0,
                             'pb2_v': 0.0, 'pb2_i': 0.0}
        # Peaks come from the RAW samples, not the painted mean.
        self._peak_i = 0.0
        self._peak_w = 0.0
        # When each 1 kHz stream last delivered. 0.0 = nothing ever arrived,
        # which must not be reported as "stale" before the bridge is up.
        self._last_power_at = 0.0
        self._last_motor_at = 0.0
        self._streams_stale = False
        
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

        # Ask the sbRIO what is actually running, rather than assuming the
        # driver is down because this panel has not started one.
        self._power_paint_timer = QTimer(self)
        self._power_paint_timer.timeout.connect(self._repaint_power_badges)
        self._power_paint_timer.timeout.connect(self._repaint_cmd_source)
        # _update_button_states is driven by the 5 Hz paint timer rather
        # than by the 1 kHz state callbacks. It makes nineteen setEnabled()
        # calls and depends on nothing that changes faster than a user can
        # act; every user action also calls it directly, so the only
        # latency added is up to 200 ms on an FSM-driven enable.
        self._power_paint_timer.timeout.connect(self._update_button_states)
        self._power_paint_timer.start(200)

        self._fpga_busy = False
        self.fpga_status_ready.connect(self._on_fpga_status)
        if not self.use_sim_time:
            self._fpga_poll_timer = QTimer(self)
            self._fpga_poll_timer.timeout.connect(self._poll_fpga_status)
            self._fpga_poll_timer.start(30000)
            QTimer.singleShot(300, self._poll_fpga_status)

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
        main_layout.addWidget(self._create_stale_banner())
        
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
    
    def _create_stale_banner(self):
        """A permanently reserved one-line row for the stream-staleness warning.

        Reserved rather than shown-on-demand, and that is the whole point.
        Toggling a widget's visibility is a layout change, and a layout change
        is what the complaint was about: this label used to live in the top
        bar, where making it visible raised the layout's minimum width by 889
        px (measured) and Qt force-grew the window, because a window cannot be
        drawn narrower than its layout demands.

        Always present, empty when healthy, so the panel's geometry never
        moves in either axis whatever the streams do. Costs ~18 px of height,
        permanently, which is a fair price for a warning that cannot be
        missed and cannot shove the window around.

        Horizontally Ignored so its text still cannot raise the minimum width,
        and elided rather than wrapped in _repaint_stream_staleness -- wrapping
        would grow the row to two lines on a narrow window and reintroduce the
        same complaint in the other axis.
        """
        self.lbl_stream_stale.setSizePolicy(QSizePolicy.Ignored,
                                            QSizePolicy.Preferred)
        self.lbl_stream_stale.setMinimumWidth(0)
        self.lbl_stream_stale.setWordWrap(False)
        fm = QFontMetrics(self.lbl_stream_stale.font())
        self.lbl_stream_stale.setFixedHeight(fm.height() + 4)
        self.lbl_stream_stale.setVisible(True)
        return self.lbl_stream_stale

    def _create_top_bar(self) -> QHBoxLayout:
        """Create top bar with power display and E-Stop button"""
        top_bar = QHBoxLayout()
        
        # Power summary badges
        power_box = QHBoxLayout()
        power_box.setSpacing(8)

        pb1_container = QVBoxLayout()
        pb1_container.setSpacing(3)
        # Which board is which side, established on the robot 2026-09-01
        # (Alex). Correlating the banked runs could not separate them -- in an
        # air pronk all four legs move in phase.
        pb1_title = QLabel("PB1  —  LEFT side")
        pb1_title.setObjectName("PowerBoardTitle")
        pb1_title.setAlignment(Qt.AlignCenter)
        pb1_frame = QFrame()
        pb1_frame.setObjectName("PowerBoardFrame")
        pb1_frame_layout = QHBoxLayout()
        pb1_frame_layout.setSpacing(10)
        pb1_frame_layout.setContentsMargins(8, 8, 8, 8)

        pb2_container = QVBoxLayout()
        pb2_container.setSpacing(3)
        pb2_title = QLabel("PB2  —  RIGHT side")
        pb2_title.setObjectName("PowerBoardTitle")
        pb2_title.setAlignment(Qt.AlignCenter)
        pb2_frame = QFrame()
        pb2_frame.setObjectName("PowerBoardFrame")
        pb2_frame_layout = QHBoxLayout()
        pb2_frame_layout.setSpacing(10)
        pb2_frame_layout.setContentsMargins(8, 8, 8, 8)
        
        self.lbl_pb1_voltage = QLabel('--.- V')
        self.lbl_pb1_voltage.setObjectName('PowerBadge')
        self.lbl_pb1_current = QLabel('-.-- A')
        self.lbl_pb1_current.setObjectName('PowerBadge')
        self.lbl_pb1_power = QLabel('--.- W')
        self.lbl_pb1_power.setObjectName('PowerBadge')

        self.lbl_pb2_voltage = QLabel('--.- V')
        self.lbl_pb2_voltage.setObjectName('PowerBadge')
        self.lbl_pb2_current = QLabel('-.-- A')
        self.lbl_pb2_current.setObjectName('PowerBadge')
        self.lbl_pb2_power = QLabel('--.- W')
        self.lbl_pb2_power.setObjectName('PowerBadge')
        
        pb1_frame_layout.addWidget(self.lbl_pb1_voltage)
        pb1_frame_layout.addWidget(self.lbl_pb1_current)
        pb1_frame_layout.addWidget(self.lbl_pb1_power)
        pb1_frame.setLayout(pb1_frame_layout)
        pb1_container.addWidget(pb1_title)
        pb1_container.addWidget(pb1_frame)

        pb2_frame_layout.addWidget(self.lbl_pb2_voltage)
        pb2_frame_layout.addWidget(self.lbl_pb2_current)
        pb2_frame_layout.addWidget(self.lbl_pb2_power)
        pb2_frame.setLayout(pb2_frame_layout)
        pb2_container.addWidget(pb2_title)
        pb2_container.addWidget(pb2_frame)

        # TOTAL -- what the bench supply is actually delivering, which is
        # the number worth watching against its current limit. Neither
        # per-board badge answers that on its own.
        total_container = QVBoxLayout()
        total_container.setSpacing(3)
        total_title = QLabel("TOTAL  (both boards)")
        total_title.setObjectName("PowerBoardTitle")
        total_title.setAlignment(Qt.AlignCenter)
        total_frame = QFrame()
        total_frame.setObjectName("PowerBoardFrame")
        total_frame_layout = QHBoxLayout()
        total_frame_layout.setSpacing(10)
        total_frame_layout.setContentsMargins(8, 8, 8, 8)

        self.lbl_total_current = QLabel('-.-- A')
        self.lbl_total_current.setObjectName('PowerBadge')
        self.lbl_total_power = QLabel('--.- W')
        self.lbl_total_power.setObjectName('PowerBadge')
        for lbl in (self.lbl_total_current, self.lbl_total_power):
            lbl.setToolTip(
                'PB1 + PB2, load channels i_1..i_7 only.\n'
                'Channel 0 is excluded: a near-constant ~13.5 A that does '
                'not track load and correlates negatively with torque.\n'
                '200 ms mean, painted at 5 Hz.')
            total_frame_layout.addWidget(lbl)

        total_frame.setLayout(total_frame_layout)
        total_container.addWidget(total_title)
        total_container.addWidget(total_frame)

        power_box.addLayout(pb1_container)
        power_box.addLayout(pb2_container)
        power_box.addLayout(total_container)

        # PEAK -- the largest single sample since the last reset. Sizing the
        # supply and reading a pronk both depend on the spike, not the mean.
        peak_container = QVBoxLayout()
        peak_container.setSpacing(3)
        peak_header = QHBoxLayout()
        peak_header.setSpacing(4)
        peak_title = QLabel("PEAK")
        peak_title.setObjectName("PowerBoardTitle")
        peak_title.setAlignment(Qt.AlignCenter)
        self.btn_peak_reset = QPushButton('\u21ba')
        self.btn_peak_reset.setMaximumWidth(28)
        self.btn_peak_reset.setToolTip('Reset the peak readings')
        self.btn_peak_reset.clicked.connect(lambda: self._reset_power_peaks('manual'))
        peak_header.addStretch(1)
        peak_header.addWidget(peak_title)
        peak_header.addWidget(self.btn_peak_reset)
        peak_header.addStretch(1)

        peak_frame = QFrame()
        peak_frame.setObjectName("PowerBoardFrame")
        peak_frame_layout = QHBoxLayout()
        peak_frame_layout.setSpacing(10)
        peak_frame_layout.setContentsMargins(8, 8, 8, 8)

        self.lbl_peak_current = QLabel('-.-- A')
        self.lbl_peak_current.setObjectName('PowerBadge')
        self.lbl_peak_power = QLabel('--.- W')
        self.lbl_peak_power.setObjectName('PowerBadge')
        for lbl in (self.lbl_peak_current, self.lbl_peak_power):
            lbl.setToolTip(
                'Largest SINGLE SAMPLE of PB1+PB2 since the last reset, at '
                'the full 1 kHz \u2014 not the largest painted average.\n'
                'A 20 ms spike barely moves the 200 ms mean shown under '
                'TOTAL, and the spike is the point.\n'
                'Resets automatically when the trigger goes ON, so it reads '
                '"peak during this run"; the outgoing value is logged first.')
            peak_frame_layout.addWidget(lbl)

        peak_frame.setLayout(peak_frame_layout)
        peak_container.addLayout(peak_header)
        peak_container.addWidget(peak_frame)
        power_box.addLayout(peak_container)

        # Shown only while a 1 kHz stream has stopped. Placed with the power
        # row because that is where a frozen number is most believable.
        self.lbl_stream_stale = QLabel('')
        self.lbl_stream_stale.setStyleSheet(
            'color: #ffa726; font-weight: bold; padding-left: 10px;')
        # Geometry and size policy are set in _create_stale_banner().
        top_bar.addLayout(power_box)
        # The staleness banner is NOT in this bar. Measured: at the default
        # 1024-wide window the power boxes and buttons claim ~892 px, leaving
        # ~130 px of slack, so a label here was allocated 63 px of a
        # 90-character warning. It lives on its own reserved row instead --
        # see _create_stale_banner().
        
        # E-Stop Button
        self.btn_estop = QPushButton('E-STOP')
        self.btn_estop.setObjectName("EstopBtn")
        self.btn_estop.setMinimumWidth(100)
        self.btn_estop.setMinimumHeight(50)
        self.btn_estop.clicked.connect(self._on_estop_clicked)
        self._estop_timer = QTimer(self)
        self._estop_timer.timeout.connect(self._estop_retry)
        self._estop_deadline = 0.0

        self.btn_stop_gait = QPushButton('Stop Gait')
        self.btn_stop_gait.setObjectName("StopGaitBtn")
        self.btn_stop_gait.setMinimumWidth(100)
        self.btn_stop_gait.setMinimumHeight(50)
        self.btn_stop_gait.clicked.connect(self._on_stop_gait_clicked)
        self.btn_stop_gait.setEnabled(False)
        self.btn_stop_gait.setToolTip(
            'End the run and LEAVE THE ROBOT HOLDING.\n'
            'Drops the gait trigger, then requests Energised (IDLE) -- the '
            'motors stay powered.\n'
            'Use this to finish a run. Use E-STOP when something is wrong.')
        self.btn_estop.setToolTip(
            'EMERGENCY STOP -- one press, one outcome, every time.\n'
            '  1. drops the gait trigger (the controller\'s own abort)\n'
            '  2. requests Motors Off (SYSTEM_ON): motors DE-ENERGISED.\n'
            '     THE ROBOT GOES LIMP and will fall if unsupported.\n'
            'Re-sent every 200 ms until the robot confirms. After 5 s with '
            'no confirmation it says so and tells you to cut the supply.\n'
            'No dialog, and no dependence on the current mode.\n'
            'To end a run without dropping the robot, use Stop Gait.\n'
            'This is a gRPC REQUEST and needs the ROS bridge. The bench '
            'supply output-off is the real emergency stop.')
        self._estop_timer = QTimer(self)
        self._estop_timer.timeout.connect(self._estop_retry)
        self._estop_deadline = 0.0

        self.btn_stop_gait = QPushButton('Stop Gait')
        self.btn_stop_gait.setObjectName("StopGaitBtn")
        self.btn_stop_gait.setMinimumWidth(100)
        self.btn_stop_gait.setMinimumHeight(50)
        self.btn_stop_gait.clicked.connect(self._on_stop_gait_clicked)
        self.btn_stop_gait.setEnabled(False)
        self.btn_stop_gait.setToolTip(
            'End the run and LEAVE THE ROBOT HOLDING.\n'
            'Drops the gait trigger, then requests Energised (IDLE) -- the '
            'motors stay powered.\n'
            'Use this to finish a run. Use E-STOP when something is wrong.')
        self.btn_estop.setToolTip(
            'EMERGENCY STOP. Drops the gait trigger, then requests a mode '
            'change:\n'
            '  from Commands Enabled (STANDBY) -> Power Up && Hold (IDLE), '
            'motors still holding\n'
            '  from anywhere else -> Motors Off (SYSTEM_ON), MOTORS '
            'DE-ENERGISED and the robot goes limp\n'
            'A second press within 5 s asks before de-energising, so a '
            'double-tap cannot drop a standing robot. The trigger is always '
            'dropped first, unconditionally.\n'
            'This is a REQUEST over gRPC: no ack, no retry, and it needs the '
            'ROS bridge. The supply output-off is the real emergency stop.')
        self.btn_estop.setEnabled(False)
        
        top_bar.addStretch(1)
        top_bar.addWidget(self.btn_stop_gait)
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
        self.btn_systemon = QPushButton('1 \u00b7 Motors Off  (safe)')
        self.btn_systemon.setToolTip(
            'SYSTEM_ON (0) — the firmware\'s "safe state", and NOT what its '
            'name suggests.\nMotors go to REST (every command zeroed) and '
            'the power switches go OFF, so THE ROBOT GOES LIMP. Support it '
            'before pressing this.\nAlso where the e-stop retreats to from '
            'anywhere but Standby.')
        self.btn_systemon.setObjectName("SystemOnBtn")
        self.btn_systemon.setCheckable(True)
        self.btn_systemon.clicked.connect(lambda: self._request_robot_mode(ROBOTMODE.SYSTEM_ON))
        self.btn_systemon.setEnabled(False)
        
        self.btn_idle = QPushButton('2 \u00b7 Power Up  \u2014  hall calib')
        self.btn_idle.setToolTip(
            'IDLE (2). Coming from Motors Off this routes through INIT: '
            'power sequence, set-zero, then HALL CALIBRATION — a driven '
            'sweep, SO THE LEGS MOVE. Keep clear.\n'
            'It then settles into a closed pose, energised but only SOFTLY '
            'held — no firm holding torque, because the gains still in force '
            'are the calibration ones (kp 50, kd 1.5). Firm torque arrives '
            'after Set Home.\n'
            'Commands are NOT accepted in this state.\n'
            'Also where the e-stop drops you from Commands Enabled.')
        self.btn_idle.setCheckable(True)
        self.btn_idle.clicked.connect(lambda: self._request_robot_mode(ROBOTMODE.IDLE))
        self.btn_idle.setEnabled(False)
        
        self.btn_standby = QPushButton('3 \u00b7 Enable Commands')
        self.btn_standby.setToolTip(
            'STANDBY (3). Motors accept commands. Nothing moves on entry — '
            'this only opens the door.\n'
            'It does NOT home: press "Set Home" for that (it runs '
            'corgi_homing), and the firm holding torque appears once homing '
            'has set the zero.\n'
            'Gaits run in this state and nowhere else.\nDown: Power Up.')
        self.btn_standby.setCheckable(True)
        self.btn_standby.clicked.connect(lambda: self._request_robot_mode(ROBOTMODE.STANDBY))
        self.btn_standby.setEnabled(False)
        
        self.btn_motorconfig = QPushButton('Enter Motor Config')
        self.btn_motorconfig.setToolTip(
            'MOTORCONFIG (4). Side branch off Motors Off / Power Up && Hold, '
            'not a rung on the ladder.\nCONFIG sets the motors to REST, so '
            'THE ROBOT GOES LIMP here too while parameters are read and '
            'written.\nOpens the config panel on entry.')
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
        self.btn_home.setToolTip(
            'Runs corgi_homing, which drives the legs to the reference pose '
            'and sets the joint zero.\n'
            'Only available in Commands Enabled (STANDBY).\n'
            'This is what produces the FIRM holding torque — before it, the '
            'motors are energised but softly held.')
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
        # ---- Command Source -------------------------------------------
        # What the robot is being TOLD. The panel showed motor/state and
        # never motor/command, so a node commanding theta=0 with kp=50, and
        # two nodes writing the topic at once, were both invisible here.
        grp_src = QGroupBox("Command Source")
        grp_src_layout = QVBoxLayout()

        self._cmd_mode_label = QLabel("no command stream")
        self._cmd_mode_label.setStyleSheet(
            "color: #888; font-size: 13px; font-weight: bold;")
        self._cmd_mode_label.setWordWrap(True)

        self._cmd_pub_label = QLabel("publishers on motor/command: -")
        self._cmd_pub_label.setStyleSheet("color: #aaa; font-size: 11px;")
        # Wraps rather than widening the sidebar: this label can read
        # "publishers on motor/command: 2 -- TWO WRITERS, they will fight at
        # 1 kHz", which is the same unbounded-sizeHint trap as the staleness
        # banner above, one column over.
        self._cmd_pub_label.setWordWrap(True)

        self._cmd_gain_label = QLabel("kp  -  /  -  /  -")
        self._cmd_gain_label.setStyleSheet(
            "color: #aaa; font-size: 11px; font-family: monospace;")

        grp_src_layout.addWidget(self._cmd_mode_label)
        grp_src_layout.addWidget(self._cmd_gain_label)
        grp_src_layout.addWidget(self._cmd_pub_label)
        grp_src.setLayout(grp_src_layout)
        sidebar.addWidget(grp_src)

        # Stored by the callback, painted at 5 Hz. Never paint per message:
        # this topic runs at 1 kHz and repainting a QLabel that often wedges
        # the GUI thread -- the same trap the power readout already solved.
        self._last_cmd = None
        self._last_cmd_t = 0.0
        self._cmd_pub_count = -1
        self._cmd_pub_poll = 0

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
        # Angles in degrees, then the three motor torques in N*m. The
        # torques are what say whether a leg is WORKING -- a pose alone
        # cannot distinguish holding gently from grinding into a stop,
        # which is exactly the state the home pose used to sit in.
        ANG = [('theta', 'θ'), ('beta', 'β'), ('gamma', 'γ')]
        TRQ = [('torque_r', 'τ R'), ('torque_l', 'τ L'), ('torque_h', 'τ H')]
        legs = [
            ('LF', 0, 0, ANG + TRQ),
            ('RF', 0, 1, ANG + TRQ),
            ('LH', 1, 0, ANG + TRQ),
            ('RH', 1, 1, ANG + TRQ)
        ]

        for leg_name, r, c, fields in legs:
            leg_group = QGroupBox(leg_name)
            # Two columns of three: angles left, torques right. Stacked, the
            # six labels made each leg box tall enough to shove the rest of
            # the column around, and the reading that matters is per row --
            # theta against tau R, and so on.
            leg_layout = QGridLayout()
            leg_layout.setHorizontalSpacing(18)
            leg_layout.setVerticalSpacing(2)

            for idx, (field_name, display_name) in enumerate(fields):
                label_key = f"{leg_name}_{field_name}"
                lbl = QLabel(f"{display_name}: --")
                lbl.setObjectName("MotorLabel")
                col = idx // 3
                if col:
                    # Fixed width so the column does not twitch as the sign
                    # and digit count change on every repaint.
                    lbl.setMinimumWidth(120)
                leg_layout.addWidget(lbl, idx % 3, col)
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
        self.ros_worker.motor_cmd_updated.connect(self._handle_motor_cmd_update)
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

    def _poll_fpga_status(self):
        """Kick off a status check on a worker thread. Never blocks the GUI."""
        if self.use_sim_time or self._fpga_busy:
            return
        self._fpga_busy = True

        def work():
            try:
                token, lines = sbrio.fpga_driver_status()
            except Exception as exc:                        # pragma: no cover
                token, lines = 'FAILED', ['status check raised: %s' % exc]
            self.fpga_status_ready.emit(token, lines)

        threading.Thread(target=work, daemon=True,
                         name='fpga-status').start()

    def _on_fpga_status(self, token, lines):
        """Apply a status result on the GUI thread."""
        self._fpga_busy = False
        was = self._fpga_running

        if token == 'ALREADY_RUNNING':
            self._fpga_running = True
        elif token == 'NOT_RUNNING':
            self._fpga_running = False
        else:
            # NO_KEY / UNREACHABLE / TIMEOUT: we do not know, and saying
            # "not running" would be a guess that invites a duplicate.
            self._fpga_running = None

        self._refresh_fpga_button()

        if was is not self._fpga_running:
            if self._fpga_running is True:
                self._log('FPGA driver is already running on %s'
                          % sbrio.SBRIO_HOST, LOGLEVEL.INFO, 'fpga_driver')
            elif self._fpga_running is False and was is True:
                self._log('FPGA driver is NO LONGER running on %s — the '
                          'terminal window was closed, or it exited'
                          % sbrio.SBRIO_HOST, LOGLEVEL.WARN, 'fpga_driver')
            elif self._fpga_running is None and was is not None:
                self._log('cannot check the FPGA driver (%s) — its state is '
                          'now unknown' % token, LOGLEVEL.WARN, 'fpga_driver')

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
                why.append('the robot is in Commands Enabled (STANDBY)')
            self._log('REFUSED to stop the FPGA driver: %s. Drop to Power '
                      'Up (IDLE) and stop the trigger first -- pulling the command '
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
        self._fpga_busy = True
        self.btn_fpga_driver.setEnabled(False)
        self.btn_fpga_driver.setText('Stopping FPGA driver…')
        QApplication.setOverrideCursor(Qt.WaitCursor)
        QApplication.processEvents()
        try:
            token, lines = sbrio.stop_fpga_driver()
        finally:
            QApplication.restoreOverrideCursor()
            self._fpga_busy = False
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

        # `ros2 run corgi_ros_bridge corgi_ros_bridge` is a WRAPPER: the
        # actual node is its child, with a different pid. ProcessManager
        # tracks the wrapper, so excluding only that pid leaves our own
        # bridge looking like someone else's. Exclude the children too.
        mine = {os.getpid()}
        tracked = self.process_manager.get_pid('ros_bridge')
        if tracked:
            mine.add(tracked)
            try:
                kids = sp.run(['pgrep', '-P', str(tracked)],
                              capture_output=True, text=True, timeout=5)
                mine.update(int(k) for k in kids.stdout.split() if k.isdigit())
            except Exception:
                pass

        return [tok for tok in out.stdout.split()
                if tok.isdigit() and int(tok) not in mine]

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

        # Launching a window is not starting a driver, but verifying it
        # here was worse than not verifying: eight back-to-back ssh checks
        # take about two seconds, and the window needs longer than that just
        # to log in and start grpccore. It reported FAILED on a driver that
        # came up fine 8 s later.
        #
        # Hand it to the async poll instead. These fire on the GUI thread's
        # timer, each spawning the same worker the periodic poll uses, so
        # nothing blocks and the button updates itself the moment the driver
        # appears.
        for delay_ms in (3000, 6000, 10000, 15000, 22000, 30000):
            QTimer.singleShot(delay_ms, self._poll_fpga_status)

        return 'LAUNCHED_TERMINAL', [
            'the button will switch to "Stop FPGA Driver" on its own once the '
            'sbRIO reports the driver up (checked at 3, 6, 10, 15, 22 and '
            '30 s).',
            'If it does not, read that terminal window — it holds the '
            'reason, and stays open on exit.']

    def _start_fpga_driver(self, manual: bool) -> bool:
        """Bring the driver up over ssh and report what happened.

        This blocks the GUI, deliberately: it is bounded (5 s to connect, 30 s
        overall), it only ever runs at bring-up, and nothing else in the panel
        is useful until it finishes. A worker thread here would buy an
        unresponsive-looking wait instead of an honest one.
        """
        self._fpga_busy = True
        self.btn_fpga_driver.setEnabled(False)
        self.btn_fpga_driver.setText('Starting FPGA driver…')
        QApplication.setOverrideCursor(Qt.WaitCursor)
        QApplication.processEvents()
        try:
            token, lines = self._launch_fpga(allow_prompt=manual)
        finally:
            QApplication.restoreOverrideCursor()
            self._fpga_busy = False
            self.btn_fpga_driver.setEnabled(True)

        ok = token in ('ALREADY_RUNNING', 'STARTED', 'UNVERIFIED',
                       'LAUNCHED_TERMINAL')
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
            'LAUNCHED_TERMINAL': 'FPGA driver launching in a terminal window '
                                 'on %s' % sbrio.SBRIO_HOST,
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
        if ok and token != 'LAUNCHED_TERMINAL':
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
    
    def _estop_retry(self):
        """Re-send SYSTEM_ON until the robot confirms it, or time out loudly."""
        mode = getattr(getattr(self, 'robot_state', None), 'robot_mode', None)
        if mode is not None and int(mode) == int(ROBOTMODE.SYSTEM_ON):
            self._estop_timer.stop()
            self.log_widget.add_log(
                'E-STOP CONFIRMED: robot reports Motors Off (SYSTEM_ON)',
                LOGLEVEL.WARN, 'orin')
            return

        if time.monotonic() > self._estop_deadline:
            self._estop_timer.stop()
            self.log_widget.add_log(
                'E-STOP NOT CONFIRMED after 5 s -- the robot has not reported '
                'Motors Off. The gait trigger IS dropped, but do NOT assume '
                'the motors are de-energised: CUT THE BENCH SUPPLY OUTPUT.',
                LOGLEVEL.FATAL, 'orin')
            return

        if not self.ros_worker.is_running:
            return
        robot_cmd = RobotCmdStamped()
        robot_cmd.header.seq = self._robot_cmd_seq + 1
        robot_cmd.header.stamp = self.ros_worker.node.get_clock().now().to_msg()
        robot_cmd.header.frame_id = ''
        robot_cmd.request_robot_mode = int(ROBOTMODE.SYSTEM_ON)
        self.ros_worker.send_robot_command(robot_cmd)
        self._robot_cmd_seq += 1

    def _on_stop_gait_clicked(self):
        """End the run and leave the robot holding. NOT an emergency control.

        Drops the trigger (the controller's own abort) and requests IDLE, so
        the motors stay energised. This is what e-stop used to do on its
        first press -- which is exactly why it needed its own button: an
        operator reaching for a red button in a hurry should not have to
        know the robot's current mode to predict what happens.
        """
        if not self.ros_worker.is_running:
            return

        if self.btn_trigger.isChecked():
            self.btn_trigger.setChecked(False)
            self._on_trigger_clicked()

        robot_cmd = RobotCmdStamped()
        robot_cmd.header.seq = self._robot_cmd_seq + 1
        robot_cmd.header.stamp = self.ros_worker.node.get_clock().now().to_msg()
        robot_cmd.header.frame_id = ''
        robot_cmd.request_robot_mode = int(ROBOTMODE.IDLE)
        self._pending_robot_mode = int(ROBOTMODE.IDLE)
        self.ros_worker.send_robot_command(robot_cmd)
        self._robot_cmd_seq += 1
        self.log_widget.add_log(
            'STOP GAIT: trigger dropped, -> Energised (IDLE), motors still '
            'holding', LOGLEVEL.WARN, 'orin')
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
        
        # UNCONDITIONAL. Not "IDLE from STANDBY, SYSTEM_ON otherwise":
        # an emergency control whose action depends on invisible state is
        # not an emergency control. One press, one outcome, every time --
        # motors de-energised. The gentle variant is its own button.
        del current
        robot_cmd.request_robot_mode = int(ROBOTMODE.SYSTEM_ON)
        self._pending_robot_mode = int(ROBOTMODE.SYSTEM_ON)
        self.log_widget.add_log(
            'E-STOP: -> Motors Off (SYSTEM_ON), de-energising -- retrying '
            'until the robot confirms', LOGLEVEL.WARN, 'orin')

        self._last_estop_at = time.monotonic()
        self.ros_worker.send_robot_command(robot_cmd)
        self._robot_cmd_seq += 1
        # The mode change is an UNACKNOWLEDGED gRPC request. Sent once and
        # dropped, nothing says so -- a plausible reading of "the first
        # press didn't stop the motors". Keep asking until /robot/state
        # agrees, and say so loudly if it never does.
        self._estop_deadline = time.monotonic() + 5.0
        self._estop_timer.start(200)
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
            # Peak should mean "during this run", not "since the panel
            # opened". The outgoing value is logged by the reset itself.
            self._reset_power_peaks('run start')
            filename = trigger_cmd.output_filename if trigger_cmd.output_filename else "no filename"
            self.log_widget.add_log(f'Trigger enabled: {filename}', LOGLEVEL.INFO, 'orin')
        else:
            self.log_widget.add_log('Trigger stopped', LOGLEVEL.INFO, 'orin')
    
    # ========================================================================
    # ROS Message Handlers
    # ========================================================================
    
    def _handle_power_state_update(self, state):
        """Accumulate power state. The badges are painted on a timer.

        This arrives at the bridge's 1 kHz. Repainting four labels per board
        per message churns the digits far faster than anyone can read them,
        and makes Qt re-lay-out the row a thousand times a second to serve
        an eye that manages about five. Sum here; average and paint in
        _repaint_power_badges().
        """
        self.power_state = state
        self._last_power_at = time.monotonic()
        
        pb1_v = self._get_float_field(state, 'pb1_v_0')
        pb1_i = self._sum_powerboard_current(state, 'pb1')
        pb2_v = self._get_float_field(state, 'pb2_v_0')
        pb2_i = self._sum_powerboard_current(state, 'pb2')

        a = self._power_accum
        a['n'] += 1
        a['pb1_v'] += pb1_v
        a['pb1_i'] += pb1_i
        a['pb2_v'] += pb2_v
        a['pb2_i'] += pb2_i

        # Peak from THIS sample. Doing it on the 5 Hz painted average would
        # miss the transients entirely: a 20 ms spike barely moves a 200 ms
        # mean, and a spike is the whole reason to have a peak reading.
        self._peak_i = max(self._peak_i, pb1_i + pb2_i)
        self._peak_w = max(self._peak_w, pb1_v * pb1_i + pb2_v * pb2_i)

        # painted by _repaint_power_badges on a 5 Hz timer
        
        # Buttons are NOT refreshed here. This callback runs at the
        # bridge's 1 kHz and _update_button_states() makes nineteen
        # setEnabled() calls, none of which depend on power state --
        # it reads only the bridge toggle, use_sim_time, the robot
        # mode and _homing_active. It is driven by the 5 Hz paint
        # timer instead, and directly by every user action.
    
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

        # Everything below repaints the mode display, and reads ONLY
        # state.robot_mode. robot/state streams at the bridge's 1 kHz
        # while the mode changes a handful of times per session, so
        # repainting per message is pure waste -- and setStyleSheet() is
        # not cheap: Qt reparses the CSS and repolishes the widget on
        # every call, on the GUI thread, competing with the log pane and
        # the power badges.
        #
        # Guarding on the mode is exact: if it has not changed, the
        # display is already correct. The pending-transition latch above
        # stays per-message on purpose -- it must not miss the tick the
        # transition lands on.
        if current_mode == getattr(self, "_displayed_mode", None):
            return
        self._displayed_mode = current_mode
        
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
        
        # Refreshed on the 5 Hz paint timer, not here: robot/state also
        # streams at 1 kHz, so this call site doubled the ~19,000
        # setEnabled() calls a second. The mode transition logic above
        # still runs per message; only the button repaint is throttled.
    
    def _handle_motor_cmd_update(self, msg):
        """Store the newest motor command. Painting happens at 5 Hz."""
        import time
        self._last_cmd = msg
        self._last_cmd_t = time.monotonic()

    def _repaint_cmd_source(self):
        """Name what is commanding the robot, from the gain fingerprint.

        The label says "inferred" because a fingerprint is not an identity.
        kp 90 / kd 1.75 is used by BOTH corgi_homing (homing.cpp:120-126)
        and corgi_csv_control (corgi_csv_control.cpp:103-111), so it cannot
        name one of them, and it does not try to.
        """
        import time
        if not hasattr(self, "_cmd_mode_label"):
            return

        # Publisher count is a ROS API call; poll it every ~2 s, not at 5 Hz.
        self._cmd_pub_poll += 1
        if self._cmd_pub_poll >= 10:
            self._cmd_pub_poll = 0
            try:
                if self.ros_worker.is_running and self.ros_worker.node is not None:
                    self._cmd_pub_count = self.ros_worker.node.count_publishers(
                        "motor/command")
            except Exception:
                self._cmd_pub_count = -1

        n = self._cmd_pub_count
        if n < 0:
            self._cmd_pub_label.setText("publishers on motor/command: ?")
            self._cmd_pub_label.setStyleSheet("color: #888; font-size: 11px;")
        elif n > 1:
            self._cmd_pub_label.setText(
                "publishers on motor/command: %d  -- TWO WRITERS, they will "
                "fight at 1 kHz" % n)
            self._cmd_pub_label.setStyleSheet(
                "color: #e06c5a; font-size: 11px; font-weight: bold;")
        else:
            self._cmd_pub_label.setText("publishers on motor/command: %d" % n)
            self._cmd_pub_label.setStyleSheet("color: #aaa; font-size: 11px;")

        stale = (self._last_cmd is None
                 or time.monotonic() - self._last_cmd_t > 0.5)
        if stale:
            self._cmd_mode_label.setText("NO COMMAND STREAM")
            self._cmd_mode_label.setStyleSheet(
                "color: #888; font-size: 13px; font-weight: bold;")
            self._cmd_gain_label.setText("kp  -  /  -  /  -")
            return

        m = self._last_cmd.module_a
        kp_l, kp_r, kp_h = m.kp_l, m.kp_r, m.kp_h
        self._cmd_gain_label.setText(
            "kp %6.1f / %6.1f / %6.1f   (l/r/h)" % (kp_l, kp_r, kp_h))

        def near(a, b):
            return abs(a - b) < 0.5

        if near(kp_l, 50) and near(kp_r, 50) and near(kp_h, 50):
            text = ("POSITION FALLBACK (kp 50) -- impedance gains are ZERO. "
                    "This is force_control's position_control branch and "
                    "almost certainly a bug.")
            colour = "#e06c5a"
        elif near(kp_l, 90) and near(kp_r, 90) and near(kp_h, 90):
            text = ("FIXED-GAIN JOINT PD (kp 90) -- inferred: homing OR csv "
                    "control. force_control is not in the loop.")
            colour = "#5a9fd4"
        else:
            text = "IMPEDANCE (force_control) -- inferred from computed gains"
            colour = "#6fbf73"

        self._cmd_mode_label.setText(text)
        self._cmd_mode_label.setStyleSheet(
            "color: %s; font-size: 13px; font-weight: bold;" % colour)

    def _handle_motor_state_update(self, state):
        """Handle motor state update from ROS"""
        self.motor_state = state
        self._last_motor_at = time.monotonic()
        
        if not hasattr(state, 'module_a'):
            return
        
        modules = [
            ('LF', state.module_a),
            ('RF', state.module_b),
            ('LH', state.module_d),
            ('RH', state.module_c),
        ]
        # (field, symbol, is_angle) -- angles convert to degrees, torques
        # are already N*m and must NOT be run through np.degrees.
        state_fields = [
            ('theta', 'θ', True),
            ('beta', 'β', True),
            ('gamma', 'γ', True),
            ('torque_r', 'τ R', False),
            ('torque_l', 'τ L', False),
            ('torque_h', 'τ H', False),
        ]
        
        for leg_name, module in modules:
            if module is None:
                continue
            
            for field_name, display_name, is_angle in state_fields:
                key = f"{leg_name}_{field_name}"
                if key not in self.motor_labels:
                    continue
                
                raw = self._get_float_field(module, field_name)
                if is_angle:
                    self.motor_labels[key].setText(
                        f"{display_name}: {np.degrees(raw):.1f}°")
                    self.motor_labels[key].setStyleSheet("color: #aaa;")
                else:
                    self.motor_labels[key].setText(
                        f"{display_name}: {raw:+.2f} N·m")
                    # The ABAD clips silently at 45-64 N*m against a 44.25
                    # stall (#15), so a torque worth noticing should look
                    # different from one that is not.
                    mag = abs(raw)
                    colour = ('#ef5350' if mag > 20.0 else
                              '#ffa726' if mag > 8.0 else '#aaa')
                    self.motor_labels[key].setStyleSheet("color: %s;" % colour)
    
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
        current_label: QLabel,
        power_label: QLabel,
    ):
        """Update one powerboard summary row using v_0 and summed current."""
        power = voltage * current

        voltage_label.setText(f"{voltage:.1f} V")
        current_label.setText(f"{current:.2f} A")
        power_label.setText(f"{power:.1f} W")

        voltage_label.setToolTip('Bus voltage (channel v_0), 200 ms mean.')

        note = ('Sum of i_1..i_7. Channel 0 is excluded: it sits at a near '
                'constant ~13.5 A, does not move with the gait, and '
                'correlates negatively with torque — it is not load current. '
                'Including it read ~1.4 kW against a supply showing ~96 W.')
        current_label.setToolTip(note)
        power_label.setToolTip(note)
    
    STALE_AFTER_S = 0.75

    def _check_stream_freshness(self):
        """Dim what is no longer live, and say which stream stopped.

        Runs on the 5 Hz paint timer. A value held on screen after its
        stream stopped is indistinguishable from a live one that happens to
        be steady -- which is exactly how a frozen reading gets believed.
        """
        now = time.monotonic()
        stale = []
        if self._last_power_at and now - self._last_power_at > self.STALE_AFTER_S:
            stale.append(('power/state', now - self._last_power_at))
        if self._last_motor_at and now - self._last_motor_at > self.STALE_AFTER_S:
            stale.append(('motor/state', now - self._last_motor_at))

        if stale:
            worst = max(age for _, age in stale)
            full = ('\u26a0  %s STOPPED %.1f s ago \u2014 the values below '
                    'are frozen, not live'
                    % (' and '.join(n for n, _ in stale), worst))
            # Elide to the width the label actually has. Without this the
            # Ignored size policy above would simply clip mid-word with no
            # indication. Recomputed here on the 5 Hz timer, so it tracks a
            # manual window resize by itself; the tooltip always has it all.
            avail = self.lbl_stream_stale.width() - 14
            if avail > 40:
                fm = QFontMetrics(self.lbl_stream_stale.font())
                self.lbl_stream_stale.setText(
                    fm.elidedText(full, Qt.ElideRight, avail))
            else:
                self.lbl_stream_stale.setText(full)
            self.lbl_stream_stale.setToolTip(full)
        else:
            # Blanked, never hidden -- the row is reserved so the geometry
            # cannot move. See _create_stale_banner().
            self.lbl_stream_stale.setText('')
            self.lbl_stream_stale.setToolTip('')

        power_stale = any(n == 'power/state' for n, _ in stale)
        dim = "color: #6a6a6a;"
        for lbl in (self.lbl_pb1_voltage, self.lbl_pb1_current,
                    self.lbl_pb1_power, self.lbl_pb2_voltage,
                    self.lbl_pb2_current, self.lbl_pb2_power,
                    self.lbl_total_current, self.lbl_total_power):
            lbl.setStyleSheet(dim if power_stale else "")

        if any(n == 'motor/state' for n, _ in stale):
            for lbl in self.motor_labels.values():
                lbl.setStyleSheet(dim)

        if bool(stale) != self._streams_stale:
            self._streams_stale = bool(stale)
            if self._streams_stale:
                self._log('%s stopped — panel values are FROZEN until it '
                          'resumes. Expected during HALL_CALIBRATE: the sbRIO '
                          'driver runs the sweep in its own loop and is not '
                          'servicing gRPC meanwhile.'
                          % ' and '.join(n for n, _ in stale),
                          LOGLEVEL.WARN, 'system')
            else:
                self._log('state streams live again — values are current',
                          LOGLEVEL.INFO, 'system')

    def _reset_power_peaks(self, why: str):
        """Zero the peaks, logging the outgoing value so none is lost."""
        if self._peak_w > 0.0:
            self._log('peak power %.1f W at %.2f A (%s) — resetting'
                      % (self._peak_w, self._peak_i, why),
                      LOGLEVEL.INFO, 'system')
        self._peak_i = 0.0
        self._peak_w = 0.0

    def _repaint_power_badges(self):
        """Paint the 200 ms mean, at 5 Hz. Readable, and steadier than
        whichever single sample happened to land on the repaint."""
        self._check_stream_freshness()

        a = self._power_accum
        if not a['n']:
            # No samples this window. Deliberately does NOT clear the badges:
            # the last real value is worth keeping on screen, but
            # _check_stream_freshness has already dimmed it and said so.
            return
        n = float(a['n'])
        pb1_v, pb1_i = a['pb1_v'] / n, a['pb1_i'] / n
        pb2_v, pb2_i = a['pb2_v'] / n, a['pb2_i'] / n
        for k in a:
            a[k] = 0

        self._update_power_badges(
            pb1_v, pb1_i,
            self.lbl_pb1_voltage,
            self.lbl_pb1_current, self.lbl_pb1_power,
        )
        self._update_power_badges(
            pb2_v, pb2_i,
            self.lbl_pb2_voltage,
            self.lbl_pb2_current, self.lbl_pb2_power,
        )

        total_i = pb1_i + pb2_i
        total_w = pb1_v * pb1_i + pb2_v * pb2_i
        self.lbl_total_current.setText(f"{total_i:.2f} A")
        self.lbl_total_power.setText(f"{total_w:.1f} W")
        self.lbl_peak_current.setText(f"{self._peak_i:.2f} A")
        self.lbl_peak_power.setText(f"{self._peak_w:.1f} W")

    def _update_button_states(self):
        """Update button enabled/disabled states based on current state"""
        bridge_on = self.btn_ros_bridge.isChecked()
        
        # In simulation mode, enable basic buttons even without bridge
        # In real mode, require bridge for all operations
        enable_basic = bridge_on or self.use_sim_time
        
        # Basic buttons
        self.btn_estop.setEnabled(bridge_on)  # E-stop always requires bridge (safety critical)
        self.btn_stop_gait.setEnabled(bridge_on)
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
