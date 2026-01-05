#!/usr/bin/env python3
import os
import sys
import threading
import subprocess
import signal
import numpy as np
from datetime import datetime
import yaml
from enum import IntEnum

import rclpy
from rclpy.executors import SingleThreadedExecutor

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QComboBox, QGroupBox, QStackedWidget,
                             QLineEdit, QPushButton, QTabWidget, QFormLayout, 
                             QGridLayout, QTableWidget, QTableWidgetItem, QHeaderView,
                             QMessageBox, QTextEdit, QSizePolicy)
from PyQt5.QtGui import QIntValidator, QDoubleValidator, QFont, QBrush, QColor, QFontDatabase
from PyQt5.QtCore import Qt, pyqtSignal, QTimer

from corgi_msgs.msg import (
    MotorCmdStamped, MotorStateStamped, PowerCmdStamped, PowerStateStamped,
    RobotCmdStamped, RobotStateStamped, TriggerStamped, LogStamped
)

PLACEHOLDER = "- Please choose -"

class ROBOTMODE(IntEnum):
    SYSTEM_ON = 0
    INIT = 1
    IDLE = 2
    STANDBY = 3
    MOTORCONFIG = 4

class LOGLEVEL(IntEnum):
    DEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3
    FATAL = 4

STYLESHEET = """
QWidget {
    background-color: #2b2b2b;
    color: #ffffff;
    font-family: 'Segoe UI', 'Ubuntu', sans-serif;
    font-size: 14px;
}
QGroupBox {
    border: 1px solid #555;
    border-radius: 5px;
    margin-top: 20px;
    font-weight: bold;
    color: #ccc;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top center;
    padding: 0 3px;
}
QPushButton {
    background-color: #404040;
    border: 1px solid #555;
    border-radius: 4px;
    padding: 8px;
    min-height: 25px;
}
QPushButton:hover { background-color: #505050; }
QPushButton:pressed { background-color: #2d2d2d; }
QPushButton:checked { background-color: #3a6ea5; color: white; border: 1px solid #5a9ed5; }
QPushButton:disabled { background-color: #333; color: #777; border: 1px solid #444; }

QPushButton#PowerOnBtn { background-color: #2e7d32; font-weight: bold; font-size: 16px; }
QPushButton#PowerOnBtn:hover { background-color: #1b5e20; }
QPushButton#PowerOffBtn { background-color: #d32f2f; font-weight: bold; font-size: 16px; }
QPushButton#PowerOffBtn:hover { background-color: #b71c1c; }

QLabel#HeaderLabel { font-size: 18px; font-weight: bold; color: #eee; }
QLabel#StatusLabel { font-size: 24px; font-weight: bold; color: #4fc3f7; }
QLineEdit { background-color: #202020; border: 1px solid #555; color: white; padding: 5px; border-radius: 3px; }
QTextEdit { background-color: #1e1e1e; border: 1px solid #444; color: #00e676; font-family: 'Consolas', monospace; }
QComboBox { background-color: #404040; border: 1px solid #555; color: white; padding: 5px; }
QComboBox:disabled { background-color: #333; color: #777; }
QComboBox::drop-down { border: none; }
QComboBox QAbstractItemView { background-color: #404040; color: white; selection-background-color: #3a6ea5; }
QTabWidget::pane { border: 1px solid #555; }
QTabBar::tab { background-color: #404040; color: #ccc; padding: 8px 16px; border: 1px solid #555; }
QTabBar::tab:selected { background-color: #3a6ea5; color: white; }
QTableWidget { background-color: #2b2b2b; color: white; gridline-color: #555; }
QHeaderView::section { background-color: #404040; color: white; padding: 5px; border: 1px solid #555; }
"""

# TODO: Load parameters
# Placeholder for motor parameters will be loaded
FULL_PARAM_DATA = []
    
class ParameterTabWidget(QWidget):
    def __init__(self, module_name, motor_name, main_window=None):
        super().__init__()
        self.module_name = module_name
        self.motor_name = motor_name
        self.main_window = main_window
        self.input_fields = {}
        self.default_values = {}
        self.is_modified = False 
        self.is_unlocked = False 
        self.initial_values = {} 
        self.is_edited = False
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)

        title = f"<h2 style=\"font-family: 'Segoe UI','Arial','Helvetica','Tahoma','Verdana'; color: #2196F3;\">Configuration: {self.module_name} / {self.motor_name}</h2>"
        main_layout.addWidget(QLabel(title))
        main_layout.addWidget(QLabel("<hr>"))
        
        tab_widget = QTabWidget()
        
        groups = {}
        for p in FULL_PARAM_DATA:
            group_name = p.get('Group', 'Unassigned')
            if group_name not in groups:
                groups[group_name] = []
            groups[group_name].append(p)
            
        sorted_groups = sorted(groups.items(), key=lambda item: {
            'Control': 1, 'Limits': 2, 'Motor': 3, 'System': 4, 'Reserved': 5
        }.get(item[0], 99))
        
        DISPLAY_NAMES = {
            "Control": "Control", "Limits": "Limits", "Motor": "Motor",
            "System": "System", "Reserved": "Reserved"
        }
        
        for group_name, params in sorted_groups:
            tab = self.create_functional_tab(group_name, params)
            display_name = DISPLAY_NAMES.get(group_name, group_name)
            tab_widget.addTab(tab, display_name)
            
        tab_widget.addTab(self.create_raw_table_tab(), "All Parameters")
        
        main_layout.addWidget(tab_widget)

        button_layout = QHBoxLayout()
        
        # Yellow button: Edit mode toggle
        self.edit_button = QPushButton("Edit")
        self.edit_button.setStyleSheet("background-color: #FBC02D; color: black; font-weight: bold; padding: 10px;")
        self.edit_button.clicked.connect(self.toggle_edit_mode)
        self.edit_button.setEnabled(True)
        
        # Blue button: Write parameters temporarily
        self.write_only_button = QPushButton("Write(Temp)")
        self.write_only_button.setStyleSheet("background-color: #2196F3; color: black; font-weight: bold; padding: 10px;")
        self.write_only_button.clicked.connect(self.handle_write_only)
        self.write_only_button.setEnabled(False)

        # Green button: Reset all parameters to default values
        self.reset_default_button = QPushButton("Reset to Default")
        self.reset_default_button.setStyleSheet("background-color: #4CAF50; color: black; font-weight: bold; padding: 10px;")
        self.reset_default_button.clicked.connect(self.handle_reset_to_default)
        self.reset_default_button.setEnabled(False)

        button_layout.addWidget(self.edit_button)
        button_layout.addWidget(self.reset_default_button)
        button_layout.addWidget(self.write_only_button)

        main_layout.addLayout(button_layout)
    
    def log(self, message, level='INFO'):
        """Log message to main window log area"""
        
        if self.main_window:
            self.main_window.log(message, level)
        else:
            print(message)

    def create_functional_tab(self, group_name, params):
        """Create a functional parameter tab - currently placeholder until parameter loading is implemented"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Placeholder message
        placeholder = QLabel(f"<h3>{group_name} Parameters</h3><p>Parameter configuration will be available once parameter loading is implemented.</p>")
        placeholder.setAlignment(Qt.AlignCenter)
        placeholder.setStyleSheet("color: #888; padding: 50px;")
        layout.addWidget(placeholder)
        
        return tab
        
        # TODO: Restore full parameter UI once parameter loading is implemented
        # tab = QWidget()
        # form_layout = QFormLayout(tab)
        # INPUT_FIELD_WIDTH = 150
        # IDEAL_FIELD_WIDTH = 150
        # RANGE_FIELD_WIDTH = 200
        # TOTAL_CONTENT_WIDTH = INPUT_FIELD_WIDTH + IDEAL_FIELD_WIDTH + RANGE_FIELD_WIDTH
        # FIXED_WIDTH = TOTAL_CONTENT_WIDTH + 15

        # # Headers
        # header_widget = QWidget()
        # header_layout = QHBoxLayout(header_widget)
        # header_layout.setContentsMargins(0, 0, 0, 0)
        # header_layout.setSpacing(5) 

        # label_value = QLabel("<b>Current Value</b>")
        # label_value.setFixedWidth(INPUT_FIELD_WIDTH)
        # label_value.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        # label_ideal = QLabel("<b>Default</b>")
        # label_ideal.setFixedWidth(IDEAL_FIELD_WIDTH)
        # label_ideal.setAlignment(Qt.AlignCenter) 

        # label_range = QLabel("<b>Range (Min ~ Max)</b>")
        # label_range.setFixedWidth(RANGE_FIELD_WIDTH)
        # label_range.setAlignment(Qt.AlignLeft | Qt.AlignVCenter) 
        
        # header_layout.addWidget(label_value)
        # header_layout.addWidget(label_ideal)
        # header_layout.addWidget(label_range)
        
        # header_widget.setFixedWidth(FIXED_WIDTH)
        # form_layout.addRow(QLabel(""), header_widget)

        # TODO: Restore parameter field creation once parameter loading is implemented
        # for param in params:
        #     # Parameter label
        #     variable_name = param['Variable']
        #     desc = param.get('Desc', variable_name)
        #     unit = param.get('Unit', variable_name)
        #     
        #     # Label displays description, unit, and variable name
        #     param_label_text = f"<b>{desc}{unit} [{variable_name}]:</b>" 
        #     param_label_widget = QLabel(param_label_text)
        #     param_label_widget.setToolTip(desc)
        #     # param_label_widget.setMinimumWidth(450) 
        #     
        #     init_val = param.get('Init')
        #     is_float = isinstance(init_val, float) or (isinstance(init_val, (int, str)) and '.' in str(str(init_val)))
        #     is_writable = param['Writeable'] != 0

        #     right_widget = QWidget()
        #     h_layout = QHBoxLayout(right_widget)
        #     h_layout.setContentsMargins(0, 0, 0, 0)
        #     h_layout.setSpacing(5) 

        #     if is_writable:
        #         # Editable parameter
        #         input_field = QLineEdit()

        #         min_val = float(param.get('Min') if param.get('Min') != '' else -1e308)
        #         max_val = float(param.get('Max') if param.get('Max') != '' else 1e308)
        #         
        #         # Remove validator, use editingFinished event to limit range instead
        #         input_field.setText(str(init_val))
        #         input_field.setFixedWidth(INPUT_FIELD_WIDTH)
        #         input_field.setReadOnly(True)
        #         
        #         # Save parameter range information
        #         input_field.setProperty('min_val', min_val)
        #         input_field.setProperty('max_val', max_val)
        #         input_field.setProperty('is_float', is_float)
        #         
        #         # When editing is finished, check and limit value range
        #         input_field.editingFinished.connect(lambda field=input_field: self.clamp_value(field))
        #         
        #         self.input_fields[variable_name] = input_field
        #         self.initial_values[variable_name] = str(init_val)
        #         self.default_values[variable_name] = str(init_val)
        #         input_field.textChanged.connect(self.check_modified_state) 
        #         
        #         ideal_value_label = QLabel(str(init_val))
        #         ideal_value_label.setFixedWidth(IDEAL_FIELD_WIDTH)
        #         ideal_value_label.setAlignment(Qt.AlignCenter)
        #         ideal_value_label.setStyleSheet("color: #4CAF50;") 

        #         range_label = QLabel(f"({param['Min']} ~ {param['Max']})")
        #         range_label.setFixedWidth(RANGE_FIELD_WIDTH)
        #         range_label.setAlignment(Qt.AlignVCenter)

        #         h_layout.addWidget(input_field) 
        #         h_layout.addWidget(ideal_value_label)
        #         h_layout.addWidget(range_label) 
        #         
        #     else:
        #         # Read-only parameter
        #         value_label = QLabel(str(init_val))
        #         value_label.setStyleSheet("background-color: #EEEEEE; padding: 3px; border-radius: 3px; color: #555555;")
        #         value_label.setFixedWidth(INPUT_FIELD_WIDTH)

        #         ideal_value_label = QLabel(str(init_val))
        #         ideal_value_label.setFixedWidth(IDEAL_FIELD_WIDTH)
        #         ideal_value_label.setAlignment(Qt.AlignCenter)
        #         ideal_value_label.setStyleSheet("color: #757575;") 

        #         range_label = QLabel(f"({param['Min']} ~ {param['Max']})")
        #         range_label.setFixedWidth(RANGE_FIELD_WIDTH)
        #         range_label.setAlignment(Qt.AlignVCenter)
        #         range_label.setStyleSheet("color: #757575;")

        #         h_layout.addWidget(value_label)
        #         h_layout.addWidget(ideal_value_label)
        #         h_layout.addWidget(range_label)

        #     right_widget.setFixedWidth(FIXED_WIDTH)
        #     
        #     form_layout.addRow(param_label_widget, right_widget)
        #     
        # return tab
    
    def clamp_value(self, input_field):
        """Automatically clamp input value to min/max range when out of bounds"""
        try:
            min_val = input_field.property('min_val')
            max_val = input_field.property('max_val')
            is_float = input_field.property('is_float')
            
            text = input_field.text().strip()
            if not text:
                return
            
            # Try to convert to numeric value
            if is_float:
                value = float(text)
            else:
                value = int(float(text))  # Allow input format like "5.0"
            
            # Clamp value within range
            clamped_value = max(min_val, min(value, max_val))
            
            # If value was changed, update input field
            if value != clamped_value:
                if is_float:
                    input_field.setText(str(clamped_value))
                else:
                    input_field.setText(str(int(clamped_value)))
                
                # Log the clamping action
                if value < min_val:
                    self.log(f"Value clamped to minimum: {clamped_value}")
                else:
                    self.log(f"Value clamped to maximum: {clamped_value}")
            else:
                # Format display (unified format)
                if is_float:
                    input_field.setText(str(value))
                else:
                    input_field.setText(str(int(value)))
                    
        except ValueError:
            # If input is invalid (e.g., text), restore to previous value
            variable_name = None
            for var_name, field in self.input_fields.items():
                if field == input_field:
                    variable_name = var_name
                    break
            
            if variable_name and variable_name in self.initial_values:
                input_field.setText(self.initial_values[variable_name])
                self.log(f"Invalid input. Restored to previous value: {self.initial_values[variable_name]}", 'WARN')
    
    def check_modified_state(self):
        """Check parameter modification state and update button enabled status based on is_unlocked state"""     
        is_modified_now = False
        for variable_name, input_field in self.input_fields.items():
            current_text = input_field.text()
            initial_value = self.initial_values.get(variable_name)
            
            if current_text != initial_value:
                is_modified_now = True
                break
        
        if is_modified_now != self.is_modified:
            self.is_modified = is_modified_now
            # Note: No need to log here - modification state is reflected in UI button states

        # Set button based on latest state
        self.write_only_button.setEnabled(self.is_modified)

    def save_current_as_initial_and_unlock(self, permanent=False):
        """Save all input field current values as new initial values, and unlock or disable buttons accordingly"""
        # No need to check validity anymore, as values are automatically clamped within range
        for variable_name, input_field in self.input_fields.items():
            self.initial_values[variable_name] = input_field.text() 
        
        self.is_modified = False 
        
        if permanent:
            self.is_unlocked = False
            self.write_only_button.setEnabled(False)
            self.log("Parameters PERMANENTLY saved.", 'INFO')
        else:
            self.is_unlocked = True
            self.write_only_button.setEnabled(False)
            self.log("Parameters TEMPORARILY saved.", 'INFO')

        return True

    def create_raw_table_tab(self):
        """Create a raw data table tab"""
        """Create raw parameter table tab - currently placeholder until parameter loading is implemented"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # Placeholder message
        placeholder = QLabel("<h3>All Parameters</h3><p>Parameter table will be available once parameter loading is implemented.</p>")
        placeholder.setAlignment(Qt.AlignCenter)
        placeholder.setStyleSheet("color: #888; padding: 50px;")
        layout.addWidget(placeholder)
        
        return tab
        
        # TODO: Restore full table once parameter loading is implemented
        # table = QTableWidget()
        # table.setRowCount(len(FULL_PARAM_DATA))
        # table.setColumnCount(6)
        # table.setHorizontalHeaderLabels(["Variable", "Description", "Writeable", "Ideal Value", "Min Value", "Max Value"])
        # 
        # for row, param in enumerate(FULL_PARAM_DATA):
        #     is_readonly = param.get("Writeable", 0) == 0
        #     
        #     item_variable = QTableWidgetItem(str(param.get("Variable", "")))
        #     item_desc = QTableWidgetItem(str(param.get("Desc", "")))
        #     item_writeable = QTableWidgetItem(str(param.get("Writeable", "")))
        #     item_init = QTableWidgetItem(str(param.get("Init", "")))
        #     item_min = QTableWidgetItem(str(param.get("Min", "")))
        #     item_max = QTableWidgetItem(str(param.get("Max", "")))

        #     if is_readonly:
        #         gray_background = QBrush(QColor("#B8B8B8"))
        #         item_variable.setBackground(gray_background)
        #         item_desc.setBackground(gray_background)
        #         item_writeable.setBackground(gray_background)
        #         item_init.setBackground(gray_background)
        #         item_min.setBackground(gray_background)
        #         item_max.setBackground(gray_background)
        #     
        #     table.setItem(row, 0, item_variable)
        #     table.setItem(row, 1, item_desc)
        #     table.setItem(row, 2, item_writeable)
        #     table.setItem(row, 3, item_init)
        #     table.setItem(row, 4, item_min)
        #     table.setItem(row, 5, item_max)

        # table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        # table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        # 
        # layout.addWidget(table)
        # return tab

    def check_parameters_validity(self):
        """Check validity of all input parameters and mark errors"""
        has_error = False
        for variable_name, input_field in self.input_fields.items():
            if not input_field.hasAcceptableInput():
                 self.log(f"Error: Parameter {variable_name} Value '{input_field.text()}' is invalid or out of range.", 'ERROR')
                 input_field.setStyleSheet("background-color: #FFDDEE; border: 2px solid #F44336;") 
                 has_error = True
            else:
                 if self.is_edited:
                     input_field.setStyleSheet("background-color: #FFFFFF; border: 1px solid #2196F3;")
                 else:
                     input_field.setStyleSheet("background-color: #F5F5F5; border: 1px solid #CCCCCC;")
        return not has_error
    
    def handle_write_only(self):
        """Blue button function: Write parameters temporarily"""
        if not self.check_parameters_validity():
            QMessageBox.critical(self, "Validation Error", "Parameters contain errors. Please fix them before proceeding.", 
                                 QMessageBox.StandardButton.Ok)
            self.log("Parameters contain errors. Please fix them before proceeding.", 'ERROR')
            return

        self.log(f"Writing parameters for {self.module_name} / {self.motor_name}...", 'INFO')
        
        if self.save_current_as_initial_and_unlock(permanent=False): 
            QMessageBox.information(self, "Write Successful", f"{self.motor_name} Parameters is written successfully (temporary).", 
                                    QMessageBox.StandardButton.Ok)

    def toggle_edit_mode(self):
        """Edit button: Toggle edit mode, allow or prohibit parameter editing"""
        self.is_edited = not self.is_edited
        for input_field in self.input_fields.values():
            input_field.setReadOnly(not self.is_edited)
            if self.is_edited:
                # Editable mode
                input_field.setStyleSheet("background-color: #FFFFFF; border: 1px solid #2196F3;") 
            else:
                # Locked mode
                input_field.setStyleSheet("background-color: #F5F5F5; border: 1px solid #CCCCCC;") 
        
        # Update button text and style, and reset button enabled status
        if self.is_edited:
            self.edit_button.setText("Lock Parameters")
            self.edit_button.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold; padding: 10px;")
            self.reset_default_button.setEnabled(True)  # Enable reset button in edit mode
            self.log("Edit mode ENABLED. You can now modify parameters.", 'INFO')
        else:
            self.edit_button.setText("Edit Parameters")
            self.edit_button.setStyleSheet("background-color: #FBC02D; color: black; font-weight: bold; padding: 10px;")
            self.reset_default_button.setEnabled(False)  # Disable reset button in locked mode
            self.log("Edit mode DISABLED. Parameter editing is locked.", 'INFO')

    def handle_reset_to_default(self):
        """Green button function: Reset all modifiable parameters to default values"""
        reply = QMessageBox.question(
            self,
            "Reset to Default Confirmation",
            f"Are you sure you want to RESET all parameters to their DEFAULT values for {self.motor_name}?\n\nAll current changes will be lost.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            # Reset all modifiable parameters to default values
            for variable_name, input_field in self.input_fields.items():
                default_value = self.default_values.get(variable_name)
                if default_value is not None:
                    input_field.setText(default_value)
            
            self.log(f"All parameters for {self.motor_name} have been reset to DEFAULT values.", 'INFO')
            QMessageBox.information(
                self,
                "Reset Successful",
                f"All parameters for {self.motor_name} have been reset to their DEFAULT values.",
                QMessageBox.StandardButton.Ok
            )
        else:
            self.log("Reset to Default action cancelled by user.", 'WARN')

    def handle_set_default(self):
        """Green button function: Set current parameters as default values"""
        if not self.check_parameters_validity():
            QMessageBox.critical(self, "Validation Error", "Parameters contain errors. Please fix them before proceeding.", 
                                 QMessageBox.StandardButton.Ok)
            self.log("Parameters contain errors. Please fix them before proceeding.")
            return

        reply = QMessageBox.question(
            self,
            "Set as Default Confirmation",
            "Are you sure you want to set the current parameters as DEFAULT values?\n\nThis action cannot be undone.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            if self.save_current_as_initial_and_unlock(permanent=True): 
                QMessageBox.information(self, "Set as Default Successful", f"{self.motor_name} Parameters are set as DEFAULT successfully.", 
                                        QMessageBox.StandardButton.Ok)
        else:
            self.log("Set as Default action cancelled by user.", 'WARN')

class MainWindow(QWidget):
    robot_state_signal = pyqtSignal(object)
    log_state_signal = pyqtSignal(object)
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Configuration")
        self.setGeometry(100, 100, 1100, 700)
        self.setStyleSheet(STYLESHEET)
        self.motor_data = {
            "A": ["Direct (A1)", "Belt (A2)"],
            "B": ["Direct (B1)", "Belt (B2)"],
            "C": ["Direct (C1)", "Belt (C2)"],
            "D": ["Direct (D1)", "Belt (D2)"]
        }
        self.motor_index_map = {}
        self.robot_state = RobotStateStamped()
        self._robot_cmd_seq = 0
        self._last_logged_robot_mode = None  # Track last logged mode to avoid redundant logs
        
        # Initialize UI first so log_text exists before init_ros tries to log
        self.init_ui()
        self.init_ros()
        
        # Connect signals
        self.robot_state_signal.connect(self._handle_robot_state_update)
        self.log_state_signal.connect(self._handle_log_update)

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        motor_group = QHBoxLayout()
        
        left_panel = QWidget()
        left_panel.setFixedWidth(300) 
        left_layout = QVBoxLayout(left_panel)
        left_group = QGroupBox("Selection")
        self.left_group = left_group  # Save reference for later enable/disable operations
        group_layout = QVBoxLayout()
        
        module_layout = QHBoxLayout()
        self.module_combo = QComboBox()
        module_list = [PLACEHOLDER] + list(self.motor_data.keys())
        self.module_combo.addItems(module_list)
        module_layout.addWidget(QLabel("Module:"))
        module_layout.addWidget(self.module_combo)
        group_layout.addLayout(module_layout)
        
        motor_layout = QHBoxLayout()
        self.motor_combo = QComboBox()
        self.motor_combo.setEnabled(False) 
        motor_layout.addWidget(QLabel("Motor:"))
        motor_layout.addWidget(self.motor_combo)
        group_layout.addLayout(motor_layout)

        left_group.setLayout(group_layout)
        left_layout.addWidget(left_group)

        # Power control area (left panel)
        power_group = QGroupBox("Power Control")
        power_layout = QVBoxLayout()
        
        self.power_button = QPushButton("Power OFF")
        self.power_button.setObjectName("PowerOffBtn")
        self.power_button.setFixedHeight(50)
        self.power_button.clicked.connect(self.toggle_power)
        
        power_layout.addWidget(self.power_button)
        power_group.setLayout(power_layout)
        
        # Place power control at the top
        left_layout.insertWidget(0, power_group)
        left_layout.addStretch(1) 
        motor_group.addWidget(left_panel)

        self.stacked_widget = QStackedWidget()
        
        self.placeholder_page = QWidget()
        placeholder_layout = QVBoxLayout(self.placeholder_page)
        placeholder_layout.addWidget(QLabel("<h1 style=\"font-family: 'Segoe UI','Arial','Helvetica','Tahoma','Verdana';\">Please select a motor on the left for configuration.</h1>"))
        placeholder_layout.setAlignment(Qt.AlignCenter)
        self.stacked_widget.addWidget(self.placeholder_page)
        
        self.create_config_pages()

        motor_group.addWidget(self.stacked_widget)
        
        # Add main content to container layout
        main_layout.addLayout(motor_group)

        # Add log display area
        log_group = QGroupBox("Log")
        self.log_group = log_group  # Save reference for later enable/disable operations
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        self.log_text.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        log_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)
        # Allocate vertical space ratio: upper content : log = 3 : 1 (adjustable)
        main_layout.setStretch(0, 3)  # motor_group
        main_layout.setStretch(1, 1)  # log_group

        self.module_combo.currentTextChanged.connect(self.update_motor_combo)
        self.motor_combo.currentTextChanged.connect(self.switch_config_page)

        self.update_motor_combo(self.module_combo.currentText())

        # Apply UI lock based on current power state (default is off)
        self.apply_power_state()

    def log(self, message, level='INFO'):
        """Add message to log display area"""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        
        # Color map matching control panel
        color_map = {
            'DEBUG': '#2196f3',
            'INFO': '#00e676',
            'WARN': '#ffea00',
            'ERROR': '#ff5252',
            'FATAL': '#d32f2f',
            'SYSTEM': '#bb86fc',
        }
        color = color_map.get(level, '#ffffff')
        
        level_padded = f'{level:5s}'
        
        log_html = f'<span style="color:#888;">[{timestamp}]</span> '
        log_html += f'<span style="color:{color}; font-weight:bold;">[{level_padded}]</span> '
        log_html += f'<span style="color:#aaa;">[orin]</span> '
        log_html += f'<span style="color:#ddd;">{message}</span>'
        
        self.log_text.append(log_html)
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def create_config_pages(self):
        index = 1
        for module, motors in self.motor_data.items():
            for motor in motors:
                config_page = ParameterTabWidget(module, motor, self) 
                self.stacked_widget.addWidget(config_page)
                self.motor_index_map[motor] = index
                index += 1
        
    def update_motor_combo(self, selected_module):
        self.motor_combo.clear()
        
        if selected_module == PLACEHOLDER:
            self.motor_combo.setEnabled(False)
            self.motor_combo.addItem(PLACEHOLDER)
            self.stacked_widget.setCurrentIndex(0)
        elif selected_module in self.motor_data:
            self.motor_combo.setEnabled(True)
            motor_list = [PLACEHOLDER] + self.motor_data[selected_module]
            self.motor_combo.addItems(motor_list)

    def switch_config_page(self, selected_motor):
        if selected_motor in self.motor_index_map:
            target_index = self.motor_index_map[selected_motor]
            self.stacked_widget.setCurrentIndex(target_index)
            self.current_motor = selected_motor
        else:
            self.stacked_widget.setCurrentIndex(0)
            self.current_motor = None

    def apply_power_state(self):
        """Enable/disable entire interface based on power state (except power button)"""
        current_mode = self.robot_state.robot_mode if hasattr(self.robot_state, 'robot_mode') else -1
        
        # Enable interface when in MOTORCONFIG mode
        enabled = (current_mode == ROBOTMODE.MOTORCONFIG)
        
        # Left Selection area and right config page
        if hasattr(self, 'left_group'):
            self.left_group.setEnabled(enabled)
        if hasattr(self, 'stacked_widget'):
            self.stacked_widget.setEnabled(enabled)
        
        # Update power button appearance based on mode
        if current_mode == ROBOTMODE.SYSTEM_ON:
            # System ON = Motors OFF
            self.power_button.setText("Power ON")
            self.power_button.setObjectName("PowerOnBtn")
            self.power_button.setEnabled(True)
        elif current_mode == ROBOTMODE.IDLE:
            # IDLE = Motors ON
            self.power_button.setText("Power OFF")
            self.power_button.setObjectName("PowerOffBtn")
            self.power_button.setEnabled(True)
        elif current_mode == ROBOTMODE.MOTORCONFIG:
            # In config mode, show current state
            self.power_button.setText("Power ON")
            self.power_button.setObjectName("PowerOnBtn")
            self.power_button.setEnabled(True)
        else:
            self.power_button.setEnabled(False)
        
        # Refresh button style
        self.power_button.style().unpolish(self.power_button)
        self.power_button.style().polish(self.power_button)
        
        # Refresh Motor list enabled state based on current module state
        if enabled:
            self.update_motor_combo(self.module_combo.currentText())

    def toggle_power(self):
        """Toggle power state - based on robot_state"""
        current_mode = self.robot_state.robot_mode if hasattr(self.robot_state, 'robot_mode') else -1
        
        if current_mode == ROBOTMODE.SYSTEM_ON:
            # System ON => Power OFF, clicking will power ON (send IDLE command)
            self._pub_robot_mode(ROBOTMODE.IDLE)
            self.log("Powering ON motors...", 'SYSTEM')
        elif current_mode == ROBOTMODE.IDLE:
            # IDLE => Power ON, clicking will power OFF (send SYSTEM_ON command with warning)
            reply = QMessageBox.warning(
                self,
                "POWER OFF WARNING",
                "You are about to power OFF all motors.\n\nThis will interrupt current operation.\n\nAre you sure you want to proceed?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if reply == QMessageBox.StandardButton.Yes:
                self._pub_robot_mode(ROBOTMODE.SYSTEM_ON)
                self.log("Powering OFF motors...", 'SYSTEM')
            else:
                self.log("Power OFF cancelled by user.", 'WARN')
        else:
            self.log(f"Cannot toggle power from current mode: {current_mode}", 'WARN')
    
    def _pub_robot_mode(self, mode):
        """Publish robot mode command"""
        robot_cmd = RobotCmdStamped()
        robot_cmd.header.seq = self._robot_cmd_seq + 1
        robot_cmd.header.stamp = self.node.get_clock().now().to_msg()
        robot_cmd.header.frame_id = ''
        robot_cmd.request_robot_mode = int(mode)
        self.robot_cmd_pub.publish(robot_cmd)
        self._robot_cmd_seq += 1
        self.log(f'Sent Robot Mode Command: {mode.name} ({mode.value}), seq={self._robot_cmd_seq}', 'SYSTEM')
    
    def init_ros(self):
        """Initialize ROS2 node and subscriptions"""
        rclpy.init(args=None)
        self.node = rclpy.create_node('corgi_config_panel')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._executor_thread.start()
        
        # Publishers
        self.robot_cmd_pub = self.node.create_publisher(RobotCmdStamped, 'robot/command', 10)
        
        # Subscribers
        self.robot_state_sub = self.node.create_subscription(
            RobotStateStamped, 'robot/state', self.robot_state_cb, 10)
        self.log_sub = self.node.create_subscription(
            LogStamped, 'log', self.log_cb, 10)
        
        self.log('[SYSTEM] Config Panel Initialized', 'INFO')
    
    def robot_state_cb(self, state):
        """ROS2 callback - emit signal for thread safety"""
        self.robot_state_signal.emit(state)
    
    def log_cb(self, log_msg):
        """ROS2 callback - emit signal for thread safety"""
        self.log_state_signal.emit(log_msg)
    
    def _handle_robot_state_update(self, state):
        """Handle robot state updates in main thread"""
        self.robot_state = state
        current_mode = int(state.robot_mode)
        
        if self._last_logged_robot_mode != current_mode:
            try:
                mode_enum = ROBOTMODE(state.robot_mode)
                mode_text = mode_enum.name
                self.log(f'Robot mode changed to: {mode_text}', 'INFO')
                self._last_logged_robot_mode = current_mode
            except ValueError:
                mode_text = "UNKNOWN"
        
        self.apply_power_state()
    
    def _handle_log_update(self, log_msg):

        level = log_msg.level
        node_name = log_msg.node_name if hasattr(log_msg, 'node_name') else 'unknown'
        message = log_msg.message if hasattr(log_msg, 'message') else ''

        if hasattr(log_msg.header, 'stamp'):
            stamp = log_msg.header.stamp
            timestamp = datetime.fromtimestamp(stamp.sec + stamp.nanosec / 1e9).strftime('%Y-%m-%d %H:%M:%S.%f')
        else:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')

        level_map = {
            LOGLEVEL.DEBUG: ('DEBUG', '#2196f3'),
            LOGLEVEL.INFO: ('INFO ', '#00e676'),
            LOGLEVEL.WARN: ('WARN ', '#ffea00'),
            LOGLEVEL.ERROR: ('ERROR', '#ff5252'),
            LOGLEVEL.FATAL: ('FATAL', '#d32f2f'),
        }

        level_name, color = level_map.get(level, ('UNKNOWN', '#ffffff'))
        log_html = f'<span style="color:#888;">[{timestamp}]</span> '
        log_html += f'<span style="color:{color}; font-weight:bold;">[{level_name}]</span> '
        log_html += f'<span style="color:#aaa;">[{node_name}]</span> '
        log_html += f'<span style="color:#ddd;">{message}</span>'
        
        self.log_text.append(log_html)
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())
    
    def closeEvent(self, event):
        try:
            self.node.destroy_subscription(self.robot_state_sub)
        except:
            pass
        try:
            self.node.destroy_subscription(self.log_sub)
        except:
            pass
        try:
            rclpy.try_shutdown()
        except:
            pass
        super(MainWindow, self).closeEvent(event)
   
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion') 
    candidates = ["Segoe UI", "Arial", "Helvetica", "Tahoma", "Verdana"]
    db = QFontDatabase()
    for fam in candidates:
        if fam in db.families():
            app.setFont(QFont(fam, 10))
            break
    # TODO: Load parameters
    # FULL_PARAM_DATA = load_parameters()
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())