#!/usr/bin/env python3
"""
Constants and Enumerations for Corgi UI
Centralized definitions for robot modes, log levels, error codes, and colors.
"""
from enum import IntEnum
import logging
import os

# ============================================================================
# Robot State Enumerations
# ============================================================================

class ROBOTMODE(IntEnum):
    """Robot finite state machine modes"""
    SYSTEM_ON = 0
    INIT = 1
    IDLE = 2
    STANDBY = 3
    MOTORCONFIG = 4


class Module(IntEnum):
    """Motor module identifiers (legs)"""
    MODULE_A = 0  # LF (Left Front)
    MODULE_B = 1  # RF (Right Front)
    MODULE_C = 2  # RH (Right Hind)
    MODULE_D = 3  # LH (Left Hind)


class Motor(IntEnum):
    """Motor identifiers within each module"""
    MOTOR_R = 0  # Right motor (Belt drive)
    MOTOR_L = 1  # Left motor (Direct drive)


class ConfigMode(IntEnum):
    """Motor configuration operation modes"""
    READ = 0
    WRITE = 1


# ============================================================================
# Error Codes of Motor Configuration
# ============================================================================

class ErrorCode(IntEnum):
    """Motor configuration error codes"""
    CODE_CONFIG_SUCCESS = 0
    CODE_INVALID_VALUE = 1
    CODE_READ_ONLY = 2
    CODE_INVALID_ADDR = 3
    CODE_INVALID_CMD = 4
    INVALIDE_MODULE_INDEX = 5
    INVALIDE_MOTOR_INDEX = 6
    INVALIDE_SEQ = 7


# ============================================================================
# Logging System
# ============================================================================

class LOGLEVEL(IntEnum):
    """Log severity levels"""
    DEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3
    FATAL = 4


# Mapping from LOGLEVEL enum to Python logging levels
LOGLEVEL_TO_LOGGING_MAP = {
    LOGLEVEL.DEBUG: logging.DEBUG,
    LOGLEVEL.INFO: logging.INFO,
    LOGLEVEL.WARN: logging.WARNING,
    LOGLEVEL.ERROR: logging.ERROR,
    LOGLEVEL.FATAL: logging.CRITICAL,
}


# ============================================================================
# UI Color Definitions
# ============================================================================

class COLORS:
    """Centralized color palette for UI elements"""
    
    # Log Level Colors (for HTML logging)
    LOG_COLORS = {
        LOGLEVEL.DEBUG: '#2196f3',  # Blue
        LOGLEVEL.INFO:  '#00e676',  # Green
        LOGLEVEL.WARN:  '#ffea00',  # Yellow
        LOGLEVEL.ERROR: '#ff5252',  # Red
        LOGLEVEL.FATAL: '#d32f2f',  # Dark Red
    }
    
    # Log Level Names (formatted for display)
    LOG_NAMES = {
        LOGLEVEL.DEBUG: 'DEBUG',
        LOGLEVEL.INFO:  'INFO ',
        LOGLEVEL.WARN:  'WARN ',
        LOGLEVEL.ERROR: 'ERROR',
        LOGLEVEL.FATAL: 'FATAL',
    }
    
    # Status Indication Colors
    STATUS_SUCCESS = '#00e676'  # Green
    STATUS_WARNING = '#ffea00'  # Yellow
    STATUS_ERROR = '#ff5252'    # Red
    STATUS_INFO = '#4fc3f7'     # Light Blue
    STATUS_NEUTRAL = '#bdbdbd'  # Gray
    
    # UI Element Colors
    TIMESTAMP = '#888'
    SOURCE = '#aaa'
    MESSAGE = '#ddd'
    
    # Button Colors
    BTN_PRIMARY = '#3a6ea5'
    BTN_PRIMARY_HOVER = '#4a7eb5'
    BTN_SUCCESS = '#2e7d32'
    BTN_SUCCESS_HOVER = '#3e8d42'
    BTN_DANGER = '#d32f2f'
    BTN_DANGER_HOVER = '#b71c1c'


# ============================================================================
# System Paths (for subprocess management)
# ============================================================================

class PATHS:
    """System paths for ROS 2 packages and executables"""
    # These can be overridden or made configurable
    HOME_DIR = os.path.expanduser('~')
    DEFAULT_CSV_DIR = os.path.join(HOME_DIR, 'corgi_ws/corgi_ros2_ws/input_csv')
    DEFAULT_OUTPUT_DIR = os.path.join(HOME_DIR, 'corgi_ws/corgi_ros2_ws/output_data')
    DEFAULT_LOG_DIR = os.path.join(HOME_DIR, 'corgi_ws/corgi_ros2_ws/log_file')


# ============================================================================
# Helper Functions
# ============================================================================

def get_log_color(level: LOGLEVEL) -> str:
    """Get HTML color code for a log level"""
    return COLORS.LOG_COLORS.get(level, '#ffffff')


def get_log_name(level: LOGLEVEL) -> str:
    """Get formatted name for a log level"""
    return COLORS.LOG_NAMES.get(level, 'UNKNOWN')


def format_log_html(timestamp: str, level: LOGLEVEL, source: str, message: str) -> str:
    """
    Format a log message as HTML with appropriate colors
    
    Args:
        timestamp: Formatted timestamp string (e.g., '12:34:56.789')
        level: Log level enum value
        source: Source identifier (e.g., 'system', 'orin', 'fpga_driver')
        message: Log message content
    
    Returns:
        HTML-formatted log string
    """
    color = get_log_color(level)
    level_name = get_log_name(level)
    
    html = f'<span style="color:{COLORS.TIMESTAMP};">[{timestamp}]</span> '
    html += f'<span style="color:{color}; font-weight:bold;">[{level_name}]</span> '
    html += f'<span style="color:{COLORS.SOURCE};">[{source}]</span> '
    html += f'<span style="color:{COLORS.MESSAGE};">{message}</span>'
    
    return html


def setup_file_logger(name: str, log_filepath: str):
    """
    Create a standardized file logger
    
    Args:
        name: Logger name (e.g., 'CorgiControlPanel', 'CorgiConfigPanel')
        log_filepath: Full path to log file
    
    Returns:
        Configured logging.Logger instance
    """

    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    
    # Remove existing handlers to avoid duplicates
    logger.handlers.clear()
    
    handler = logging.FileHandler(log_filepath, mode='w')
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        '[%(asctime)s] [%(levelname)-5s] [%(name)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    
    return logger


def log_to_file(logger, level: LOGLEVEL, source: str, message: str):
    """
    Write a log message to file logger
    
    Args:
        logger: logging.Logger instance
        level: LOGLEVEL enum value
        source: Source identifier
        message: Log message
    """
    file_level = LOGLEVEL_TO_LOGGING_MAP.get(level, logging.INFO)
    logger.log(file_level, f'[{source}] {message}')


def close_file_logger(logger, log_filepath: str, panel_name: str = "Panel"):
    """
    Close file logger and display save confirmation
    
    Args:
        logger: logging.Logger instance to close
        log_filepath: Path where log file was saved
        panel_name: Name of the panel (for display message)
    """
    try:
        # Close all handlers
        for handler in logger.handlers[:]:
            handler.close()
            logger.removeHandler(handler)
        print(f'{panel_name} log saved to: {log_filepath}')
    except Exception as e:
        print(f'Error closing {panel_name} log file: {e}')
