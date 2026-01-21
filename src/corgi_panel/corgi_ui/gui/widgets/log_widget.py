#!/usr/bin/env python3
"""
Log Widget for Corgi UI
Reusable log display component with level filtering and HTML formatting.
"""
from datetime import datetime
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTextEdit, 
    QComboBox, QLabel
)
from PyQt5.QtCore import pyqtSignal

from corgi_ui.core.constants import LOGLEVEL, COLORS, format_log_html


class LogWidget(QWidget):
    """
    Reusable log display widget with level filtering.
    
    Features:
    - HTML-formatted log messages with color coding
    - Log level filtering (DEBUG, INFO, WARN, ERROR, FATAL)
    - Auto-scroll to latest message
    - Timestamp display
    
    Signals:
    - log_level_changed: Emitted when minimum log level filter changes
    
    Example usage:
        log_widget = LogWidget()
        log_widget.log_level_changed.connect(on_filter_changed)
        log_widget.add_log("System initialized", LOGLEVEL.INFO, "system")
        log_widget.set_min_level(LOGLEVEL.WARN)
    """
    
    # Signal emitted when log level filter changes
    log_level_changed = pyqtSignal(int)  # LOGLEVEL value
    
    def __init__(self, parent=None, default_level: LOGLEVEL = LOGLEVEL.DEBUG):
        """
        Initialize LogWidget
        
        Args:
            parent: Parent widget
            default_level: Initial minimum log level to display
        """
        super().__init__(parent)
        
        self.min_log_level = default_level
        self._init_ui()
    
    def _init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        
        # Log level filter row
        filter_layout = QHBoxLayout()
        filter_layout.setSpacing(8)
        
        filter_label = QLabel('Min Level:')
        filter_label.setStyleSheet('color: #aaa; font-size: 12px;')
        
        self.combo_log_level = QComboBox()
        self.combo_log_level.addItems(['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'])
        self.combo_log_level.setCurrentIndex(self.min_log_level)
        self.combo_log_level.currentIndexChanged.connect(self._on_level_changed)
        
        # Apply consistent styling
        self.combo_log_level.setStyleSheet("""
            QComboBox {
                background-color: #404040;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 5px;
                color: white;
                min-width: 80px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 5px solid #aaa;
            }
            QComboBox QAbstractItemView {
                background-color: #404040;
                color: white;
                selection-background-color: #3a6ea5;
                border: 1px solid #555;
            }
        """)
        
        filter_layout.addWidget(filter_label)
        filter_layout.addWidget(self.combo_log_level)
        filter_layout.addStretch(1)
        
        # Log text display
        self.text_log = QTextEdit()
        self.text_log.setReadOnly(True)
        
        # Apply monospace font for better log readability
        self.text_log.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                border: 1px solid #444;
                color: #ddd;
                font-family: 'Consolas', 'Monaco', 'Courier New', monospace;
                font-size: 12px;
                padding: 5px;
            }
        """)
        
        # Add components to layout
        layout.addLayout(filter_layout)
        layout.addWidget(self.text_log)
    
    def _on_level_changed(self, index: int):
        """
        Handle log level filter change
        
        Args:
            index: Selected index in combo box
        """
        self.min_log_level = LOGLEVEL(index)
        self.log_level_changed.emit(index)
    
    def add_log(self, message: str, level: LOGLEVEL = LOGLEVEL.INFO, source: str = "system"):
        """
        Add a log message to the display
        
        Args:
            message: Log message content
            level: Log severity level
            source: Source identifier (e.g., 'system', 'orin', 'fpga_driver', 'user')
        """
        # Filter based on minimum level
        if level < self.min_log_level:
            return
        
        # Generate timestamp with date
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        # Format log message as HTML
        log_html = format_log_html(timestamp, level, source, message)
        
        # Append to text widget
        self.text_log.append(log_html)
        
        # Auto-scroll to bottom
        scrollbar = self.text_log.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def set_min_level(self, level: LOGLEVEL):
        """
        Set minimum log level filter
        
        Args:
            level: LOGLEVEL enum value
        """
        self.min_log_level = level
        self.combo_log_level.setCurrentIndex(int(level))
    
    def get_min_level(self) -> LOGLEVEL:
        """
        Get current minimum log level
        
        Returns:
            Current LOGLEVEL filter value
        """
        return self.min_log_level
    
    def clear(self):
        """Clear all log messages"""
        self.text_log.clear()
    
    def set_max_height(self, height: int):
        """
        Set maximum height for the log display
        
        Args:
            height: Maximum height in pixels
        """
        self.text_log.setMaximumHeight(height)
    
    def get_text(self) -> str:
        """
        Get plain text content of logs (without HTML formatting)
        
        Returns:
            Plain text log content
        """
        return self.text_log.toPlainText()
    
    def save_to_file(self, filepath: str):
        """
        Save log content to a file
        
        Args:
            filepath: Path to save file
        """
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(self.get_text())
            return True
        except Exception as e:
            print(f"Failed to save log to {filepath}: {e}")
            return False


class CompactLogWidget(LogWidget):
    """
    Compact version of LogWidget without the filter dropdown.
    Useful for embedded displays or when filter control is external.
    """
    
    def __init__(self, parent=None, default_level: LOGLEVEL = LOGLEVEL.DEBUG):
        """
        Initialize CompactLogWidget
        
        Args:
            parent: Parent widget
            default_level: Initial minimum log level to display
        """
        super().__init__(parent, default_level)
        
        # Remove the filter layout (hide combo box)
        self.combo_log_level.hide()
        self.combo_log_level.parent().parent().layout().itemAt(0).widget().hide()
