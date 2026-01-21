#!/usr/bin/env python3
"""
Launch script for Corgi Configuration Panel
Entry point for: ros2 run corgi_panel corgi_config_panel
"""
import sys
import os
import signal
from PyQt5.QtWidgets import QApplication

# ROS 2 import (will be initialized by the panel itself)
import rclpy


def load_stylesheet() -> str:
    """
    Load QSS stylesheet from assets directory
    
    Returns:
        Stylesheet content as string, or empty string if not found
    """
    try:
        # Try to find assets directory
        # First check in installed location
        from ament_index_python.packages import get_package_share_directory
        try:
            pkg_share = get_package_share_directory('corgi_panel')
            theme_path = os.path.join(pkg_share, 'assets', 'theme.qss')
        except Exception:
            # Fallback to development location
            current_dir = os.path.dirname(os.path.abspath(__file__))
            pkg_dir = os.path.dirname(current_dir)
            theme_path = os.path.join(pkg_dir, 'corgi_ui', 'assets', 'theme.qss')
        
        if os.path.exists(theme_path):
            with open(theme_path, 'r', encoding='utf-8') as f:
                return f.read()
        else:
            print(f"Warning: Theme file not found at {theme_path}")
            return ""
    except Exception as e:
        print(f"Warning: Failed to load stylesheet: {e}")
        return ""


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\nShutting down Corgi Configuration Panel...")
    QApplication.quit()


def main():
    """Main entry point for Configuration Panel"""
    # Setup signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create QApplication
    app = QApplication(sys.argv)
    
    # Load and apply stylesheet
    stylesheet = load_stylesheet()
    if stylesheet:
        app.setStyleSheet(stylesheet)
    
    # Allow Ctrl+C to work
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    # Import and create config panel
    try:
        from corgi_ui.gui.config_panel import CorgiConfigPanel
        
        # Create main window
        window = CorgiConfigPanel()
        window.show()
        
        # Run application
        exit_code = app.exec_()
        
        # Cleanup ROS if needed
        try:
            rclpy.try_shutdown()
        except Exception:
            pass
        
        sys.exit(exit_code)
        
    except ImportError as e:
        print(f"Error: Failed to import CorgiConfigPanel: {e}")
        print("Make sure the corgi_panel package is properly installed.")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
