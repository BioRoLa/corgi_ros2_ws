#!/usr/bin/env python3
"""
Launch script for Corgi Control Panel
Entry point for: ros2 run corgi_panel corgi_control_panel
"""
import sys
import os
import signal
import multiprocessing
import subprocess
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer


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
                stylesheet = f.read()
                print(f"✓ Loaded stylesheet from: {theme_path}")
                print(f"  Stylesheet length: {len(stylesheet)} characters")
                return stylesheet
        else:
            print(f"Warning: Theme file not found at {theme_path}")
            # Try alternative paths
            alt_paths = [
                os.path.join(current_dir, '..', 'corgi_ui', 'assets', 'theme.qss'),
                os.path.join(os.path.dirname(current_dir), 'corgi_ui', 'assets', 'theme.qss'),
            ]
            for alt_path in alt_paths:
                if os.path.exists(alt_path):
                    with open(alt_path, 'r', encoding='utf-8') as f:
                        stylesheet = f.read()
                        print(f"✓ Loaded stylesheet from alternative path: {alt_path}")
                        return stylesheet
            return ""
    except Exception as e:
        print(f"Warning: Failed to load stylesheet: {e}")
        return ""


def signal_handler(sig, frame):
    """Handle Ctrl+C / SIGTERM gracefully — triggers closeEvent so child processes are cleaned up"""
    print(f"\nShutting down Corgi Control Panel (signal {sig})...")
    # closeAllWindows() triggers each window's closeEvent, which calls
    # process_manager.cleanup_all() to terminate csv_control / data_recorder etc.
    QApplication.closeAllWindows()
    QApplication.quit()


def main():
    """Main entry point for Control Panel"""
    # Create QApplication
    app = QApplication(sys.argv)
    
    # Load and apply stylesheet
    stylesheet = load_stylesheet()
    if stylesheet:
        app.setStyleSheet(stylesheet)
    
    # Import and create control panel
    try:
        from corgi_ui.gui.control_panel import CorgiControlPanel
        
        # Create main window
        window = CorgiControlPanel()
        window.show()

        # Set up signal handlers AFTER the window exists so closeEvent can run.
        # Reset to SIG_DFL first so Python receives the signal at all (Qt blocks it
        # otherwise), then install our handler.
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # QTimer that fires every 500 ms — this gives the Python interpreter a
        # chance to process the signal even while Qt's event loop is running.
        noop_timer = QTimer()
        noop_timer.timeout.connect(lambda: None)
        noop_timer.start(500)
        
        # Run application
        exit_code = app.exec_()
        
        # Cleanup ROS if needed
        try:
            rclpy.try_shutdown()
        except Exception:
            pass

        sys.exit(exit_code)
        
    except ImportError as e:
        print(f"Error: Failed to import CorgiControlPanel: {e}")
        print("Make sure the corgi_panel package is properly installed.")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
