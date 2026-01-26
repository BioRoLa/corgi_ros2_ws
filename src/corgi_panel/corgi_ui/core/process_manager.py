#!/usr/bin/env python3
"""
Process Manager for Corgi UI
Manages subprocess lifecycle (ROS nodes, external tools) with thread-safe operations.
"""
import subprocess
import signal
import threading
import logging
from typing import Optional, Dict, List
from enum import IntEnum


class ProcessState(IntEnum):
    """Process state enumeration"""
    STOPPED = 0
    STARTING = 1
    RUNNING = 2
    STOPPING = 3
    FAILED = 4


class ManagedProcess:
    """Container for a managed subprocess with metadata"""
    
    def __init__(self, name: str, command: List[str]):
        self.name = name
        self.command = command
        self.process: Optional[subprocess.Popen] = None
        self.state = ProcessState.STOPPED
        self._lock = threading.Lock()
    
    @property
    def is_running(self) -> bool:
        """Check if process is currently running"""
        with self._lock:
            if self.process is None:
                return False
            return self.process.poll() is None
    
    @property
    def pid(self) -> Optional[int]:
        """Get process ID if running"""
        with self._lock:
            return self.process.pid if self.process else None
    
    @property
    def returncode(self) -> Optional[int]:
        """Get return code if process has terminated"""
        with self._lock:
            return self.process.returncode if self.process else None


class ProcessManager:
    """
    Centralized manager for subprocess operations.
    Provides thread-safe start/stop operations for ROS nodes and external tools.
    
    Example usage:
        manager = ProcessManager()
        manager.start_process('ros_bridge', ['ros2', 'launch', 'pkg', 'file.py'])
        manager.stop_process('ros_bridge')
        manager.cleanup_all()
    """
    
    def __init__(self, logger: Optional[logging.Logger] = None):
        """
        Initialize ProcessManager
        
        Args:
            logger: Optional logger instance. If None, creates a default logger.
        """
        self._processes: Dict[str, ManagedProcess] = {}
        self._lock = threading.RLock()  # Reentrant lock for nested calls
        self.logger = logger or self._create_default_logger()
    
    def _create_default_logger(self) -> logging.Logger:
        """Create a default logger for ProcessManager"""
        logger = logging.getLogger('ProcessManager')
        logger.setLevel(logging.DEBUG)
        if not logger.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(
                logging.Formatter('[%(asctime)s] [%(name)s] %(levelname)s: %(message)s')
            )
            logger.addHandler(handler)
        return logger
    
    def start_process(
        self, 
        name: str, 
        command: List[str],
        cwd: Optional[str] = None,
        shell: bool = False,
        capture_output: bool = True
    ) -> bool:
        """
        Start a new subprocess
        
        Args:
            name: Unique identifier for this process
            command: Command as list of strings (e.g., ['ros2', 'launch', 'pkg', 'file.py'])
            cwd: Working directory for the process
            shell: Whether to run command through shell
            capture_output: Whether to capture stdout/stderr (False = inherit from parent)
        
        Returns:
            True if process started successfully, False otherwise
        """
        with self._lock:
            # Check if process already exists
            if name in self._processes:
                existing = self._processes[name]
                if existing.is_running:
                    self.logger.warning(f"Process '{name}' is already running (PID: {existing.pid})")
                    return False
                else:
                    self.logger.info(f"Removing terminated process '{name}' (returncode: {existing.returncode})")
                    del self._processes[name]
            
            try:
                # Prepare subprocess arguments
                kwargs = {
                    'cwd': cwd,
                    'shell': shell,
                }
                
                if capture_output:
                    kwargs['stdout'] = subprocess.PIPE
                    kwargs['stderr'] = subprocess.PIPE
                else:
                    # Inherit parent's stdout/stderr
                    kwargs['stdout'] = None
                    kwargs['stderr'] = None
                
                # Start the process
                self.logger.info(f"Starting process '{name}': {' '.join(command)}")
                proc = subprocess.Popen(command, **kwargs)
                
                # Create managed process container
                managed = ManagedProcess(name, command)
                managed.process = proc
                managed.state = ProcessState.RUNNING
                self._processes[name] = managed
                
                self.logger.info(f"Process '{name}' started successfully (PID: {proc.pid})")
                return True
                
            except FileNotFoundError as e:
                self.logger.error(f"Failed to start '{name}': Command not found - {e}")
                return False
            except PermissionError as e:
                self.logger.error(f"Failed to start '{name}': Permission denied - {e}")
                return False
            except Exception as e:
                self.logger.error(f"Failed to start '{name}': {e}")
                return False
    
    def stop_process(
        self, 
        name: str, 
        timeout: float = 5.0,
        force: bool = False
    ) -> bool:
        """
        Stop a running subprocess gracefully
        
        Args:
            name: Process identifier
            timeout: Time to wait for graceful termination before force kill
            force: If True, send SIGKILL immediately instead of SIGTERM
        
        Returns:
            True if process was stopped successfully, False otherwise
        """
        with self._lock:
            if name not in self._processes:
                self.logger.warning(f"Process '{name}' not found in registry")
                return False
            
            managed = self._processes[name]
            
            if not managed.is_running:
                self.logger.info(f"Process '{name}' is not running (already terminated)")
                del self._processes[name]
                return True
            
            try:
                proc = managed.process
                self.logger.info(f"Stopping process '{name}' (PID: {proc.pid})")
                managed.state = ProcessState.STOPPING
                
                if force:
                    # Force kill immediately
                    proc.kill()  # SIGKILL
                    proc.wait(timeout=timeout)
                    self.logger.info(f"Process '{name}' force killed")
                else:
                    # Graceful termination
                    proc.terminate()  # SIGTERM
                    try:
                        proc.wait(timeout=timeout)
                        self.logger.info(f"Process '{name}' terminated gracefully")
                    except subprocess.TimeoutExpired:
                        # Timeout - force kill
                        self.logger.warning(f"Process '{name}' did not respond to SIGTERM, force killing")
                        proc.kill()
                        proc.wait(timeout=2.0)
                        self.logger.info(f"Process '{name}' force killed")
                
                managed.state = ProcessState.STOPPED
                del self._processes[name]
                return True
                
            except Exception as e:
                self.logger.error(f"Error stopping process '{name}': {e}")
                managed.state = ProcessState.FAILED
                return False
    
    def is_running(self, name: str) -> bool:
        """
        Check if a process is currently running
        
        Args:
            name: Process identifier
        
        Returns:
            True if process is running, False otherwise
        """
        with self._lock:
            if name not in self._processes:
                return False
            return self._processes[name].is_running
    
    def get_process_state(self, name: str) -> Optional[ProcessState]:
        """
        Get current state of a process
        
        Args:
            name: Process identifier
        
        Returns:
            ProcessState enum value, or None if process not found
        """
        with self._lock:
            if name not in self._processes:
                return None
            
            managed = self._processes[name]
            # Update state based on actual process status
            if not managed.is_running and managed.state == ProcessState.RUNNING:
                managed.state = ProcessState.STOPPED
            
            return managed.state
    
    def get_pid(self, name: str) -> Optional[int]:
        """
        Get process ID of a running process
        
        Args:
            name: Process identifier
        
        Returns:
            Process ID if running, None otherwise
        """
        with self._lock:
            if name not in self._processes:
                return None
            return self._processes[name].pid
    
    def list_processes(self) -> Dict[str, ProcessState]:
        """
        Get dictionary of all managed processes and their states
        
        Returns:
            Dictionary mapping process names to their current states
        """
        with self._lock:
            return {
                name: self.get_process_state(name) 
                for name in self._processes.keys()
            }
    
    def cleanup_all(self, timeout: float = 5.0) -> None:
        """
        Stop all running processes (typically called on shutdown)
        
        Args:
            timeout: Time to wait for each process to terminate
        """
        with self._lock:
            process_names = list(self._processes.keys())
            
            if not process_names:
                self.logger.info("No processes to clean up")
                return
            
            self.logger.info(f"Cleaning up {len(process_names)} process(es)")
            
            for name in process_names:
                if self.is_running(name):
                    self.stop_process(name, timeout=timeout)
            
            self.logger.info("Process cleanup complete")
    
    def restart_process(
        self, 
        name: str, 
        new_command: Optional[List[str]] = None,
        timeout: float = 5.0
    ) -> bool:
        """
        Restart a process (stop if running, then start again)
        
        Args:
            name: Process identifier
            new_command: New command to use (if None, uses original command)
            timeout: Time to wait for process to stop
        
        Returns:
            True if restart successful, False otherwise
        """
        with self._lock:
            # Get original command if not provided
            if new_command is None:
                if name not in self._processes:
                    self.logger.error(f"Cannot restart '{name}': Process not found and no command provided")
                    return False
                new_command = self._processes[name].command
            
            # Stop if running
            if self.is_running(name):
                if not self.stop_process(name, timeout=timeout):
                    self.logger.error(f"Failed to stop process '{name}' for restart")
                    return False
            
            # Start with new command
            return self.start_process(name, new_command)
    
    def __del__(self):
        """Cleanup on object destruction"""
        try:
            self.cleanup_all(timeout=2.0)
        except Exception:
            pass  # Suppress errors during cleanup
