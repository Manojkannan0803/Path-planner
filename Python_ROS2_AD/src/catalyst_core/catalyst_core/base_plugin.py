#!/usr/bin/env python3
"""
CATALYST Base Plugin Class - TwinSim-inspired Plugin Architecture

This abstract base class defines the interface that all CATALYST plugins must implement.
Similar to TwinSim's plugin structure, this provides standardized initialization,
execution, and cleanup methods.

Architecture:
- All plugins inherit from CatalystPlugin
- Supports both synchronous and asynchronous execution
- Provides performance monitoring and error handling
- Enables hot-swapping of algorithm components
"""

from abc import ABC, abstractmethod
import time
import threading
from typing import Any, Dict, Optional
import rclpy
from rclpy.node import Node
from catalyst_interfaces.msg import PluginStatus, AlgorithmConfig


class CatalystPlugin(ABC, Node):
    """
    Abstract base class for all CATALYST plugins.
    
    Inspired by TwinSim plugin architecture, this provides:
    - Standardized plugin lifecycle management
    - Performance monitoring capabilities  
    - Configuration management
    - Error handling and logging
    """
    
    def __init__(self, plugin_name: str, plugin_type: str):
        """Initialize the plugin with name and type."""
        super().__init__(plugin_name + '_node')
        
        self.plugin_id = plugin_name
        self.plugin_type = plugin_type
        self.status = PluginStatus.STATUS_UNLOADED
        self.config: Optional[AlgorithmConfig] = None
        
        # Performance monitoring
        self.execution_times = []
        self.total_executions = 0
        self.successful_executions = 0
        self.failed_executions = 0
        
        # Threading support
        self._lock = threading.Lock()
        self._shutdown_requested = False
        
        # Status publisher (TwinSim-style monitoring)
        self.status_publisher = self.create_publisher(
            PluginStatus, f'/catalyst/plugin_status/{self.plugin_id}', 10)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self._publish_status)
        
        self.get_logger().info(f"CATALYST Plugin '{self.plugin_id}' initialized")
    
    @abstractmethod
    def initialize(self, config: AlgorithmConfig) -> bool:
        """
        Initialize the plugin with configuration.
        
        Args:
            config: Plugin-specific configuration
            
        Returns:
            bool: True if initialization successful, False otherwise
        """
        pass
    
    @abstractmethod
    def execute(self, input_data: Any) -> Any:
        """
        Execute the plugin's main functionality.
        
        Args:
            input_data: Input data for processing
            
        Returns:
            Any: Processed output data
        """
        pass
    
    @abstractmethod
    def cleanup(self) -> None:
        """Clean up plugin resources."""
        pass
    
    def safe_execute(self, input_data: Any) -> tuple[Any, bool]:
        """
        Execute plugin with error handling and performance monitoring.
        
        Args:
            input_data: Input data for processing
            
        Returns:
            tuple: (result, success_flag)
        """
        start_time = time.time()
        
        try:
            with self._lock:
                self.status = PluginStatus.STATUS_RUNNING
                result = self.execute(input_data)
                
            execution_time = time.time() - start_time
            self._update_performance_metrics(execution_time, True)
            
            self.status = PluginStatus.STATUS_READY
            return result, True
            
        except Exception as e:
            execution_time = time.time() - start_time
            self._update_performance_metrics(execution_time, False)
            
            self.get_logger().error(f"Plugin '{self.plugin_id}' execution failed: {str(e)}")
            self.status = PluginStatus.STATUS_ERROR
            return None, False
    
    def load_plugin(self, config: AlgorithmConfig) -> bool:
        """
        Load and initialize the plugin.
        
        Args:
            config: Plugin configuration
            
        Returns:
            bool: True if loaded successfully
        """
        try:
            self.status = PluginStatus.STATUS_LOADING
            
            if self.initialize(config):
                self.config = config
                self.status = PluginStatus.STATUS_READY
                self.get_logger().info(f"Plugin '{self.plugin_id}' loaded successfully")
                return True
            else:
                self.status = PluginStatus.STATUS_ERROR
                self.get_logger().error(f"Plugin '{self.plugin_id}' initialization failed")
                return False
                
        except Exception as e:
            self.status = PluginStatus.STATUS_ERROR
            self.get_logger().error(f"Plugin '{self.plugin_id}' loading failed: {str(e)}")
            return False
    
    def unload_plugin(self) -> bool:
        """
        Unload the plugin and clean up resources.
        
        Returns:
            bool: True if unloaded successfully
        """
        try:
            self.status = PluginStatus.STATUS_STOPPING
            self.cleanup()
            self.status = PluginStatus.STATUS_UNLOADED
            self.get_logger().info(f"Plugin '{self.plugin_id}' unloaded successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Plugin '{self.plugin_id}' unloading failed: {str(e)}")
            return False
    
    def _update_performance_metrics(self, execution_time: float, success: bool):
        """Update internal performance metrics."""
        with self._lock:
            self.execution_times.append(execution_time)
            # Keep only last 100 execution times
            if len(self.execution_times) > 100:
                self.execution_times.pop(0)
                
            self.total_executions += 1
            if success:
                self.successful_executions += 1
            else:
                self.failed_executions += 1
    
    def _publish_status(self):
        """Publish plugin status (TwinSim-style monitoring)."""
        status_msg = PluginStatus()
        status_msg.plugin_id = self.plugin_id
        status_msg.plugin_name = self.plugin_id
        status_msg.plugin_type = self.plugin_type
        status_msg.status = self.status
        
        with self._lock:
            status_msg.total_executions = self.total_executions
            status_msg.successful_executions = self.successful_executions
            status_msg.failed_executions = self.failed_executions
            
            if self.execution_times:
                status_msg.execution_time_avg = sum(self.execution_times) / len(self.execution_times) * 1000  # ms
                status_msg.execution_time_max = max(self.execution_times) * 1000  # ms
        
        status_msg.stamp = self.get_clock().now().to_msg()
        self.status_publisher.publish(status_msg)
    
    def get_performance_metrics(self) -> Dict[str, float]:
        """Get current performance metrics."""
        with self._lock:
            if not self.execution_times:
                return {
                    'avg_execution_time': 0.0,
                    'max_execution_time': 0.0,
                    'success_rate': 0.0,
                    'total_executions': self.total_executions
                }
            
            return {
                'avg_execution_time': sum(self.execution_times) / len(self.execution_times),
                'max_execution_time': max(self.execution_times),
                'success_rate': self.successful_executions / self.total_executions if self.total_executions > 0 else 0.0,
                'total_executions': self.total_executions
            }
