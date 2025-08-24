#!/usr/bin/env python3
"""
CATALYST Platform Manager - TwinSim-inspired Platform Layer

This is the main orchestrator class, equivalent to TwinSim's Platform component.
It manages the entire simulation platform including:
- Plugin lifecycle management
- Configuration management  
- Simulation execution orchestration
- Performance monitoring and logging

Architecture follows TwinSim's layered design:
Platform Layer -> Interface Layer -> Algorithm Layer -> Model Layer
"""

import rclpy
from rclpy.node import Node
import time
import threading
from typing import Dict, Any, Optional, List
import yaml
import os

from catalyst_interfaces.msg import (
    VehicleState, ObstacleMap, PathPlan, AlgorithmConfig, PluginStatus
)
from catalyst_interfaces.srv import PlanPath, LoadPlugin
from .plugin_registry import PluginRegistry
from .configuration_manager import ConfigurationManager


class PlatformManager(Node):
    """
    Main CATALYST Platform Manager - TwinSim Platform equivalent.
    
    Responsibilities:
    1. Plugin lifecycle management (load, execute, unload)
    2. Simulation orchestration and coordination
    3. Performance monitoring and analysis
    4. Configuration management
    5. Inter-plugin communication
    """
    
    def __init__(self):
        super().__init__('catalyst_platform_manager')
        
        # Core components (TwinSim architecture)
        self.plugin_registry = PluginRegistry()
        self.config_manager = ConfigurationManager()
        
        # State management
        self.current_vehicle_state: Optional[VehicleState] = None
        self.current_obstacle_map: Optional[ObstacleMap] = None
        self.active_plugins: Dict[str, Any] = {}
        
        # Performance monitoring
        self.execution_metrics = {}
        self.simulation_active = False
        
        # ROS2 Services (TwinSim-style external interface)
        self.plan_path_service = self.create_service(
            PlanPath, '/catalyst/plan_path', self.plan_path_callback)
        
        self.load_plugin_service = self.create_service(
            LoadPlugin, '/catalyst/load_plugin', self.load_plugin_callback)
        
        # ROS2 Subscriptions (sensor data interceptors)
        self.vehicle_state_sub = self.create_subscription(
            VehicleState, '/catalyst/vehicle_state', self.vehicle_state_callback, 10)
        
        self.obstacle_map_sub = self.create_subscription(
            ObstacleMap, '/catalyst/obstacle_map', self.obstacle_map_callback, 10)
        
        # ROS2 Publishers (simulation output)
        self.path_plan_pub = self.create_publisher(
            PathPlan, '/catalyst/path_plan', 10)
        
        self.platform_status_pub = self.create_publisher(
            PluginStatus, '/catalyst/platform_status', 10)
        
        # Monitoring timer
        self.status_timer = self.create_timer(2.0, self.publish_platform_status)
        
        # Initialize platform
        self._initialize_platform()
        
        self.get_logger().info("CATALYST Platform Manager initialized")
    
    def _initialize_platform(self):
        """Initialize the platform with default configuration."""
        try:
            # Load default configuration
            config_path = self.get_package_share_directory('catalyst_core') + '/config/default_config.yaml'
            if os.path.exists(config_path):
                self.config_manager.load_configuration(config_path)
            
            # Load default plugins
            self._load_default_plugins()
            
            self.get_logger().info("Platform initialization completed")
            
        except Exception as e:
            self.get_logger().error(f"Platform initialization failed: {str(e)}")
    
    def _load_default_plugins(self):
        """Load default plugins for basic operation."""
        try:
            # Load A* path planning plugin (converted from MATLAB)
            astar_config = AlgorithmConfig()
            astar_config.algorithm_name = "astar"
            astar_config.plugin_id = "astar_pathplanner"
            astar_config.heuristic_weight = 2.5  # From h_cost.m
            astar_config.penalty_factor = 1000.0  # From h_cost.m
            astar_config.max_planning_time = 30.0
            astar_config.max_iterations = 10000
            
            self.plugin_registry.load_plugin("astar_pathplanner", "algorithm", astar_config)
            
            # Load environment model (converted from DPDscenario.m)
            env_config = AlgorithmConfig()
            env_config.algorithm_name = "distribution_center_model"
            env_config.plugin_id = "dc_environment"
            
            self.plugin_registry.load_plugin("dc_environment", "model", env_config)
            
            self.get_logger().info("Default plugins loaded successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load default plugins: {str(e)}")
    
    def plan_path_callback(self, request: PlanPath.Request, response: PlanPath.Response):
        """
        Handle path planning service request.
        
        This orchestrates the entire planning process similar to TwinSim's
        simulation execution model.
        """
        start_time = time.time()
        
        try:
            self.get_logger().info("Path planning request received")
            
            # Get active path planning plugin
            planner_plugin = self.plugin_registry.get_plugin("astar_pathplanner")
            
            if not planner_plugin:
                response.path_plan.success = False
                response.path_plan.error_message = "No path planning plugin available"
                return response
            
            # Prepare input data
            planning_input = {
                'start_state': request.start_state,
                'goal_state': request.goal_state,
                'obstacle_map': request.obstacle_map,
                'config': request.config
            }
            
            # Execute path planning (TwinSim-style plugin execution)
            result, success = planner_plugin.safe_execute(planning_input)
            
            if success and result:
                response.path_plan = result
                response.path_plan.planning_time = time.time() - start_time
                response.path_plan.success = True
                
                # Publish result for other systems
                self.path_plan_pub.publish(response.path_plan)
                
                self.get_logger().info(
                    f"Path planning completed successfully in {response.path_plan.planning_time:.3f}s")
            else:
                response.path_plan.success = False
                response.path_plan.error_message = "Path planning execution failed"
                response.path_plan.planning_time = time.time() - start_time
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Path planning service error: {str(e)}")
            response.path_plan.success = False
            response.path_plan.error_message = str(e)
            response.path_plan.planning_time = time.time() - start_time
            return response
    
    def load_plugin_callback(self, request: LoadPlugin.Request, response: LoadPlugin.Response):
        """Handle plugin loading service request."""
        try:
            success = self.plugin_registry.load_plugin(
                request.plugin_name, request.plugin_type, request.config)
            
            response.success = success
            if success:
                response.message = f"Plugin '{request.plugin_name}' loaded successfully"
                plugin = self.plugin_registry.get_plugin(request.plugin_name)
                if plugin and hasattr(plugin, '_publish_status'):
                    # Get plugin status
                    pass  # Status will be published by plugin itself
            else:
                response.message = f"Failed to load plugin '{request.plugin_name}'"
            
            return response
            
        except Exception as e:
            response.success = False
            response.message = f"Plugin loading error: {str(e)}"
            return response
    
    def vehicle_state_callback(self, msg: VehicleState):
        """Handle vehicle state updates from sensors."""
        self.current_vehicle_state = msg
        
        # Trigger any state-dependent processing
        self._update_simulation_state()
    
    def obstacle_map_callback(self, msg: ObstacleMap):
        """Handle obstacle map updates from sensors."""
        self.current_obstacle_map = msg
        
        # Update environment model
        env_plugin = self.plugin_registry.get_plugin("dc_environment")
        if env_plugin:
            env_plugin.safe_execute({'obstacle_map': msg})
    
    def _update_simulation_state(self):
        """Update overall simulation state based on new sensor data."""
        if self.current_vehicle_state and self.current_obstacle_map:
            # Simulation state is complete, could trigger autonomous planning
            pass
    
    def execute_simulation_cycle(self):
        """
        Execute one complete simulation cycle.
        
        This is the main orchestration method similar to TwinSim's
        simulation execution engine.
        """
        try:
            if not self.simulation_active:
                return
            
            # 1. Data Collection Phase (Interceptor Layer)
            # Sensor data is collected via ROS2 subscriptions
            
            # 2. Environment Update Phase (Model Layer)
            env_plugin = self.plugin_registry.get_plugin("dc_environment")
            if env_plugin and self.current_obstacle_map:
                env_result, _ = env_plugin.safe_execute(self.current_obstacle_map)
            
            # 3. Path Planning Phase (Algorithm Layer)
            if (self.current_vehicle_state and self.current_obstacle_map and 
                hasattr(self, 'goal_state')):
                
                planner_plugin = self.plugin_registry.get_plugin("astar_pathplanner")
                if planner_plugin:
                    planning_input = {
                        'start_state': self.current_vehicle_state,
                        'goal_state': self.goal_state,
                        'obstacle_map': self.current_obstacle_map
                    }
                    
                    path_result, success = planner_plugin.safe_execute(planning_input)
                    
                    if success:
                        self.path_plan_pub.publish(path_result)
            
            # 4. Performance Monitoring (Platform Layer)
            self._update_performance_metrics()
            
        except Exception as e:
            self.get_logger().error(f"Simulation cycle error: {str(e)}")
    
    def start_simulation(self):
        """Start continuous simulation execution."""
        self.simulation_active = True
        self.get_logger().info("Simulation started")
    
    def stop_simulation(self):
        """Stop simulation execution."""
        self.simulation_active = False
        self.get_logger().info("Simulation stopped")
    
    def _update_performance_metrics(self):
        """Update platform performance metrics."""
        # Collect metrics from all active plugins
        for plugin_id, plugin in self.active_plugins.items():
            if hasattr(plugin, 'get_performance_metrics'):
                self.execution_metrics[plugin_id] = plugin.get_performance_metrics()
    
    def publish_platform_status(self):
        """Publish platform status for monitoring."""
        status = PluginStatus()
        status.plugin_id = "platform_manager"
        status.plugin_name = "CATALYST Platform Manager"
        status.plugin_type = "platform"
        status.status = PluginStatus.STATUS_READY if not self.simulation_active else PluginStatus.STATUS_RUNNING
        status.stamp = self.get_clock().now().to_msg()
        
        self.platform_status_pub.publish(status)
    
    def shutdown(self):
        """Shutdown platform and cleanup all resources."""
        self.get_logger().info("Shutting down CATALYST Platform Manager")
        
        # Stop simulation
        self.stop_simulation()
        
        # Unload all plugins
        self.plugin_registry.unload_all_plugins()
        
        # Cleanup resources
        self.destroy_node()


def main(args=None):
    """Main entry point for the platform manager."""
    rclpy.init(args=args)
    
    try:
        platform_manager = PlatformManager()
        
        # Start the platform
        platform_manager.get_logger().info("CATALYST Platform Manager starting...")
        
        # Spin the node
        rclpy.spin(platform_manager)
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'platform_manager' in locals():
            platform_manager.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
