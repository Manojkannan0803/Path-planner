#!/usr/bin/env python3
"""
CATALYST Configuration Manager - XX (work)-inspired Configuration System

This class manages system and plugin configurations, similar to XX (work)'s
configuration management. It provides:
- YAML-based configuration loading and saving
- Runtime configuration updates
- Plugin-specific configuration management
- Environment-specific configurations (dev, test, production)
"""

import yaml
import os
from typing import Dict, Any, Optional
import threading
from catalyst_interfaces.msg import AlgorithmConfig


class ConfigurationManager:
    """
    Configuration manager for CATALYST system.
    
    Handles all configuration aspects similar to XX (work)'s configuration system:
    - System-wide configuration
    - Plugin-specific configurations  
    - Runtime configuration updates
    - Environment-specific settings
    """
    
    def __init__(self):
        self.system_config: Dict[str, Any] = {}
        self.plugin_configs: Dict[str, Dict[str, Any]] = {}
        self.config_file_path: Optional[str] = None
        self._lock = threading.Lock()
        
        # Default configuration
        self._load_default_config()
    
    def _load_default_config(self):
        """Load default system configuration."""
        self.system_config = {
            'platform': {
                'name': 'CATALYST',
                'version': '1.0.0',
                'log_level': 'INFO',
                'max_plugins': 50,
                'monitoring_interval': 1.0
            },
            'simulation': {
                'real_time': True,
                'time_step': 0.1,
                'max_simulation_time': 3600.0,
                'auto_start': False
            },
            'algorithms': {
                'default_planner': 'astar',
                'fallback_planner': 'rrt_star',
                'planning_timeout': 30.0,
                'replanning_threshold': 5.0
            },
            'environment': {
                'distribution_center': {
                    'length': 328.0,  # From DPDscenario.m
                    'width': 200.0,   # From DPDscenario.m
                    'scale': 1.0      # From DPDscenario.m
                }
            },
            'vehicle': {
                'articulated_vehicle': {
                    'trailer_wheelbase': 8.475,    # L_1f from staticobs_check.m
                    'tractor_wheelbase': 3.8,      # L_0f from staticobs_check.m
                    'kingpin_distance': 0.3,       # L_0b from staticobs_check.m
                    'trailer_overhang_rear': 5.0,  # oh_1b from staticobs_check.m
                    'trailer_overhang_front': 9.475, # oh_1f from staticobs_check.m
                    'trailer_width': 2.5,          # w_1 from staticobs_check.m
                    'tractor_overhang_front': 1.5, # oh_0f from staticobs_check.m
                    'tractor_overhang_rear': 0.94  # oh_0b from staticobs_check.m
                }
            },
            'path_planning': {
                'astar': {
                    'heuristic_weight': 2.5,       # From h_cost.m
                    'penalty_factor': 1000.0,      # From h_cost.m
                    'use_rectangular_heuristic': True,
                    'max_iterations': 10000,
                    'goal_tolerance': 1.0,
                    'angle_tolerance': 0.1
                }
            }
        }
    
    def load_configuration(self, config_file: str) -> bool:
        """
        Load configuration from YAML file.
        
        Args:
            config_file: Path to YAML configuration file
            
        Returns:
            bool: True if loaded successfully
        """
        try:
            with self._lock:
                if not os.path.exists(config_file):
                    print(f"Configuration file not found: {config_file}")
                    return False
                
                with open(config_file, 'r') as file:
                    loaded_config = yaml.safe_load(file)
                
                # Merge with existing configuration
                self._deep_merge(self.system_config, loaded_config)
                self.config_file_path = config_file
                
                print(f"Configuration loaded from: {config_file}")
                return True
                
        except Exception as e:
            print(f"Failed to load configuration: {str(e)}")
            return False
    
    def save_configuration(self, config_file: Optional[str] = None) -> bool:
        """
        Save current configuration to YAML file.
        
        Args:
            config_file: Path to save configuration (uses loaded path if None)
            
        Returns:
            bool: True if saved successfully
        """
        try:
            with self._lock:
                save_path = config_file or self.config_file_path
                if not save_path:
                    print("No configuration file path specified")
                    return False
                
                # Ensure directory exists
                os.makedirs(os.path.dirname(save_path), exist_ok=True)
                
                with open(save_path, 'w') as file:
                    yaml.dump(self.system_config, file, default_flow_style=False, indent=2)
                
                print(f"Configuration saved to: {save_path}")
                return True
                
        except Exception as e:
            print(f"Failed to save configuration: {str(e)}")
            return False
    
    def get_system_config(self, key_path: str = "") -> Any:
        """
        Get system configuration value by key path.
        
        Args:
            key_path: Dot-separated key path (e.g., "platform.name")
            
        Returns:
            Configuration value or None if not found
        """
        with self._lock:
            if not key_path:
                return self.system_config.copy()
            
            return self._get_nested_value(self.system_config, key_path)
    
    def set_system_config(self, key_path: str, value: Any) -> bool:
        """
        Set system configuration value by key path.
        
        Args:
            key_path: Dot-separated key path (e.g., "platform.name")
            value: Value to set
            
        Returns:
            bool: True if set successfully
        """
        try:
            with self._lock:
                self._set_nested_value(self.system_config, key_path, value)
                return True
        except Exception as e:
            print(f"Failed to set configuration: {str(e)}")
            return False
    
    def get_algorithm_config(self, algorithm_name: str) -> AlgorithmConfig:
        """
        Get algorithm configuration as ROS2 message.
        
        Args:
            algorithm_name: Name of the algorithm
            
        Returns:
            AlgorithmConfig message
        """
        config_msg = AlgorithmConfig()
        config_msg.algorithm_name = algorithm_name
        config_msg.plugin_id = f"{algorithm_name}_plugin"
        
        # Get algorithm-specific configuration
        algo_config = self.get_system_config(f"path_planning.{algorithm_name}")
        
        if algo_config and algorithm_name == "astar":
            config_msg.heuristic_weight = algo_config.get('heuristic_weight', 2.5)
            config_msg.penalty_factor = algo_config.get('penalty_factor', 1000.0)
            config_msg.use_rectangular_heuristic = algo_config.get('use_rectangular_heuristic', True)
            config_msg.max_planning_time = self.get_system_config('algorithms.planning_timeout') or 30.0
            config_msg.max_iterations = algo_config.get('max_iterations', 10000)
            config_msg.goal_tolerance = algo_config.get('goal_tolerance', 1.0)
            config_msg.angle_tolerance = algo_config.get('angle_tolerance', 0.1)
        
        # Get vehicle parameters
        vehicle_config = self.get_system_config('vehicle.articulated_vehicle')
        if vehicle_config:
            # Convert to parameter arrays for flexibility
            param_names = []
            param_values = []
            for key, value in vehicle_config.items():
                param_names.append(f"vehicle.{key}")
                param_values.append(str(value))
            
            config_msg.parameter_names = param_names
            config_msg.parameter_values = param_values
        
        config_msg.stamp = self._get_current_time()
        return config_msg
    
    def get_plugin_config(self, plugin_id: str) -> Dict[str, Any]:
        """
        Get plugin-specific configuration.
        
        Args:
            plugin_id: Plugin identifier
            
        Returns:
            Plugin configuration dictionary
        """
        return self.plugin_configs.get(plugin_id, {})
    
    def set_plugin_config(self, plugin_id: str, config: Dict[str, Any]):
        """
        Set plugin-specific configuration.
        
        Args:
            plugin_id: Plugin identifier
            config: Plugin configuration dictionary
        """
        with self._lock:
            self.plugin_configs[plugin_id] = config.copy()
    
    def update_config_runtime(self, config_updates: Dict[str, Any]) -> bool:
        """
        Update configuration at runtime.
        
        Args:
            config_updates: Dictionary of configuration updates
            
        Returns:
            bool: True if updated successfully
        """
        try:
            with self._lock:
                self._deep_merge(self.system_config, config_updates)
                return True
        except Exception as e:
            print(f"Failed to update configuration: {str(e)}")
            return False
    
    def _deep_merge(self, target: Dict[str, Any], source: Dict[str, Any]):
        """Deep merge source dictionary into target."""
        for key, value in source.items():
            if key in target and isinstance(target[key], dict) and isinstance(value, dict):
                self._deep_merge(target[key], value)
            else:
                target[key] = value
    
    def _get_nested_value(self, config: Dict[str, Any], key_path: str) -> Any:
        """Get nested configuration value by dot-separated key path."""
        keys = key_path.split('.')
        value = config
        
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return None
        
        return value
    
    def _set_nested_value(self, config: Dict[str, Any], key_path: str, value: Any):
        """Set nested configuration value by dot-separated key path."""
        keys = key_path.split('.')
        current = config
        
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        
        current[keys[-1]] = value
    
    def _get_current_time(self):
        """Get current time for timestamps."""
        import time
        current_time = time.time()
        seconds = int(current_time)
        nanoseconds = int((current_time - seconds) * 1e9)
        
        # Create a simple time structure
        class TimeStruct:
            def __init__(self, sec, nanosec):
                self.sec = sec
                self.nanosec = nanosec
        
        return TimeStruct(seconds, nanoseconds)
    
    def get_environment_config(self) -> Dict[str, Any]:
        """Get environment-specific configuration."""
        return self.get_system_config('environment') or {}
    
    def get_vehicle_config(self) -> Dict[str, Any]:
        """Get vehicle-specific configuration.""" 
        return self.get_system_config('vehicle') or {}
    
    def validate_configuration(self) -> tuple[bool, List[str]]:
        """
        Validate current configuration.
        
        Returns:
            tuple: (is_valid, list_of_errors)
        """
        errors = []
        
        # Validate required sections
        required_sections = ['platform', 'simulation', 'algorithms', 'environment', 'vehicle']
        for section in required_sections:
            if section not in self.system_config:
                errors.append(f"Missing required configuration section: {section}")
        
        # Validate specific values
        if self.get_system_config('platform.max_plugins') and self.get_system_config('platform.max_plugins') <= 0:
            errors.append("platform.max_plugins must be positive")
        
        if self.get_system_config('simulation.time_step') and self.get_system_config('simulation.time_step') <= 0:
            errors.append("simulation.time_step must be positive")
        
        # Validate vehicle parameters
        vehicle_config = self.get_vehicle_config()
        if 'articulated_vehicle' in vehicle_config:
            av_config = vehicle_config['articulated_vehicle']
            for param in ['trailer_wheelbase', 'tractor_wheelbase', 'trailer_width']:
                if param not in av_config or av_config[param] <= 0:
                    errors.append(f"vehicle.articulated_vehicle.{param} must be positive")
        
        return len(errors) == 0, errors


def main(args=None):
    """Main entry point for standalone configuration manager testing."""
    config_manager = ConfigurationManager()
    
    # Test configuration
    print("System configuration:")
    print(yaml.dump(config_manager.get_system_config(), default_flow_style=False, indent=2))
    
    # Test algorithm configuration
    astar_config = config_manager.get_algorithm_config("astar")
    print(f"A* configuration: {astar_config}")
    
    # Validate configuration
    is_valid, errors = config_manager.validate_configuration()
    print(f"Configuration valid: {is_valid}")
    if errors:
        print("Errors:", errors)


if __name__ == '__main__':
    main()
