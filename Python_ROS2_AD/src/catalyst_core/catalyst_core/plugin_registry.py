#!/usr/bin/env python3
"""
CATALYST Plugin Registry - XX (work)-inspired Plugin Management

This class manages plugin lifecycle and registration, similar to XX (work)'s
plugin management system. It provides:
- Dynamic plugin loading and unloading
- Plugin discovery and registration  
- Plugin health monitoring
- Inter-plugin communication coordination
"""

import importlib
import sys
from typing import Dict, Any, Optional, List
import threading
from catalyst_interfaces.msg import AlgorithmConfig, PluginStatus
from .base_plugin import CatalystPlugin


class PluginRegistry:
    """
    Plugin registry for managing CATALYST plugins.
    
    Similar to XX (work)'s plugin management, this handles:
    - Plugin discovery and loading
    - Plugin lifecycle management
    - Plugin health monitoring
    - Plugin communication coordination
    """
    
    def __init__(self):
        self.plugins: Dict[str, CatalystPlugin] = {}
        self.plugin_configs: Dict[str, AlgorithmConfig] = {}
        self.plugin_types = {
            'algorithm': 'catalyst_algorithms',
            'model': 'catalyst_models', 
            'sensor': 'catalyst_sensors',
            'visualization': 'catalyst_visualization'
        }
        self._lock = threading.Lock()
    
    def discover_plugins(self, plugin_type: str) -> List[str]:
        """
        Discover available plugins of a given type.
        
        Args:
            plugin_type: Type of plugins to discover (algorithm, model, sensor, etc.)
            
        Returns:
            List of available plugin names
        """
        available_plugins = []
        
        try:
            package_name = self.plugin_types.get(plugin_type)
            if not package_name:
                return available_plugins
            
            # Try to import the package and discover plugins
            package = importlib.import_module(package_name)
            
            # Look for plugin classes
            for attr_name in dir(package):
                attr = getattr(package, attr_name)
                if (isinstance(attr, type) and 
                    issubclass(attr, CatalystPlugin) and 
                    attr != CatalystPlugin):
                    available_plugins.append(attr_name)
                    
        except ImportError:
            pass  # Package not available
        except Exception as e:
            print(f"Error discovering plugins: {e}")
        
        return available_plugins
    
    def load_plugin(self, plugin_name: str, plugin_type: str, 
                   config: AlgorithmConfig) -> bool:
        """
        Load a plugin with given configuration.
        
        Args:
            plugin_name: Name/ID of the plugin to load
            plugin_type: Type of plugin (algorithm, model, etc.)
            config: Plugin configuration
            
        Returns:
            bool: True if loaded successfully
        """
        with self._lock:
            try:
                # Check if plugin already loaded
                if plugin_name in self.plugins:
                    self.unload_plugin(plugin_name)
                
                # Get package name for plugin type
                package_name = self.plugin_types.get(plugin_type)
                if not package_name:
                    print(f"Unknown plugin type: {plugin_type}")
                    return False
                
                # Import the plugin module
                module_name = f"{package_name}.{plugin_name.lower()}_plugin"
                try:
                    plugin_module = importlib.import_module(module_name)
                except ImportError:
                    # Try alternative naming convention
                    module_name = f"{package_name}.{plugin_name.lower()}"
                    plugin_module = importlib.import_module(module_name)
                
                # Find the plugin class
                plugin_class = None
                for attr_name in dir(plugin_module):
                    attr = getattr(plugin_module, attr_name)
                    if (isinstance(attr, type) and 
                        issubclass(attr, CatalystPlugin) and 
                        attr != CatalystPlugin):
                        plugin_class = attr
                        break
                
                if not plugin_class:
                    print(f"No plugin class found in module {module_name}")
                    return False
                
                # Instantiate and initialize the plugin
                plugin_instance = plugin_class(plugin_name, plugin_type)
                
                if plugin_instance.load_plugin(config):
                    self.plugins[plugin_name] = plugin_instance
                    self.plugin_configs[plugin_name] = config
                    print(f"Plugin '{plugin_name}' loaded successfully")
                    return True
                else:
                    print(f"Plugin '{plugin_name}' initialization failed")
                    return False
                    
            except Exception as e:
                print(f"Failed to load plugin '{plugin_name}': {str(e)}")
                return False
    
    def unload_plugin(self, plugin_name: str) -> bool:
        """
        Unload a plugin and cleanup its resources.
        
        Args:
            plugin_name: Name/ID of the plugin to unload
            
        Returns:
            bool: True if unloaded successfully
        """
        with self._lock:
            try:
                if plugin_name not in self.plugins:
                    print(f"Plugin '{plugin_name}' not found")
                    return False
                
                plugin = self.plugins[plugin_name]
                
                if plugin.unload_plugin():
                    del self.plugins[plugin_name]
                    if plugin_name in self.plugin_configs:
                        del self.plugin_configs[plugin_name]
                    
                    # Cleanup the plugin node
                    plugin.destroy_node()
                    
                    print(f"Plugin '{plugin_name}' unloaded successfully")
                    return True
                else:
                    print(f"Failed to unload plugin '{plugin_name}'")
                    return False
                    
            except Exception as e:
                print(f"Error unloading plugin '{plugin_name}': {str(e)}")
                return False
    
    def get_plugin(self, plugin_name: str) -> Optional[CatalystPlugin]:
        """
        Get a loaded plugin instance.
        
        Args:
            plugin_name: Name/ID of the plugin
            
        Returns:
            Plugin instance or None if not found
        """
        return self.plugins.get(plugin_name)
    
    def list_plugins(self) -> Dict[str, str]:
        """
        List all loaded plugins.
        
        Returns:
            Dictionary mapping plugin names to their types
        """
        return {name: plugin.plugin_type for name, plugin in self.plugins.items()}
    
    def get_plugin_status(self, plugin_name: str) -> Optional[int]:
        """
        Get the current status of a plugin.
        
        Args:
            plugin_name: Name/ID of the plugin
            
        Returns:
            Plugin status code or None if not found
        """
        plugin = self.plugins.get(plugin_name)
        return plugin.status if plugin else None
    
    def get_all_plugin_metrics(self) -> Dict[str, Dict[str, float]]:
        """
        Get performance metrics for all loaded plugins.
        
        Returns:
            Dictionary mapping plugin names to their performance metrics
        """
        metrics = {}
        for name, plugin in self.plugins.items():
            if hasattr(plugin, 'get_performance_metrics'):
                metrics[name] = plugin.get_performance_metrics()
        return metrics
    
    def reload_plugin(self, plugin_name: str) -> bool:
        """
        Reload a plugin with its existing configuration.
        
        Args:
            plugin_name: Name/ID of the plugin to reload
            
        Returns:
            bool: True if reloaded successfully
        """
        if plugin_name not in self.plugins:
            return False
        
        # Get existing config
        config = self.plugin_configs.get(plugin_name)
        plugin_type = self.plugins[plugin_name].plugin_type
        
        # Unload and reload
        if self.unload_plugin(plugin_name) and config:
            return self.load_plugin(plugin_name, plugin_type, config)
        
        return False
    
    def unload_all_plugins(self):
        """Unload all plugins and cleanup resources."""
        plugin_names = list(self.plugins.keys())
        for plugin_name in plugin_names:
            self.unload_plugin(plugin_name)
    
    def is_plugin_healthy(self, plugin_name: str) -> bool:
        """
        Check if a plugin is healthy and responsive.
        
        Args:
            plugin_name: Name/ID of the plugin
            
        Returns:
            bool: True if plugin is healthy
        """
        plugin = self.plugins.get(plugin_name)
        if not plugin:
            return False
        
        # Check plugin status
        return plugin.status in [PluginStatus.STATUS_READY, PluginStatus.STATUS_RUNNING]


def main(args=None):
    """Main entry point for standalone plugin registry testing."""
    registry = PluginRegistry()
    
    # Discover available plugins
    for plugin_type in registry.plugin_types.keys():
        plugins = registry.discover_plugins(plugin_type)
        print(f"Available {plugin_type} plugins: {plugins}")


if __name__ == '__main__':
    main()
