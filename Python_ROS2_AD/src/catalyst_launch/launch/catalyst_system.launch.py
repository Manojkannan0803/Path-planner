#!/usr/bin/env python3
"""
CATALYST System Launch File - TwinSim-inspired Complete System Launch

This launch file starts the complete CATALYST system with all layers:
1. Platform Layer (catalyst_core)
2. Interface Layer (catalyst_interfaces) 
3. Algorithm Layer (catalyst_algorithms)
4. Model Layer (catalyst_models)

Similar to TwinSim's system startup, this orchestrates all components.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate the complete CATALYST system launch description."""
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable RViz2 visualization'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            FindPackageShare('catalyst_core').find('catalyst_core'),
            'config', 'default_config.yaml'
        ),
        description='Path to configuration file'
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    enable_visualization = LaunchConfiguration('enable_visualization')
    config_file = LaunchConfiguration('config_file')
    
    # ========================================================================
    # PLATFORM LAYER (TwinSim Platform equivalent)
    # ========================================================================
    
    platform_manager_node = Node(
        package='catalyst_core',
        executable='platform_manager',
        name='catalyst_platform_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'config_file': config_file}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0
    )
    
    plugin_registry_node = Node(
        package='catalyst_core', 
        executable='plugin_registry',
        name='catalyst_plugin_registry',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    configuration_manager_node = Node(
        package='catalyst_core',
        executable='configuration_manager', 
        name='catalyst_configuration_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'config_file': config_file}
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # ========================================================================
    # ALGORITHM LAYER (TwinSim Plugins equivalent)
    # ========================================================================
    
    astar_planner_node = Node(
        package='catalyst_algorithms',
        executable='astar_plugin',
        name='catalyst_astar_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('enable_astar'), "' == 'true'"]))
    )
    
    cost_calculator_node = Node(
        package='catalyst_algorithms',
        executable='cost_calculators',
        name='catalyst_cost_calculator',
        output='screen', 
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    collision_detector_node = Node(
        package='catalyst_algorithms',
        executable='collision_detection',
        name='catalyst_collision_detector', 
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # ========================================================================
    # MODEL LAYER (TwinSim Models equivalent)  
    # ========================================================================
    
    environment_model_node = Node(
        package='catalyst_models',
        executable='environment_model',
        name='catalyst_environment_model',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    vehicle_model_node = Node(
        package='catalyst_models',
        executable='vehicle_model',
        name='catalyst_vehicle_model',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    motion_primitives_node = Node(
        package='catalyst_models',
        executable='motion_primitives',
        name='catalyst_motion_primitives',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}], 
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # ========================================================================
    # SENSOR LAYER (TwinSim Interceptors equivalent)
    # ========================================================================
    
    sensor_interceptor_node = Node(
        package='catalyst_sensors',
        executable='sensor_interceptors',
        name='catalyst_sensor_interceptors',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # ========================================================================
    # VISUALIZATION LAYER  
    # ========================================================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='catalyst_rviz',
        arguments=['-d', os.path.join(
            FindPackageShare('catalyst_visualization').find('catalyst_visualization'),
            'rviz', 'catalyst_system.rviz'
        )],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_visualization)
    )
    
    visualization_node = Node(
        package='catalyst_visualization',
        executable='path_visualizer',
        name='catalyst_path_visualizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(enable_visualization)
    )
    
    # ========================================================================
    # GROUP ACTIONS FOR ORGANIZED STARTUP
    # ========================================================================
    
    # Platform Layer Group (starts first)
    platform_group = GroupAction([
        PushRosNamespace('catalyst/platform'),
        platform_manager_node,
        plugin_registry_node,
        configuration_manager_node
    ])
    
    # Algorithm Layer Group  
    algorithm_group = GroupAction([
        PushRosNamespace('catalyst/algorithms'),
        astar_planner_node,
        cost_calculator_node,
        collision_detector_node
    ])
    
    # Model Layer Group
    model_group = GroupAction([
        PushRosNamespace('catalyst/models'),
        environment_model_node,
        vehicle_model_node,
        motion_primitives_node
    ])
    
    # Sensor Layer Group
    sensor_group = GroupAction([
        PushRosNamespace('catalyst/sensors'),
        sensor_interceptor_node
    ])
    
    # Visualization Group
    visualization_group = GroupAction([
        PushRosNamespace('catalyst/visualization'),
        rviz_node,
        visualization_node
    ])
    
    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        log_level_arg,
        enable_visualization_arg,
        config_file_arg,
        
        # Additional arguments for optional components
        DeclareLaunchArgument('enable_astar', default_value='true', 
                            description='Enable A* path planner'),
        DeclareLaunchArgument('enable_rrt', default_value='false',
                            description='Enable RRT* path planner'),
        
        # System startup sequence (TwinSim-style layered startup)
        platform_group,      # Start platform layer first
        algorithm_group,     # Then algorithm plugins
        model_group,         # Then simulation models
        sensor_group,        # Then sensor interceptors
        visualization_group  # Finally visualization
    ])
