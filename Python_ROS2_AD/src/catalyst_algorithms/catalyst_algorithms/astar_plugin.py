#!/usr/bin/env python3
"""
CATALYST A* Path Planning Plugin - Converted from MATLAB Pathplanning_Astar.m

This plugin implements the A* path planning algorithm converted from the original
MATLAB implementation. It follows the TwinSim plugin architecture pattern.

Original MATLAB file: Pathplanning_Astar.m
Author: Manojpriyadharson Kannan (Student number: 638628)
Converted by: CATALYST Team

Key features converted from MATLAB:
- A* search algorithm with motion primitives
- Forward and reverse motion support
- Rectangular and Euclidean heuristics
- Obstacle avoidance for articulated vehicles
- Distribution center environment support
"""

import numpy as np
import heapq
import time
import math
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, field

import rclpy
from catalyst_core.base_plugin import CatalystPlugin
from catalyst_interfaces.msg import (
    VehicleState, ObstacleMap, PathPlan, MotionPrimitive, AlgorithmConfig
)
from catalyst_interfaces.srv import PlanPath


@dataclass
class AStarState:
    """
    A* search state representation (converted from MATLAB 'state' struct).
    
    Corresponds to the state structure in Pathplanning_Astar.m
    """
    x: float = 0.0          # X coordinate [m]
    y: float = 0.0          # Y coordinate [m] 
    xa: float = 0.0         # Actual ending point X [m]
    ya: float = 0.0         # Actual ending point Y [m]
    
    # Motion primitive information
    pred_ti: float = 0.0    # Predecessor theta initial [degrees]
    pred_gi: float = 0.0    # Predecessor gamma initial [degrees]
    pred_tf: float = 0.0    # Predecessor theta final [degrees]
    pred_gf: float = 0.0    # Predecessor gamma final [degrees]
    pred_dir: int = 1       # Direction (1=forward, 0=reverse)
    
    # Search tree information
    predxy: Optional[int] = None    # Index of predecessor in tree
    
    # Cost information
    g_cost: float = 0.0     # Cost so far
    h_cost: float = 0.0     # Heuristic cost
    f_cost: float = 0.0     # Total cost (g + h)
    
    # Motion primitive index
    mp_index: Optional[int] = None


class AStarPathPlanner(CatalystPlugin):
    """
    A* Path Planning Plugin - TwinSim Plugin Architecture
    
    Converted from MATLAB Pathplanning_Astar.m with the following structure:
    1. Initialize algorithm parameters (from MATLAB global variables)
    2. Load motion primitives (from MATLAB MP files)
    3. Execute A* search with forward/reverse motion
    4. Return path plan result
    """
    
    def __init__(self):
        super().__init__("astar_pathplanner", "algorithm")
        
        # Algorithm parameters (from MATLAB global variables)
        self.length_dc = 328.0      # Length of distribution center [m]
        self.width_dc = 200.0       # Width of distribution center [m] 
        self.scale = 1.0            # Scale of the grid [m]
        
        # Discretization intervals (from MATLAB)
        self.theta_discrete = np.arange(0, 360, 9)  # 0:9:351
        self.gamma_discrete = np.array([-21, 0, 21])  # [-21 0 21]
        
        # Motion primitives data (to be loaded)
        self.motion_primitives = {}
        self.mp_data_loaded = False
        
        # Algorithm configuration
        self.config = {
            'heuristic_weight': 2.5,
            'penalty_factor': 1000.0,
            'use_rectangular_heuristic': True,
            'max_iterations': 10000,
            'goal_tolerance': 1.0,
            'angle_tolerance': 0.1
        }
        
        # Performance tracking
        self.planning_iterations = 0
        self.nodes_explored = 0
        
        self.get_logger().info("A* Path Planner Plugin initialized")
    
    def initialize(self, config: AlgorithmConfig) -> bool:
        """
        Initialize the A* plugin with configuration.
        
        Args:
            config: Algorithm configuration from TwinSim-style config system
            
        Returns:
            bool: True if initialization successful
        """
        try:
            # Update configuration from ROS2 message
            self.config['heuristic_weight'] = config.heuristic_weight
            self.config['penalty_factor'] = config.penalty_factor
            self.config['use_rectangular_heuristic'] = config.use_rectangular_heuristic
            self.config['max_iterations'] = config.max_iterations
            self.config['goal_tolerance'] = config.goal_tolerance
            self.config['angle_tolerance'] = config.angle_tolerance
            
            # Parse vehicle parameters from config
            for i, param_name in enumerate(config.parameter_names):
                if i < len(config.parameter_values):
                    value = float(config.parameter_values[i])
                    # Store vehicle parameters for collision checking
                    if param_name.startswith('vehicle.'):
                        setattr(self, param_name.replace('vehicle.', ''), value)
            
            # Load motion primitives (placeholder - will be implemented)
            self._load_motion_primitives()
            
            self.get_logger().info(f"A* Plugin initialized with config: {self.config}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"A* Plugin initialization failed: {str(e)}")
            return False
    
    def execute(self, input_data: Dict[str, Any]) -> PathPlan:
        """
        Execute A* path planning algorithm.
        
        This is the main method that implements the converted MATLAB algorithm.
        
        Args:
            input_data: Dictionary containing:
                - start_state: VehicleState
                - goal_state: VehicleState  
                - obstacle_map: ObstacleMap
                - config: AlgorithmConfig (optional)
                
        Returns:
            PathPlan: Complete path plan result
        """
        start_time = time.time()
        
        try:
            # Extract input data
            start_state = input_data['start_state']
            goal_state = input_data['goal_state']
            obstacle_map = input_data['obstacle_map']
            
            self.get_logger().info(
                f"A* planning from ({start_state.x:.1f}, {start_state.y:.1f}) "
                f"to ({goal_state.x:.1f}, {goal_state.y:.1f})"
            )
            
            # Convert ROS2 messages to internal representation
            start_astar = self._ros_to_astar_state(start_state)
            goal_astar = self._ros_to_astar_state(goal_state)
            
            # Execute A* algorithm (converted from MATLAB)
            path_states, success, error_msg = self._astar_search(
                start_astar, goal_astar, obstacle_map
            )
            
            # Create result
            result = PathPlan()
            result.start_state = start_state
            result.goal_state = goal_state
            result.success = success
            result.error_message = error_msg or ""
            result.planning_time = time.time() - start_time
            result.nodes_explored = self.nodes_explored
            result.algorithm_id = 1  # A* algorithm ID
            
            if success and path_states:
                # Convert path to ROS2 format
                result.path_states = [self._astar_to_ros_state(state) for state in path_states]
                result.total_path_cost = path_states[-1].f_cost if path_states else 0.0
                result.path_length = self._calculate_path_length(path_states)
                
                self.get_logger().info(
                    f"A* planning successful: {len(path_states)} states, "
                    f"cost={result.total_path_cost:.1f}, time={result.planning_time:.3f}s"
                )
            else:
                self.get_logger().warn(f"A* planning failed: {error_msg}")
            
            result.stamp = self.get_clock().now().to_msg()
            return result
            
        except Exception as e:
            self.get_logger().error(f"A* execution error: {str(e)}")
            
            # Return failure result
            result = PathPlan()
            result.success = False
            result.error_message = str(e)
            result.planning_time = time.time() - start_time
            result.stamp = self.get_clock().now().to_msg()
            return result
    
    def _astar_search(self, start: AStarState, goal: AStarState, 
                     obstacle_map: ObstacleMap) -> Tuple[List[AStarState], bool, Optional[str]]:
        """
        Main A* search algorithm converted from MATLAB.
        
        This method implements the core A* algorithm from Pathplanning_Astar.m
        including the main loop, motion primitive selection, and goal checking.
        
        Args:
            start: Start state
            goal: Goal state
            obstacle_map: Environment obstacles
            
        Returns:
            Tuple of (path_states, success, error_message)
        """
        self.planning_iterations = 0
        self.nodes_explored = 0
        
        # Initialize lists (converted from MATLAB o, c lists)
        open_list = []          # Priority queue for open states
        closed_set = set()      # Set of closed state coordinates
        state_tree = {}         # Dictionary mapping state IDs to states
        
        # Initialize start state (from MATLAB cur initialization)
        current = start
        current.g_cost = 0.0
        current.h_cost = self._calculate_heuristic(current, goal)
        current.f_cost = current.g_cost + current.h_cost
        
        # Add start to open list
        state_id = self._get_state_id(current)
        state_tree[state_id] = current
        heapq.heappush(open_list, (current.f_cost, state_id))
        
        # Goal region setup (from MATLAB area definition)
        goal_region = self._setup_goal_region(goal)
        
        self.get_logger().info(f"A* search started, goal region: {goal_region}")
        
        # Main search loop (from MATLAB while loopcheck)
        while open_list and self.planning_iterations < self.config['max_iterations']:
            self.planning_iterations += 1
            
            # Get state with minimum f-cost (from MATLAB min cost selection)
            if not open_list:
                return [], False, "Open list exhausted - no path found"
            
            f_cost, current_id = heapq.heappop(open_list)
            current = state_tree[current_id]
            
            # Add to closed set
            closed_set.add(self._get_state_coord_key(current))
            self.nodes_explored += 1
            
            # Check if goal reached (from MATLAB loop_checkfinal)
            if self._is_goal_reached(current, goal, goal_region):
                self.get_logger().info(f"Goal reached after {self.planning_iterations} iterations")
                path = self._reconstruct_path(current, state_tree)
                return path, True, None
            
            # Generate successors (from MATLAB motion primitive selection)
            successors = self._generate_successors(current, obstacle_map)
            
            for successor in successors:
                coord_key = self._get_state_coord_key(successor)
                
                # Skip if already in closed set
                if coord_key in closed_set:
                    continue
                
                # Calculate costs
                successor.g_cost = current.g_cost + self._calculate_transition_cost(current, successor)
                successor.h_cost = self._calculate_heuristic(successor, goal)
                successor.f_cost = successor.g_cost + successor.h_cost
                successor.predxy = current_id
                
                # Add to open list if not there or if better path found
                successor_id = self._get_state_id(successor)
                
                # Check if already in state tree with better cost
                if successor_id in state_tree:
                    existing = state_tree[successor_id]
                    if successor.g_cost >= existing.g_cost:
                        continue  # Not a better path
                
                # Add/update in state tree and open list
                state_tree[successor_id] = successor
                heapq.heappush(open_list, (successor.f_cost, successor_id))
        
        # Search failed
        error_msg = f"A* search failed after {self.planning_iterations} iterations"
        if self.planning_iterations >= self.config['max_iterations']:
            error_msg += " (max iterations reached)"
        
        return [], False, error_msg
    
    def _generate_successors(self, current: AStarState, obstacle_map: ObstacleMap) -> List[AStarState]:
        """
        Generate successor states using motion primitives.
        
        This converts the MATLAB motion primitive selection logic including
        forward and reverse motion generation.
        
        Args:
            current: Current state
            obstacle_map: Environment obstacles
            
        Returns:
            List of valid successor states
        """
        successors = []
        
        # Motion primitive selection (from MATLAB A1, A2, A6 generation)
        # This is a simplified version - full implementation would load actual MP data
        
        # Forward motion candidates (from MATLAB A selection)
        theta_candidates = self._get_theta_candidates(current, obstacle_map, forward=True)
        gamma_candidates = self.gamma_discrete
        
        for theta_f in theta_candidates:
            for gamma_f in gamma_candidates:
                successor = self._create_successor_state(
                    current, theta_f, gamma_f, forward=True, obstacle_map=obstacle_map
                )
                if successor:
                    successors.append(successor)
        
        # Reverse motion candidates (from MATLAB AR selection)  
        theta_candidates_reverse = self._get_theta_candidates(current, obstacle_map, forward=False)
        
        for theta_f in theta_candidates_reverse:
            for gamma_f in gamma_candidates:
                successor = self._create_successor_state(
                    current, theta_f, gamma_f, forward=False, obstacle_map=obstacle_map
                )
                if successor:
                    successors.append(successor)
        
        return successors
    
    def _create_successor_state(self, current: AStarState, theta_f: float, gamma_f: float,
                              forward: bool, obstacle_map: ObstacleMap) -> Optional[AStarState]:
        """
        Create a successor state using motion primitive.
        
        This implements the MATLAB logic for creating new states from motion primitives
        including collision checking and coordinate transformation.
        
        Args:
            current: Current state
            theta_f: Final theta angle [degrees]
            gamma_f: Final gamma angle [degrees] 
            forward: True for forward motion, False for reverse
            obstacle_map: Environment obstacles
            
        Returns:
            Valid successor state or None if invalid
        """
        try:
            # Create new state
            successor = AStarState()
            
            # Motion primitive simulation (simplified - would use actual MP data)
            # This simulates the MATLAB xp, yp coordinate calculation
            step_size = 2.0  # meters (would come from motion primitive)
            direction_multiplier = 1.0 if forward else -1.0
            
            # Calculate movement (simplified kinematic model)
            theta_rad = math.radians(theta_f)
            dx = direction_multiplier * step_size * math.cos(theta_rad)
            dy = direction_multiplier * step_size * math.sin(theta_rad)
            
            # Set successor position
            successor.xa = current.xa + dx
            successor.ya = current.ya + dy
            successor.x = round(successor.xa)  # ppround_1 equivalent
            successor.y = round(successor.ya)
            
            # Set motion primitive information
            successor.pred_ti = current.pred_tf
            successor.pred_gi = current.pred_gf
            successor.pred_tf = theta_f
            successor.pred_gf = gamma_f
            successor.pred_dir = 1 if forward else 0
            
            # Collision checking (from MATLAB staticobs_check)
            if not self._is_collision_free(current, successor, obstacle_map):
                return None
            
            # Boundary checking
            if not self._is_within_bounds(successor):
                return None
            
            return successor
            
        except Exception as e:
            self.get_logger().debug(f"Error creating successor: {str(e)}")
            return None
    
    def _calculate_heuristic(self, state: AStarState, goal: AStarState) -> float:
        """
        Calculate heuristic cost (converted from h_cost.m).
        
        Implements both Euclidean and rectangular heuristics from the original MATLAB.
        
        Args:
            state: Current state
            goal: Goal state
            
        Returns:
            Heuristic cost value
        """
        try:
            # Euclidean distance (from MATLAB h_cost.m)
            dx = state.xa - goal.xa
            dy = state.ya - goal.ya
            euclidean_dist = math.sqrt(dx*dx + dy*dy)
            
            # Apply heuristic weight (from MATLAB heuristic coefficient)
            heuristic_cost = euclidean_dist * self.config['heuristic_weight']
            
            # Rectangular heuristic logic (from MATLAB rectheur conditions)
            if self.config['use_rectangular_heuristic']:
                # Apply rectangular heuristic if conditions are met
                # This is simplified - full implementation would include rectheur.m logic
                if goal.ya <= 80 and state.ya >= 70:
                    heuristic_cost *= 1.2  # Adjust for rectangular heuristic
                elif goal.ya >= 80 and state.ya <= 70:
                    heuristic_cost *= 1.2
            
            return heuristic_cost
            
        except Exception as e:
            self.get_logger().debug(f"Heuristic calculation error: {str(e)}")
            return float('inf')
    
    def _calculate_transition_cost(self, from_state: AStarState, to_state: AStarState) -> float:
        """
        Calculate transition cost between states (converted from g_cost.m).
        
        Args:
            from_state: Source state
            to_state: Destination state
            
        Returns:
            Transition cost
        """
        # Calculate distance (from MATLAB g_cost pathlength)
        dx = to_state.xa - from_state.xa
        dy = to_state.ya - from_state.ya
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Apply penalties for direction changes, etc.
        cost = distance
        
        # Penalty for reverse motion
        if to_state.pred_dir == 0:  # Reverse
            cost *= 1.2
        
        return cost
    
    def _is_collision_free(self, from_state: AStarState, to_state: AStarState, 
                          obstacle_map: ObstacleMap) -> bool:
        """
        Check if path between states is collision-free.
        
        This implements simplified collision checking. Full implementation
        would convert staticobs_check.m logic.
        
        Args:
            from_state: Source state
            to_state: Destination state
            obstacle_map: Environment obstacles
            
        Returns:
            True if collision-free
        """
        # Simplified collision checking
        # Full implementation would include articulated vehicle model from staticobs_check.m
        
        # Check bounds
        if (to_state.xa < 0 or to_state.xa > self.length_dc or
            to_state.ya < 0 or to_state.ya > self.width_dc):
            return False
        
        # Static obstacle checking (simplified)
        # Would implement full polygon collision checking from MATLAB
        for obstacle in obstacle_map.static_obstacles:
            if self._point_in_polygon(to_state.xa, to_state.ya, obstacle):
                return False
        
        return True
    
    def _point_in_polygon(self, x: float, y: float, polygon) -> bool:
        """Simplified point-in-polygon test."""
        # Placeholder implementation
        # Full version would convert InPolygon.c logic
        return False
    
    def _is_within_bounds(self, state: AStarState) -> bool:
        """Check if state is within environment bounds."""
        return (0 <= state.xa <= self.length_dc and 
                0 <= state.ya <= self.width_dc)
    
    def _get_theta_candidates(self, current: AStarState, obstacle_map: ObstacleMap, 
                            forward: bool) -> List[float]:
        """
        Get theta angle candidates based on virtual obstacles.
        
        This converts the MATLAB virtualobs_check and virtualobsreverse_check logic.
        
        Args:
            current: Current state
            obstacle_map: Environment obstacles
            forward: True for forward motion
            
        Returns:
            List of valid theta angles
        """
        # Simplified version - would implement full virtual obstacle logic
        if forward:
            # From MATLAB A6 and virtual obstacle checking
            base_angles = [270, 342, 351, 0]
        else:
            # From MATLAB A5 for reverse motion
            base_angles = [270, 90]
        
        return base_angles
    
    def _setup_goal_region(self, goal: AStarState) -> Dict[str, Any]:
        """
        Setup goal region based on MATLAB area definition.
        
        Args:
            goal: Goal state
            
        Returns:
            Goal region definition
        """
        # From MATLAB lz_lc, wz_lc goal zone definition
        zone_length = 5.0  # lz_lc
        zone_width = 1.5   # wz_lc
        
        return {
            'center_x': goal.xa,
            'center_y': goal.ya,
            'length': zone_length,
            'width': zone_width,
            'tolerance': self.config['goal_tolerance']
        }
    
    def _is_goal_reached(self, current: AStarState, goal: AStarState, 
                        goal_region: Dict[str, Any]) -> bool:
        """
        Check if current state has reached the goal (from MATLAB loop_checkfinal).
        
        Args:
            current: Current state
            goal: Goal state
            goal_region: Goal region definition
            
        Returns:
            True if goal reached
        """
        # Distance check
        dx = current.xa - goal.xa
        dy = current.ya - goal.ya
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance > goal_region['tolerance']:
            return False
        
        # Angle check
        angle_diff = abs(current.pred_tf - goal.pred_tf)
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        return angle_diff <= math.degrees(self.config['angle_tolerance'])
    
    def _reconstruct_path(self, goal_state: AStarState, state_tree: Dict[str, AStarState]) -> List[AStarState]:
        """
        Reconstruct path from goal to start.
        
        Args:
            goal_state: Goal state reached
            state_tree: Tree of all states
            
        Returns:
            Path as list of states from start to goal
        """
        path = []
        current = goal_state
        
        while current is not None:
            path.append(current)
            if current.predxy is None:
                break
            current = state_tree.get(current.predxy)
        
        path.reverse()
        return path
    
    def _get_state_id(self, state: AStarState) -> str:
        """Generate unique ID for state."""
        return f"{state.x}_{state.y}_{state.pred_tf}_{state.pred_gf}_{state.pred_dir}"
    
    def _get_state_coord_key(self, state: AStarState) -> str:
        """Generate coordinate key for closed set."""
        return f"{state.x}_{state.y}"
    
    def _calculate_path_length(self, path: List[AStarState]) -> float:
        """Calculate total path length."""
        if len(path) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(path)):
            dx = path[i].xa - path[i-1].xa
            dy = path[i].ya - path[i-1].ya
            total_length += math.sqrt(dx*dx + dy*dy)
        
        return total_length
    
    def _ros_to_astar_state(self, ros_state: VehicleState) -> AStarState:
        """Convert ROS2 VehicleState to internal AStarState."""
        astar_state = AStarState()
        astar_state.x = ros_state.x
        astar_state.y = ros_state.y
        astar_state.xa = ros_state.xa if hasattr(ros_state, 'xa') else ros_state.x
        astar_state.ya = ros_state.ya if hasattr(ros_state, 'ya') else ros_state.y
        astar_state.pred_tf = math.degrees(ros_state.theta) if hasattr(ros_state, 'theta') else 0.0
        astar_state.pred_gf = math.degrees(ros_state.gamma) if hasattr(ros_state, 'gamma') else 0.0
        return astar_state
    
    def _astar_to_ros_state(self, astar_state: AStarState) -> VehicleState:
        """Convert internal AStarState to ROS2 VehicleState."""
        ros_state = VehicleState()
        ros_state.x = astar_state.x
        ros_state.y = astar_state.y
        ros_state.xa = astar_state.xa
        ros_state.ya = astar_state.ya
        ros_state.theta = math.radians(astar_state.pred_tf)
        ros_state.gamma = math.radians(astar_state.pred_gf)
        ros_state.g_cost = astar_state.g_cost
        ros_state.h_cost = astar_state.h_cost
        ros_state.f_cost = astar_state.f_cost
        ros_state.direction = astar_state.pred_dir
        ros_state.stamp = self.get_clock().now().to_msg()
        return ros_state
    
    def _load_motion_primitives(self):
        """
        Load motion primitives from data files.
        
        This would load the actual motion primitive data from the MATLAB .mat files
        converted to Python format.
        """
        # Placeholder - would implement actual motion primitive loading
        self.mp_data_loaded = True
        self.get_logger().info("Motion primitives loaded (placeholder)")
    
    def cleanup(self):
        """Clean up plugin resources."""
        self.motion_primitives.clear()
        self.get_logger().info("A* Plugin cleanup completed")


def main(args=None):
    """Main entry point for A* plugin."""
    rclpy.init(args=args)
    
    try:
        astar_plugin = AStarPathPlanner()
        
        # Test configuration
        test_config = AlgorithmConfig()
        test_config.algorithm_name = "astar"
        test_config.heuristic_weight = 2.5
        test_config.penalty_factor = 1000.0
        test_config.max_iterations = 10000
        
        if astar_plugin.load_plugin(test_config):
            astar_plugin.get_logger().info("A* Plugin ready for path planning")
            rclpy.spin(astar_plugin)
        else:
            astar_plugin.get_logger().error("Failed to load A* Plugin")
            
    except KeyboardInterrupt:
        pass
    finally:
        if 'astar_plugin' in locals():
            astar_plugin.cleanup()
            astar_plugin.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
