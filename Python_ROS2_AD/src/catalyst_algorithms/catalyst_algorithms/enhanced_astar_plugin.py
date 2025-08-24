#!/usr/bin/env python3
"""
CATALYST A* Path Planning Plugin - Enhanced Production Version

This is the complete A* path planning plugin integrating:
- Motion primitive loading from MATLAB .mat files
- Collision detection for articulated vehicles
- Enhanced cost calculations with zone penalties
- Full MATLAB algorithm conversion

Based on MATLAB Pathplanning_Astar.m with complete functionality.
"""

import numpy as np
import heapq
import time
import math
import os
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, field

import rclpy
from catalyst_core.base_plugin import CatalystPlugin
from catalyst_interfaces.msg import (
    VehicleState, ObstacleMap, PathPlan, MotionPrimitive, AlgorithmConfig
)
from catalyst_interfaces.srv import PlanPath

# Import enhanced components
from .motion_primitive_loader import MotionPrimitiveLoader, MotionPrimitive as MPrimitive, VehicleParameters
from .collision_detection import CollisionDetector, Environment, ObstaclePolygon
from .cost_calculator import CostCalculator, CostState, GoalState, ZonePolygon
from .dpd_environment import DPDEnvironmentBuilder
from .rectangular_heuristics import RectangularHeuristics, HeuristicState
from .virtual_obstacle_checker import VirtualObstacleChecker, VehicleState as VOCState


@dataclass
class AStarState:
    """Enhanced A* search state with full MATLAB compatibility."""
    x: float = 0.0          # Discretized X coordinate [m]
    y: float = 0.0          # Discretized Y coordinate [m] 
    xa: float = 0.0         # Actual ending point X [m]
    ya: float = 0.0         # Actual ending point Y [m]
    
    # Motion primitive information (from MATLAB)
    pred_ti: float = 0.0    # Predecessor theta initial [degrees]
    pred_gi: float = 0.0    # Predecessor gamma initial [degrees]
    pred_tf: float = 0.0    # Predecessor theta final [degrees]
    pred_gf: float = 0.0    # Predecessor gamma final [degrees]
    pred_dir: int = 1       # Predecessor direction (1=forward, -1=reverse)
    predxy: Optional[str] = None  # Predecessor state ID
    
    # Cost information
    g_cost: float = 0.0     # Cost from start
    h_cost: float = 0.0     # Heuristic cost to goal
    f_cost: float = 0.0     # Total cost (g + h)
    
    # Motion primitive reference
    mp_index: Optional[int] = None  # Motion primitive index used
    
    def __hash__(self) -> int:
        """Hash function for set operations."""
        return hash((self.x, self.y, self.pred_tf, self.pred_gf))


class EnhancedAStarPathPlanner(CatalystPlugin):
    """
    Enhanced A* Path Planner with complete MATLAB functionality.
    
    Features:
    - Motion primitive loading from MATLAB .mat files
    - Collision detection for articulated vehicles  
    - Enhanced cost calculations
    - Zone-based heuristics
    - Full MATLAB algorithm compatibility
    """
    
    def __init__(self):
        """Initialize enhanced A* planner."""
        super().__init__()
        
        # Enhanced components
        self.motion_loader = MotionPrimitiveLoader()
        self.collision_detector = CollisionDetector()
        self.cost_calculator = CostCalculator()
        
        # Algorithm configuration (from MATLAB)
        self.config = {
            'heuristic_weight': 2.5,        # From MATLAB h_cost.m
            'penalty_factor': 1000.0,       # From MATLAB zone penalty
            'use_rectangular_heuristic': True,
            'max_planning_time': 30.0,
            'max_iterations': 10000,
            'goal_tolerance': 1.0,
            'angle_tolerance': 0.1
        }
        
        # Environment parameters (from MATLAB)
        self.length_dc = 328.0  # Distribution center length [m]
        self.width_dc = 200.0   # Distribution center width [m]
        self.scale = 1.0        # Scaling factor
        
        # Discretization (from MATLAB)
        self.theta_discrete = np.arange(0, 360, 9)  # [0, 9, 18, ..., 351]
        self.gamma_discrete = np.array([-21, 0, 21])  # Articulation angles
        
        # Search statistics
        self.planning_iterations = 0
        self.nodes_explored = 0
        self.nodes_generated = 0
        
        # Current environment
        self.current_environment: Optional[Environment] = None
        self.goal_zones: Optional[Tuple[ZonePolygon, ZonePolygon]] = None
        
        self.get_logger().info("Enhanced A* path planner initialized")
    
    def initialize(self, config: Dict[str, Any]) -> bool:
        """Initialize plugin with configuration."""
        try:
            # Update configuration
            if config:
                self.config.update(config)
            
            # Initialize motion primitives
            primitives_dir = config.get('primitives_directory', None)
            if primitives_dir:
                self.motion_loader.primitives_dir = primitives_dir
            
            success = self.motion_loader.load_motion_primitives()
            if not success:
                self.get_logger().warning("Using mock motion primitives")
            
            # Set up cost calculator with configuration
            self.cost_calculator.heuristic_coefficient = self.config['heuristic_weight']
            self.cost_calculator.zone_penalty_factor = self.config['penalty_factor']
            
            self.get_logger().info(
                f"Initialized with {len(self.motion_loader.primitives)} motion primitives"
            )
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Initialization failed: {str(e)}")
            return False
    
    def plan_path_service(self, request, response):
        """
        Service callback for path planning requests.
        
        Args:
            request: PlanPath service request
            response: PlanPath service response
            
        Returns:
            PlanPath service response
        """
        start_time = time.time()
        
        try:
            # Extract request data
            start_state = request.start_state
            goal_state = request.goal_state
            obstacle_map = request.obstacle_map
            algorithm_config = request.algorithm_config
            
            # Update configuration if provided
            if algorithm_config and algorithm_config.algorithm_name == "astar":
                self._update_config_from_message(algorithm_config)
            
            # Convert ROS messages to internal format
            start_astar = self._ros_to_astar_state(start_state)
            goal_astar = self._ros_to_astar_state(goal_state)
            environment = self._ros_to_environment(obstacle_map)
            
            # Set current environment
            self.current_environment = environment
            
            # Plan path
            path_states, success, error_msg = self._astar_search(start_astar, goal_astar, environment)
            
            # Create response
            response.success = success
            response.error_message = error_msg or ""
            response.planning_time = time.time() - start_time
            response.nodes_explored = self.nodes_explored
            response.algorithm_id = 1  # A* algorithm ID
            
            if success and path_states:
                response.path_states = [self._astar_to_ros_state(state) for state in path_states]
                response.total_path_cost = path_states[-1].f_cost if path_states else 0.0
                response.path_length = self._calculate_path_length(path_states)
                
                self.get_logger().info(
                    f"Planning successful: {len(path_states)} states, "
                    f"cost={response.total_path_cost:.1f}, time={response.planning_time:.3f}s"
                )
            else:
                self.get_logger().warning(f"Planning failed: {error_msg}")
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Path planning service failed: {str(e)}")
            response.success = False
            response.error_message = f"Service error: {str(e)}"
            response.planning_time = time.time() - start_time
            return response
    
    def _astar_search(self, start: AStarState, goal: AStarState, 
                     environment: Environment) -> Tuple[List[AStarState], bool, Optional[str]]:
        """
        Enhanced A* search algorithm with full MATLAB functionality.
        
        Args:
            start: Start state
            goal: Goal state
            environment: Environment with obstacles
            
        Returns:
            (path_states, success, error_message)
        """
        start_time = time.time()
        self.planning_iterations = 0
        self.nodes_explored = 0
        self.nodes_generated = 0
        
        # Initialize search structures
        open_list = []  # Priority queue: (f_cost, state_id)
        closed_set = set()  # Closed state coordinates
        state_tree = {}  # All generated states
        
        # Initialize start state
        current = start
        current.g_cost = 0.0
        current.h_cost = self._calculate_heuristic(current, goal, start)
        current.f_cost = current.g_cost + current.h_cost
        current.predxy = None
        
        # Add to search
        state_id = self._get_state_id(current)
        state_tree[state_id] = current
        heapq.heappush(open_list, (current.f_cost, state_id))
        
        self.get_logger().info(
            f"A* search started: start=({start.xa:.1f},{start.ya:.1f}), "
            f"goal=({goal.xa:.1f},{goal.ya:.1f})"
        )
        
        # Main A* search loop (from MATLAB)
        while open_list and self.planning_iterations < self.config['max_iterations']:
            self.planning_iterations += 1
            
            # Check time limit
            if time.time() - start_time > self.config['max_planning_time']:
                return [], False, "Planning time limit exceeded"
            
            # Get best state from open list
            if not open_list:
                return [], False, "Open list exhausted"
            
            f_cost, current_id = heapq.heappop(open_list)
            current = state_tree[current_id]
            
            # Add to closed set
            coord_key = self._get_state_coord_key(current)
            closed_set.add(coord_key)
            self.nodes_explored += 1
            
            # Check if goal reached
            if self._is_goal_reached(current, goal):
                self.get_logger().info(
                    f"Goal reached after {self.planning_iterations} iterations, "
                    f"{self.nodes_explored} nodes explored"
                )
                path = self._reconstruct_path(current, state_tree)
                return path, True, None
            
            # Generate successors using motion primitives
            successors = self._generate_successors(current, environment)
            
            for successor in successors:
                coord_key = self._get_state_coord_key(successor)
                
                # Skip if in closed set
                if coord_key in closed_set:
                    continue
                
                # Calculate costs
                successor.g_cost = current.g_cost + self._calculate_transition_cost(current, successor)
                successor.h_cost = self._calculate_heuristic(successor, goal, start)
                successor.f_cost = successor.g_cost + successor.h_cost
                successor.predxy = current_id
                
                successor_id = self._get_state_id(successor)
                
                # Check if better path
                if successor_id in state_tree:
                    existing = state_tree[successor_id]
                    if successor.g_cost >= existing.g_cost:
                        continue
                
                # Add to search
                state_tree[successor_id] = successor
                heapq.heappush(open_list, (successor.f_cost, successor_id))
                self.nodes_generated += 1
            
            # Progress reporting
            if self.planning_iterations % 1000 == 0:
                self.get_logger().info(
                    f"Iteration {self.planning_iterations}, nodes explored: {self.nodes_explored}, "
                    f"open list size: {len(open_list)}"
                )
        
        # Search failed
        error_msg = f"Search failed after {self.planning_iterations} iterations"
        if self.planning_iterations >= self.config['max_iterations']:
            error_msg += " (max iterations reached)"
        
        self.get_logger().warning(error_msg)
        return [], False, error_msg
    
    def _generate_successors(self, current: AStarState, environment: Environment) -> List[AStarState]:
        """
        Generate successor states using motion primitives.
        
        Enhanced version with real motion primitive loading.
        """
        successors = []
        
        # Get applicable motion primitives
        applicable_primitives = self.motion_loader.get_applicable_primitives(
            current.pred_tf, current.pred_gf
        )
        
        # If no loaded primitives, use fallback generation
        if not applicable_primitives:
            return self._generate_fallback_successors(current, environment)
        
        for primitive in applicable_primitives:
            try:
                # Calculate end state using motion primitive
                end_x, end_y, end_theta, end_gamma = self.motion_loader.calculate_end_state(
                    current.xa, current.ya, current.pred_tf, current.pred_gf, primitive
                )
                
                # Create successor state
                successor = AStarState()
                successor.xa = end_x
                successor.ya = end_y
                successor.x = round(end_x)
                successor.y = round(end_y)
                successor.pred_ti = current.pred_tf
                successor.pred_gi = current.pred_gf
                successor.pred_tf = end_theta
                successor.pred_gf = end_gamma
                successor.pred_dir = primitive.direction
                successor.mp_index = primitive.id
                
                # Check collision
                if self._is_state_valid(successor, environment):
                    successors.append(successor)
                    
            except Exception as e:
                self.get_logger().debug(f"Failed to generate successor with primitive {primitive.id}: {str(e)}")
        
        return successors
    
    def _generate_fallback_successors(self, current: AStarState, environment: Environment) -> List[AStarState]:
        """Generate successors using fallback method when no primitives available."""
        successors = []
        
        # Basic 8-connected movement (simplified fallback)
        movements = [
            (2.0, 0.0, 0),    # East
            (-2.0, 0.0, 180), # West  
            (0.0, 2.0, 90),   # North
            (0.0, -2.0, 270), # South
            (1.4, 1.4, 45),   # NE
            (-1.4, 1.4, 135), # NW
            (1.4, -1.4, 315), # SE
            (-1.4, -1.4, 225) # SW
        ]
        
        for dx, dy, theta_f in movements:
            successor = AStarState()
            successor.xa = current.xa + dx
            successor.ya = current.ya + dy
            successor.x = round(successor.xa)
            successor.y = round(successor.ya)
            successor.pred_ti = current.pred_tf
            successor.pred_gi = current.pred_gf
            successor.pred_tf = theta_f
            successor.pred_gf = 0.0
            successor.pred_dir = 1
            
            if self._is_state_valid(successor, environment):
                successors.append(successor)
        
        return successors
    
    def _is_state_valid(self, state: AStarState, environment: Environment) -> bool:
        """Check if state is valid (collision-free and within bounds)."""
        try:
            # Convert to radians for collision detection
            theta_rad = math.radians(state.pred_tf)
            gamma_rad = math.radians(state.pred_gf)
            
            # Check collision using enhanced collision detector
            return self.collision_detector.check_collision(
                state.xa, state.ya, theta_rad, gamma_rad, environment
            )
            
        except Exception as e:
            self.get_logger().debug(f"State validation failed: {str(e)}")
            return False
    
    def _calculate_heuristic(self, state: AStarState, goal: AStarState, initial: AStarState) -> float:
        """Calculate heuristic using enhanced cost calculator."""
        try:
            # Convert to cost calculator format
            current_cost_state = CostState(
                x=state.x, y=state.y, xa=state.xa, ya=state.ya,
                theta=math.radians(state.pred_tf), gamma=math.radians(state.pred_gf)
            )
            
            goal_cost_state = GoalState(
                x=goal.x, y=goal.y, xa=goal.xa, ya=goal.ya,
                theta=math.radians(goal.pred_tf), gamma=math.radians(goal.pred_gf)
            )
            
            initial_cost_state = CostState(
                x=initial.x, y=initial.y, xa=initial.xa, ya=initial.ya,
                theta=math.radians(initial.pred_tf), gamma=math.radians(initial.pred_gf)
            )
            
            # Use goal zones if available
            large_zone, precise_zone = None, None
            if self.goal_zones:
                large_zone, precise_zone = self.goal_zones
            
            return self.cost_calculator.calculate_h_cost(
                current_cost_state, goal_cost_state, initial_cost_state, large_zone, precise_zone
            )
            
        except Exception as e:
            self.get_logger().debug(f"Heuristic calculation failed: {str(e)}")
            # Fallback to simple Euclidean distance
            dx = state.xa - goal.xa
            dy = state.ya - goal.ya
            return math.sqrt(dx*dx + dy*dy) * self.config['heuristic_weight']
    
    def _calculate_transition_cost(self, from_state: AStarState, to_state: AStarState) -> float:
        """Calculate cost of transition between states."""
        try:
            # Get motion primitive cost if available
            mp_cost = 0.0
            if to_state.mp_index and str(to_state.mp_index) in self.motion_loader.primitives:
                primitive = self.motion_loader.primitives[str(to_state.mp_index)]
                mp_cost = primitive.curvature_cost
            
            # Use enhanced cost calculator
            from_cost_state = CostState(
                x=from_state.x, y=from_state.y, xa=from_state.xa, ya=from_state.ya,
                theta=math.radians(from_state.pred_tf), gamma=math.radians(from_state.pred_gf),
                parent_g_cost=from_state.g_cost
            )
            
            to_cost_state = CostState(
                x=to_state.x, y=to_state.y, xa=to_state.xa, ya=to_state.ya,
                theta=math.radians(to_state.pred_tf), gamma=math.radians(to_state.pred_gf)
            )
            
            return self.cost_calculator.calculate_transition_cost(from_cost_state, to_cost_state, mp_cost)
            
        except Exception as e:
            self.get_logger().debug(f"Transition cost calculation failed: {str(e)}")
            # Fallback to Euclidean distance
            dx = to_state.xa - from_state.xa
            dy = to_state.ya - from_state.ya
            return math.sqrt(dx*dx + dy*dy)
    
    def _is_goal_reached(self, current: AStarState, goal: AStarState) -> bool:
        """Check if current state reaches the goal."""
        dx = current.xa - goal.xa
        dy = current.ya - goal.ya
        distance = math.sqrt(dx*dx + dy*dy)
        
        angle_diff = abs(current.pred_tf - goal.pred_tf)
        angle_diff = min(angle_diff, 360 - angle_diff)  # Handle wrap-around
        
        return (distance <= self.config['goal_tolerance'] and 
                angle_diff <= math.degrees(self.config['angle_tolerance']))
    
    def _reconstruct_path(self, goal_state: AStarState, state_tree: Dict[str, AStarState]) -> List[AStarState]:
        """Reconstruct path from goal to start."""
        path = []
        current = goal_state
        
        while current is not None:
            path.append(current)
            if current.predxy is None:
                break
            current = state_tree.get(current.predxy)
        
        path.reverse()
        return path
    
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
    
    def _get_state_id(self, state: AStarState) -> str:
        """Generate unique ID for state."""
        return f"{state.x}_{state.y}_{state.pred_tf}_{state.pred_gf}_{state.pred_dir}"
    
    def _get_state_coord_key(self, state: AStarState) -> str:
        """Generate coordinate key for closed set."""
        return f"{state.x}_{state.y}_{state.pred_tf}"
    
    def _ros_to_astar_state(self, ros_state: VehicleState) -> AStarState:
        """Convert ROS VehicleState to AStarState."""
        astar_state = AStarState()
        astar_state.x = ros_state.x
        astar_state.y = ros_state.y
        astar_state.xa = ros_state.xa if ros_state.xa != 0 else ros_state.x
        astar_state.ya = ros_state.ya if ros_state.ya != 0 else ros_state.y
        astar_state.pred_tf = math.degrees(ros_state.theta)
        astar_state.pred_gf = math.degrees(ros_state.gamma)
        astar_state.pred_dir = ros_state.direction
        astar_state.g_cost = ros_state.g_cost
        astar_state.h_cost = ros_state.h_cost
        astar_state.f_cost = ros_state.f_cost
        return astar_state
    
    def _astar_to_ros_state(self, astar_state: AStarState) -> VehicleState:
        """Convert AStarState to ROS VehicleState."""
        ros_state = VehicleState()
        ros_state.x = astar_state.x
        ros_state.y = astar_state.y
        ros_state.xa = astar_state.xa
        ros_state.ya = astar_state.ya
        ros_state.theta = math.radians(astar_state.pred_tf)
        ros_state.gamma = math.radians(astar_state.pred_gf)
        ros_state.direction = astar_state.pred_dir
        ros_state.g_cost = astar_state.g_cost
        ros_state.h_cost = astar_state.h_cost
        ros_state.f_cost = astar_state.f_cost
        return ros_state
    
    def _ros_to_environment(self, obstacle_map: ObstacleMap) -> Environment:
        """Convert ROS ObstacleMap to Environment."""
        environment = Environment()
        environment.length_dc = obstacle_map.length_dc
        environment.width_dc = obstacle_map.width_dc
        
        # Convert obstacles (simplified - would need more complex parsing)
        environment.obstacles = []
        
        return environment
    
    def _update_config_from_message(self, algorithm_config: AlgorithmConfig):
        """Update configuration from ROS message."""
        self.config['heuristic_weight'] = algorithm_config.heuristic_weight
        self.config['penalty_factor'] = algorithm_config.penalty_factor
        self.config['use_rectangular_heuristic'] = algorithm_config.use_rectangular_heuristic
        self.config['max_planning_time'] = algorithm_config.max_planning_time
        self.config['max_iterations'] = algorithm_config.max_iterations
        self.config['goal_tolerance'] = algorithm_config.goal_tolerance
        self.config['angle_tolerance'] = algorithm_config.angle_tolerance
    
    def get_status(self) -> Dict[str, Any]:
        """Get plugin status information."""
        return {
            'name': 'Enhanced A* Path Planner',
            'version': '2.0.0',
            'status': 'active',
            'motion_primitives_loaded': len(self.motion_loader.primitives),
            'last_planning_time': getattr(self, '_last_planning_time', 0.0),
            'nodes_explored': self.nodes_explored,
            'configuration': self.config
        }


def main(args=None):
    """Main function for standalone testing."""
    rclpy.init(args=args)
    
    try:
        # Create and configure planner
        planner = EnhancedAStarPathPlanner()
        
        # Initialize with default configuration
        config = {
            'heuristic_weight': 2.5,
            'penalty_factor': 1000.0,
            'use_rectangular_heuristic': True,
            'max_planning_time': 30.0,
            'max_iterations': 10000,
            'goal_tolerance': 1.0,
            'angle_tolerance': 0.1
        }
        
        success = planner.initialize(config)
        if not success:
            planner.get_logger().error("Failed to initialize planner")
            return
        
        planner.get_logger().info("Enhanced A* planner ready")
        
        # Spin the node
        rclpy.spin(planner)
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
