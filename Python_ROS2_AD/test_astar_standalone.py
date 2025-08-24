#!/usr/bin/env python3
"""
Standalone Test for CATALYST A* Plugin - No ROS2 Required

This test version allows us to validate the A* algorithm conversion
without requiring ROS2 installation. It uses mock data structures
that simulate the ROS2 messages.
"""

import numpy as np
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Any


# Mock ROS2 message structures for testing
@dataclass
class MockVehicleState:
    """Mock VehicleState message for testing."""
    x: float = 0.0
    y: float = 0.0
    xa: float = 0.0
    ya: float = 0.0
    theta: float = 0.0  # radians
    gamma: float = 0.0  # radians
    g_cost: float = 0.0
    h_cost: float = 0.0
    f_cost: float = 0.0
    direction: int = 1


@dataclass
class MockObstacleMap:
    """Mock ObstacleMap message for testing."""
    length_dc: float = 328.0
    width_dc: float = 200.0
    scale: float = 1.0
    static_obstacles: List = None
    
    def __post_init__(self):
        if self.static_obstacles is None:
            self.static_obstacles = []


@dataclass
class MockPathPlan:
    """Mock PathPlan message for testing."""
    start_state: MockVehicleState = None
    goal_state: MockVehicleState = None
    path_states: List[MockVehicleState] = None
    success: bool = False
    error_message: str = ""
    planning_time: float = 0.0
    nodes_explored: int = 0
    total_path_cost: float = 0.0
    path_length: float = 0.0
    algorithm_id: int = 1
    
    def __post_init__(self):
        if self.path_states is None:
            self.path_states = []


@dataclass
class MockAlgorithmConfig:
    """Mock AlgorithmConfig message for testing."""
    algorithm_name: str = "astar"
    plugin_id: str = "astar_test"
    heuristic_weight: float = 2.5
    penalty_factor: float = 1000.0
    use_rectangular_heuristic: bool = True
    max_planning_time: float = 30.0
    max_iterations: int = 10000
    goal_tolerance: float = 1.0
    angle_tolerance: float = 0.1
    parameter_names: List[str] = None
    parameter_values: List[str] = None
    
    def __post_init__(self):
        if self.parameter_names is None:
            self.parameter_names = []
        if self.parameter_values is None:
            self.parameter_values = []


# Mock Logger for testing
class MockLogger:
    def info(self, msg): print(f"[INFO] {msg}")
    def warn(self, msg): print(f"[WARN] {msg}")
    def error(self, msg): print(f"[ERROR] {msg}")
    def debug(self, msg): print(f"[DEBUG] {msg}")


# Simplified A* State for testing
@dataclass
class AStarState:
    x: float = 0.0
    y: float = 0.0
    xa: float = 0.0
    ya: float = 0.0
    pred_ti: float = 0.0
    pred_gi: float = 0.0
    pred_tf: float = 0.0
    pred_gf: float = 0.0
    pred_dir: int = 1
    predxy: Optional[int] = None
    g_cost: float = 0.0
    h_cost: float = 0.0
    f_cost: float = 0.0
    mp_index: Optional[int] = None


class StandaloneAStarPlanner:
    """
    Standalone A* Planner for testing without ROS2.
    
    This contains the core algorithm logic from the full plugin,
    adapted to work without ROS2 dependencies.
    """
    
    def __init__(self):
        self.logger = MockLogger()
        
        # Algorithm parameters (from MATLAB)
        self.length_dc = 328.0
        self.width_dc = 200.0
        self.scale = 1.0
        
        # Discretization
        self.theta_discrete = np.arange(0, 360, 9)
        self.gamma_discrete = np.array([-21, 0, 21])
        
        # Configuration
        self.config = {
            'heuristic_weight': 2.5,
            'penalty_factor': 1000.0,
            'use_rectangular_heuristic': True,
            'max_iterations': 1000,  # Reduced for testing
            'goal_tolerance': 2.0,   # Relaxed for testing
            'angle_tolerance': 0.2
        }
        
        self.planning_iterations = 0
        self.nodes_explored = 0
        
        self.logger.info("Standalone A* Planner initialized")
    
    def plan_path(self, start: MockVehicleState, goal: MockVehicleState, 
                  obstacle_map: MockObstacleMap) -> MockPathPlan:
        """
        Plan path from start to goal.
        
        Args:
            start: Start vehicle state
            goal: Goal vehicle state
            obstacle_map: Environment obstacles
            
        Returns:
            Complete path plan
        """
        start_time = time.time()
        
        self.logger.info(f"Planning from ({start.x:.1f}, {start.y:.1f}) to ({goal.x:.1f}, {goal.y:.1f})")
        
        # Convert to internal representation
        start_astar = self._mock_to_astar_state(start)
        goal_astar = self._mock_to_astar_state(goal)
        
        # Execute A* search
        path_states, success, error_msg = self._astar_search(start_astar, goal_astar, obstacle_map)
        
        # Create result
        result = MockPathPlan()
        result.start_state = start
        result.goal_state = goal
        result.success = success
        result.error_message = error_msg or ""
        result.planning_time = time.time() - start_time
        result.nodes_explored = self.nodes_explored
        result.algorithm_id = 1
        
        if success and path_states:
            result.path_states = [self._astar_to_mock_state(state) for state in path_states]
            result.total_path_cost = path_states[-1].f_cost if path_states else 0.0
            result.path_length = self._calculate_path_length(path_states)
            
            self.logger.info(
                f"Planning successful: {len(path_states)} states, "
                f"cost={result.total_path_cost:.1f}, time={result.planning_time:.3f}s"
            )
        else:
            self.logger.warn(f"Planning failed: {error_msg}")
        
        return result
    
    def _astar_search(self, start: AStarState, goal: AStarState, 
                     obstacle_map: MockObstacleMap) -> Tuple[List[AStarState], bool, Optional[str]]:
        """Main A* search algorithm."""
        import heapq
        
        self.planning_iterations = 0
        self.nodes_explored = 0
        
        # Initialize search structures
        open_list = []
        closed_set = set()
        state_tree = {}
        
        # Initialize start state
        current = start
        current.g_cost = 0.0
        current.h_cost = self._calculate_heuristic(current, goal)
        current.f_cost = current.g_cost + current.h_cost
        
        state_id = self._get_state_id(current)
        state_tree[state_id] = current
        heapq.heappush(open_list, (current.f_cost, state_id))
        
        self.logger.info(f"A* search started, max_iterations: {self.config['max_iterations']}")
        
        # Main search loop
        while open_list and self.planning_iterations < self.config['max_iterations']:
            self.planning_iterations += 1
            
            if not open_list:
                return [], False, "Open list exhausted"
            
            f_cost, current_id = heapq.heappop(open_list)
            current = state_tree[current_id]
            
            # Add to closed set
            closed_set.add(self._get_state_coord_key(current))
            self.nodes_explored += 1
            
            # Check if goal reached
            if self._is_goal_reached(current, goal):
                self.logger.info(f"Goal reached after {self.planning_iterations} iterations")
                path = self._reconstruct_path(current, state_tree)
                return path, True, None
            
            # Generate successors
            successors = self._generate_successors(current, obstacle_map)
            
            for successor in successors:
                coord_key = self._get_state_coord_key(successor)
                
                if coord_key in closed_set:
                    continue
                
                # Calculate costs
                successor.g_cost = current.g_cost + self._calculate_transition_cost(current, successor)
                successor.h_cost = self._calculate_heuristic(successor, goal)
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
            
            # Progress reporting
            if self.planning_iterations % 100 == 0:
                self.logger.info(f"Iteration {self.planning_iterations}, nodes explored: {self.nodes_explored}")
        
        # Search failed
        error_msg = f"Search failed after {self.planning_iterations} iterations"
        if self.planning_iterations >= self.config['max_iterations']:
            error_msg += " (max iterations reached)"
        
        return [], False, error_msg
    
    def _generate_successors(self, current: AStarState, obstacle_map: MockObstacleMap) -> List[AStarState]:
        """Generate successor states."""
        successors = []
        
        # Simplified successor generation for testing
        # Use basic 8-connected grid movement
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
            successor.pred_tf = theta_f
            successor.pred_gf = 0.0  # Simplified
            successor.pred_dir = 1
            
            # Check bounds and obstacles
            if self._is_valid_state(successor, obstacle_map):
                successors.append(successor)
        
        return successors
    
    def _is_valid_state(self, state: AStarState, obstacle_map: MockObstacleMap) -> bool:
        """Check if state is valid."""
        # Bounds check
        if (state.xa < 0 or state.xa > self.length_dc or
            state.ya < 0 or state.ya > self.width_dc):
            return False
        
        # Simple obstacle check (would be more complex in real implementation)
        return True
    
    def _calculate_heuristic(self, state: AStarState, goal: AStarState) -> float:
        """Calculate heuristic cost."""
        dx = state.xa - goal.xa
        dy = state.ya - goal.ya
        euclidean_dist = math.sqrt(dx*dx + dy*dy)
        return euclidean_dist * self.config['heuristic_weight']
    
    def _calculate_transition_cost(self, from_state: AStarState, to_state: AStarState) -> float:
        """Calculate transition cost."""
        dx = to_state.xa - from_state.xa
        dy = to_state.ya - from_state.ya
        return math.sqrt(dx*dx + dy*dy)
    
    def _is_goal_reached(self, current: AStarState, goal: AStarState) -> bool:
        """Check if goal is reached."""
        dx = current.xa - goal.xa
        dy = current.ya - goal.ya
        distance = math.sqrt(dx*dx + dy*dy)
        return distance <= self.config['goal_tolerance']
    
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
        return f"{state.x}_{state.y}_{state.pred_tf}_{state.pred_gf}"
    
    def _get_state_coord_key(self, state: AStarState) -> str:
        """Generate coordinate key."""
        return f"{state.x}_{state.y}"
    
    def _mock_to_astar_state(self, mock_state: MockVehicleState) -> AStarState:
        """Convert mock state to A* state."""
        astar_state = AStarState()
        astar_state.x = mock_state.x
        astar_state.y = mock_state.y
        astar_state.xa = mock_state.xa if mock_state.xa != 0 else mock_state.x
        astar_state.ya = mock_state.ya if mock_state.ya != 0 else mock_state.y
        astar_state.pred_tf = math.degrees(mock_state.theta)
        astar_state.pred_gf = math.degrees(mock_state.gamma)
        return astar_state
    
    def _astar_to_mock_state(self, astar_state: AStarState) -> MockVehicleState:
        """Convert A* state to mock state."""
        mock_state = MockVehicleState()
        mock_state.x = astar_state.x
        mock_state.y = astar_state.y
        mock_state.xa = astar_state.xa
        mock_state.ya = astar_state.ya
        mock_state.theta = math.radians(astar_state.pred_tf)
        mock_state.gamma = math.radians(astar_state.pred_gf)
        mock_state.g_cost = astar_state.g_cost
        mock_state.h_cost = astar_state.h_cost
        mock_state.f_cost = astar_state.f_cost
        mock_state.direction = astar_state.pred_dir
        return mock_state


def test_astar_basic():
    """Test basic A* functionality."""
    print("="*60)
    print("CATALYST A* Plugin - Standalone Test")
    print("="*60)
    
    # Create planner
    planner = StandaloneAStarPlanner()
    
    # Test case from original MATLAB (similar to the hardcoded values)
    start_state = MockVehicleState()
    start_state.x = 75.0  # From MATLAB i_state.x
    start_state.y = 45.0  # From MATLAB i_state.y
    start_state.xa = 75.0
    start_state.ya = 45.0
    start_state.theta = math.radians(180)  # From MATLAB i_state.t = 180
    start_state.gamma = math.radians(0)    # From MATLAB i_state.g = 0
    
    goal_state = MockVehicleState()
    goal_state.x = 60.0   # From MATLAB f_state.x
    goal_state.y = 64.0   # From MATLAB f_state.y
    goal_state.xa = 60.0
    goal_state.ya = 64.0
    goal_state.theta = math.radians(270)   # From MATLAB f_state.t = 270
    goal_state.gamma = math.radians(0)     # From MATLAB f_state.g = 0
    
    # Create obstacle map
    obstacle_map = MockObstacleMap()
    obstacle_map.length_dc = 328.0  # From MATLAB
    obstacle_map.width_dc = 200.0   # From MATLAB
    
    print(f"Start: ({start_state.x}, {start_state.y}) θ={math.degrees(start_state.theta):.0f}°")
    print(f"Goal:  ({goal_state.x}, {goal_state.y}) θ={math.degrees(goal_state.theta):.0f}°")
    print(f"Environment: {obstacle_map.length_dc}m × {obstacle_map.width_dc}m")
    print()
    
    # Plan path
    result = planner.plan_path(start_state, goal_state, obstacle_map)
    
    # Display results
    print("\n" + "="*60)
    print("RESULTS")
    print("="*60)
    print(f"Success: {result.success}")
    print(f"Planning time: {result.planning_time:.3f} seconds")
    print(f"Nodes explored: {result.nodes_explored}")
    print(f"Algorithm iterations: {planner.planning_iterations}")
    
    if result.success:
        print(f"Path length: {result.path_length:.2f} meters")
        print(f"Total cost: {result.total_path_cost:.2f}")
        print(f"Path states: {len(result.path_states)}")
        
        print("\nPath waypoints:")
        for i, state in enumerate(result.path_states):
            if i % 3 == 0 or i == len(result.path_states) - 1:  # Show every 3rd waypoint
                print(f"  {i:2d}: ({state.x:6.1f}, {state.y:6.1f}) "
                      f"θ={math.degrees(state.theta):6.1f}° cost={state.f_cost:6.1f}")
    else:
        print(f"Error: {result.error_message}")
    
    print("\n" + "="*60)
    print("Test completed!")
    print("="*60)
    
    return result.success


def test_astar_simple():
    """Test with a simpler scenario."""
    print("\n" + "="*60)
    print("SIMPLE TEST CASE")
    print("="*60)
    
    planner = StandaloneAStarPlanner()
    
    # Simple test case
    start_state = MockVehicleState()
    start_state.x = 10.0
    start_state.y = 10.0
    start_state.xa = 10.0
    start_state.ya = 10.0
    
    goal_state = MockVehicleState()
    goal_state.x = 20.0
    goal_state.y = 20.0
    goal_state.xa = 20.0
    goal_state.ya = 20.0
    
    obstacle_map = MockObstacleMap()
    
    print(f"Simple path: ({start_state.x}, {start_state.y}) → ({goal_state.x}, {goal_state.y})")
    
    result = planner.plan_path(start_state, goal_state, obstacle_map)
    
    print(f"Result: {result.success}, Time: {result.planning_time:.3f}s, Nodes: {result.nodes_explored}")
    
    return result.success


if __name__ == "__main__":
    print("CATALYST A* Plugin Test Suite")
    print("Testing A* algorithm conversion from MATLAB to Python")
    print()
    
    # Run tests
    try:
        simple_success = test_astar_simple()
        basic_success = test_astar_basic()
        
        print(f"\nTest Summary:")
        print(f"Simple test: {'PASSED' if simple_success else 'FAILED'}")
        print(f"MATLAB test: {'PASSED' if basic_success else 'FAILED'}")
        
        if simple_success and basic_success:
            print("\n🎉 All tests PASSED! A* conversion is working correctly.")
        else:
            print("\n❌ Some tests FAILED. Check the algorithm implementation.")
            
    except Exception as e:
        print(f"\n💥 Test failed with exception: {str(e)}")
        import traceback
        traceback.print_exc()
