# Path Planning System - Working Flow Documentation

## Overview

This document provides a comprehensive breakdown of the execution flow in the refactored ADAS path planning system, from entry point to final path generation. The system follows a 5-layer architecture with clear separation of concerns.

## 🔄 Complete Working Flow

### 📍 ENTRY POINT

```
main_demo.m (Line 188) - Main entry point
└── Calls: main_demo() function
```

**File:** `main_demo.m`
- **Purpose:** Demonstration script that replaces the original monolithic `Pathplanning_Astar.m`
- **Location:** Root directory
- **Key Function:** Shows how to use the new layered architecture

---

## 🏗️ LAYER 5: APPLICATION INITIALIZATION

### 1. PlanningSession Creation

**Flow:** `main_demo.m` → `PlanningSession.m`

```matlab
% main_demo.m (Line 13)
session = PlanningSession('scenarios/DPDScenario.json');
```

**PlanningSession Constructor Sequence:**
```matlab
% PlanningSession.m (Lines 14-28)
1. Load configuration file → ConfigManager().load_config()
2. Create PlanningEngine(config) 
3. Create ScenarioManager(config.scenario)
4. Create ResultsManager()
```

**Key Components Initialized:**
- **ConfigManager**: Loads JSON configuration
- **PlanningEngine**: Core planning orchestrator (Layer 4)
- **ScenarioManager**: Manages test scenarios and environments
- **ResultsManager**: Stores and analyzes planning results

---

## 🎯 LAYER 4: PLANNING ENGINE INITIALIZATION

### 2. PlanningEngine Creation

**Flow:** `PlanningSession.m` → `PlanningEngine.m`

```matlab
% PlanningEngine.m (Lines 17-31)
```

**Initialization Sequence:**

#### **Layer 3 Services Creation:**
```matlab
1. CostCalculator(config.planning.cost_function)
2. HeuristicCalculator(config.planning.heuristic) 
3. ObstacleChecker(config.planning.obstacle_checker)  ← Critical collision detection service
4. MotionPrimitiveEngine(config.motion_primitives)
```

#### **Layer 2 Framework Creation:**
```matlab
5. StateSpace(config) → State management and validation
```

#### **Algorithm Factory Setup:**
```matlab
6. AlgorithmFactory() → Registers available algorithms (A*, Dijkstra, RRT, etc.)
7. set_algorithm(config.planning.algorithm.name) → Sets initial algorithm
```

---

## ⚙️ LAYER 3: SERVICE INITIALIZATION

### 3. ObstacleChecker Creation

**File:** `+services/+collision_detection/ObstacleChecker.m`

```matlab
% ObstacleChecker.m (Lines 11-24)
Constructor Process:
1. Set default collision_buffer = 0.5m
2. Initialize static_obstacles = []
3. Initialize vehicle_model = []
4. Override with configuration if provided
```

**Key Properties:**
- **collision_buffer**: Safety margin around obstacles
- **static_obstacles**: Obstacle geometry data
- **vehicle_model**: Tractor-trailer dimensions and parameters

### 4. Other Service Initialization

#### **CostCalculator**
- **Purpose**: Calculate g-costs (cost so far)
- **Features**: Distance costs, direction change penalties, reverse motion penalties

#### **HeuristicCalculator** 
- **Purpose**: Calculate h-costs (estimated cost to goal)
- **Features**: Multiple heuristic types (Manhattan, Euclidean)

#### **MotionPrimitiveEngine**
- **Purpose**: Provide kinematically feasible motion primitives
- **Features**: Pre-computed trajectories for tractor-trailer dynamics

---

## 🚀 EXECUTION FLOW

### 4. Single Planning Execution

**Flow:** `main_demo.m` → `PlanningSession.execute_single_planning`

```matlab
% main_demo.m (Line 20)
[path, success, stats] = session.execute_single_planning(start_state, goal_state);
```

**PlanningSession.execute_single_planning Sequence:**
```matlab
% PlanningSession.m (Lines 32-65)
1. Get scenario data → scenario_manager.get_scenario_data()
2. Set algorithm if specified in options
3. Execute planning → planning_engine.plan_path(start, goal, scenario_data)
4. Display results → display_planning_results()
5. Store results → results_manager.add_result()
6. Visualize if enabled → visualize_results()
```

### 5. PlanningEngine.plan_path Execution

**Core Planning Logic:**
```matlab
% PlanningEngine.m (Lines 36-56)
1. Validate states → state_space.is_state_valid(start_state/goal_state)
2. Load obstacles → obstacle_checker.load_obstacles(scenario_data)  ← ObstacleChecker integration
3. Create planning context → create_planning_context()
4. Execute algorithm → current_algorithm.execute_planning(context)
5. Post-process path → post_process_path() (if successful)
6. Enhance statistics → enhance_statistics()
```

---

## 🧠 LAYER 4: ALGORITHM EXECUTION

### 6. Algorithm Creation & Selection

**Flow:** `AlgorithmFactory` → `AStarAlgorithm`

```matlab
% AlgorithmFactory.m (Lines 18-26)
Algorithm Creation Process:
1. Check registration → registered_algorithms.isKey(algorithm_name)
2. Get algorithm class → registered_algorithms(algorithm_name)  
3. Create instance → feval(algorithm_class, config, services)
```

**Available Algorithms:**
- **astar**: A* Search Algorithm
- **dijkstra**: Dijkstra's Algorithm  
- **rrt**: Rapidly-exploring Random Trees
- **hybrid_astar**: Hybrid A* Algorithm
- **theta_star**: Theta* Algorithm

### 7. AStarAlgorithm.execute_planning

**Main A* Search Loop:**
```matlab
% AStarAlgorithm.m (Lines 34-101)
1. Extract planning context (start_state, goal_state, services)
2. Update services if provided
3. Validate inputs → validate_inputs()
4. Initialize search structures:
   - open_list (priority queue)
   - closed_list (visited states)
   - search_tree (for path reconstruction)
   - statistics tracking
5. Create start node with initial costs
6. MAIN A* SEARCH LOOP:
   a. Get minimum f-cost node → open_list.pop()
   b. Add to closed list → closed_list.add()
   c. Check goal condition → is_goal_reached()
   d. Generate successors → generate_successors()  ← Core expansion
   e. Update open list with new valid nodes
   f. Continue until goal found or open list empty
7. Reconstruct path if goal reached
8. Return results with statistics
```

### 8. Successor Generation (Core Path Finding)

**Critical Function:** `generate_successors()`
```matlab
% AStarAlgorithm.m (Lines 190-240)
For each current node:
1. Get available motion primitives → motion_primitive_engine.get_primitives()
2. For each motion primitive:
   a. Apply primitive → apply_motion_primitive()
   b. Check state validity → state_space.is_state_valid()
   c. Check collision safety → check_collision_free()  ← ObstacleChecker called here
   d. Check if already visited → closed_list.contains_state()
   e. Create successor node → StateNode(successor_state)
   f. Calculate costs:
      - G-cost → cost_calculator.calculate_g_cost()
      - H-cost → heuristic_calculator.calculate_h_cost()
   g. Update parent relationships
   h. Add to open list → open_list.push()
```

---

## 🛡️ COLLISION CHECKING FLOW (ObstacleChecker in Action)

### 9. Primary Collision Check Call

**Trigger:** A* algorithm calls collision check for each motion primitive

```matlab
% AStarAlgorithm.m (Line 262)
is_collision_free = obj.services.obstacle_checker.check_static_obstacles(
    current_state, motion_primitive.trajectory,
    motion_primitive.theta_array, motion_primitive.gamma_array,
    motion_primitive.direction, [], [], []);
```

### 10. ObstacleChecker.check_static_obstacles

**Detailed Collision Checking Process:**
```matlab
% ObstacleChecker.m (Lines 31-62)
1. Validate inputs (obstacle data, trajectory)
2. For each point along the trajectory:
   a. Calculate vehicle pose:
      - vehicle_x = current_state.x + trajectory.x(i)
      - vehicle_y = current_state.y + trajectory.y(i)  
      - vehicle_theta = theta_array(i)
      - vehicle_gamma = gamma_array(i)
   b. Generate vehicle footprint → generate_vehicle_footprint()
   c. For each obstacle in environment:
      - Check collision → check_vehicle_obstacle_collision()
      - Return FALSE immediately if collision detected
3. Return TRUE if entire trajectory is safe
```

### 11. Vehicle Footprint Generation

**Multi-Body Vehicle Modeling:**
```matlab
% ObstacleChecker.m (Lines 108-125)
1. Create default vehicle model if needed → create_default_vehicle_model()
2. Generate tractor footprint → generate_tractor_footprint(x, y, theta)
3. Generate trailer footprint → generate_trailer_footprint(x, y, theta, gamma)
4. Combine into complete vehicle_footprint structure
```

**Vehicle Model Parameters (Default):**
```matlab
tractor_length: 6.0m
tractor_width: 2.5m  
trailer_length: 10.5m
trailer_width: 2.5m
hitch_offset: 3.0m
```

### 12. Tractor Footprint Generation

```matlab
% ObstacleChecker.m (Lines 127-142)
1. Get tractor dimensions from vehicle model
2. Define corner points in local coordinates:
   corners_local = [
       -length_t/2, -width_t/2;  % Rear-left
        length_t/2, -width_t/2;  % Front-left  
        length_t/2,  width_t/2;  % Front-right
       -length_t/2,  width_t/2   % Rear-right
   ];
3. Transform to global coordinates → transform_points(corners, x, y, theta)
```

### 13. Trailer Footprint Generation

```matlab
% ObstacleChecker.m (Lines 144-167)
1. Get trailer dimensions and hitch offset
2. Calculate hitch point (connection between tractor and trailer):
   hitch_x = x - hitch_offset * cosd(theta)
   hitch_y = y - hitch_offset * sind(theta)
3. Calculate trailer orientation:
   trailer_theta = theta + gamma  ← Articulation angle
4. Define trailer corners in local coordinates
5. Transform to global coordinates using hitch point and trailer orientation
```

### 14. Coordinate Transformation

**Mathematical Transformation:**
```matlab
% ObstacleChecker.m (Lines 169-184)
1. Convert angle to radians: theta_rad = deg2rad(theta)
2. Create 2D rotation matrix:
   R = [cos(theta), -sin(theta);
        sin(theta),  cos(theta)]
3. For each point:
   rotated_point = R * local_point
   global_point = [x + rotated_point(1), y + rotated_point(2)]
```

### 15. Vehicle-Obstacle Collision Detection

```matlab
% ObstacleChecker.m (Lines 186-198)
1. Check tractor collision → check_polygon_collision(tractor_footprint, obstacle)
2. Check trailer collision → check_polygon_collision(trailer_footprint, obstacle)  
3. Return TRUE if EITHER tractor OR trailer collides
```

### 16. Polygon Collision Algorithm

**Two-Phase Collision Detection:**
```matlab
% ObstacleChecker.m (Lines 200-224)
1. Add safety buffer to obstacle → add_buffer_to_polygon()
2. Phase 1 - Check if any vehicle vertex is inside buffered obstacle:
   for each vehicle_vertex:
       if inpolygon(vertex.x, vertex.y, buffered_obstacle.x, buffered_obstacle.y):
           return COLLISION
3. Phase 2 - Check if any obstacle vertex is inside vehicle:
   for each obstacle_vertex:
       if inpolygon(vertex.x, vertex.y, vehicle_polygon.x, vehicle_polygon.y):
           return COLLISION
4. Return NO_COLLISION if both phases pass
```

### 17. Polygon Buffering

**Safety Margin Implementation:**
```matlab
% ObstacleChecker.m (Lines 226-253)
Simplified polygon buffering algorithm:
1. For each vertex in polygon:
   a. Get adjacent vertices (previous and next)
   b. Calculate edge direction vector
   c. Calculate outward normal vector (perpendicular)
   d. Offset vertex by buffer_size along normal direction
2. Return buffered polygon coordinates
```

---

## 📊 RESULT FLOW

### 18. Path Reconstruction & Results

**Success Path:**
```matlab
If goal found in A* search:
1. Reconstruct path → reconstruct_path(goal_node)
2. Return to PlanningEngine → post_process_path()
3. Return to PlanningSession → display_planning_results()
4. Store results → results_manager.add_result()
5. Visualize if enabled → visualize_results()
```

**Failure Handling:**
```matlab
If planning fails:
1. Return empty path with failure status
2. Provide termination reason:
   - 'max_iterations_reached'
   - 'open_list_exhausted'
   - 'invalid_inputs'
3. Log statistics for analysis
```

---

## 🔗 Key Data Flow Connections

### Configuration Flow
```
JSON Config → ConfigManager → PlanningEngine → Services (ObstacleChecker, CostCalculator, etc.)
```

### Obstacle Data Flow  
```
Scenario File → ScenarioManager → ObstacleChecker.load_obstacles() → Collision Detection
```

### Motion Planning Flow
```
Motion Primitives → State Generation → ObstacleChecker → Cost Calculation → A* Decision
```

### Service Dependencies
```
Algorithm Uses: {
    ObstacleChecker    → Collision safety
    CostCalculator     → Movement costs  
    HeuristicCalculator → Goal estimation
    MotionPrimitiveEngine → Valid motions
}
```

---

## 🎯 ObstacleChecker's Critical Role

The `ObstacleChecker` service is integrated at multiple points:

### **Initialization Phase:**
- Created by PlanningEngine with configuration parameters
- Loaded with scenario obstacle data

### **Planning Phase:**  
- Called by A* algorithm for every motion primitive evaluation
- Determines safety of each potential vehicle movement
- Prevents collision with static obstacles

### **Real-time Decisions:**
- Filters available motion primitives based on collision safety
- Ensures generated paths are feasible for tractor-trailer execution

---

## 📝 Summary

This working flow demonstrates how the refactored system transforms a monolithic 364-line script into a modular, maintainable architecture. The `ObstacleChecker` plays a crucial role as a Layer 3 service, providing collision detection capabilities that enable safe path planning for complex tractor-trailer vehicles in obstacle-rich environments.

The layered design ensures:
- **Modularity**: Each component has single responsibility
- **Extensibility**: Easy to add new algorithms and services  
- **Testability**: Layer-by-layer validation possible
- **Maintainability**: Changes isolated to specific components
- **Reusability**: Services shared across different algorithms

---

## 🔧 Configuration Files

### Key Configuration Elements:
- **Scenario Configuration**: `scenarios/DPDScenario.json`
- **Algorithm Parameters**: Cost function weights, heuristic types
- **Vehicle Model**: Tractor-trailer dimensions and constraints
- **Obstacle Data**: Static obstacle coordinates and shapes
- **Planning Parameters**: Search limits, visualization settings

This comprehensive flow ensures robust, safe, and efficient path planning for autonomous driving applications.
