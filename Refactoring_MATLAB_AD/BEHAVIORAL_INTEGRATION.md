# Behavioral Planner Integration with 5-Layer Architecture

## Overview

This document describes how the `BehavioralPlannerSystemObject` integrates with your existing 5-layer path planning architecture to create a complete **6-layer hierarchical planning system**.

## 🏗️ Enhanced Architecture

### **Before: 5-Layer Architecture**
```
┌─────────────────────────────────────────────────────────┐
│  LAYER 5: APPLICATIONS                                  │
│  - PlanningSession (main orchestrator)                 │
│  - Scenario management, Batch processing               │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│  LAYER 4: ALGORITHMS (MODULAR)                         │
│  - PlanningEngine, AlgorithmFactory, A*, Dijkstra, RRT │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│  LAYER 3: SERVICES                                     │
│  - ObstacleChecker, CostCalculator, HeuristicCalculator│
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│  LAYER 2: FRAMEWORK                                    │
│  - StateSpace, StateNode, OpenList/ClosedList          │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│  LAYER 1: FOUNDATION                                   │
│  - DataLoader, GeometryUtils, Math utilities           │
└─────────────────────────────────────────────────────────┘
```

### **After: 6-Layer Architecture with Behavioral Planning**
```
┌─────────────────────────────────────────────────────────┐
│  LAYER 6: MISSION PLANNING (NEW)                       │
│  - BehavioralPlannerSystemObject                       │
│  - Route sequencing, Segment management                │
│  - High-level navigation decisions                     │
└─────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────┐
│  LAYER 5: APPLICATIONS                                  │
│  - PlanningSession (receives goals from Layer 6)       │
│  - Scenario management, Batch processing               │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│  LAYER 4: ALGORITHMS (MODULAR)                         │
│  - PlanningEngine, AlgorithmFactory, A*, Dijkstra, RRT │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│  LAYER 3: SERVICES                                     │
│  - ObstacleChecker, CostCalculator, HeuristicCalculator│
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│  LAYER 2: FRAMEWORK                                    │
│  - StateSpace, StateNode, OpenList/ClosedList          │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│  LAYER 1: FOUNDATION                                   │
│  - DataLoader, GeometryUtils, Math utilities           │
└─────────────────────────────────────────────────────────┘
```

## 🎯 Integration Strategy

### **1. Hierarchical Planning Levels**

| Planning Level | Time Horizon | Spatial Scale | Responsibility |
|---------------|--------------|---------------|----------------|
| **Mission Planning** (Layer 6) | Minutes to Hours | Kilometers | Route sequencing, segment goals |
| **Path Planning** (Layer 5-1) | Seconds to Minutes | Meters to Hundreds of meters | Detailed trajectory generation |
| **Motion Control** | Milliseconds to Seconds | Vehicle dimensions | Vehicle dynamics and control |

### **2. Data Flow Integration**

```
Route Plan (JSON) ──┐
                    ▼
    ┌─────────────────────────────────────┐
    │    BehavioralPlannerSystemObject    │  ← Layer 6
    │    - Load route segments            │
    │    - Sequence goals                 │
    │    - Adaptive configuration         │
    └─────────────────┬───────────────────┘
                      │ next_goal, segment_config
                      ▼
    ┌─────────────────────────────────────┐
    │         PlanningSession             │  ← Layer 5
    │    - Execute detailed planning      │
    │    - Use configured algorithm       │
    │    - Generate precise trajectory    │
    └─────────────────┬───────────────────┘
                      │ detailed_path
                      ▼
            Vehicle Control System
```

## 🔧 Key Integration Components

### **1. BehavioralPlannerSystemObject**

**Purpose:** Acts as Layer 6 - Mission Planning level

**Key Features:**
- **Route Sequencing:** Breaks mission into executable segments
- **Goal Management:** Provides next goal to Layer 5 planning
- **Adaptive Configuration:** Adjusts algorithm and parameters per segment
- **Progress Monitoring:** Tracks segment completion and mission progress

**Integration Points:**
```matlab
% Creates PlanningSession from your Layer 5
obj.planning_session = PlanningSession('scenarios/DPDScenario.json');

% Provides goals to your existing system
next_goal = [segment.end_pose(1), segment.end_pose(2), ...
           segment.end_pose(3), segment.end_pose(4)];

% Configures your algorithms adaptively
segment_config.algorithm = attributes.algorithm; % 'astar', 'rrt', etc.
```

### **2. Route Plan Structure**

**JSON Configuration Format:**
```json
{
  "segments": [
    {
      "start_pose": [x, y, theta, gamma],
      "end_pose": [x, y, theta, gamma],
      "type": "straight|turn|parking",
      "attributes": {
        "algorithm": "astar|dijkstra|rrt",
        "max_speed": 5.0,
        "turn_maneuver": true/false,
        "stop_line": true/false
      }
    }
  ]
}
```

**Integration with Your System:**
- **start_pose/end_pose:** Compatible with your state format `[x, y, theta, gamma]`
- **algorithm:** Directly maps to your `AlgorithmFactory` algorithms
- **attributes:** Configure your `CostCalculator` and planning parameters

### **3. Simulink Integration**

**Complete System Object Chain:**
```
┌─────────────────────┐    ┌─────────────────────┐    ┌─────────────────────┐
│ BehavioralPlanner   │───▶│ PathPlanning        │───▶│ PathFollowing       │
│ SystemObject        │    │ SystemObject        │    │ Controller          │
│ (Layer 6)           │    │ (Layer 5-1)         │    │                     │
└─────────────────────┘    └─────────────────────┘    └─────────────────────┘
         │                           │                           │
         ▼                           ▼                           ▼
   Route Segments            Detailed Paths              Control Commands
```

## 📊 Modified Data Flow

### **Enhanced Planning Workflow**

```matlab
% 1. Mission Level (Layer 6)
[next_goal, planning_trigger, segment_config, mission_status] = ...
    behavioral_planner.step(current_pose, current_speed, planning_complete, sim_time);

% 2. Path Planning Level (Layer 5-1)  
if planning_trigger
    % Configure your existing system
    planning_session.set_algorithm(segment_config.algorithm);
    
    % Execute planning with behavioral goal
    start_state = struct('x', current_x, 'y', current_y, 'theta', current_theta, 'gamma', current_gamma);
    goal_state = struct('x', next_goal(1), 'y', next_goal(2), 'theta', next_goal(3), 'gamma', next_goal(4));
    
    [path, success, stats] = planning_session.execute_single_planning(start_state, goal_state);
end

% 3. Control Level
[velocity_cmd, steering_cmd] = path_controller.step(current_pose, path);
```

## 🎛️ Configuration Adaptations

### **1. Algorithm Selection per Segment**

```matlab
% Behavioral planner adapts algorithm based on segment type
if attributes.turn_maneuver
    segment_config.algorithm = 'rrt';      % Better for tight maneuvers
else
    segment_config.algorithm = 'astar';    % Efficient for straight segments
end

if strcmp(segment.type, 'parking')
    segment_config.algorithm = 'rrt';      % Complex parking maneuvers
end
```

### **2. Parameter Adaptation**

```matlab
% Adapt planning parameters based on segment attributes
if attributes.turn_maneuver
    config.planning_tolerance = [0.5, 0.5, 10];  % Tighter tolerance
    config.connection_distance = 5.0;            % Shorter connections
else
    config.planning_tolerance = [1.0, 1.0, 15];  % Relaxed tolerance
    config.connection_distance = 10.0;           % Longer connections
end
```

### **3. Speed Profile Integration**

```matlab
% Speed profiles from behavioral planner
speed_config.max_speed = segment_attributes.max_speed;
speed_config.end_speed = segment_attributes.end_speed;

% Integrate with your CostCalculator
cost_weights.velocity_penalty = 1.0 / speed_config.max_speed;
```

## 🔄 Execution Flow

### **Step-by-Step Integration Process**

1. **Mission Initialization**
   ```matlab
   % Behavioral planner loads route plan
   route_plan = load_route_plan('scenarios/route_plan.json');
   planning_session = PlanningSession('scenarios/DPDScenario.json');
   ```

2. **Segment Execution Loop**
   ```matlab
   for each simulation step:
       % Layer 6: Get next segment goal
       [next_goal, trigger, config, status] = behavioral_planner.step(...);
       
       % Layer 5-1: Execute detailed planning when triggered
       if trigger
           [path, success] = planning_session.execute_single_planning(current_state, goal_state);
       end
       
       % Control: Follow generated path
       [v_cmd, steer_cmd] = controller.step(current_pose, path);
   end
   ```

3. **Segment Transition**
   ```matlab
   % Automatically advance when segment completed
   if segment_completed
       behavioral_planner.advance_to_next_segment();
       % Next iteration will plan for new segment
   end
   ```

## 🎯 Benefits of This Integration

### **1. Hierarchical Planning**
- **Mission-level reasoning:** Route sequencing and goal management
- **Tactical planning:** Your existing 5-layer detailed path planning
- **Clear separation:** Each layer handles appropriate time/space scales

### **2. Adaptive Behavior**
- **Algorithm switching:** Different algorithms for different segment types
- **Parameter adaptation:** Optimized settings per maneuver type
- **Speed management:** Segment-specific velocity profiles

### **3. Extensibility**
- **Easy route modification:** JSON-based route definition
- **New maneuver types:** Add new segment types and behaviors
- **Integration flexibility:** Works with your existing Simulink architecture

### **4. Real-world Applicability**
- **GPS-like navigation:** High-level route with detailed execution
- **Mission planning:** Complex multi-segment autonomous missions
- **Behavioral adaptation:** Different driving styles per road segment

## 🔧 Implementation Example

### **Complete Simulink Integration**

```matlab
% In your enhanced Simulink model:

% 1. Add BehavioralPlannerSystemObject
behavioral_planner = BehavioralPlannerSystemObject;
behavioral_planner.RoutePlanFile = 'scenarios/route_plan.json';

% 2. Connect to existing PathPlanningSystemObject
path_planner = PathPlanningSystemObject;

% 3. Signal flow:
% BehavioralPlanner outputs -> PathPlanner inputs
% next_goal -> start/goal states
% planning_trigger -> trigger_planning
% segment_config -> algorithm configuration
```

This integration creates a **complete autonomous driving planning stack** that bridges the gap between high-level mission planning and your detailed path planning algorithms, maintaining the modularity and extensibility of your existing 5-layer architecture while adding mission-level intelligence.

## 🎯 Usage Summary

Your enhanced system now supports:

1. **Route-based missions:** Load multi-segment routes from JSON
2. **Adaptive planning:** Algorithm and parameter selection per segment  
3. **Progress monitoring:** Real-time mission and segment progress tracking
4. **Seamless integration:** Works with all your existing components
5. **Simulink ready:** Complete System Object for real-time simulation

This transforms your path planning system from a **single-goal planner** into a **complete mission execution system** capable of handling complex autonomous driving scenarios.
