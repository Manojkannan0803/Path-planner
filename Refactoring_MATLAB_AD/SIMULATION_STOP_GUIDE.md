# Simulation Stop Checker - Complete Guide

## Overview

The `SimulationStopChecker` is a service in Layer 3 of your path planning architecture that provides intelligent stopping conditions for autonomous driving simulations. It determines when to stop the simulation based on multiple criteria including goal achievement, timeout conditions, planning failures, and emergency situations.

## 🏗️ Architecture Integration

### **Layer 3 Service Integration**
```
┌─────────────────────────────────────────────────────────┐
│  LAYER 5: APPLICATIONS                                  │
│  - PlanningSession uses SimulationStopChecker          │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│  LAYER 4: ALGORITHMS                                    │
│  - Algorithm results fed to SimulationStopChecker      │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│  LAYER 3: SERVICES                                     │
│  ✓ SimulationStopChecker (NEW)                         │
│  - ObstacleChecker (collision detection)               │
│  - CostCalculator, HeuristicCalculator                 │
└─────────────────────────────────────────────────────────┘
```

## 🎯 Key Features

### **1. Multiple Stopping Conditions**
- **Goal Achievement**: Checks position, orientation, and articulation tolerances
- **Simulation Timeout**: Prevents infinite simulation loops
- **Planning Failure**: Detects when path planning consistently fails
- **Path Completion**: Recognizes when planned path is fully executed
- **Collision Warning**: Emergency stop for imminent collision detection

### **2. Configurable Tolerances**
- **Position Tolerance**: Distance error acceptance (default: 2.0m)
- **Orientation Tolerance**: Angular error acceptance (default: 15°)
- **Articulation Tolerance**: Gamma angle error acceptance (default: 10°)

### **3. Adaptive Behavior**
- **Tractor-Trailer Aware**: Handles complex vehicle dynamics
- **Planning Context**: Integrates with your existing algorithms
- **Statistics Tracking**: Monitors simulation performance
- **Debug Mode**: Detailed logging for development

## 📁 File Structure

```
+services/
└── +validation/
    └── SimulationStopChecker.m     ← Main service class

Root Directory:
├── simulation_stop_demo.m          ← Basic usage examples  
├── integration_example.m           ← Integration patterns
└── SIMULATION_STOP_GUIDE.md        ← This documentation
```

## 🚀 Quick Start

### **Basic Usage**

```matlab
% 1. Create stop checker with default configuration
stop_checker = services.validation.SimulationStopChecker();

% 2. Define states
current_state = struct('x', 60.1, 'y', 64.1, 'theta', 271, 'gamma', -1);
goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);

% 3. Check stopping conditions
[should_stop, stop_reason, stop_details] = stop_checker.check_stopping_conditions(...
    current_state, goal_state, simulation_time);

% 4. Handle result
if should_stop
    fprintf('Simulation stopped: %s\n', stop_reason);
    if strcmp(stop_reason, 'goal_reached')
        fprintf('SUCCESS! Position error: %.3fm\n', stop_details.position_error);
    end
end
```

### **Custom Configuration**

```matlab
% Configure for your specific requirements
config = struct(...
    'goal_tolerance', struct(...
        'position_tolerance', 1.0, ...      % Tight position tolerance
        'orientation_tolerance', 5.0, ...   % Tight orientation tolerance
        'articulation_tolerance', 3.0), ... % Tight articulation tolerance
    'max_simulation_time', 120.0, ...      % 2 minute timeout
    'max_planning_attempts', 3, ...        % Max replanning attempts
    'debug_mode', true);                   % Enable detailed logging

stop_checker = services.validation.SimulationStopChecker(config);
```

## 🔧 Integration Patterns

### **1. PlanningSession Integration**

```matlab
% In your planning loop
function execute_planning_with_stop_check()
    session = PlanningSession('scenarios/DPDScenario.json');
    stop_checker = services.validation.SimulationStopChecker();
    
    start_state = struct('x', 75, 'y', 45, 'theta', 180, 'gamma', 0);
    goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
    
    current_state = start_state;
    simulation_time = 0;
    dt = 0.5;
    
    while true
        simulation_time = simulation_time + dt;
        
        % Execute path planning
        [path, success, stats] = session.execute_single_planning(current_state, goal_state);
        
        % Prepare planning result for stop checker
        planning_result = struct('success', success, 'path', path, 'stats', stats);
        
        % Check stopping conditions
        [should_stop, stop_reason, stop_details] = stop_checker.check_stopping_conditions(...
            current_state, goal_state, simulation_time, planning_result);
        
        if should_stop
            fprintf('Planning completed: %s\n', stop_reason);
            break;
        end
        
        % Update vehicle state (simplified)
        if success
            current_state = update_vehicle_state(current_state, path);
        end
    end
end
```

### **2. Simulink System Object Integration**

```matlab
classdef PathPlanningSystemObject < matlab.System
    properties (Access = private)
        planning_session
        stop_checker
        consecutive_failures
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            % Initialize components
            obj.planning_session = PlanningSession('scenarios/DPDScenario.json');
            
            stop_config = struct(...
                'goal_tolerance', struct('position_tolerance', 1.5, ...
                                       'orientation_tolerance', 10.0), ...
                'max_simulation_time', 180.0);
            obj.stop_checker = services.validation.SimulationStopChecker(stop_config);
            obj.consecutive_failures = 0;
        end
        
        function [stop_simulation, stop_reason] = stepImpl(obj, current_pose, goal_pose, sim_time)
            % Execute planning
            [path, success, stats] = obj.planning_session.execute_single_planning(...
                current_pose, goal_pose);
            
            % Track failures
            if ~success
                obj.consecutive_failures = obj.consecutive_failures + 1;
            else
                obj.consecutive_failures = 0;
            end
            
            % Prepare additional info
            additional_info = struct('consecutive_failures', obj.consecutive_failures);
            planning_result = struct('success', success, 'path', path, 'stats', stats);
            
            % Check stopping conditions
            [stop_simulation, stop_reason, ~] = obj.stop_checker.check_stopping_conditions(...
                current_pose, goal_pose, sim_time, planning_result, additional_info);
        end
    end
end
```

### **3. Behavioral Planner Integration**

```matlab
% In BehavioralPlannerSystemObject
function [mission_complete, completion_reason] = check_mission_completion(obj, current_segment, vehicle_state, sim_time)
    
    % Get current segment goal
    segment_goal = obj.route_plan.segments(current_segment).end_pose;
    goal_state = struct('x', segment_goal(1), 'y', segment_goal(2), ...
                       'theta', segment_goal(3), 'gamma', segment_goal(4));
    
    % Check if segment is complete
    [segment_complete, reason, details] = obj.stop_checker.check_stopping_conditions(...
        vehicle_state, goal_state, sim_time);
    
    if segment_complete && strcmp(reason, 'goal_reached')
        % Advance to next segment
        obj.current_segment_index = obj.current_segment_index + 1;
        
        % Check if mission complete
        if obj.current_segment_index > length(obj.route_plan.segments)
            mission_complete = true;
            completion_reason = 'mission_accomplished';
        else
            mission_complete = false;
            completion_reason = 'segment_complete';
        end
    else
        mission_complete = segment_complete;
        completion_reason = reason;
    end
end
```

## 📊 Stopping Conditions Details

### **1. Goal Achievement**

**Condition**: All tolerances satisfied simultaneously
```matlab
position_error <= position_tolerance AND
orientation_error <= orientation_tolerance AND
articulation_error <= articulation_tolerance
```

**Typical Values**:
- **Parking scenarios**: 0.5m, 5°, 3°
- **Highway scenarios**: 2.0m, 15°, 10°
- **Intersection scenarios**: 1.0m, 8°, 5°

### **2. Simulation Timeout**

**Condition**: `simulation_time >= max_simulation_time`

**Use Cases**:
- Prevent infinite loops in complex scenarios
- Ensure simulation completion within time bounds
- Testing timeout handling

### **3. Planning Failure**

**Conditions**:
- `consecutive_failures >= max_planning_attempts`
- Unrecoverable failure types: `'invalid_start_state'`, `'invalid_goal_state'`, `'no_valid_path'`

**Triggers**:
- Algorithm cannot find valid path
- Start or goal state in collision
- Kinematic constraints violated

### **4. Path Completion**

**Conditions**:
- Path following controller reports completion
- Vehicle reaches end of planned trajectory
- Path completion percentage >= 100%

### **5. Collision Warning**

**Conditions**:
- `collision_warning = true` in additional_info
- `obstacle_distance < safety_threshold`
- Emergency stop triggered

## 🎛️ Configuration Options

### **Goal Tolerance Configuration**

```matlab
goal_tolerance = struct(...
    'position_tolerance', 1.5, ...      % meters
    'orientation_tolerance', 10.0, ...  % degrees
    'articulation_tolerance', 8.0);     % degrees (gamma angle)
```

### **Stopping Conditions Enable/Disable**

```matlab
stopping_conditions = struct(...
    'enable_goal_reached', true, ...        % Primary success condition
    'enable_timeout', true, ...             % Safety timeout
    'enable_planning_failure', true, ...    % Algorithm failure detection
    'enable_collision_imminent', false, ... % Emergency collision stop
    'enable_path_completion', true);        % Path following completion
```

### **Simulation Limits**

```matlab
limits = struct(...
    'max_simulation_time', 300.0, ...   % Maximum simulation duration (seconds)
    'max_planning_attempts', 5);        % Max replanning before failure
```

## 📈 Statistics and Monitoring

### **Available Statistics**

```matlab
stats = stop_checker.get_simulation_statistics();

% Statistics structure:
% stats.total_checks          - Number of condition checks performed
% stats.goal_achievements     - Number of successful goal achievements
% stats.timeouts             - Number of timeout conditions
% stats.planning_failures    - Number of planning failure detections
% stats.start_time          - First check timestamp
% stats.last_check_time     - Most recent check timestamp
```

### **Performance Monitoring**

```matlab
% Real-time monitoring
last_result = stop_checker.last_check_result;
fprintf('Last check: %s at %.1fs\n', last_result.stop_reason, last_result.check_time);

% Reset statistics for new simulation
stop_checker.reset_statistics();
```

## 🔍 Debug and Troubleshooting

### **Enable Debug Mode**

```matlab
stop_checker.set_debug_mode(true);

% Debug output examples:
% [SimStopChecker] Goal reached! Position error: 0.15m, Orientation error: 3.2°
% [SimStopChecker] Simulation timeout reached: 305.2s
% [SimStopChecker] Planning failure detected: max_attempts_exceeded
```

### **Common Issues and Solutions**

#### **Issue**: Goal never reached despite vehicle being close
**Solution**: Check tolerance values - they might be too tight
```matlab
tolerance = stop_checker.get_goal_tolerance();
fprintf('Current tolerances: %.2fm, %.1f°, %.1f°\n', ...
    tolerance.position_tolerance, tolerance.orientation_tolerance, tolerance.articulation_tolerance);
```

#### **Issue**: Simulation times out frequently  
**Solution**: Increase timeout or optimize planning performance
```matlab
stop_checker.set_max_simulation_time(600.0); % Increase to 10 minutes
```

#### **Issue**: Planning failures not detected
**Solution**: Ensure planning results are properly formatted
```matlab
planning_result = struct(...
    'success', false, ...
    'path', [], ...
    'stats', struct('termination_reason', 'no_feasible_path'));
```

## 🎯 Usage Examples

### **Example 1: Tight Parking Scenario**

```matlab
% Parking requires very precise positioning
parking_config = struct(...
    'goal_tolerance', struct(...
        'position_tolerance', 0.3, ...      % Very tight: 30cm
        'orientation_tolerance', 3.0, ...   % Very tight: 3°
        'articulation_tolerance', 2.0), ... % Very tight: 2°
    'max_simulation_time', 90.0);          % Parking can take time

parking_checker = services.validation.SimulationStopChecker(parking_config);
```

### **Example 2: Highway Merging**

```matlab
% Highway scenarios allow more tolerance but need timeout protection
highway_config = struct(...
    'goal_tolerance', struct(...
        'position_tolerance', 3.0, ...      % Relaxed: 3m
        'orientation_tolerance', 20.0, ...  % Relaxed: 20°
        'articulation_tolerance', 15.0), ... % Relaxed: 15°
    'max_simulation_time', 45.0);          % Quick merging required

highway_checker = services.validation.SimulationStopChecker(highway_config);
```

### **Example 3: Complex Maneuvering**

```matlab
% Complex maneuvers need collision detection and failure recovery
complex_config = struct(...
    'goal_tolerance', struct(...
        'position_tolerance', 1.0, ...
        'orientation_tolerance', 8.0, ...
        'articulation_tolerance', 6.0), ...
    'stopping_conditions', struct(...
        'enable_collision_imminent', true, ...  % Enable collision detection
        'enable_planning_failure', true), ...
    'max_planning_attempts', 3, ...            % Allow replanning
    'max_simulation_time', 180.0);

complex_checker = services.validation.SimulationStopChecker(complex_config);
```

## 🔄 Complete Simulation Loop

```matlab
function run_complete_simulation()
    % Complete example of simulation with stop checking
    
    % Setup
    session = PlanningSession('scenarios/DPDScenario.json');
    stop_checker = services.validation.SimulationStopChecker();
    
    start_state = struct('x', 75, 'y', 45, 'theta', 180, 'gamma', 0);
    goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
    
    current_state = start_state;
    simulation_time = 0;
    dt = 0.5;
    consecutive_failures = 0;
    
    fprintf('Starting simulation: (%.1f,%.1f) → (%.1f,%.1f)\n', ...
        start_state.x, start_state.y, goal_state.x, goal_state.y);
    
    % Main simulation loop
    while true
        simulation_time = simulation_time + dt;
        
        % Execute path planning
        [path, success, stats] = session.execute_single_planning(current_state, goal_state);
        
        % Track planning failures
        if ~success
            consecutive_failures = consecutive_failures + 1;
        else
            consecutive_failures = 0;
        end
        
        % Prepare information for stop checker
        planning_result = struct('success', success, 'path', path, 'stats', stats);
        additional_info = struct('consecutive_failures', consecutive_failures);
        
        % Check stopping conditions
        [should_stop, stop_reason, stop_details] = stop_checker.check_stopping_conditions(...
            current_state, goal_state, simulation_time, planning_result, additional_info);
        
        % Log progress
        if mod(simulation_time * 2, 10) == 0 % Every 5 seconds
            fprintf('Time: %5.1fs, Position: (%5.1f, %5.1f), Status: %s\n', ...
                simulation_time, current_state.x, current_state.y, ...
                should_stop ? stop_reason : 'Running');
        end
        
        % Handle stopping condition
        if should_stop
            fprintf('\n=== SIMULATION COMPLETED ===\n');
            fprintf('Stop reason: %s\n', stop_reason);
            fprintf('Total time: %.1fs\n', simulation_time);
            
            switch stop_reason
                case 'goal_reached'
                    fprintf('SUCCESS: Goal achieved!\n');
                    fprintf('Final errors: Position %.3fm, Orientation %.1f°\n', ...
                        stop_details.position_error, stop_details.orientation_error);
                case 'simulation_timeout'
                    fprintf('TIMEOUT: Simulation exceeded time limit\n');
                case 'planning_failure'
                    fprintf('FAILURE: Planning failed (%s)\n', stop_details.failure_type);
                otherwise
                    fprintf('STOPPED: %s\n', stop_reason);
            end
            break;
        end
        
        % Update vehicle state (simplified vehicle model)
        if success && ~isempty(path.x)
            % Move along planned path
            current_state = update_vehicle_position(current_state, goal_state, dt);
        end
    end
    
    % Display final statistics
    final_stats = stop_checker.get_simulation_statistics();
    fprintf('\nSimulation Statistics:\n');
    fprintf('Total condition checks: %d\n', final_stats.total_checks);
    fprintf('Planning attempts: %d\n', consecutive_failures + (success ? 1 : 0));
end

function new_state = update_vehicle_position(current_state, goal_state, dt)
    % Simple vehicle model for demonstration
    speed = 2.0; % m/s
    
    % Calculate direction to goal
    dx = goal_state.x - current_state.x;
    dy = goal_state.y - current_state.y;
    distance = sqrt(dx^2 + dy^2);
    
    if distance > 0.1
        % Move toward goal
        move_distance = min(speed * dt, distance);
        new_state = current_state;
        new_state.x = current_state.x + (dx/distance) * move_distance;
        new_state.y = current_state.y + (dy/distance) * move_distance;
        
        % Gradually adjust orientation
        target_theta = atan2d(dy, dx);
        theta_diff = target_theta - current_state.theta;
        theta_diff = mod(theta_diff + 180, 360) - 180; % Normalize to [-180, 180]
        new_state.theta = current_state.theta + sign(theta_diff) * min(abs(theta_diff), 10 * dt);
        
        % Gradually adjust articulation
        new_state.gamma = current_state.gamma + (goal_state.gamma - current_state.gamma) * 0.1 * dt;
    else
        new_state = current_state;
    end
end
```

## 📝 Best Practices

1. **Configure for Scenario**: Adjust tolerances based on maneuver complexity
2. **Enable Debug Mode**: Use during development for detailed insights
3. **Monitor Statistics**: Track performance across multiple simulations
4. **Handle All Stop Reasons**: Implement appropriate responses for each condition
5. **Test Edge Cases**: Verify behavior with extreme tolerance values
6. **Integration Testing**: Test with actual PlanningSession and algorithms

## 🚀 Integration with Your System

The `SimulationStopChecker` seamlessly integrates with your existing 5-layer architecture:

- **Layer 5**: `PlanningSession` uses it to manage simulation lifecycle
- **Layer 4**: Algorithm results feed into stopping decision logic  
- **Layer 3**: Works alongside `ObstacleChecker` and other services
- **Simulink**: Perfect for System Object real-time simulation
- **Behavioral Planning**: Enables hierarchical goal management

This creates a complete, intelligent simulation framework that knows when to stop based on mission success, safety conditions, and system performance.

---

**File Location**: `+services/+validation/SimulationStopChecker.m`  
**Dependencies**: None (standalone service)  
**Compatibility**: Full integration with existing path planning architecture  
**Version**: 1.0 - Initial release with comprehensive stopping conditions
