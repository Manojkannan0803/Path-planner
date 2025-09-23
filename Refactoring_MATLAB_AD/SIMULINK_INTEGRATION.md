# ADAS Path Planning Simulink Integration

## Overview

This document describes the complete Simulink integration strategy for the MATLAB-based ADAS path planning system. The integration enables real-time simulation, hardware-in-the-loop testing, and rapid prototyping for autonomous vehicle development.

## 🏗️ Architecture Design

### **Simulink System Architecture**

```
┌─────────────────────────────────────────────────────────────────┐
│                    SIMULINK MODEL ARCHITECTURE                  │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌──────────────────┐    ┌─────────────┐ │
│  │   INPUT LAYER   │    │   PROCESSING     │    │   OUTPUT    │ │
│  │                 │    │     LAYER        │    │   LAYER     │ │
│  │ • Start/Goal    │───▶│ • Path Planning  │───▶│ • Vehicle   │ │
│  │ • Obstacles     │    │ • Collision Det. │    │   Control   │ │
│  │ • Triggers      │    │ • Path Following │    │ • Status    │ │
│  │ • Parameters    │    │ • Vehicle Dyn.   │    │ • Logs      │ │
│  └─────────────────┘    └──────────────────┘    └─────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

### **MATLAB System Objects Created**

| System Object | Purpose | Layer Integration |
|--------------|---------|------------------|
| `PathPlanningSystemObject` | Main path planning orchestrator | Wraps entire MATLAB system |
| `CollisionDetectionSystemObject` | Real-time collision checking | Layer 3 Services |
| `VehicleDynamicsSystemObject` | Tractor-trailer kinematics | Physical simulation |
| `PathFollowingControllerSystemObject` | Path tracking control | Control layer |

---

## 🎯 System Objects Detail

### **1. PathPlanningSystemObject**

**Purpose:** Main Simulink interface to the entire path planning system

**Inputs:**
- `current_x, current_y, current_theta, current_gamma` - Current vehicle state
- `goal_x, goal_y, goal_theta, goal_gamma` - Goal state
- `trigger_planning` - Planning trigger signal
- `simulation_time` - Current simulation time

**Outputs:**
- `path_x, path_y, path_theta, path_gamma` - Planned path arrays (100 points max)
- `success` - Planning success flag
- `stats_out` - Planning statistics [iterations, nodes_explored, computation_time, path_length]

**Key Features:**
- Wraps entire `main_demo` functionality
- Configurable algorithm selection (A*, Dijkstra, RRT)
- Real-time path updates based on trigger or timer
- JSON configuration file support

**Configuration Properties:**
```matlab
ConfigurationFile = 'scenarios/DPDScenario.json'
EnableVisualization = false
PlanningAlgorithm = 'astar'
UpdateRate = 0.1  % seconds
```

### **2. CollisionDetectionSystemObject**

**Purpose:** Real-time collision detection for vehicle safety

**Inputs:**
- `vehicle_x, vehicle_y, vehicle_theta, vehicle_gamma` - Vehicle pose
- `obs_x_array, obs_y_array` - Obstacle coordinate arrays
- `obs_count` - Number of obstacles

**Outputs:**
- `is_safe` - Overall safety status
- `tractor_collision` - Tractor collision flag
- `trailer_collision` - Trailer collision flag

**Key Features:**
- Integrates `ObstacleChecker` service
- Multi-body collision detection (tractor + trailer)
- Configurable safety buffers
- Real-time polygon-based collision algorithms

**Configuration Properties:**
```matlab
VehicleLength = 6.0       % Tractor length (m)
VehicleWidth = 2.5        % Tractor width (m)
TrailerLength = 10.5      % Trailer length (m)
TrailerWidth = 2.5        % Trailer width (m)
HitchOffset = 3.0         % Hitch offset (m)
CollisionBuffer = 0.5     % Safety buffer (m)
```

### **3. VehicleDynamicsSystemObject**

**Purpose:** Kinematic simulation of tractor-trailer vehicle

**Inputs:**
- `velocity_cmd` - Velocity command (m/s)
- `steering_cmd` - Steering command (degrees)
- `x_init, y_init, theta_init, gamma_init` - Initial state
- `sim_time` - Simulation time

**Outputs:**
- `x_out, y_out, theta_out, gamma_out` - Current vehicle state
- `velocity_out` - Current velocity

**Key Features:**
- Kinematic model for articulated vehicles
- Realistic articulation dynamics
- Configurable vehicle parameters
- Integration with motion constraints

**Configuration Properties:**
```matlab
TractorLength = 6.0           % Tractor length (m)
MaxSteeringAngle = 30         % Max steering (degrees)
MaxArticulationAngle = 60     % Max articulation (degrees)
SampleTime = 0.1              % Integration time step
```

### **4. PathFollowingControllerSystemObject**

**Purpose:** Path tracking controller for autonomous navigation

**Inputs:**
- `current_x, current_y, current_theta` - Current vehicle pose
- `path_x, path_y, path_theta, path_gamma` - Reference path
- `path_valid` - Path validity flag
- `sim_time` - Simulation time

**Outputs:**
- `velocity_cmd` - Velocity command for vehicle
- `steering_cmd` - Steering command for vehicle
- `at_goal` - Goal reached flag
- `path_progress` - Path completion percentage

**Key Features:**
- Pure pursuit path following algorithm
- Adaptive velocity control
- Cross-track error minimization
- Goal reaching detection

**Configuration Properties:**
```matlab
LookaheadDistance = 5.0       % Lookahead distance (m)
MaxVelocity = 5.0             % Maximum velocity (m/s)
PositionTolerance = 1.0       % Goal tolerance (m)
VelocityGain = 1.0            % Velocity control gain
SteeringGain = 2.0            % Steering control gain
```

---

## 🔧 Integration Steps

### **Step 1: Setup MATLAB Environment**

```matlab
% Add paths to MATLAB system
addpath(genpath('Path-planner/Refactoring_MATLAB_AD'));

% Verify System Objects are accessible
which PathPlanningSystemObject
which CollisionDetectionSystemObject
which VehicleDynamicsSystemObject
which PathFollowingControllerSystemObject
```

### **Step 2: Create Simulink Model**

```matlab
% Run the model creation script
simulink.create_simulink_model;

% Or create manually using Simulink Library Browser:
% - Add MATLAB System blocks
% - Configure SystemObjectName property for each block
% - Connect inputs/outputs as shown in architecture
```

### **Step 3: Configure System Objects**

For each MATLAB System block:
1. Set `SystemObjectName` to appropriate class
2. Configure parameters in block dialog
3. Set sample times appropriately
4. Connect input/output signals

### **Step 4: Model Configuration**

```matlab
% Set solver parameters
set_param(model, 'SolverType', 'Fixed-step');
set_param(model, 'Solver', 'ode4');
set_param(model, 'FixedStep', '0.1');
set_param(model, 'StopTime', '100');

% Enable data logging
set_param(model, 'SaveOutput', 'on');
set_param(model, 'SaveState', 'on');
```

---

## 📊 Signal Flow Architecture

### **Main Control Loop**

```
Start/Goal ──┐
Obstacles ───┼──▶ Path Planning ──▶ Planned Path ──┐
Triggers ────┘                                     │
                                                   ▼
Vehicle State ◄── Vehicle Dynamics ◄── Controller ◄┘
     │                    ▲
     └──▶ Collision ──────┘
         Detection
```

### **Detailed Signal Connections**

| From Block | Signal | To Block | Purpose |
|------------|--------|----------|---------|
| Clock | time | All Systems | Synchronization |
| Start Position | [x,y,θ,γ] | Path Planning | Planning start |
| Goal Position | [x,y,θ,γ] | Path Planning | Planning goal |
| Path Planning | path arrays | Controller | Reference trajectory |
| Controller | [v_cmd, δ_cmd] | Vehicle Dynamics | Control commands |
| Vehicle Dynamics | [x,y,θ,γ] | Controller | State feedback |
| Vehicle Dynamics | [x,y,θ,γ] | Collision Detection | Safety monitoring |
| Obstacle Data | obstacle coords | Collision Detection | Environment data |

---

## 🚀 Usage Examples

### **Basic Simulation**

```matlab
% Load the created model
model_name = 'ADAS_PathPlanning_System';
load_system(model_name);

% Configure simulation parameters
set_param(model_name, 'StopTime', '50');

% Run simulation
simOut = sim(model_name);

% Access logged data
vehicle_states = simOut.logsout.getElement('vehicle_states').Values;
path_data = simOut.logsout.getElement('path_data').Values;

% Plot results
figure;
plot(vehicle_states.Data(:,1), vehicle_states.Data(:,2));
title('Vehicle Trajectory');
xlabel('X Position (m)');
ylabel('Y Position (m)');
```

### **Real-time Parameter Tuning**

```matlab
% Change planning algorithm during simulation
set_param([model_name '/Path Planning System'], ...
          'PlanningAlgorithm', 'dijkstra');

% Adjust controller parameters
set_param([model_name '/Path Following Controller'], ...
          'LookaheadDistance', '7.0');

% Update vehicle model
set_param([model_name '/Vehicle Dynamics'], ...
          'MaxSteeringAngle', '25');
```

### **Hardware-in-the-Loop Integration**

```matlab
% Configure for real-time execution
set_param(model_name, 'SimulationMode', 'external');

% Set up Real-Time Workshop
rtwbuild(model_name);

% Deploy to target hardware
% (Requires Simulink Real-Time or similar)
```

---

## 🎛️ Advanced Features

### **1. Multi-Scenario Testing**

```matlab
% Batch testing with different scenarios
scenarios = {'DPDScenario.json', 'WarehouseScenario.json', 'HighwayScenario.json'};

for i = 1:length(scenarios)
    set_param([model_name '/Path Planning System'], ...
              'ConfigurationFile', scenarios{i});
    
    simOut = sim(model_name);
    results(i) = analyze_simulation_results(simOut);
end
```

### **2. Monte Carlo Analysis**

```matlab
% Statistical analysis with random obstacles
num_runs = 100;
success_rate = zeros(num_runs, 1);

for run = 1:num_runs
    % Generate random obstacle configuration
    obstacles = generate_random_obstacles();
    
    % Update model
    set_param([model_name '/Obstacle Data'], 'Value', mat2str(obstacles));
    
    % Run simulation
    simOut = sim(model_name);
    success_rate(run) = evaluate_success(simOut);
end

fprintf('Success Rate: %.1f%%\n', mean(success_rate) * 100);
```

### **3. Real-time Visualization**

```matlab
% Enable real-time plotting during simulation
set_param([model_name '/Path Planning System'], ...
          'EnableVisualization', 'true');

% Use Simulink Dashboard for real-time monitoring
% Add gauges, sliders, and plots for interactive control
```

---

## 🔍 Debugging and Validation

### **Common Issues and Solutions**

| Issue | Symptoms | Solution |
|-------|----------|----------|
| System Object not found | "Cannot find class" error | Check MATLAB path, verify file location |
| Simulation too slow | Real-time factor < 1 | Increase fixed step size, disable visualization |
| Path planning fails | Empty path output | Check start/goal validity, obstacle configuration |
| Vehicle instability | Oscillating motion | Reduce controller gains, check vehicle parameters |

### **Validation Checklist**

- [ ] All System Objects compile without errors
- [ ] Signal dimensions match between blocks
- [ ] Sample times are consistent
- [ ] Initial conditions are properly set
- [ ] Obstacle data format is correct
- [ ] Path planning triggers at expected times
- [ ] Vehicle dynamics are realistic
- [ ] Controller tracks path smoothly
- [ ] Collision detection works correctly

---

## 📈 Performance Optimization

### **Real-time Performance Tips**

1. **Fixed-point arithmetic**: Convert to fixed-point for embedded targets
2. **Code generation**: Use Simulink Coder for optimized C code
3. **Vectorization**: Minimize loops in System Object step functions
4. **Memory management**: Pre-allocate arrays in setupImpl
5. **Parallel execution**: Use Parallel Computing Toolbox for batch runs

### **Simulation Speed Optimization**

```matlab
% Optimize solver settings
set_param(model, 'Solver', 'ode1');  % Euler method for speed
set_param(model, 'FixedStep', '0.2'); % Larger time step if stable

% Disable unnecessary logging
set_param(model, 'DSMLogging', 'off');

% Use compiled algorithms
set_param(model, 'SimulationMode', 'accelerator');
```

---

## 🎯 Benefits of Simulink Integration

### **1. Real-time Simulation**
- Continuous-time vehicle dynamics
- Real-time path planning updates
- Hardware timing validation

### **2. Rapid Prototyping**
- Visual block-based development
- Parameter tuning during simulation
- Interactive dashboard controls

### **3. Hardware Integration**
- Code generation for embedded systems
- Hardware-in-the-loop testing
- Real-time target deployment

### **4. Validation & Testing**
- Systematic scenario testing
- Monte Carlo analysis
- Performance benchmarking

### **5. Visualization**
- Real-time plotting
- 3D vehicle animation
- Dashboard monitoring

This Simulink integration transforms your MATLAB path planning system into a comprehensive simulation and validation environment for ADAS development, enabling seamless transition from algorithm development to real-world deployment.
