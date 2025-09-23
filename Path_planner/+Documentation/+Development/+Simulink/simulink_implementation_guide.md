# Simulink Block Implementation Guide

**Date:** 2025-09-23  
**Target:** MATLAB/Simulink Developer  
**Approach:** Simulink blocks containing MATLAB function/class implementations

---

## 1. Simulink Block Architecture Overview

### **Top-Level Model: `AutonomousValetParking.slx`**
```
┌─────────────────────────────────────────────────────────────────┐
│                    AutonomousValetParking.slx                   │
├─────────────────────────────────────────────────────────────────┤
│  [ScenarioLoader] → [GlobalPlanner] → [LocalPathAdapter]       │
│       ↓                   ↓                    ↓               │
│  [MapBus]            [PathBus]           [LocalBandBus]        │
│                           ↓                    ↓               │
│  [VehicleState] → [TrajectoryGenerator] → [SafetyMonitor]      │
│       ↓                   ↓                    ↓               │
│  [SensorBus]        [TrajectoryBus]     [SafetyFlagBus]       │
│                           ↓                    ↓               │
│                    [Controller] ←── [Logger]                   │
│                           ↓                                    │
│                   [ActuatorCmdBus]                             │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Simulink Block Specifications

### **Block 1: ScenarioLoader**
**Type:** MATLAB Function Block  
**Execution:** Initialize (one-time)  
**MATLAB Implementation:** `ScenarioLoader.m` class

```matlab
function [mapBus, vehicleParams] = fcn_ScenarioLoader(scenarioFile)
    persistent loader
    if isempty(loader)
        loader = ScenarioLoader(scenarioFile);
    end
    [mapBus, vehicleParams] = loader.loadScenario();
end
```

**Internal Class:** `ScenarioLoader.m`
```matlab
classdef ScenarioLoader < handle
    properties (Access = private)
        configFile
        mapData
        vehicleConfig
    end
    
    methods
        function obj = ScenarioLoader(configFile)
            obj.configFile = configFile;
            obj.loadConfiguration();
        end
        
        function [mapBus, vehicleParams] = loadScenario(obj)
            % Load YAML config, create MapBus structure
            % Return structured bus objects
        end
    end
end
```

### **Block 2: GlobalPlanner**
**Type:** MATLAB Function Block  
**Execution:** Triggered/1Hz  
**MATLAB Implementation:** `HybridAStar.m` class

```matlab
function [pathBus, statsOut] = fcn_GlobalPlanner(mapBus, startPose, goalPose, ...
                                                 primitiveLib, plannerOptions, trigger)
    persistent planner
    if isempty(planner)
        planner = HybridAStar(primitiveLib, plannerOptions);
    end
    
    if trigger > 0
        [pathBus, statsOut] = planner.planPath(mapBus, startPose, goalPose);
    else
        pathBus = getEmptyPathBus();
        statsOut = getEmptyStatsBus();
    end
end
```

**Internal Class:** `HybridAStar.m`
```matlab
classdef HybridAStar < handle
    properties (Access = private)
        primitiveLib
        options
        openList      % PriorityQueue instance
        closedHash    % StateHash instance
        heuristic     % HeuristicCalculator instance
    end
    
    methods
        function obj = HybridAStar(primitiveLib, options)
            obj.primitiveLib = primitiveLib;
            obj.options = options;
            obj.openList = PriorityQueue(options.maxNodes);
            obj.closedHash = StateHash(options.resolution);
            obj.heuristic = HeuristicCalculator(options.heuristicType);
        end
        
        function [pathBus, stats] = planPath(obj, mapBus, startPose, goalPose)
            % Main A* search implementation
            % Returns PathBus with poses, directions, cost
        end
    end
end
```

### **Block 3: LocalPathAdapter**
**Type:** MATLAB Function Block  
**Execution:** 10-20Hz  
**MATLAB Implementation:** `ElasticBand.m` class

```matlab
function localBandBus = fcn_LocalPathAdapter(pathBus, localCostmap, dynObstacles, vehicleState)
    persistent adapter
    if isempty(adapter)
        adapter = ElasticBand();
    end
    
    localBandBus = adapter.adaptPath(pathBus, localCostmap, dynObstacles, vehicleState);
end
```

### **Block 4: TrajectoryGenerator**
**Type:** MATLAB Function Block  
**Execution:** 20-50Hz  
**MATLAB Implementation:** `SCurveGenerator.m` class

```matlab
function trajectoryBus = fcn_TrajectoryGenerator(localBandBus, vehicleParams, limits)
    persistent generator
    if isempty generator
        generator = SCurveGenerator(vehicleParams, limits);
    end
    
    trajectoryBus = generator.generateTrajectory(localBandBus);
end
```

### **Block 5: SafetyMonitor**
**Type:** MATLAB Function Block  
**Execution:** 50-100Hz  
**MATLAB Implementation:** `CollisionChecker.m` class

```matlab
function safetyFlagBus = fcn_SafetyMonitor(trajectoryBus, localCostmap, vehicleParams)
    persistent monitor
    if isempty(monitor)
        monitor = CollisionChecker(vehicleParams);
    end
    
    safetyFlagBus = monitor.checkSafety(trajectoryBus, localCostmap);
end
```

---

## 3. Supporting MATLAB Classes

### **Class 1: PriorityQueue.m**
```matlab
classdef PriorityQueue < handle
    % Binary heap implementation for open list management
    properties (Access = private)
        heap
        size
        capacity
    end
    
    methods
        function obj = PriorityQueue(maxSize)
            obj.capacity = maxSize;
            obj.heap = zeros(maxSize, 3); % [f_cost, g_cost, node_id]
            obj.size = 0;
        end
        
        function push(obj, node, f_cost, g_cost)
            % Insert with heap property maintenance
        end
        
        function [node, f_cost] = pop(obj)
            % Extract minimum f_cost node
        end
        
        function isEmpty = empty(obj)
            isEmpty = (obj.size == 0);
        end
    end
end
```

### **Class 2: StateHash.m**
```matlab
classdef StateHash < handle
    % Discretized state hashing for closed list
    properties (Access = private)
        hashTable
        resolution
        thetaBins
    end
    
    methods
        function obj = StateHash(resolution)
            obj.resolution = resolution;
            obj.thetaBins = 360 / 7.5; % 48 bins
            obj.hashTable = containers.Map();
        end
        
        function key = getKey(obj, x, y, theta)
            x_disc = round(x / obj.resolution);
            y_disc = round(y / obj.resolution);
            theta_disc = round(theta / 7.5);
            key = sprintf('%d_%d_%d', x_disc, y_disc, theta_disc);
        end
        
        function exists = hasState(obj, x, y, theta)
            key = obj.getKey(x, y, theta);
            exists = obj.hashTable.isKey(key);
        end
        
        function addState(obj, x, y, theta, g_cost)
            key = obj.getKey(x, y, theta);
            obj.hashTable(key) = g_cost;
        end
    end
end
```

### **Class 3: VehicleFootprint.m**
```matlab
classdef VehicleFootprint < handle
    % Vehicle collision boundary management
    properties (Access = private)
        length
        width
        wheelbase
        corners % [4 x 2] corner coordinates
    end
    
    methods
        function obj = VehicleFootprint(vehicleParams)
            obj.length = vehicleParams.length;
            obj.width = vehicleParams.width;
            obj.wheelbase = vehicleParams.wheelbase;
            obj.updateCorners(0, 0, 0); % Initialize at origin
        end
        
        function updateCorners(obj, x, y, theta)
            % Update corner positions for given pose
            half_length = obj.length / 2;
            half_width = obj.width / 2;
            
            % Local corners
            local_corners = [-half_length, -half_width;
                            half_length, -half_width;
                            half_length, half_width;
                            -half_length, half_width];
            
            % Rotate and translate
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            obj.corners = (R * local_corners')' + [x, y];
        end
        
        function inCollision = checkCollision(obj, x, y, theta, obstacles)
            obj.updateCorners(x, y, theta);
            inCollision = false;
            
            for i = 1:length(obstacles)
                if obj.polygonIntersection(obj.corners, obstacles{i})
                    inCollision = true;
                    return;
                end
            end
        end
    end
end
```

---

## 4. Bus Object Definitions

### **Create Bus Objects Script: `defineBusObjects.m`**
```matlab
function defineBusObjects()
    % Run this script once to define all bus objects in workspace
    
    % MapBus definition
    clear elems;
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'staticObstacles';
    elems(1).DataType = 'Bus: ObstacleBus';
    
    elems(2) = Simulink.BusElement;
    elems(2).Name = 'parkingSlots';
    elems(2).DataType = 'Bus: ParkingSlotBus';
    
    elems(3) = Simulink.BusElement;
    elems(3).Name = 'timestamp';
    elems(3).DataType = 'double';
    
    MapBus = Simulink.Bus;
    MapBus.Elements = elems;
    assignin('base', 'MapBus', MapBus);
    
    % PathBus definition
    clear elems;
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'poses';
    elems(1).DataType = 'Bus: PoseBus';
    elems(1).Dimensions = [100 1]; % Max 100 waypoints
    
    elems(2) = Simulink.BusElement;
    elems(2).Name = 'directions';
    elems(2).DataType = 'int8';
    elems(2).Dimensions = [100 1]; % 1=forward, -1=reverse
    
    elems(3) = Simulink.BusElement;
    elems(3).Name = 'length';
    elems(3).DataType = 'uint16';
    
    elems(4) = Simulink.BusElement;
    elems(4).Name = 'cost_total';
    elems(4).DataType = 'double';
    
    PathBus = Simulink.Bus;
    PathBus.Elements = elems;
    assignin('base', 'PathBus', PathBus);
    
    % Continue for other bus types...
end
```

---

## 5. Simulink Model Configuration

### **Model Settings for Code Generation**
```matlab
% Model Configuration Parameters
set_param(gcs, 'Solver', 'FixedStepDiscrete');
set_param(gcs, 'FixedStep', '0.02'); % 50Hz base rate
set_param(gcs, 'StartTime', '0.0');
set_param(gcs, 'StopTime', '60.0');

% Code Generation Settings
set_param(gcs, 'RTWSystemTargetFile', 'ert.tlc');
set_param(gcs, 'GenCodeOnly', 'off');
set_param(gcs, 'MakeCommand', 'make_rtw');

% Optimization Settings
set_param(gcs, 'DefaultParameterBehavior', 'Tunable');
set_param(gcs, 'OptimizeBlockIOStorage', 'on');
set_param(gcs, 'BufferReuse', 'on');
```

### **Block Sample Times**
```matlab
% ScenarioLoader: Triggered (startup only)
% GlobalPlanner: 1.0 (1Hz) or triggered
% LocalPathAdapter: 0.05 (20Hz)
% TrajectoryGenerator: 0.02 (50Hz)  
% SafetyMonitor: 0.01 (100Hz)
% Logger: 0.5 (2Hz)
```

---

## 6. Implementation Sequence for Developer

### **Week 1-2: Infrastructure**
1. **Create bus object definitions** (`defineBusObjects.m`)
2. **Set up basic Simulink model** with empty function blocks
3. **Implement ScenarioLoader** class and block
4. **Create VehicleFootprint** class for collision detection

### **Week 3-4: Core Planning**
5. **Implement PriorityQueue** and **StateHash** classes
6. **Create HybridAStar** class with basic Euclidean heuristic
7. **Integrate GlobalPlanner** function block
8. **Add basic collision checking**

### **Week 5-6: Trajectory & Safety**
9. **Implement SCurveGenerator** class
10. **Create TrajectoryGenerator** function block
11. **Add CollisionChecker** class and SafetyMonitor block
12. **Implement ElasticBand** for LocalPathAdapter

### **Week 7-9: Integration & Testing**
13. **Connect all blocks with proper bus interfaces**
14. **Add logging and diagnostics**
15. **Create test scenarios and validation suite**
16. **Optimize for code generation**

---

## 7. Testing Strategy per Block

### **Unit Testing Approach**
```matlab
% Test each MATLAB class independently
testRunner = matlab.unittest.TestRunner.withTextOutput();
testSuite = matlab.unittest.TestSuite.fromClass(?HybridAStarTest);
results = testRunner.run(testSuite);

% Test each Simulink block with synthetic bus data
sim('test_GlobalPlanner_block.slx');
```

### **Integration Testing**
```matlab
% Full model simulation with predefined scenarios
load('test_scenario_01.mat'); % Contains MapBus, start/goal poses
simOut = sim('AutonomousValetParking.slx', 'LoadExternalInput', 'on');
validateResults(simOut);
```

---

## 8. Code Generation Considerations

### **Fixed-Size Arrays**
```matlab
% In all MATLAB functions, use fixed-size arrays
function pathBus = fcn_GlobalPlanner(...)
    % Pre-allocate fixed size
    maxWaypoints = 100;
    pathBus.poses = zeros(maxWaypoints, 3); % [x, y, theta]
    pathBus.directions = zeros(maxWaypoints, 1, 'int8');
    % ... populate with actual path data
    pathBus.length = actualLength; % Track actual used length
end
```

### **Memory Management**
```matlab
% Use persistent objects to avoid recreation
function result = fcn_MyBlock(input)
    persistent myProcessor
    if isempty(myProcessor)
        myProcessor = MyProcessorClass();
        myProcessor.preallocateMemory(1000); % Fixed size allocation
    end
    result = myProcessor.process(input);
end
```

---

**✅ This guide provides the complete Simulink+MATLAB implementation approach the developer needs!**

The key insight is that **Simulink provides the architecture and data flow, while MATLAB classes provide the algorithmic implementation** - exactly as you described! 🎯