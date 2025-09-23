# Simulink Model Creation Instructions

**File:** `AutonomousValetParking.slx` (To be created manually)  
**Developer Action Required:** Please create this Simulink model manually  
**Priority:** Phase 1A Foundation  

---

## Model Configuration Settings

### **1. Create New Simulink Model**
```matlab
% Run these commands in MATLAB to create and configure the model
new_system('AutonomousValetParking');
open_system('AutonomousValetParking');

% Configure model settings
set_param('AutonomousValetParking', 'Solver', 'FixedStepDiscrete');
set_param('AutonomousValetParking', 'FixedStep', '0.02');  % 50Hz base rate
set_param('AutonomousValetParking', 'StartTime', '0.0');
set_param('AutonomousValetParking', 'StopTime', '60.0');

% Code generation settings
set_param('AutonomousValetParking', 'RTWSystemTargetFile', 'ert.tlc');
set_param('AutonomousValetParking', 'DefaultParameterBehavior', 'Tunable');
set_param('AutonomousValetParking', 'OptimizeBlockIOStorage', 'on');

save_system('AutonomousValetParking');
```

### **2. Block Layout (Manual Placement)**

#### **Top-Level Architecture:**
```
[Scenario] → [GlobalPlanner] → [LocalAdapter] → [TrajGen] → [Controller]
     ↓             ↓               ↓             ↓           ↓
 [MapBus]      [PathBus]    [LocalBandBus] [TrajBus] [ActuatorBus]
                    ↓               ↑          ↑
            [VehicleState] → [SafetyMonitor] ←┘
                    ↓               ↓
               [SensorBus]    [SafetyFlagBus]
```

#### **Block-by-Block Placement:**

##### **Block 1: ScenarioLoader (Initialize)**
- **Type:** MATLAB Function Block
- **Sample Time:** `-1` (inherited/triggered)
- **Function Name:** `fcn_ScenarioLoader`
- **Position:** [50, 50, 150, 100]

##### **Block 2: GlobalPlanner (1Hz)**
- **Type:** MATLAB Function Block  
- **Sample Time:** `1.0` (1Hz)
- **Function Name:** `fcn_GlobalPlanner`
- **Position:** [200, 50, 300, 100]

##### **Block 3: LocalPathAdapter (20Hz)**
- **Type:** MATLAB Function Block
- **Sample Time:** `0.05` (20Hz)
- **Function Name:** `fcn_LocalPathAdapter`  
- **Position:** [350, 50, 450, 100]

##### **Block 4: TrajectoryGenerator (50Hz)**
- **Type:** MATLAB Function Block
- **Sample Time:** `0.02` (50Hz)
- **Function Name:** `fcn_TrajectoryGenerator`
- **Position:** [500, 50, 600, 100]

##### **Block 5: SafetyMonitor (100Hz)**
- **Type:** MATLAB Function Block
- **Sample Time:** `0.01` (100Hz)
- **Function Name:** `fcn_SafetyMonitor`
- **Position:** [350, 150, 450, 200]

##### **Block 6: VehicleState (50Hz)**
- **Type:** MATLAB Function Block
- **Sample Time:** `0.02` (50Hz)
- **Function Name:** `fcn_VehicleState`
- **Position:** [200, 150, 300, 200]

##### **Block 7: Logger (2Hz)**
- **Type:** MATLAB Function Block
- **Sample Time:** `0.5` (2Hz)
- **Function Name:** `fcn_Logger`
- **Position:** [650, 100, 750, 150]

### **3. Bus Connections (Manual Wire Routing)**

#### **Data Buses Required:**
1. **MapBus:** ScenarioLoader → GlobalPlanner, LocalPathAdapter, SafetyMonitor
2. **PathBus:** GlobalPlanner → LocalPathAdapter, Logger
3. **LocalBandBus:** LocalPathAdapter → TrajectoryGenerator
4. **TrajectoryBus:** TrajectoryGenerator → SafetyMonitor, Logger
5. **VehicleStateBus:** VehicleState → LocalPathAdapter, TrajectoryGenerator, SafetyMonitor
6. **SensorBus:** VehicleState → SafetyMonitor
7. **SafetyFlagBus:** SafetyMonitor → All planning blocks
8. **ActuatorCmdBus:** TrajectoryGenerator → Output (Controller interface)

### **4. Function Block Stubs (To be implemented)**

#### **Create These MATLAB Function Files:**
```matlab
% In src/ directory - create placeholders for now

% src/fcn_ScenarioLoader.m
function [mapBus, vehicleParams] = fcn_ScenarioLoader(scenarioFile)
    % TODO: Implement scenario loading
    mapBus = createEmptyMapBus();
    vehicleParams = createEmptyVehicleParams();
end

% src/fcn_GlobalPlanner.m  
function [pathBus, statsOut] = fcn_GlobalPlanner(mapBus, startPose, goalPose, trigger)
    % TODO: Implement Hybrid A* global planner
    pathBus = createEmptyPathBus();
    statsOut = createEmptyStatsBus();
end

% src/fcn_LocalPathAdapter.m
function localBandBus = fcn_LocalPathAdapter(pathBus, localCostmap, dynObstacles, vehicleState)
    % TODO: Implement elastic band local adaptation
    localBandBus = createEmptyLocalBandBus();
end

% src/fcn_TrajectoryGenerator.m
function trajectoryBus = fcn_TrajectoryGenerator(localBandBus, vehicleParams, limits)
    % TODO: Implement S-curve trajectory generation
    trajectoryBus = createEmptyTrajectoryBus();
end

% src/fcn_SafetyMonitor.m
function safetyFlagBus = fcn_SafetyMonitor(trajectoryBus, localCostmap, vehicleParams)
    % TODO: Implement collision checking and safety monitoring
    safetyFlagBus = createEmptySafetyFlagBus();
end

% src/fcn_VehicleState.m
function [vehicleStateBus, sensorBus] = fcn_VehicleState(inputs)
    % TODO: Implement vehicle state estimation
    vehicleStateBus = createEmptyVehicleStateBus();
    sensorBus = createEmptySensorBus();
end

% src/fcn_Logger.m
function fcn_Logger(pathBus, trajectoryBus, safetyFlagBus)
    % TODO: Implement data logging and diagnostics
    % No output - logging only
end
```

### **5. Model Validation Commands**
```matlab
% After creating the model structure
% Check model configuration
configSet = getActiveConfigSet('AutonomousValetParking');
get_param(configSet, 'Solver')         % Should be 'FixedStepDiscrete'
get_param(configSet, 'FixedStep')      % Should be '0.02'

% Validate model (after adding blocks)
sldiagnostics('AutonomousValetParking');
```

---

## Next Development Steps After Model Creation

### **Phase 1A Continuation:**
1. ✅ **VehicleParams & Configuration** (COMPLETED)
2. 🔄 **Create Simulink Model** (Manual - this document)
3. ⏳ **Bus Structures & Interfaces** (Next implementation)
4. ⏳ **Motion Primitive Infrastructure** (Following implementation)

### **Post-Model Creation Tasks:**
1. **Test model compilation** with empty function blocks
2. **Verify sample times** are correctly configured  
3. **Check signal dimensions** for bus interfaces
4. **Validate code generation** settings for future embedded deployment

### **Developer Notes:**
- **Bus structures** will be implemented next to properly define signal interfaces
- **Function block implementations** will be populated progressively through Phases 1B-1C
- **Model testing** should begin with simple signal generation and empty function calls

---

**⚠️ DEVELOPER ACTION REQUIRED: Please create the Simulink model manually using the instructions above, then proceed with bus structure implementation.**