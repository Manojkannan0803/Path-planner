%% CREATE_SIMULINK_MODEL - Create ADAS Path Planning Simulink Model
% This script creates a complete Simulink model integrating the MATLAB
% path planning system with real-time simulation capabilities

function create_adas_simulink_model()
    fprintf('=== CREATING ADAS PATH PLANNING SIMULINK MODEL ===\n');
    
    % Model name
    model_name = 'ADAS_PathPlanning_System';
    
    % Create new model
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    
    new_system(model_name);
    open_system(model_name);
    
    % Set model parameters
    set_param(model_name, 'SolverType', 'Fixed-step');
    set_param(model_name, 'Solver', 'ode4');
    set_param(model_name, 'FixedStep', '0.1');
    set_param(model_name, 'StopTime', '100');
    
    fprintf('Created model: %s\n', model_name);
    
    %% Add System Objects
    add_path_planning_system(model_name);
    add_vehicle_dynamics_system(model_name);
    add_collision_detection_system(model_name);
    add_path_following_controller(model_name);
    
    %% Add Input/Output blocks
    add_input_output_blocks(model_name);
    
    %% Add Visualization blocks
    add_visualization_blocks(model_name);
    
    %% Connect blocks
    connect_blocks(model_name);
    
    %% Configure subsystems
    configure_subsystems(model_name);
    
    % Save model
    save_system(model_name);
    
    fprintf('=== SIMULINK MODEL CREATION COMPLETED ===\n');
    fprintf('Model saved as: %s.slx\n', model_name);
    fprintf('To run simulation: sim(''%s'')\n', model_name);
end

function add_path_planning_system(model_name)
    % Add Path Planning System Object
    
    block_name = [model_name '/Path Planning System'];
    add_block('simulink/User-Defined Functions/MATLAB System', block_name);
    
    % Configure System Object
    set_param(block_name, 'SystemObjectName', 'PathPlanningSystemObject');
    set_param(block_name, 'Position', [100, 100, 200, 200]);
    
    fprintf('Added Path Planning System block\n');
end

function add_vehicle_dynamics_system(model_name)
    % Add Vehicle Dynamics System Object
    
    block_name = [model_name '/Vehicle Dynamics'];
    add_block('simulink/User-Defined Functions/MATLAB System', block_name);
    
    % Configure System Object
    set_param(block_name, 'SystemObjectName', 'VehicleDynamicsSystemObject');
    set_param(block_name, 'Position', [400, 300, 500, 400]);
    
    fprintf('Added Vehicle Dynamics block\n');
end

function add_collision_detection_system(model_name)
    % Add Collision Detection System Object
    
    block_name = [model_name '/Collision Detection'];
    add_block('simulink/User-Defined Functions/MATLAB System', block_name);
    
    % Configure System Object
    set_param(block_name, 'SystemObjectName', 'CollisionDetectionSystemObject');
    set_param(block_name, 'Position', [100, 300, 200, 400]);
    
    fprintf('Added Collision Detection block\n');
end

function add_path_following_controller(model_name)
    % Add Path Following Controller System Object
    
    block_name = [model_name '/Path Following Controller'];
    add_block('simulink/User-Defined Functions/MATLAB System', block_name);
    
    % Configure System Object
    set_param(block_name, 'SystemObjectName', 'PathFollowingControllerSystemObject');
    set_param(block_name, 'Position', [700, 200, 800, 300]);
    
    fprintf('Added Path Following Controller block\n');
end

function add_input_output_blocks(model_name)
    % Add input and output blocks
    
    % Clock for simulation time
    add_block('simulink/Sources/Clock', [model_name '/Clock']);
    set_param([model_name '/Clock'], 'Position', [20, 50, 50, 80]);
    
    % Start/Goal position constants
    add_block('simulink/Sources/Constant', [model_name '/Start Position']);
    set_param([model_name '/Start Position'], 'Value', '[75, 45, 180, 0]');
    set_param([model_name '/Start Position'], 'Position', [20, 120, 80, 150]);
    
    add_block('simulink/Sources/Constant', [model_name '/Goal Position']);
    set_param([model_name '/Goal Position'], 'Value', '[60, 64, 270, 0]');
    set_param([model_name '/Goal Position'], 'Position', [20, 170, 80, 200]);
    
    % Planning trigger
    add_block('simulink/Sources/Pulse Generator', [model_name '/Planning Trigger']);
    set_param([model_name '/Planning Trigger'], 'Period', '5');
    set_param([model_name '/Planning Trigger'], 'PulseWidth', '10');
    set_param([model_name '/Planning Trigger'], 'Position', [20, 220, 80, 250]);
    
    % Obstacle data (simplified)
    add_block('simulink/Sources/Constant', [model_name '/Obstacle Data']);
    set_param([model_name '/Obstacle Data'], 'Value', 'zeros(200, 1)');
    set_param([model_name '/Obstacle Data'], 'Position', [20, 270, 80, 300]);
    
    % Output displays
    add_block('simulink/Sinks/Display', [model_name '/Vehicle Position']);
    set_param([model_name '/Vehicle Position'], 'Position', [900, 100, 950, 130]);
    
    add_block('simulink/Sinks/Display', [model_name '/Planning Success']);
    set_param([model_name '/Planning Success'], 'Position', [300, 100, 350, 130]);
    
    add_block('simulink/Sinks/Display', [model_name '/Collision Status']);
    set_param([model_name '/Collision Status'], 'Position', [300, 300, 350, 330]);
    
    fprintf('Added input/output blocks\n');
end

function add_visualization_blocks(model_name)
    % Add visualization blocks
    
    % XY Graph for vehicle trajectory
    add_block('simulink/Sinks/XY Graph', [model_name '/Vehicle Trajectory']);
    set_param([model_name '/Vehicle Trajectory'], 'Position', [900, 200, 950, 250]);
    set_param([model_name '/Vehicle Trajectory'], 'xmin', '0');
    set_param([model_name '/Vehicle Trajectory'], 'xmax', '120');
    set_param([model_name '/Vehicle Trajectory'], 'ymin', '0');
    set_param([model_name '/Vehicle Trajectory'], 'ymax', '100');
    
    % Scope for control signals
    add_block('simulink/Sinks/Scope', [model_name '/Control Signals']);
    set_param([model_name '/Control Signals'], 'Position', [900, 300, 950, 350]);
    
    % To Workspace blocks for data logging
    add_block('simulink/Sinks/To Workspace', [model_name '/Log Vehicle States']);
    set_param([model_name '/Log Vehicle States'], 'VariableName', 'vehicle_states');
    set_param([model_name '/Log Vehicle States'], 'Position', [900, 400, 950, 430]);
    
    add_block('simulink/Sinks/To Workspace', [model_name '/Log Path Data']);
    set_param([model_name '/Log Path Data'], 'VariableName', 'path_data');
    set_param([model_name '/Log Path Data'], 'Position', [300, 150, 350, 180]);
    
    fprintf('Added visualization blocks\n');
end

function connect_blocks(model_name)
    % Connect all blocks with signal lines
    
    try
        % Clock connections
        add_line(model_name, 'Clock/1', 'Path Planning System/10');
        add_line(model_name, 'Clock/1', 'Vehicle Dynamics/7');
        add_line(model_name, 'Clock/1', 'Path Following Controller/10');
        
        % Input connections to Path Planning System
        add_line(model_name, 'Start Position/1', 'Path Planning System/1');
        add_line(model_name, 'Goal Position/1', 'Path Planning System/5');
        add_line(model_name, 'Planning Trigger/1', 'Path Planning System/9');
        
        % Path Planning to Path Following Controller
        add_line(model_name, 'Path Planning System/1', 'Path Following Controller/5');
        add_line(model_name, 'Path Planning System/2', 'Path Following Controller/6');
        add_line(model_name, 'Path Planning System/3', 'Path Following Controller/7');
        add_line(model_name, 'Path Planning System/4', 'Path Following Controller/8');
        add_line(model_name, 'Path Planning System/5', 'Path Following Controller/9');
        
        % Vehicle Dynamics to feedback
        add_line(model_name, 'Vehicle Dynamics/1', 'Path Following Controller/1');
        add_line(model_name, 'Vehicle Dynamics/2', 'Path Following Controller/2');
        add_line(model_name, 'Vehicle Dynamics/3', 'Path Following Controller/3');
        add_line(model_name, 'Vehicle Dynamics/4', 'Path Following Controller/4');
        
        % Controller to Vehicle Dynamics
        add_line(model_name, 'Path Following Controller/1', 'Vehicle Dynamics/1');
        add_line(model_name, 'Path Following Controller/2', 'Vehicle Dynamics/2');
        
        % Collision Detection connections
        add_line(model_name, 'Vehicle Dynamics/1', 'Collision Detection/1');
        add_line(model_name, 'Vehicle Dynamics/2', 'Collision Detection/2');
        add_line(model_name, 'Vehicle Dynamics/3', 'Collision Detection/3');
        add_line(model_name, 'Vehicle Dynamics/4', 'Collision Detection/4');
        add_line(model_name, 'Obstacle Data/1', 'Collision Detection/5');
        
        % Output connections
        add_line(model_name, 'Path Planning System/5', 'Planning Success/1');
        add_line(model_name, 'Collision Detection/1', 'Collision Status/1');
        add_line(model_name, 'Vehicle Dynamics/1', 'Vehicle Position/1');
        
        % Visualization connections
        add_line(model_name, 'Vehicle Dynamics/1', 'Vehicle Trajectory/1');
        add_line(model_name, 'Vehicle Dynamics/2', 'Vehicle Trajectory/2');
        add_line(model_name, 'Path Following Controller/1', 'Control Signals/1');
        
        % Data logging
        add_line(model_name, 'Vehicle Dynamics/1', 'Log Vehicle States/1');
        add_line(model_name, 'Path Planning System/1', 'Log Path Data/1');
        
        fprintf('Connected all blocks\n');
        
    catch ME
        warning('SimulinkConnection:Error', 'Some connections may have failed: %s', ME.message);
    end
end

function configure_subsystems(model_name)
    % Configure subsystems and parameters
    
    % Set model configuration parameters
    cs = getActiveConfigSet(model_name);
    cs.set_param('Solver', 'ode4');
    cs.set_param('FixedStep', '0.1');
    cs.set_param('SaveOutput', 'on');
    cs.set_param('SaveState', 'on');
    cs.set_param('SaveTime', 'on');
    
    % Configure data logging
    cs.set_param('DSMLogging', 'on');
    cs.set_param('LoggingToFile', 'on');
    
    fprintf('Configured subsystems\n');
end

%% Main execution
if isempty(dbstack())
    create_adas_simulink_model();
end
