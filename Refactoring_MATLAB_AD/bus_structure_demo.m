%% VEHICLE BUS STRUCTURE DEMO
% This script demonstrates the usage of the new bus structure system
% for vehicle goal checking with different vehicle types.

clear; clc;
fprintf('VEHICLE BUS STRUCTURE DEMONSTRATION\n');
fprintf('===================================\n\n');

%% 1. CREATE BUS STRUCTURES
fprintf('1. Creating bus structures...\n');
run('create_vehicle_bus_structures.m');

%% 2. ARTICULATED VEHICLE EXAMPLE
fprintf('\n2. ARTICULATED VEHICLE EXAMPLE\n');
fprintf('------------------------------\n');

% Create input bus for articulated vehicle
input_bus = struct();

% Create vehicle states
input_bus.current_state = create_vehicle_state(60.1, 64.2, 268, 1, 'articulated');
input_bus.goal_state = create_vehicle_state(60, 64, 270, 0, 'articulated');

% Create tolerance configuration
input_bus.tolerances = create_tolerance_config(1.0, 10.0, 5.0);

% Create vehicle configuration
input_bus.vehicle_config = create_vehicle_config('articulated');

% Test goal checking with bus structure
fprintf('Testing articulated vehicle with bus structure:\n');
[should_stop, stop_reason, goal_errors] = check_goal_reached_bus(input_bus);

fprintf('Current state: (%.1f, %.1f, %.0f°, %.0f°)\n', ...
    input_bus.current_state.x, input_bus.current_state.y, ...
    input_bus.current_state.theta, input_bus.current_state.gamma);
fprintf('Goal state: (%.1f, %.1f, %.0f°, %.0f°)\n', ...
    input_bus.goal_state.x, input_bus.goal_state.y, ...
    input_bus.goal_state.theta, input_bus.goal_state.gamma);
fprintf('Result: %s\n', stop_reason);
fprintf('Errors: Pos=%.2fm, Orient=%.1f°, Artic=%.1f°\n', ...
    goal_errors.position_error, goal_errors.orientation_error, goal_errors.articulation_error);

%% 3. CAR VEHICLE EXAMPLE
fprintf('\n3. CAR VEHICLE EXAMPLE\n');
fprintf('----------------------\n');

% Create input bus for car vehicle
car_input_bus = struct();

% Create vehicle states for car
car_input_bus.current_state = create_vehicle_state(60.1, 64.2, 268, 0, 'car');
car_input_bus.goal_state = create_vehicle_state(60, 64, 270, 0, 'car');

% Create tolerance configuration (same as before)
car_input_bus.tolerances = create_tolerance_config(1.0, 10.0, 5.0);

% Create vehicle configuration for car
car_input_bus.vehicle_config = create_vehicle_config('car');

% Test goal checking with bus structure
fprintf('Testing car vehicle with bus structure:\n');
[should_stop_car, stop_reason_car, goal_errors_car] = check_goal_reached_bus(car_input_bus);

fprintf('Current state: (%.1f, %.1f, %.0f°) [car]\n', ...
    car_input_bus.current_state.x, car_input_bus.current_state.y, ...
    car_input_bus.current_state.theta);
fprintf('Goal state: (%.1f, %.1f, %.0f°) [car]\n', ...
    car_input_bus.goal_state.x, car_input_bus.goal_state.y, ...
    car_input_bus.goal_state.theta);
fprintf('Result: %s\n', stop_reason_car);
fprintf('Errors: Pos=%.2fm, Orient=%.1f° (No articulation)\n', ...
    goal_errors_car.position_error, goal_errors_car.orientation_error);

%% 4. DIFFERENT TOLERANCE SCENARIOS
fprintf('\n4. DIFFERENT TOLERANCE SCENARIOS\n');
fprintf('--------------------------------\n');

% Define different scenarios
scenarios = {
    struct('name', 'Parking (Tight)', 'tolerances', [0.5, 5.0, 3.0]);
    struct('name', 'Highway (Relaxed)', 'tolerances', [3.0, 20.0, 15.0]);
    struct('name', 'Default', 'tolerances', [2.0, 15.0, 10.0]);
};

for i = 1:length(scenarios)
    scenario = scenarios{i};
    tol = scenario.tolerances;
    
    fprintf('\nTesting %s tolerances:\n', scenario.name);
    
    % Update tolerance configuration
    test_input_bus = input_bus;  % Use articulated vehicle
    test_input_bus.tolerances = create_tolerance_config(tol(1), tol(2), tol(3));
    
    [should_stop_test, stop_reason_test, goal_errors_test] = check_goal_reached_bus(test_input_bus);
    
    fprintf('  Tolerances: Pos=%.1fm, Orient=%.1f°, Artic=%.1f°\n', tol(1), tol(2), tol(3));
    fprintf('  Result: %s\n', stop_reason_test);
    fprintf('  Satisfied: Pos=%s, Orient=%s, Artic=%s\n', ...
        bool2str(goal_errors_test.position_ok), ...
        bool2str(goal_errors_test.orientation_ok), ...
        bool2str(goal_errors_test.articulation_ok));
end

%% 5. COMPARISON WITH ORIGINAL FUNCTION
fprintf('\n5. COMPARISON WITH ORIGINAL FUNCTION\n');
fprintf('------------------------------------\n');

% Test both functions with same inputs
fprintf('Testing both original and bus-based functions:\n');

% Original function call
[stop_orig, reason_orig, errors_orig] = check_goal_reached(...
    input_bus.current_state, input_bus.goal_state, ...
    1.0, 10.0, 5.0, 'articulated');

% Bus function call (already done above)
stop_bus = should_stop;
reason_bus = stop_reason;
errors_bus = goal_errors;

fprintf('Original function: %s (pos_error=%.2fm)\n', reason_orig, errors_orig.position_error);
fprintf('Bus function:      %s (pos_error=%.2fm)\n', reason_bus, errors_bus.position_error);

if stop_orig == stop_bus && abs(errors_orig.position_error - errors_bus.position_error) < 1e-6
    fprintf('✓ Both functions produce identical results!\n');
else
    fprintf('✗ Functions produce different results!\n');
end

%% 6. SIMULINK INTEGRATION EXAMPLE
fprintf('\n6. SIMULINK INTEGRATION EXAMPLE\n');
fprintf('-------------------------------\n');

fprintf('For Simulink integration:\n');
fprintf('1. Use create_vehicle_bus_structures.m to define bus objects\n');
fprintf('2. Create MATLAB Function block with check_goal_reached_bus\n');
fprintf('3. Configure input/output ports with bus data types:\n');
fprintf('   - Input: GoalCheckInputBus\n');
fprintf('   - Outputs: boolean (should_stop), GoalCheckOutputBus (errors)\n\n');

fprintf('Example Simulink MATLAB Function code:\n');
fprintf('function [should_stop, goal_errors] = fcn(input_bus)\n');
fprintf('  [should_stop, ~, goal_errors] = check_goal_reached_bus(input_bus);\n');
fprintf('end\n\n');

%% Helper function
function str = bool2str(bool_val)
    if bool_val
        str = 'YES';
    else
        str = 'NO';
    end
end

fprintf('Demo completed! All bus structures and functions are ready to use.\n\n');
