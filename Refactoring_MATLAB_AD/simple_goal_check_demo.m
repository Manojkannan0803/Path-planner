%% SIMPLE_GOAL_CHECK_DEMO - Demonstration of simple goal checking function
% This script shows how to use the simple check_goal_reached function

function main()
    fprintf('=== GENERIC GOAL CHECKING DEMO ===\n');
    fprintf('Support for Articulated and Car Vehicles\n\n');
    
    %% 1. ARTICULATED VEHICLE TESTING
    fprintf('1. ARTICULATED VEHICLE (Tractor-Trailer) TESTING\n');
    fprintf('------------------------------------------------\n');
    
    % Define goal state for articulated vehicle
    articulated_goal = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
    
    % Test different articulated vehicle positions
    articulated_positions = [
        struct('x', 75, 'y', 45, 'theta', 180, 'gamma', 0);   % Far from goal
        struct('x', 62, 'y', 62, 'theta', 260, 'gamma', -2);  % Close to goal
        struct('x', 60.5, 'y', 63.8, 'theta', 268, 'gamma', 1); % Very close
        struct('x', 60.1, 'y', 64.1, 'theta', 271, 'gamma', -1); % At goal
    ];
    
    % Check each position with default tolerances for articulated vehicle
    fprintf('Testing articulated vehicle goal achievement:\n');
    for i = 1:length(articulated_positions)
        current_state = articulated_positions(i);
        [should_stop, stop_reason, errors] = check_goal_reached(...
            current_state, articulated_goal, [], [], [], 'articulated');
        
        fprintf('Position %d: (%.1f, %.1f, %.0f°, %.0f°) - %s\n', ...
            i, current_state.x, current_state.y, current_state.theta, current_state.gamma, stop_reason);
        
        if should_stop
            fprintf('  SUCCESS! Errors: Pos=%.2fm, Orient=%.1f°, Artic=%.1f°\n', ...
                errors.position_error, errors.orientation_error, errors.articulation_error);
            break;
        else
            fprintf('  Continue - Errors: Pos=%.2fm, Orient=%.1f°, Artic=%.1f°\n', ...
                errors.position_error, errors.orientation_error, errors.articulation_error);
        end
    end
    
    %% 2. CAR VEHICLE TESTING
    fprintf('\n2. CAR VEHICLE TESTING\n');
    fprintf('----------------------\n');
    
    % Define goal state for car vehicle (no gamma needed)
    car_goal = struct('x', 60, 'y', 64, 'theta', 270);
    
    % Test different car vehicle positions
    car_positions = [
        struct('x', 75, 'y', 45, 'theta', 180);   % Far from goal
        struct('x', 62, 'y', 62, 'theta', 260);   % Close to goal
        struct('x', 60.5, 'y', 63.8, 'theta', 268); % Very close
        struct('x', 60.1, 'y', 64.1, 'theta', 271); % At goal
    ];
    
    % Check each position for car vehicle
    fprintf('Testing car vehicle goal achievement:\n');
    for i = 1:length(car_positions)
        current_state = car_positions(i);
        [should_stop, stop_reason, errors] = check_goal_reached(...
            current_state, car_goal, [], [], [], 'car');
        
        fprintf('Position %d: (%.1f, %.1f, %.0f°) - %s\n', ...
            i, current_state.x, current_state.y, current_state.theta, stop_reason);
        
        if should_stop
            fprintf('  SUCCESS! Errors: Pos=%.2fm, Orient=%.1f° (No articulation)\n', ...
                errors.position_error, errors.orientation_error);
            break;
        else
            fprintf('  Continue - Errors: Pos=%.2fm, Orient=%.1f°\n', ...
                errors.position_error, errors.orientation_error);
        end
    end
    
    %% 3. CUSTOM TOLERANCES FOR DIFFERENT SCENARIOS
    fprintf('\n3. CUSTOM TOLERANCES FOR DIFFERENT SCENARIOS\n');
    fprintf('--------------------------------------------\n');
    
    % Test with different scenarios for both vehicle types
    test_scenarios = {
        struct('name', 'Articulated Parking', 'vehicle_type', 'articulated', ...
               'state', struct('x', 60.3, 'y', 64.2, 'theta', 267, 'gamma', 2), ...
               'goal', struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0), ...
               'tolerances', [0.5, 5.0, 3.0]);  % Tight tolerances
        struct('name', 'Car Parking', 'vehicle_type', 'car', ...
               'state', struct('x', 60.3, 'y', 64.2, 'theta', 267), ...
               'goal', struct('x', 60, 'y', 64, 'theta', 270), ...
               'tolerances', [0.5, 5.0, []]);  % Tight tolerances, no articulation
        struct('name', 'Articulated Highway', 'vehicle_type', 'articulated', ...
               'state', struct('x', 60.8, 'y', 64.5, 'theta', 275, 'gamma', 3), ...
               'goal', struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0), ...
               'tolerances', [3.0, 20.0, 15.0]);  % Relaxed tolerances
        struct('name', 'Car Highway', 'vehicle_type', 'car', ...
               'state', struct('x', 60.8, 'y', 64.5, 'theta', 275), ...
               'goal', struct('x', 60, 'y', 64, 'theta', 270), ...
               'tolerances', [3.0, 20.0, []]);  % Relaxed tolerances, no articulation
    };
    
    for i = 1:length(test_scenarios)
        scenario = test_scenarios{i};
        tol = scenario.tolerances;
        
        fprintf('\nTesting %s:\n', scenario.name);
        [should_stop, ~, errors] = check_goal_reached(...
            scenario.state, scenario.goal, tol(1), tol(2), tol(3), scenario.vehicle_type);
        
        if should_stop
            result_msg = 'goal_reached';
        else
            result_msg = 'continue';
        end
        fprintf('  Result: %s\n', result_msg);
        fprintf('  Vehicle type: %s\n', errors.vehicle_type);
        fprintf('  Errors: Pos=%.3fm (tol=%.1fm), Orient=%.1f° (tol=%.1f°)', ...
            errors.position_error, errors.position_tolerance, ...
            errors.orientation_error, errors.orientation_tolerance);
        
        if strcmp(scenario.vehicle_type, 'articulated')
            fprintf(', Artic=%.1f° (tol=%.1f°)\n', ...
                errors.articulation_error, errors.articulation_tolerance);
        else
            fprintf(' (No articulation for car)\n');
        end
    end
    
    %% 4. SIMULATION LOOP EXAMPLE (ARTICULATED VEHICLE)
    fprintf('\n4. SIMULATION LOOP EXAMPLE (ARTICULATED VEHICLE)\n');
    fprintf('------------------------------------------------\n');
    
    % Simple simulation moving toward goal
    start_state = struct('x', 75, 'y', 45, 'theta', 180, 'gamma', 0);
    current_state = start_state;
    
    fprintf('Simulating vehicle movement from (%.1f,%.1f) to (%.1f,%.1f):\n', ...
        start_state.x, start_state.y, goal_state.x, goal_state.y);
    
    max_steps = 20;
    for step = 1:max_steps
        % Move vehicle toward goal (simple linear interpolation)
        alpha = step / max_steps;
        current_state.x = start_state.x + alpha * (goal_state.x - start_state.x);
        current_state.y = start_state.y + alpha * (goal_state.y - start_state.y);
        current_state.theta = start_state.theta + alpha * (goal_state.theta - start_state.theta);
        current_state.gamma = start_state.gamma + alpha * (goal_state.gamma - start_state.gamma);
        
        % Check goal
        [should_stop, stop_reason, errors] = check_goal_reached(...
            current_state, goal_state, 1.5, 10.0, 8.0, 'articulated');
        
        fprintf('Step %2d: Pos(%.1f,%.1f) Error=%.2fm - %s\n', ...
            step, current_state.x, current_state.y, errors.position_error, stop_reason);
        
        if should_stop
            fprintf('GOAL REACHED at step %d!\n', step);
            fprintf('Final errors: Position=%.3fm, Orientation=%.1f°, Articulation=%.1f°\n', ...
                errors.position_error, errors.orientation_error, errors.articulation_error);
            break;
        end
    end
    
    if step == max_steps && ~should_stop
        fprintf('Simulation completed without reaching goal\n');
    end
    
    %% 4. INTEGRATION EXAMPLES
    fprintf('\n4. INTEGRATION EXAMPLES\n');
    fprintf('-----------------------\n');
    
    show_integration_patterns();
    
    fprintf('\n=== DEMO COMPLETED ===\n');
end

function show_integration_patterns()
    fprintf('\nIntegration Pattern 1 - Basic Planning Loop:\n');
    fprintf('--------------------------------------------\n');
    fprintf('function planning_with_goal_check()\n');
    fprintf('    start_state = struct(''x'', 75, ''y'', 45, ''theta'', 180, ''gamma'', 0);\n');
    fprintf('    goal_state = struct(''x'', 60, ''y'', 64, ''theta'', 270, ''gamma'', 0);\n');
    fprintf('    current_state = start_state;\n');
    fprintf('    \n');
    fprintf('    while true\n');
    fprintf('        %% Execute your path planning here\n');
    fprintf('        %% [path, success] = planning_session.execute_single_planning(...);\n');
    fprintf('        \n');
    fprintf('        %% Check if goal reached\n');
    fprintf('        [should_stop, stop_reason, errors] = check_goal_reached(...\n');
    fprintf('            current_state, goal_state, 1.5, 10.0, 8.0);\n');
    fprintf('        \n');
    fprintf('        if should_stop\n');
    fprintf('            fprintf(''Planning completed: %%s\\n'', stop_reason);\n');
    fprintf('            break;\n');
    fprintf('        end\n');
    fprintf('        \n');
    fprintf('        %% Update vehicle state based on planning result\n');
    fprintf('        %% current_state = update_vehicle_state(current_state, path);\n');
    fprintf('    end\n');
    fprintf('end\n\n');
    
    fprintf('Integration Pattern 2 - Simulink System Object:\n');
    fprintf('-----------------------------------------------\n');
    fprintf('function [stop_sim, stop_reason] = stepImpl(obj, current_pose, goal_pose)\n');
    fprintf('    %% In your System Object stepImpl method:\n');
    fprintf('    \n');
    fprintf('    %% Convert poses to state structures\n');
    fprintf('    current_state = struct(''x'', current_pose(1), ''y'', current_pose(2), ...\n');
    fprintf('                          ''theta'', current_pose(3), ''gamma'', current_pose(4));\n');
    fprintf('    goal_state = struct(''x'', goal_pose(1), ''y'', goal_pose(2), ...\n');
    fprintf('                       ''theta'', goal_pose(3), ''gamma'', goal_pose(4));\n');
    fprintf('    \n');
    fprintf('    %% Check goal with appropriate tolerances\n');
    fprintf('    [stop_sim, stop_reason, ~] = check_goal_reached(...\n');
    fprintf('        current_state, goal_state, obj.PositionTolerance, ...\n');
    fprintf('        obj.OrientationTolerance, obj.ArticulationTolerance);\n');
    fprintf('end\n\n');
    
    fprintf('Integration Pattern 3 - Different Scenarios:\n');
    fprintf('--------------------------------------------\n');
    fprintf('%% Parking scenario (tight tolerances)\n');
    fprintf('[stop, reason, errors] = check_goal_reached(current, goal, 0.3, 3.0, 2.0);\n\n');
    fprintf('%% Highway scenario (relaxed tolerances)\n');
    fprintf('[stop, reason, errors] = check_goal_reached(current, goal, 3.0, 20.0, 15.0);\n\n');
    fprintf('%% Normal maneuvering (balanced tolerances)\n');
    fprintf('[stop, reason, errors] = check_goal_reached(current, goal, 1.0, 8.0, 5.0);\n\n');
end

%% Entry Point
if isempty(dbstack())
    main();
end
