%% SIMULATION_STOP_DEMO - Demonstration of SimulationStopChecker Service
% This script shows how to use the SimulationStopChecker service to
% intelligently stop simulations based on goal achievement and other conditions

function main()
    fprintf('=== SIMULATION STOP CHECKER DEMO ===\n');
    fprintf('Intelligent simulation stopping based on goal achievement\n\n');
    
    %% 1. BASIC USAGE - Goal Achievement Detection
    fprintf('1. BASIC USAGE - Goal Achievement Detection\n');
    fprintf('------------------------------------------\n');
    
    % Create stop checker with default configuration
    stop_checker = services.validation.SimulationStopChecker();
    
    % Define test scenario
    goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
    
    % Simulate vehicle approaching goal
    test_positions = [
        struct('x', 75, 'y', 45, 'theta', 180, 'gamma', 0);   % Start position
        struct('x', 65, 'y', 55, 'theta', 225, 'gamma', -5);  % Intermediate
        struct('x', 62, 'y', 62, 'theta', 260, 'gamma', -2);  % Close to goal
        struct('x', 60.5, 'y', 63.8, 'theta', 268, 'gamma', 1); % Very close
        struct('x', 60.1, 'y', 64.1, 'theta', 271, 'gamma', -1); % At goal
    ];
    
    fprintf('\nSimulating vehicle approach to goal:\n');
    for i = 1:length(test_positions)
        current_state = test_positions(i);
        simulation_time = i * 10; % Simulate 10 seconds per step
        
        [should_stop, stop_reason, stop_details] = stop_checker.check_stopping_conditions(...
            current_state, goal_state, simulation_time);
        
        fprintf('Step %d: Position (%.1f, %.1f), Angle %.0f° - ', ...
            i, current_state.x, current_state.y, current_state.theta);
        
        if should_stop
            fprintf('STOP! Reason: %s\n', stop_reason);
            fprintf('  Position error: %.2fm, Orientation error: %.1f°\n', ...
                stop_details.position_error, stop_details.orientation_error);
            break;
        else
            fprintf('Continue\n');
        end
    end
    
    %% 2. CUSTOM TOLERANCE CONFIGURATION
    fprintf('\n2. CUSTOM TOLERANCE CONFIGURATION\n');
    fprintf('---------------------------------\n');
    
    % Create stop checker with tighter tolerances
    tight_config = struct(...
        'goal_tolerance', struct(...
            'position_tolerance', 0.5, ...      % Very tight: 0.5m
            'orientation_tolerance', 5.0, ...   % Very tight: 5°
            'articulation_tolerance', 3.0), ... % Very tight: 3°
        'debug_mode', true);
    
    tight_stop_checker = services.validation.SimulationStopChecker(tight_config);
    
    fprintf('\nTesting with tight tolerances (0.5m, 5°):\n');
    test_state = struct('x', 60.3, 'y', 64.2, 'theta', 267, 'gamma', 2);
    
    [should_stop, ~, stop_details] = tight_stop_checker.check_stopping_conditions(...
        test_state, goal_state, 25);
    
    if should_stop
        fprintf('Goal reached with tight tolerances!\n');
    else
        fprintf('Goal NOT reached - errors too large for tight tolerances\n');
        fprintf('Position error: %.3fm (tolerance: %.3fm)\n', ...
            stop_details.position_error, tight_config.goal_tolerance.position_tolerance);
        fprintf('Orientation error: %.1f° (tolerance: %.1f°)\n', ...
            stop_details.orientation_error, tight_config.goal_tolerance.orientation_tolerance);
    end
    
    %% 3. TIMEOUT AND FAILURE CONDITIONS
    fprintf('\n3. TIMEOUT AND FAILURE CONDITIONS\n');
    fprintf('---------------------------------\n');
    
    % Create stop checker with short timeout
    timeout_config = struct(...
        'max_simulation_time', 30.0, ...      % 30 second timeout
        'max_planning_attempts', 2, ...       % Only 2 planning attempts
        'debug_mode', true);
    
    timeout_checker = services.validation.SimulationStopChecker(timeout_config);
    
    % Test timeout condition
    fprintf('\nTesting timeout condition:\n');
    test_state = struct('x', 75, 'y', 45, 'theta', 180, 'gamma', 0);
    simulation_time = 35.0; % Exceed timeout
    
    [should_stop, stop_reason, stop_details] = timeout_checker.check_stopping_conditions(...
        test_state, goal_state, simulation_time);
    
    if should_stop && strcmp(stop_reason, 'simulation_timeout')
        fprintf('Simulation stopped due to timeout at %.1fs\n', simulation_time);
        fprintf('Maximum allowed time: %.1fs\n', stop_details.max_allowed_time);
    end
    
    % Test planning failure condition
    fprintf('\nTesting planning failure condition:\n');
    failed_planning_result = struct('success', false, 'path', [], ...
        'stats', struct('termination_reason', 'no_valid_path'));
    additional_info = struct('consecutive_failures', 3);
    
    [should_stop, stop_reason, stop_details] = timeout_checker.check_stopping_conditions(...
        test_state, goal_state, 15.0, failed_planning_result, additional_info);
    
    if should_stop && strcmp(stop_reason, 'planning_failure')
        fprintf('Simulation stopped due to planning failure\n');
        fprintf('Failure type: %s\n', stop_details.failure_type);
        fprintf('Consecutive failures: %d\n', stop_details.consecutive_failures);
    end
    
    %% 4. INTEGRATION WITH PATH PLANNING SYSTEM
    fprintf('\n4. INTEGRATION WITH PATH PLANNING SYSTEM\n');
    fprintf('----------------------------------------\n');
    
    demonstrate_planning_integration();
    
    %% 5. REAL-TIME SIMULATION LOOP EXAMPLE
    fprintf('\n5. REAL-TIME SIMULATION LOOP EXAMPLE\n');
    fprintf('------------------------------------\n');
    
    demonstrate_simulation_loop();
    
    %% 6. STATISTICS AND MONITORING
    fprintf('\n6. STATISTICS AND MONITORING\n');
    fprintf('----------------------------\n');
    
    stats = stop_checker.get_simulation_statistics();
    fprintf('Simulation Statistics:\n');
    fprintf('  Total checks performed: %d\n', stats.total_checks);
    fprintf('  Goal achievements: %d\n', stats.goal_achievements);
    fprintf('  Timeouts: %d\n', stats.timeouts);
    fprintf('  Planning failures: %d\n', stats.planning_failures);
    
    fprintf('\n=== DEMO COMPLETED ===\n');
end

function demonstrate_planning_integration()
    % Show how to integrate SimulationStopChecker with PlanningSession
    
    fprintf('\nIntegrating with path planning system:\n');
    
    % Create stop checker configured for path planning
    planning_config = struct(...
        'goal_tolerance', struct(...
            'position_tolerance', 1.5, ...
            'orientation_tolerance', 10.0, ...
            'articulation_tolerance', 8.0), ...
        'stopping_conditions', struct(...
            'enable_goal_reached', true, ...
            'enable_timeout', true, ...
            'enable_planning_failure', true, ...
            'enable_path_completion', true), ...
        'max_simulation_time', 120.0, ...
        'debug_mode', false);
    
    stop_checker = services.validation.SimulationStopChecker(planning_config);
    
    % Simulate planning session integration
    fprintf('Creating planning session...\n');
    try
        % This would normally be: session = PlanningSession('scenarios/DPDScenario.json');
        fprintf('PlanningSession created (simulated)\n');
        
        % Define states
        start_state = struct('x', 75, 'y', 45, 'theta', 180, 'gamma', 0);
        goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
        
        % Simulate planning result
        planning_result = struct(...
            'success', true, ...
            'path', struct('x', [75, 70, 65, 60], 'y', [45, 50, 57, 64]), ...
            'stats', struct('planning_time', 0.15, 'nodes_explored', 245));
        
        % Check stopping conditions after planning
        [should_stop, stop_reason, ~] = stop_checker.check_stopping_conditions(...
            start_state, goal_state, 5.0, planning_result);
        
        if should_stop
            result_msg = 'STOP';
        else
            result_msg = 'CONTINUE';
        end
        fprintf('Planning integration result: %s\n', result_msg);
        if should_stop
            fprintf('Stop reason: %s\n', stop_reason);
        end
        
    catch ME
        fprintf('Planning session simulation: %s\n', ME.message);
    end
end

function demonstrate_simulation_loop()
    % Demonstrate a complete simulation loop with stopping conditions
    
    fprintf('\nRunning complete simulation loop:\n');
    
    % Setup
    stop_checker = services.validation.SimulationStopChecker();
    start_state = struct('x', 75, 'y', 45, 'theta', 180, 'gamma', 0);
    goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
    
    current_state = start_state;
    simulation_time = 0;
    dt = 1.0; % 1 second time step
    max_iterations = 100;
    
    fprintf('Starting simulation from (%.1f, %.1f) to (%.1f, %.1f)\n', ...
        start_state.x, start_state.y, goal_state.x, goal_state.y);
    
    for iteration = 1:max_iterations
        simulation_time = simulation_time + dt;
        
        % Simulate vehicle movement (simple linear interpolation toward goal)
        alpha = min(0.05 * iteration, 1.0); % Progress factor
        current_state.x = start_state.x + alpha * (goal_state.x - start_state.x);
        current_state.y = start_state.y + alpha * (goal_state.y - start_state.y);
        current_state.theta = start_state.theta + alpha * (goal_state.theta - start_state.theta);
        current_state.gamma = start_state.gamma + alpha * (goal_state.gamma - start_state.gamma);
        
        % Simulate planning result
        planning_result = struct('success', true, 'path', struct('x', [], 'y', []));
        additional_info = struct('path_completion_percentage', alpha * 100);
        
        % Check stopping conditions
        [should_stop, stop_reason, stop_details] = stop_checker.check_stopping_conditions(...
            current_state, goal_state, simulation_time, planning_result, additional_info);
        
        % Print status every 10 iterations
        if mod(iteration, 10) == 0 || should_stop
            if should_stop
                status_msg = ['STOP (' stop_reason ')'];
            else
                status_msg = 'Continue';
            end
            fprintf('Time: %4.1fs, Pos: (%5.1f, %5.1f), Progress: %5.1f%% - %s\n', ...
                simulation_time, current_state.x, current_state.y, alpha * 100, status_msg);
        end
        
        % Exit if stopping condition met
        if should_stop
            fprintf('\nSimulation completed!\n');
            fprintf('Final state: (%.2f, %.2f, %.1f°, %.1f°)\n', ...
                current_state.x, current_state.y, current_state.theta, current_state.gamma);
            fprintf('Stop reason: %s\n', stop_reason);
            
            if isfield(stop_details, 'position_error')
                fprintf('Final position error: %.3fm\n', stop_details.position_error);
                fprintf('Final orientation error: %.1f°\n', stop_details.orientation_error);
            end
            
            break;
        end
    end
    
    if iteration == max_iterations
        fprintf('\nSimulation reached maximum iterations without stopping condition\n');
    end
end

%% Entry Point
if isempty(dbstack())
    main();
end
