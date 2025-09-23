%% INTEGRATION_EXAMPLE - Integration of SimulationStopChecker with Planning System
% This example shows how to integrate the SimulationStopChecker service 
% with your existing path planning system and Simulink System Objects

function main()
    fprintf('=== SIMULATION STOP CHECKER INTEGRATION EXAMPLE ===\n\n');
    
    %% 1. Integration with PlanningSession
    fprintf('1. INTEGRATION WITH PLANNING SESSION\n');
    fprintf('------------------------------------\n');
    
    try
        % Initialize planning session (this would be your actual session)
        fprintf('Creating PlanningSession...\n');
        % session = PlanningSession('scenarios/DPDScenario.json');
        
        % Initialize simulation stop checker
        stop_config = struct(...
            'goal_tolerance', struct(...
                'position_tolerance', 1.5, ...      % 1.5m position tolerance
                'orientation_tolerance', 10.0, ...  % 10° orientation tolerance
                'articulation_tolerance', 8.0), ... % 8° articulation tolerance
            'max_simulation_time', 180.0, ...      % 3 minute timeout
            'debug_mode', true);
        
        stop_checker = services.validation.SimulationStopChecker(stop_config);
        
        % Define planning scenario
        start_state = struct('x', 75, 'y', 45, 'theta', 180, 'gamma', 0);
        goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
        
        fprintf('Planning from (%.1f, %.1f) to (%.1f, %.1f)\n', ...
            start_state.x, start_state.y, goal_state.x, goal_state.y);
        
        % Simulate planning execution with stop checking
        [final_state, success, planning_stats] = simulate_planning_with_stop_check(...
            start_state, goal_state, stop_checker);
        
        if success
            fprintf('Planning completed successfully!\n');
            fprintf('Final position: (%.2f, %.2f, %.1f°)\n', ...
                final_state.x, final_state.y, final_state.theta);
        else
            fprintf('Planning terminated: %s\n', planning_stats.termination_reason);
        end
        
    catch ME
        fprintf('Planning session integration error: %s\n', ME.message);
    end
    
    %% 2. System Object Integration Pattern
    fprintf('\n2. SYSTEM OBJECT INTEGRATION PATTERN\n');
    fprintf('------------------------------------\n');
    
    demonstrate_system_object_integration();
    
    %% 3. Behavioral Planner Integration
    fprintf('\n3. BEHAVIORAL PLANNER INTEGRATION\n');
    fprintf('---------------------------------\n');
    
    demonstrate_behavioral_planner_integration();
    
    %% 4. Multi-Condition Stopping Example
    fprintf('\n4. MULTI-CONDITION STOPPING EXAMPLE\n');
    fprintf('------------------------------------\n');
    
    demonstrate_multi_condition_stopping();
    
    fprintf('\n=== INTEGRATION EXAMPLE COMPLETED ===\n');
end

function [final_state, success, stats] = simulate_planning_with_stop_check(start_state, goal_state, stop_checker)
    % Simulate a planning session with integrated stop checking
    
    current_state = start_state;
    simulation_time = 0;
    dt = 0.5; % 0.5 second time steps
    max_iterations = 200;
    
    success = false;
    planning_attempts = 0;
    consecutive_failures = 0;
    
    stats = struct('planning_attempts', 0, 'total_time', 0, 'termination_reason', '');
    
    for iteration = 1:max_iterations
        simulation_time = simulation_time + dt;
        planning_attempts = planning_attempts + 1;
        
        % Simulate path planning execution
        [planning_result, vehicle_moved] = simulate_planning_step(current_state, goal_state);
        
        % Update vehicle state if planning succeeded
        if planning_result.success && vehicle_moved
            % Move vehicle along planned path (simplified)
            progress = min(0.02 * iteration, 1.0);
            current_state.x = start_state.x + progress * (goal_state.x - start_state.x);
            current_state.y = start_state.y + progress * (goal_state.y - start_state.y);
            current_state.theta = start_state.theta + progress * (goal_state.theta - start_state.theta);
            current_state.gamma = start_state.gamma + progress * (goal_state.gamma - start_state.gamma);
            consecutive_failures = 0;
        else
            consecutive_failures = consecutive_failures + 1;
        end
        
        % Prepare additional info for stop checker
        additional_info = struct(...
            'consecutive_failures', consecutive_failures, ...
            'planning_attempts', planning_attempts, ...
            'path_completion_percentage', progress * 100);
        
        % Check stopping conditions
        [should_stop, stop_reason, stop_details] = stop_checker.check_stopping_conditions(...
            current_state, goal_state, simulation_time, planning_result, additional_info);
        
        % Print progress every 20 iterations
        if mod(iteration, 20) == 0 || should_stop
            fprintf('Time: %5.1fs, Position: (%5.1f, %5.1f), Planning attempts: %d\n', ...
                simulation_time, current_state.x, current_state.y, planning_attempts);
        end
        
        % Handle stopping condition
        if should_stop
            switch stop_reason
                case 'goal_reached'
                    success = true;
                    stats.termination_reason = 'Goal achieved';
                    fprintf('SUCCESS: Goal reached!\n');
                    fprintf('  Position error: %.3fm\n', stop_details.position_error);
                    fprintf('  Orientation error: %.1f°\n', stop_details.orientation_error);
                    
                case 'simulation_timeout'
                    stats.termination_reason = 'Simulation timeout';
                    fprintf('TIMEOUT: Simulation exceeded %.1fs\n', stop_details.max_allowed_time);
                    
                case 'planning_failure'
                    stats.termination_reason = 'Planning failure';
                    fprintf('FAILURE: %s\n', stop_details.failure_type);
                    
                otherwise
                    stats.termination_reason = stop_reason;
                    fprintf('STOPPED: %s\n', stop_reason);
            end
            break;
        end
    end
    
    final_state = current_state;
    stats.planning_attempts = planning_attempts;
    stats.total_time = simulation_time;
    
    if iteration == max_iterations && ~success
        stats.termination_reason = 'Maximum iterations reached';
    end
end

function [planning_result, vehicle_moved] = simulate_planning_step(current_state, goal_state)
    % Simulate a single planning step
    
    % Calculate distance to goal
    distance_to_goal = sqrt((current_state.x - goal_state.x)^2 + (current_state.y - goal_state.y)^2);
    
    % Simulate planning success probability (higher when closer to goal)
    success_probability = 0.8 + 0.15 * exp(-distance_to_goal / 10.0);
    planning_success = rand() < success_probability;
    
    if planning_success
        % Generate simple path
        path = struct(...
            'x', linspace(current_state.x, goal_state.x, 10), ...
            'y', linspace(current_state.y, goal_state.y, 10));
        
        planning_result = struct(...
            'success', true, ...
            'path', path, ...
            'stats', struct('planning_time', 0.1 + 0.1 * rand(), 'nodes_explored', randi([50, 300])));
        
        vehicle_moved = true;
    else
        % Planning failed
        planning_result = struct(...
            'success', false, ...
            'path', [], ...
            'stats', struct('termination_reason', 'no_feasible_path'));
        
        vehicle_moved = false;
    end
end

function demonstrate_system_object_integration()
    % Show how to integrate with Simulink System Objects
    
    fprintf('System Object Integration Pattern:\n\n');
    
    % This is how you would integrate SimulationStopChecker with a System Object
    fprintf('In your System Object stepImpl method:\n\n');
    
    fprintf('function [stop_simulation, stop_reason] = stepImpl(obj, current_pose, goal_pose, sim_time)\n');
    fprintf('    %% Update planning state\n');
    fprintf('    planning_result = obj.planning_system.execute_planning(current_pose, goal_pose);\n');
    fprintf('    \n');
    fprintf('    %% Prepare additional information\n');
    fprintf('    additional_info = struct(...\n');
    fprintf('        ''consecutive_failures'', obj.consecutive_planning_failures, ...\n');
    fprintf('        ''path_completion_percentage'', obj.path_completion_percentage, ...\n');
    fprintf('        ''collision_warning'', obj.collision_detected);\n');
    fprintf('    \n');
    fprintf('    %% Check stopping conditions\n');
    fprintf('    [stop_simulation, stop_reason, details] = obj.stop_checker.check_stopping_conditions(...\n');
    fprintf('        current_pose, goal_pose, sim_time, planning_result, additional_info);\n');
    fprintf('    \n');
    fprintf('    %% Handle stopping condition\n');
    fprintf('    if stop_simulation\n');
    fprintf('        obj.log_simulation_end(stop_reason, details);\n');
    fprintf('    end\n');
    fprintf('end\n\n');
    
    fprintf('Key integration points:\n');
    fprintf('- Initialize SimulationStopChecker in setupImpl\n');
    fprintf('- Call check_stopping_conditions in each stepImpl\n');
    fprintf('- Use stop_simulation output to control Simulink execution\n');
    fprintf('- Log stop_reason for analysis\n\n');
end

function demonstrate_behavioral_planner_integration()
    % Show integration with behavioral planning layer
    
    fprintf('Behavioral Planner Integration:\n\n');
    
    % Create hierarchical stop checker configuration
    behavioral_config = struct(...
        'goal_tolerance', struct(...
            'position_tolerance', 2.0, ...      % Relaxed for high-level goals
            'orientation_tolerance', 15.0, ...  % Relaxed for behavioral layer
            'articulation_tolerance', 10.0), ...
        'stopping_conditions', struct(...
            'enable_goal_reached', true, ...
            'enable_timeout', true, ...
            'enable_planning_failure', true, ...
            'enable_path_completion', true), ...
        'max_simulation_time', 300.0, ...      % 5 minutes for complex missions
        'max_planning_attempts', 3);
    
    behavioral_stop_checker = services.validation.SimulationStopChecker(behavioral_config);
    
    % Simulate behavioral planning with route segments
    route_segments = [
        struct('start', [75, 45, 180, 0], 'end', [70, 55, 225, -5], 'type', 'straight');
        struct('start', [70, 55, 225, -5], 'end', [65, 62, 260, -2], 'type', 'turn');
        struct('start', [65, 62, 260, -2], 'end', [60, 64, 270, 0], 'type', 'approach');
    ];
    
    fprintf('Simulating behavioral planning with %d route segments:\n', length(route_segments));
    
    current_segment = 1;
    simulation_time = 0;
    
    while current_segment <= length(route_segments)
        segment = route_segments(current_segment);
        
        % Convert segment to state structures (start_state used for initialization only)
        start_state = struct('x', segment.start(1), 'y', segment.start(2), ...
                           'theta', segment.start(3), 'gamma', segment.start(4));
        goal_state = struct('x', segment.end(1), 'y', segment.end(2), ...
                          'theta', segment.end(3), 'gamma', segment.end(4));
        
        fprintf('Segment %d (%s): (%.1f,%.1f) → (%.1f,%.1f)\n', ...
            current_segment, segment.type, segment.start(1), segment.start(2), ...
            segment.end(1), segment.end(2));
        
        % Simulate segment execution
        segment_time = 20 + 10 * rand(); % Random execution time
        simulation_time = simulation_time + segment_time;
        
        % Simulate reaching segment goal
        current_state = goal_state;
        planning_result = struct('success', true, 'path', struct('x', [], 'y', []));
        additional_info = struct('path_completion_percentage', 100);
        
        % Check if segment completed successfully
        [should_stop, stop_reason, ~] = behavioral_stop_checker.check_stopping_conditions(...
            current_state, goal_state, simulation_time, planning_result, additional_info);
        
        if should_stop && strcmp(stop_reason, 'goal_reached')
            fprintf('  Segment %d completed successfully\n', current_segment);
            current_segment = current_segment + 1;
        else
            fprintf('  Segment %d failed: %s\n', current_segment, stop_reason);
            break;
        end
    end
    
    if current_segment > length(route_segments)
        fprintf('All route segments completed successfully!\n');
    end
end

function demonstrate_multi_condition_stopping()
    % Demonstrate complex stopping conditions
    
    fprintf('Multi-Condition Stopping Configuration:\n\n');
    
    % Create comprehensive stopping configuration
    multi_config = struct(...
        'goal_tolerance', struct(...
            'position_tolerance', 1.0, ...
            'orientation_tolerance', 8.0, ...
            'articulation_tolerance', 5.0), ...
        'stopping_conditions', struct(...
            'enable_goal_reached', true, ...
            'enable_timeout', true, ...
            'enable_planning_failure', true, ...
            'enable_collision_imminent', true, ...
            'enable_path_completion', true), ...
        'max_simulation_time', 60.0, ...
        'max_planning_attempts', 2, ...
        'debug_mode', true);
    
    multi_stop_checker = services.validation.SimulationStopChecker(multi_config);
    
    % Test different stopping scenarios
    test_scenarios = {
        struct('name', 'Goal Achievement', 'current', [60.2, 64.1, 268, 1], 'test_time', 30);
        struct('name', 'Timeout Condition', 'current', [70, 50, 200, -3], 'test_time', 70);
        struct('name', 'Collision Warning', 'current', [65, 55, 220, -2], 'test_time', 25);
        struct('name', 'Planning Failure', 'current', [68, 52, 210, -1], 'test_time', 40);
    };
    
    goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
    
    for i = 1:length(test_scenarios)
        scenario = test_scenarios{i};
        fprintf('Testing %s:\n', scenario.name);
        
        current_state = struct('x', scenario.current(1), 'y', scenario.current(2), ...
                             'theta', scenario.current(3), 'gamma', scenario.current(4));
        
        % Prepare scenario-specific conditions
        switch scenario.name
            case 'Collision Warning'
                additional_info = struct('collision_warning', true, 'obstacle_distance', 0.5);
                planning_result = struct('success', true);
                
            case 'Planning Failure'
                additional_info = struct('consecutive_failures', 3);
                planning_result = struct('success', false, 'stats', struct('termination_reason', 'no_valid_path'));
                
            otherwise
                additional_info = struct();
                planning_result = struct('success', true);
        end
        
        [should_stop, stop_reason, stop_details] = multi_stop_checker.check_stopping_conditions(...
            current_state, goal_state, scenario.test_time, planning_result, additional_info);
        
        if should_stop
            result_msg = ['STOP - ' stop_reason];
        else
            result_msg = 'CONTINUE';
        end
        fprintf('  Result: %s\n', result_msg);
        
        if should_stop && isfield(stop_details, 'position_error')
            fprintf('  Position error: %.3fm\n', stop_details.position_error);
        end
        
        fprintf('\n');
    end
    
    % Display final statistics
    stats = multi_stop_checker.get_simulation_statistics();
    fprintf('Testing Statistics:\n');
    fprintf('  Total condition checks: %d\n', stats.total_checks);
    fprintf('  Goal achievements detected: %d\n', stats.goal_achievements);
    fprintf('  Timeouts detected: %d\n', stats.timeouts);
    fprintf('  Planning failures detected: %d\n', stats.planning_failures);
end

%% Entry Point
if isempty(dbstack())
    main();
end
