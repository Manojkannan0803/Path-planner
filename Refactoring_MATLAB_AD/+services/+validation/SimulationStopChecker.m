classdef SimulationStopChecker < handle
    % SIMULATIONSTOPCHECKER - Service for determining simulation stop conditions
    % Integrates with the 5-layer path planning architecture to provide
    % intelligent stopping conditions based on goal achievement and planning state
    
    properties (Access = private)
        goal_tolerance          % Position and orientation tolerances for goal achievement
        stopping_conditions     % Configuration for different stopping criteria
        max_simulation_time     % Maximum allowed simulation time
        max_planning_attempts   % Maximum planning attempts before failure
        vehicle_model           % Vehicle dimensions for accurate goal checking
        debug_mode              % Enable detailed logging and diagnostics
    end
    
    properties (SetAccess = private)
        last_check_result       % Result of last stopping condition check
        simulation_statistics   % Track simulation performance metrics
    end
    
    methods
        function obj = SimulationStopChecker(config)
            % Constructor with configuration
            
            % Default tolerances for tractor-trailer vehicle
            obj.goal_tolerance = struct(...
                'position_tolerance', 2.0, ...          % meters
                'orientation_tolerance', 15.0, ...      % degrees
                'articulation_tolerance', 10.0);        % degrees (gamma)
            
            % Default stopping conditions
            obj.stopping_conditions = struct(...
                'enable_goal_reached', true, ...
                'enable_timeout', true, ...
                'enable_planning_failure', true, ...
                'enable_collision_imminent', false, ...
                'enable_path_completion', true);
            
            % Default limits
            obj.max_simulation_time = 300.0;    % 5 minutes
            obj.max_planning_attempts = 5;      % Max replanning attempts
            
            % Default vehicle model (compatible with ObstacleChecker)
            obj.vehicle_model = struct(...
                'tractor_length', 6.0, ...
                'tractor_width', 2.5, ...
                'trailer_length', 10.5, ...
                'trailer_width', 2.5, ...
                'hitch_offset', 3.0);
            
            obj.debug_mode = false;
            
            % Initialize statistics
            obj.simulation_statistics = struct(...
                'total_checks', 0, ...
                'goal_achievements', 0, ...
                'timeouts', 0, ...
                'planning_failures', 0, ...
                'start_time', [], ...
                'last_check_time', []);
            
            obj.last_check_result = struct();
            
            % Override with provided configuration
            if nargin > 0
                obj.apply_configuration(config);
            end
        end
        
        function [should_stop, stop_reason, stop_details] = check_stopping_conditions(obj, current_state, goal_state, simulation_time, planning_result, additional_info)
            % Primary function to check all stopping conditions
            % 
            % Inputs:
            %   current_state   - Current vehicle state [x, y, theta, gamma]
            %   goal_state      - Target goal state [x, y, theta, gamma]
            %   simulation_time - Current simulation time (seconds)
            %   planning_result - Result from latest planning attempt
            %   additional_info - Optional struct with additional context
            %
            % Outputs:
            %   should_stop     - Boolean indicating if simulation should stop
            %   stop_reason     - String describing why simulation should stop
            %   stop_details    - Detailed information about stopping condition
            
            obj.simulation_statistics.total_checks = obj.simulation_statistics.total_checks + 1;
            obj.simulation_statistics.last_check_time = simulation_time;
            
            if isempty(obj.simulation_statistics.start_time)
                obj.simulation_statistics.start_time = simulation_time;
            end
            
            % Set default values for optional inputs
            if nargin < 6
                additional_info = struct();
            end
            if nargin < 5
                planning_result = struct('success', true, 'path', [], 'stats', struct());
            end
            
            % Initialize return values
            should_stop = false;
            stop_reason = '';
            stop_details = struct('continue_simulation', true);
            
            % Check stopping conditions in priority order
            
            % 1. GOAL ACHIEVEMENT CHECK (highest priority)
            if obj.stopping_conditions.enable_goal_reached
                [goal_reached, goal_details] = obj.check_goal_reached(current_state, goal_state);
                if goal_reached
                    should_stop = true;
                    stop_reason = 'goal_reached';
                    stop_details = goal_details;
                    obj.simulation_statistics.goal_achievements = obj.simulation_statistics.goal_achievements + 1;
                    
                    if obj.debug_mode
                        fprintf('[SimStopChecker] Goal reached! Position error: %.2fm, Orientation error: %.1f°\n', ...
                            goal_details.position_error, goal_details.orientation_error);
                    end
                    
                    obj.last_check_result = obj.create_check_result(should_stop, stop_reason, stop_details);
                    return;
                end
            end
            
            % 2. SIMULATION TIMEOUT CHECK
            if obj.stopping_conditions.enable_timeout
                [timeout_reached, timeout_details] = obj.check_timeout(simulation_time);
                if timeout_reached
                    should_stop = true;
                    stop_reason = 'simulation_timeout';
                    stop_details = timeout_details;
                    obj.simulation_statistics.timeouts = obj.simulation_statistics.timeouts + 1;
                    
                    if obj.debug_mode
                        fprintf('[SimStopChecker] Simulation timeout reached: %.1fs\n', simulation_time);
                    end
                    
                    obj.last_check_result = obj.create_check_result(should_stop, stop_reason, stop_details);
                    return;
                end
            end
            
            % 3. PLANNING FAILURE CHECK
            if obj.stopping_conditions.enable_planning_failure
                [planning_failed, failure_details] = obj.check_planning_failure(planning_result, additional_info);
                if planning_failed
                    should_stop = true;
                    stop_reason = 'planning_failure';
                    stop_details = failure_details;
                    obj.simulation_statistics.planning_failures = obj.simulation_statistics.planning_failures + 1;
                    
                    if obj.debug_mode
                        fprintf('[SimStopChecker] Planning failure detected: %s\n', failure_details.failure_type);
                    end
                    
                    obj.last_check_result = obj.create_check_result(should_stop, stop_reason, stop_details);
                    return;
                end
            end
            
            % 4. PATH COMPLETION CHECK
            if obj.stopping_conditions.enable_path_completion
                [path_completed, completion_details] = obj.check_path_completion(current_state, planning_result, additional_info);
                if path_completed
                    should_stop = true;
                    stop_reason = 'path_completed';
                    stop_details = completion_details;
                    
                    if obj.debug_mode
                        fprintf('[SimStopChecker] Path completed successfully\n');
                    end
                    
                    obj.last_check_result = obj.create_check_result(should_stop, stop_reason, stop_details);
                    return;
                end
            end
            
            % 5. COLLISION IMMINENT CHECK (if enabled)
            if obj.stopping_conditions.enable_collision_imminent
                [collision_imminent, collision_details] = obj.check_collision_imminent(current_state, additional_info);
                if collision_imminent
                    should_stop = true;
                    stop_reason = 'collision_imminent';
                    stop_details = collision_details;
                    
                    if obj.debug_mode
                        fprintf('[SimStopChecker] Collision imminent - emergency stop\n');
                    end
                    
                    obj.last_check_result = obj.create_check_result(should_stop, stop_reason, stop_details);
                    return;
                end
            end
            
            % No stopping condition met
            stop_details = struct('continue_simulation', true, 'checks_performed', obj.simulation_statistics.total_checks);
            obj.last_check_result = obj.create_check_result(should_stop, stop_reason, stop_details);
        end
        
        function [goal_reached, details] = check_goal_reached(obj, current_state, goal_state)
            % Check if vehicle has reached the goal within tolerances
            
            % Calculate position error
            position_error = sqrt((current_state.x - goal_state.x)^2 + (current_state.y - goal_state.y)^2);
            
            % Calculate orientation error (handle angle wraparound)
            orientation_error = abs(obj.normalize_angle(current_state.theta - goal_state.theta));
            
            % Calculate articulation error (gamma)
            if isfield(current_state, 'gamma') && isfield(goal_state, 'gamma')
                articulation_error = abs(obj.normalize_angle(current_state.gamma - goal_state.gamma));
            else
                articulation_error = 0;  % If gamma not provided, assume acceptable
            end
            
            % Check if all tolerances are satisfied
            position_ok = position_error <= obj.goal_tolerance.position_tolerance;
            orientation_ok = orientation_error <= obj.goal_tolerance.orientation_tolerance;
            articulation_ok = articulation_error <= obj.goal_tolerance.articulation_tolerance;
            
            goal_reached = position_ok && orientation_ok && articulation_ok;
            
            details = struct(...
                'position_error', position_error, ...
                'orientation_error', orientation_error, ...
                'articulation_error', articulation_error, ...
                'position_ok', position_ok, ...
                'orientation_ok', orientation_ok, ...
                'articulation_ok', articulation_ok, ...
                'tolerances_used', obj.goal_tolerance);
        end
        
        function [timeout_reached, details] = check_timeout(obj, simulation_time)
            % Check if simulation has exceeded maximum allowed time
            
            timeout_reached = simulation_time >= obj.max_simulation_time;
            
            details = struct(...
                'current_time', simulation_time, ...
                'max_allowed_time', obj.max_simulation_time, ...
                'time_remaining', max(0, obj.max_simulation_time - simulation_time));
        end
        
        function [planning_failed, details] = check_planning_failure(obj, planning_result, additional_info)
            % Check if planning has failed beyond recovery
            
            planning_failed = false;
            failure_type = 'none';
            
            % Check if planning result indicates failure
            if isfield(planning_result, 'success') && ~planning_result.success
                % Count consecutive failures
                if isfield(additional_info, 'consecutive_failures')
                    consecutive_failures = additional_info.consecutive_failures;
                else
                    consecutive_failures = 1;
                end
                
                if consecutive_failures >= obj.max_planning_attempts
                    planning_failed = true;
                    failure_type = 'max_attempts_exceeded';
                end
            end
            
            % Check for specific failure reasons
            if isfield(planning_result, 'stats') && isfield(planning_result.stats, 'termination_reason')
                termination_reason = planning_result.stats.termination_reason;
                
                % Certain termination reasons indicate unrecoverable failure
                unrecoverable_reasons = {'invalid_start_state', 'invalid_goal_state', 'no_valid_path'};
                if ismember(termination_reason, unrecoverable_reasons)
                    planning_failed = true;
                    failure_type = termination_reason;
                end
            end
            
            details = struct(...
                'failure_type', failure_type, ...
                'planning_success', isfield(planning_result, 'success') && planning_result.success, ...
                'consecutive_failures', isfield(additional_info, 'consecutive_failures') && additional_info.consecutive_failures || 0, ...
                'max_attempts', obj.max_planning_attempts);
        end
        
        function [path_completed, details] = check_path_completion(obj, current_state, planning_result, additional_info)
            % Check if the planned path has been successfully completed
            
            path_completed = false;
            completion_percentage = 0;
            
            % Check if path following is complete
            if isfield(additional_info, 'path_following_complete') && additional_info.path_following_complete
                path_completed = true;
                completion_percentage = 100;
            elseif isfield(additional_info, 'path_completion_percentage')
                completion_percentage = additional_info.path_completion_percentage;
                path_completed = completion_percentage >= 100;
            elseif isfield(planning_result, 'path') && ~isempty(planning_result.path)
                % Estimate completion based on distance to path end
                path = planning_result.path;
                if length(path.x) > 0
                    end_point = [path.x(end), path.y(end)];
                    current_point = [current_state.x, current_state.y];
                    distance_to_end = norm(end_point - current_point);
                    
                    % Consider path completed if very close to end
                    if distance_to_end <= obj.goal_tolerance.position_tolerance
                        path_completed = true;
                        completion_percentage = 100;
                    end
                end
            end
            
            details = struct(...
                'completion_percentage', completion_percentage, ...
                'path_available', isfield(planning_result, 'path') && ~isempty(planning_result.path), ...
                'following_complete', isfield(additional_info, 'path_following_complete') && additional_info.path_following_complete);
        end
        
        function [collision_imminent, details] = check_collision_imminent(~, ~, additional_info)
            % Check if collision is imminent (emergency stop condition)
            
            collision_imminent = false;
            
            % Check if collision warning is provided
            if isfield(additional_info, 'collision_warning') && additional_info.collision_warning
                collision_imminent = true;
            end
            
            % Check if obstacle checker reports imminent collision
            if isfield(additional_info, 'obstacle_distance') && additional_info.obstacle_distance < 1.0
                collision_imminent = true;
            end
            
            details = struct(...
                'collision_warning', isfield(additional_info, 'collision_warning') && additional_info.collision_warning, ...
                'obstacle_distance', isfield(additional_info, 'obstacle_distance') && additional_info.obstacle_distance || inf, ...
                'emergency_stop', collision_imminent);
        end
        
        function result = create_check_result(obj, should_stop, stop_reason, stop_details)
            % Create standardized check result structure
            
            result = struct(...
                'should_stop', should_stop, ...
                'stop_reason', stop_reason, ...
                'stop_details', stop_details, ...
                'check_time', obj.simulation_statistics.last_check_time, ...
                'total_checks', obj.simulation_statistics.total_checks);
        end
        
        function angle_normalized = normalize_angle(~, angle)
            % Normalize angle to [-180, 180] range
            angle_normalized = mod(angle + 180, 360) - 180;
        end
        
        function apply_configuration(obj, config)
            % Apply configuration settings
            
            if isfield(config, 'goal_tolerance')
                obj.goal_tolerance = obj.merge_structs(obj.goal_tolerance, config.goal_tolerance);
            end
            
            if isfield(config, 'stopping_conditions')
                obj.stopping_conditions = obj.merge_structs(obj.stopping_conditions, config.stopping_conditions);
            end
            
            if isfield(config, 'max_simulation_time')
                obj.max_simulation_time = config.max_simulation_time;
            end
            
            if isfield(config, 'max_planning_attempts')
                obj.max_planning_attempts = config.max_planning_attempts;
            end
            
            if isfield(config, 'vehicle_model')
                obj.vehicle_model = config.vehicle_model;
            end
            
            if isfield(config, 'debug_mode')
                obj.debug_mode = config.debug_mode;
            end
        end
        
        function merged = merge_structs(~, struct1, struct2)
            % Merge two structures, with struct2 taking precedence
            merged = struct1;
            fields = fieldnames(struct2);
            for i = 1:length(fields)
                merged.(fields{i}) = struct2.(fields{i});
            end
        end
        
        % Public getter methods
        function tolerance = get_goal_tolerance(obj)
            tolerance = obj.goal_tolerance;
        end
        
        function conditions = get_stopping_conditions(obj)
            conditions = obj.stopping_conditions;
        end
        
        function stats = get_simulation_statistics(obj)
            stats = obj.simulation_statistics;
        end
        
        % Public setter methods
        function set_goal_tolerance(obj, tolerance)
            obj.goal_tolerance = obj.merge_structs(obj.goal_tolerance, tolerance);
        end
        
        function set_stopping_conditions(obj, conditions)
            obj.stopping_conditions = obj.merge_structs(obj.stopping_conditions, conditions);
        end
        
        function set_max_simulation_time(obj, max_time)
            obj.max_simulation_time = max_time;
        end
        
        function set_debug_mode(obj, enable_debug)
            obj.debug_mode = enable_debug;
        end
        
        function reset_statistics(obj)
            % Reset simulation statistics
            obj.simulation_statistics = struct(...
                'total_checks', 0, ...
                'goal_achievements', 0, ...
                'timeouts', 0, ...
                'planning_failures', 0, ...
                'start_time', [], ...
                'last_check_time', []);
        end
    end
end
