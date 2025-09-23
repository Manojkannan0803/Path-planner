function [should_stop, stop_reason, goal_errors] = check_goal_reached_bus(input_bus)
    % CHECK_GOAL_REACHED_BUS - Goal checking function using bus structures
    %
    % This function is an enhanced version of check_goal_reached that uses
    % standardized bus structures for cleaner interface and better Simulink integration.
    %
    % INPUTS:
    %   input_bus - struct with fields:
    %     .current_state   - VehicleStateBus (x, y, theta, gamma)
    %     .goal_state      - VehicleStateBus (x, y, theta, gamma)  
    %     .tolerances      - ToleranceBus (position, orientation, articulation tolerances)
    %     .vehicle_config  - VehicleConfigBus (type, name, has_articulation)
    %
    % OUTPUTS:
    %   should_stop  - logical, true if simulation should stop (goal reached)
    %   stop_reason  - string, reason for stopping ('goal_reached' or 'continue')
    %   goal_errors  - GoalCheckOutputBus struct with detailed error information
    %
    % EXAMPLES:
    %   % Create input using helper functions
    %   input_bus.current_state = components.goal_checker.helpers.create_vehicle_state(60.1, 64.2, 268, 1, 'articulated');
    %   input_bus.goal_state = components.goal_checker.helpers.create_vehicle_state(60, 64, 270, 0, 'articulated');
    %   input_bus.tolerances = components.goal_checker.helpers.create_tolerance_config(1.0, 10.0, 5.0);
    %   input_bus.vehicle_config = components.goal_checker.helpers.create_vehicle_config('articulated');
    %   
    %   [stop, reason, errors] = check_goal_reached_bus(input_bus);
    %
    % SEE ALSO: components.goal_checker.core.check_goal_reached
    
    % Validate input structure
    if ~isstruct(input_bus)
        error('input_bus must be a structure');
    end
    
    required_fields = {'current_state', 'goal_state', 'tolerances', 'vehicle_config'};
    for i = 1:length(required_fields)
        if ~isfield(input_bus, required_fields{i})
            error('input_bus must contain field: %s', required_fields{i});
        end
    end
    
    % Extract components from bus
    current_state = input_bus.current_state;
    goal_state = input_bus.goal_state;
    tolerances = input_bus.tolerances;
    vehicle_config = input_bus.vehicle_config;
    
    % Get vehicle type
    if isfield(vehicle_config, 'name')
        % Handle bus structure with name field (character array)
        if isnumeric(vehicle_config.name)
            % Convert from uint8 character array to string
            vehicle_name_chars = vehicle_config.name(vehicle_config.name ~= 0);  % Remove padding zeros
            vehicle_type = char(vehicle_name_chars);
        else
            % Direct string assignment
            vehicle_type = vehicle_config.name;
        end
    else
        error('vehicle_config must contain name field');
    end
    
    % Determine vehicle capabilities based on type
    has_articulation = strcmp(vehicle_type, 'articulated');
    
    % Validate vehicle states
    if isfield(vehicle_config, 'validate_state')
        if ~vehicle_config.validate_state(current_state)
            error('current_state is invalid for vehicle type: %s', vehicle_type);
        end
        if ~vehicle_config.validate_state(goal_state)
            error('goal_state is invalid for vehicle type: %s', vehicle_type);
        end
    end
    
    % Extract tolerances
    if isfield(tolerances, 'position_tolerance')
        position_tolerance = tolerances.position_tolerance;
    else
        position_tolerance = 2.0;  % default
    end
    
    if isfield(tolerances, 'orientation_tolerance')
        orientation_tolerance = tolerances.orientation_tolerance;
    else
        orientation_tolerance = 15.0;  % default
    end
    
    if isfield(tolerances, 'articulation_tolerance')
        articulation_tolerance = tolerances.articulation_tolerance;
    else
        articulation_tolerance = 10.0;  % default
    end
    
    % Calculate position error (Euclidean distance)
    position_error = sqrt((current_state.x - goal_state.x)^2 + (current_state.y - goal_state.y)^2);
    
    % Calculate orientation error (handle angle wraparound)
    orientation_error = abs(normalize_angle(current_state.theta - goal_state.theta));
   
    % Check if position and orientation tolerances are satisfied
    position_ok = position_error <= position_tolerance;
    orientation_ok = orientation_error <= orientation_tolerance;
    
    % Calculate articulation error based on vehicle type
    if has_articulation
        % For articulated vehicles, check gamma angle difference
        articulation_error = abs(normalize_angle(current_state.gamma - goal_state.gamma));
        articulation_ok = articulation_error <= articulation_tolerance;
    else
        % For car vehicles, articulation is not applicable
        articulation_error = 0;
        articulation_ok = true;  % Always satisfied for car vehicles
    end
    
    % Determine if goal is reached based on vehicle type
    if has_articulation
        goal_reached = position_ok && orientation_ok && articulation_ok;
    else
        goal_reached = position_ok && orientation_ok;  % No articulation check for cars
    end
    
    % Set outputs
    if goal_reached
        should_stop = true;
        stop_reason = 'goal_reached';
        stop_reason_code = 1;
    else
        should_stop = false;
        stop_reason = 'continue';
        stop_reason_code = 0;
    end
    
    % Package error information in bus format
    goal_errors = struct(...
        'should_stop', should_stop, ...
        'stop_reason_code', stop_reason_code, ...
        'position_error', position_error, ...
        'orientation_error', orientation_error, ...
        'articulation_error', articulation_error, ...
        'position_ok', position_ok, ...
        'orientation_ok', orientation_ok, ...
        'articulation_ok', articulation_ok);
    
    % Add additional context for debugging
    goal_errors.debug_info = struct(...
        'position_tolerance', position_tolerance, ...
        'orientation_tolerance', orientation_tolerance, ...
        'articulation_tolerance', articulation_tolerance, ...
        'vehicle_type', vehicle_type, ...
        'has_articulation', has_articulation);
        
end

function angle_normalized = normalize_angle(angle)
    % Normalize angle to [-180, 180] range
    angle_normalized = mod(angle + 180, 360) - 180;
end
