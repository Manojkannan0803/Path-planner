function [should_stop, stop_reason, goal_errors] = check_goal_reached(current_state, goal_state, position_tolerance, orientation_tolerance, articulation_tolerance, vehicle_type)
    % CHECK_GOAL_REACHED - Generic function to determine if simulation should stop
    % 
    % This function checks if the vehicle has reached the goal within specified tolerances
    % and determines whether the simulation should be stopped. Supports multiple vehicle types.
    %
    % INPUTS:
    %   current_state         - struct with fields: x, y, theta, [gamma] (current vehicle pose)
    %   goal_state           - struct with fields: x, y, theta, [gamma] (target pose)
    %   position_tolerance   - scalar, position error tolerance in meters (default: 2.0)
    %   orientation_tolerance - scalar, orientation error tolerance in degrees (default: 15.0)
    %   articulation_tolerance - scalar, articulation error tolerance in degrees (default: 10.0)
    %   vehicle_type         - string, vehicle type: 'articulated' or 'car' (default: 'articulated')
    %
    % OUTPUTS:
    %   should_stop - logical, true if simulation should stop (goal reached)
    %   stop_reason - string, reason for stopping ('goal_reached' or 'continue')
    %   goal_errors - struct with position_error, orientation_error, [articulation_error]
    %
    % VEHICLE TYPES:
    %   'articulated' - Tractor-trailer with articulation angle (gamma)
    %                   Requires: x, y, theta, gamma fields in states
    %   'car'         - Standard car vehicle model  
    %                   Requires: x, y, theta fields in states (gamma ignored)
    %
    % EXAMPLES:
    %   % Articulated vehicle (tractor-trailer)
    %   current = struct('x', 60.1, 'y', 64.2, 'theta', 268, 'gamma', 1);
    %   goal = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
    %   [stop, reason, errors] = check_goal_reached(current, goal, 1.0, 10.0, 5.0, 'articulated');
    %
    %   % Car vehicle
    %   current = struct('x', 60.1, 'y', 64.2, 'theta', 268);
    %   goal = struct('x', 60, 'y', 64, 'theta', 270);
    %   [stop, reason, errors] = check_goal_reached(current, goal, 1.0, 10.0, [], 'car');
    %
    % INTEGRATION WITH SYSTEM:
    %   % In planning loop or Simulink System Object:
    %   [should_stop, stop_reason, errors] = check_goal_reached(...
    %       current_vehicle_state, target_goal_state, 1.5, 10.0, 8.0, 'articulated');
    %   
    %   if should_stop
    %       fprintf('Goal reached! Position error: %.2fm\n', errors.position_error);
    %   end
    
    % Set default tolerances and vehicle type if not provided
    if nargin < 3 || isempty(position_tolerance)
        position_tolerance = 2.0;  % meters
    end
    if nargin < 4 || isempty(orientation_tolerance)
        orientation_tolerance = 15.0;  % degrees
    end
    if nargin < 5 || isempty(articulation_tolerance)
        articulation_tolerance = 10.0;  % degrees
    end
    if nargin < 6 || isempty(vehicle_type)
        vehicle_type = 'articulated';  % default to articulated vehicle
    end
    
    % Validate inputs
    if ~isstruct(current_state) || ~isstruct(goal_state)
        error('current_state and goal_state must be structures');
    end
    
    % Validate vehicle type
    valid_vehicle_types = {'articulated', 'car'};
    if ~ismember(lower(vehicle_type), valid_vehicle_types)
        error('vehicle_type must be ''articulated'' or ''car''');
    end
    vehicle_type = lower(vehicle_type);
    
    % Check required fields based on vehicle type
    if strcmp(vehicle_type, 'articulated')
        required_fields = {'x', 'y', 'theta', 'gamma'};
        for i = 1:length(required_fields)
            field = required_fields{i};
            if ~isfield(current_state, field) || ~isfield(goal_state, field)
                error('Articulated vehicle states must have fields: x, y, theta, gamma');
            end
        end
    elseif strcmp(vehicle_type, 'car')
        required_fields = {'x', 'y', 'theta'};
        for i = 1:length(required_fields)
            field = required_fields{i};
            if ~isfield(current_state, field) || ~isfield(goal_state, field)
                error('Car vehicle states must have fields: x, y, theta');
            end
        end
    end
    
    % Calculate position error (Euclidean distance)
    position_error = sqrt((current_state.x - goal_state.x)^2 + (current_state.y - goal_state.y)^2);
    
    % Calculate orientation error (handle angle wraparound)
    orientation_error = abs(normalize_angle(current_state.theta - goal_state.theta));
   
    % Check if all tolerances are satisfied
    position_ok = position_error <= position_tolerance;
    orientation_ok = orientation_error <= orientation_tolerance;
    
    % Calculate articulation error based on vehicle type
    if strcmp(vehicle_type, 'articulated')
        % For articulated vehicles, check gamma angle difference
        articulation_error = abs(normalize_angle(current_state.gamma - goal_state.gamma));
        articulation_ok = articulation_error <= articulation_tolerance;
    else
        % For car vehicles, articulation is not applicable
        articulation_error = 0;
        articulation_ok = true;  % Always satisfied for car vehicles
    end
    
    % Determine if goal is reached based on vehicle type
    if strcmp(vehicle_type, 'articulated')
        goal_reached = position_ok && orientation_ok && articulation_ok;
    else
        goal_reached = position_ok && orientation_ok;  % No articulation check for cars
    end
    
    % Set outputs
    if goal_reached
        should_stop = true;
        stop_reason = 'goal_reached';
    else
        should_stop = false;
        stop_reason = 'continue';
    end
    
    % Package error information
    goal_errors = struct(...
        'position_error', position_error, ...
        'orientation_error', orientation_error, ...
        'articulation_error', articulation_error, ...
        'position_ok', position_ok, ...
        'orientation_ok', orientation_ok, ...
        'articulation_ok', articulation_ok, ...
        'position_tolerance', position_tolerance, ...
        'orientation_tolerance', orientation_tolerance, ...
        'articulation_tolerance', articulation_tolerance, ...
        'vehicle_type', vehicle_type);
end

function angle_normalized = normalize_angle(angle)
    % Normalize angle to [-180, 180] range
    angle_normalized = mod(angle + 180, 360) - 180;
end
