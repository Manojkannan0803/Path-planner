function vehicle_config = create_vehicle_config(vehicle_type)
    % CREATE_VEHICLE_CONFIG - Helper function to create vehicle configuration
    %
    % Creates a standardized vehicle configuration structure that defines
    % the vehicle type and its capabilities.
    %
    % INPUTS:
    %   vehicle_type - string, 'articulated' or 'car'
    %
    % OUTPUTS:
    %   vehicle_config - struct with vehicle configuration fields
    %
    % EXAMPLES:
    %   % Create articulated vehicle configuration
    %   config = components.goal_checker.helpers.create_vehicle_config('articulated');
    %
    %   % Create car vehicle configuration
    %   config = components.goal_checker.helpers.create_vehicle_config('car');
    
    % Validate input
    if nargin < 1
        vehicle_type = 'articulated';  % default
    end
    
    if ~ischar(vehicle_type) && ~isstring(vehicle_type)
        error('vehicle_type must be a string or char');
    end
    
    vehicle_type = lower(char(vehicle_type));
    
    if ~ismember(vehicle_type, {'articulated', 'car'})
        error('vehicle_type must be ''articulated'' or ''car''');
    end
    
    % Create vehicle configuration structure
    vehicle_config = struct();
    vehicle_config.name = vehicle_type;
    
    % Set type as numeric code for Simulink compatibility
    switch vehicle_type
        case 'articulated'
            vehicle_config.type_code = uint8(1);
            vehicle_config.has_articulation = true;
            vehicle_config.description = 'Tractor-trailer with articulation angle';
            vehicle_config.required_fields = {'x', 'y', 'theta', 'gamma'};
        case 'car'
            vehicle_config.type_code = uint8(2);
            vehicle_config.has_articulation = false;
            vehicle_config.description = 'Standard car vehicle without articulation';
            vehicle_config.required_fields = {'x', 'y', 'theta'};
    end
    
    % Add string representation for bus structures
    name_chars = uint8(zeros(1, 20));  % Fixed size array for bus compatibility
    name_str = vehicle_type;
    if length(name_str) <= 20
        name_chars(1:length(name_str)) = uint8(name_str);
    end
    vehicle_config.name_length = uint8(length(name_str));
    vehicle_config.name_chars = name_chars;
    
    % Add validation function handle
    vehicle_config.validate_state = @(state) validate_vehicle_state(state, vehicle_type);
    
end

function is_valid = validate_vehicle_state(state, vehicle_type)
    % Helper function to validate vehicle state structure
    is_valid = true;
    
    if ~isstruct(state)
        is_valid = false;
        return;
    end
    
    % Check required fields based on vehicle type
    if strcmp(vehicle_type, 'articulated')
        required_fields = {'x', 'y', 'theta', 'gamma'};
    else
        required_fields = {'x', 'y', 'theta'};
    end
    
    for i = 1:length(required_fields)
        if ~isfield(state, required_fields{i})
            is_valid = false;
            return;
        end
    end
end
