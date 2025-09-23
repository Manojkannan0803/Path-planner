function vehicle_state = create_vehicle_state(x, y, theta, gamma, vehicle_type)
    % CREATE_VEHICLE_STATE - Helper function to create vehicle state structures
    %
    % Creates a standardized vehicle state structure that works with both
    % articulated and car vehicle types.
    %
    % INPUTS:
    %   x           - double, X position in meters
    %   y           - double, Y position in meters  
    %   theta       - double, heading angle in degrees
    %   gamma       - double, articulation angle in degrees (ignored for car)
    %   vehicle_type - string, 'articulated' or 'car'
    %
    % OUTPUTS:
    %   vehicle_state - struct with fields based on vehicle type
    %
    % EXAMPLES:
    %   % Create articulated vehicle state
    %   state = create_vehicle_state(60, 64, 270, 5, 'articulated');
    %
    %   % Create car vehicle state  
    %   state = create_vehicle_state(60, 64, 270, 0, 'car');
    
    % Validate inputs
    if nargin < 5
        vehicle_type = 'articulated';  % default
    end
    
    if ~ischar(vehicle_type) && ~isstring(vehicle_type)
        error('vehicle_type must be a string or char');
    end
    
    vehicle_type = lower(char(vehicle_type));
    
    if ~ismember(vehicle_type, {'articulated', 'car'})
        error('vehicle_type must be ''articulated'' or ''car''');
    end
    
    % Create base structure
    vehicle_state = struct();
    vehicle_state.x = double(x);
    vehicle_state.y = double(y);
    vehicle_state.theta = double(theta);
    
    % Add gamma field based on vehicle type
    if strcmp(vehicle_type, 'articulated')
        vehicle_state.gamma = double(gamma);
    else
        % For car vehicles, gamma is always 0 (no articulation)
        vehicle_state.gamma = 0.0;
    end
    
    % Add metadata for validation
    vehicle_state.vehicle_type = vehicle_type;
    
end
