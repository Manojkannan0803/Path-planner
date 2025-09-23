function tolerance_config = create_tolerance_config(position_tolerance, orientation_tolerance, articulation_tolerance)
    % CREATE_TOLERANCE_CONFIG - Helper function to create tolerance configuration
    %
    % Creates a standardized tolerance structure for goal checking with
    % validation and default values.
    %
    % INPUTS:
    %   position_tolerance    - double, position error tolerance in meters (default: 2.0)
    %   orientation_tolerance - double, orientation error tolerance in degrees (default: 15.0)
    %   articulation_tolerance - double, articulation error tolerance in degrees (default: 10.0)
    %
    % OUTPUTS:
    %   tolerance_config - struct with tolerance fields
    %
    % EXAMPLES:
    %   % Create tight tolerances for parking
    %   tight_tol = create_tolerance_config(0.5, 5.0, 3.0);
    %
    %   % Create relaxed tolerances for highway
    %   relaxed_tol = create_tolerance_config(3.0, 20.0, 15.0);
    %
    %   % Use defaults
    %   default_tol = create_tolerance_config();
    
    % Set default values if not provided
    if nargin < 1 || isempty(position_tolerance)
        position_tolerance = 2.0;  % meters
    end
    if nargin < 2 || isempty(orientation_tolerance)
        orientation_tolerance = 15.0;  % degrees
    end
    if nargin < 3 || isempty(articulation_tolerance)
        articulation_tolerance = 10.0;  % degrees
    end
    
    % Validate inputs
    if position_tolerance <= 0
        error('position_tolerance must be positive');
    end
    if orientation_tolerance <= 0
        error('orientation_tolerance must be positive');
    end
    if articulation_tolerance <= 0
        error('articulation_tolerance must be positive');
    end
    
    % Create tolerance structure
    tolerance_config = struct();
    tolerance_config.position_tolerance = double(position_tolerance);
    tolerance_config.orientation_tolerance = double(orientation_tolerance);
    tolerance_config.articulation_tolerance = double(articulation_tolerance);
    
    % Add descriptive metadata
    tolerance_config.description = sprintf(...
        'Pos: %.1fm, Orient: %.1f°, Artic: %.1f°', ...
        position_tolerance, orientation_tolerance, articulation_tolerance);
    
end
