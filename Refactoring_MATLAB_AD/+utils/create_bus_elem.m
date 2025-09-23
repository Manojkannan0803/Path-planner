function bus_elem = create_bus_elem(name, dim, type, varargin)
    % CREATE_BUS_ELEM - Centralized function to create Simulink bus elements
    %
    % This function provides a standardized way to create bus elements with
    % validation and consistent properties for code generation compatibility.
    %
    % INPUTS:
    %   name     - string, name of the bus element
    %   dim      - scalar or vector, dimensions of the element (default: 1)
    %   type     - string, data type ('double', 'boolean', 'int32', etc.)
    %   varargin - optional name-value pairs:
    %     'Description'  - string, description of the element
    %     'Min'          - scalar, minimum value constraint
    %     'Max'          - scalar, maximum value constraint  
    %     'Units'        - string, units of the element
    %     'Check'        - logical, enable array validation (default: true)
    %     'SampleTime'   - scalar, sample time (default: -1)
    %     'Complexity'   - string, 'real' or 'complex' (default: 'real')
    %
    % OUTPUTS:
    %   bus_elem - Simulink.BusElement object
    %
    % EXAMPLES:
    %   % Simple double element
    %   elem = create_bus_elem('x', 1, 'double');
    %
    %   % Element with constraints and description
    %   elem = create_bus_elem('position', 1, 'double', ...
    %       'Description', 'Vehicle position in meters', ...
    %       'Min', -1000, 'Max', 1000, 'Units', 'm');
    %
    %   % Boolean element
    %   elem = create_bus_elem('is_valid', 1, 'boolean', ...
    %       'Description', 'Validity flag');
    %
    %   % Array element (with validation)
    %   elem = create_bus_elem('waypoints', [10, 2], 'double', ...
    %       'Description', 'Array of waypoint coordinates');
    
    % Parse inputs
    p = inputParser;
    addRequired(p, 'name', @(x) ischar(x) || isstring(x));
    addRequired(p, 'dim', @isnumeric);
    addRequired(p, 'type', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Description', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Min', [], @isnumeric);
    addParameter(p, 'Max', [], @isnumeric);
    addParameter(p, 'Units', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Check', true, @islogical);
    addParameter(p, 'SampleTime', -1, @isnumeric);
    addParameter(p, 'Complexity', 'real', @(x) ismember(x, {'real', 'complex'}));
    
    parse(p, name, dim, type, varargin{:});
    
    % Extract parsed values
    name = char(p.Results.name);
    dim = p.Results.dim;
    type = char(p.Results.type);
    description = char(p.Results.Description);
    min_val = p.Results.Min;
    max_val = p.Results.Max;
    units = char(p.Results.Units);
    check = p.Results.Check;
    sample_time = p.Results.SampleTime;
    complexity = p.Results.Complexity;
    
    % Validate dimensions
    if isscalar(dim)
        dim = double(dim);
    else
        dim = double(dim(:)');  % Ensure row vector
    end
    
    % Create bus element
    bus_elem = Simulink.BusElement;
    bus_elem.Name = name;
    bus_elem.Dimensions = dim;
    bus_elem.DataType = type;
    bus_elem.DimensionsMode = 'Fixed';
    bus_elem.SampleTime = sample_time;
    bus_elem.Complexity = complexity;
    
    % Set optional properties
    if ~isempty(description)
        bus_elem.Description = description;
    end
    
    if ~isempty(min_val)
        bus_elem.Min = min_val;
    end
    
    if ~isempty(max_val)
        bus_elem.Max = max_val;
    end
    
    if ~isempty(units)
        bus_elem.Units = units;
    end
    
    % Validation for code generation compatibility
    if check
        validate_bus_element(name, type, dim);
    end
end

function validate_bus_element(name, type, dim)
    % VALIDATE_BUS_ELEMENT - Check for code generation compatibility
    %
    % Validates bus elements to ensure they are compatible with code generation.
    % Arrays of buses are not supported and will generate warnings.
    
    % List of primitive data types supported by Simulink Coder
    primitive_types = {
        'int8', 'int16', 'int32', 'int64', ...
        'uint8', 'uint16', 'uint32', 'uint64', ...
        'boolean', 'logical', ...
        'double', 'single', ...
        'fixdt(1,16)', 'fixdt(0,16)', 'sfix16', 'ufix16', ...  % Fixed-point types
        'Enum:', 'Bus:'  % Special prefixes
    };
    
    % Check if it's a primitive type or has special prefix
    is_primitive = false;
    for i = 1:length(primitive_types)
        if strcmp(type, primitive_types{i}) || startsWith(type, primitive_types{i})
            is_primitive = true;
            break;
        end
    end
    
    % Check for arrays of non-primitive types (likely buses)
    if prod(dim) > 1
        if ~is_primitive
            warning('GOALCHECKER:ArrayOfBuses', ...
                ['Array of buses (name: %s, type: %s, size: %d) is not supported for code generation.\n' ...
                 'Consider restructuring as separate scalar bus elements or using primitive arrays.\n' ...
                 'For more info, see documentation on Simulink Coder limitations.'], ...
                name, type, prod(dim));
        end
    end
    
    % Additional validation for common issues
    if prod(dim) == 0
        warning('GOALCHECKER:ZeroDimension', ...
            'Bus element "%s" has zero dimensions, which may cause issues.', name);
    end
    
    if any(dim < 0)
        error('GOALCHECKER:NegativeDimension', ...
            'Bus element "%s" has negative dimensions, which is not allowed.', name);
    end
end
