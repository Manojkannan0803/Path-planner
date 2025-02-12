function bus_elem = create_bus_elements(name, dim, type, check)

if nargin < 4
    check = true; % default enabled
end

bus_elem = Simulink.BusElement;
bus_elem.Name = name;
bus_elem.Dimensions = dim;
bus_elem.DataType = type;
bus_elem.DimensionsMode = 'Fixed';
bus_elem.SampleTime = -1;
bus_elem.Complexity = 'real';


if check
    if prod(dim) > 1
        if ~ismember(type, {'int8', 'int16', 'int32', 'uint8', 'uint16', 'uint32', 'boolean', 'double', 'single'})
            % if not a primitive type, then it's probably a bus (or the list is incomplete?)
            warning('array of buses (name %s, type %s, size %d) is not supported for code generation', name, type, prod(dim)); % warning instead of error, because bus might not be on external interface
        end
    end
end