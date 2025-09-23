function bus_obj = create_bus_object(name, elements, description)
    % CREATE_BUS_OBJECT - Helper to create a Simulink.Bus object
    %
    % INPUTS:
    %   name - string, name of the bus
    %   elements - array of Simulink.BusElement objects
    %   description - string, description of the bus
    %
    % OUTPUTS:
    %   bus_obj - Simulink.Bus object
    
    bus_obj = Simulink.Bus;
    bus_obj.Elements = elements;
    if nargin > 2 && ~isempty(description)
        bus_obj.Description = description;
    end
    
    fprintf('  Created %s with %d elements\n', name, length(elements));
end