function create_bus_definitions(use_advanced_features)
    % CREATE_BUS_DEFINITIONS - Creates all vehicle goal checking bus structures
    %
    % This function creates standardized bus structures using the centralized
    % bus element creation system for consistent and validated bus definitions.
    %
    % INPUTS:
    %   use_advanced_features - logical, use advanced Simulink features like Units (default: false)
    %
    % CREATED BUS STRUCTURES:
    %   1. VehicleStateBus - Vehicle state (position, orientation, articulation)
    %   2. ToleranceBus - Error tolerances for goal checking
    %   3. VehicleConfigBus - Vehicle type configuration
    %   4. GoalCheckInputBus - Complete input structure
    %   5. GoalCheckOutputBus - Complete output structure
    %
    % USAGE:
    %   components.goal_checker.bus_structures.create_bus_definitions();
    %   % or for advanced features (newer Simulink versions):
    %   components.goal_checker.bus_structures.create_bus_definitions(true);
    
    if nargin < 1
        use_advanced_features = false;  % Default to maximum compatibility
    end
    
    fprintf('Creating vehicle goal checking bus structures...\n');
    if use_advanced_features
        fprintf('Using advanced Simulink features (Units, etc.)\n');
    else
        fprintf('Using basic features for maximum compatibility\n');
    end
    fprintf('\n');
    
    % Choose appropriate bus element creator
    if use_advanced_features
        create_elem = @components.goal_checker.bus_structures.create_bus_elem;
    else
        create_elem = @components.goal_checker.bus_structures.create_bus_elem_simple;
    end
    
    %% 1. VEHICLE STATE BUS
    fprintf('1. Creating VehicleStateBus...\n');
    
    if use_advanced_features
        vehicle_state_elems = [
            create_elem('x', 1, 'double', ...
                'Description', 'Vehicle X position in meters', ...
                'Min', -1000, 'Max', 1000, 'Units', 'm');
            create_elem('y', 1, 'double', ...
                'Description', 'Vehicle Y position in meters', ...
                'Min', -1000, 'Max', 1000, 'Units', 'm');
            create_elem('theta', 1, 'double', ...
                'Description', 'Vehicle heading angle in degrees', ...
                'Min', -360, 'Max', 360, 'Units', 'deg');
            create_elem('gamma', 1, 'double', ...
                'Description', 'Articulation angle in degrees (0 for car vehicles)', ...
                'Min', -90, 'Max', 90, 'Units', 'deg');
        ];
    else
        vehicle_state_elems = [
            create_elem('x', 1, 'double', ...
                'Description', 'Vehicle X position in meters', ...
                'Min', -1000, 'Max', 1000);
            create_elem('y', 1, 'double', ...
                'Description', 'Vehicle Y position in meters', ...
                'Min', -1000, 'Max', 1000);
            create_elem('theta', 1, 'double', ...
                'Description', 'Vehicle heading angle in degrees', ...
                'Min', -360, 'Max', 360);
            create_elem('gamma', 1, 'double', ...
                'Description', 'Articulation angle in degrees (0 for car vehicles)', ...
                'Min', -90, 'Max', 90);
        ];
    end
    
    VehicleStateBus = create_bus_object('VehicleStateBus', vehicle_state_elems, ...
        'Vehicle state containing position, orientation, and articulation angle');
    
    %% 2. TOLERANCE BUS
    fprintf('2. Creating ToleranceBus...\n');
    
    if use_advanced_features
        tolerance_elems = [
            create_elem('position_tolerance', 1, 'double', ...
                'Description', 'Position error tolerance in meters', ...
                'Min', 0, 'Max', 100, 'Units', 'm');
            create_elem('orientation_tolerance', 1, 'double', ...
                'Description', 'Orientation error tolerance in degrees', ...
                'Min', 0, 'Max', 180, 'Units', 'deg');
            create_elem('articulation_tolerance', 1, 'double', ...
                'Description', 'Articulation error tolerance in degrees', ...
                'Min', 0, 'Max', 90, 'Units', 'deg');
        ];
    else
        tolerance_elems = [
            create_elem('position_tolerance', 1, 'double', ...
                'Description', 'Position error tolerance in meters', ...
                'Min', 0, 'Max', 100);
            create_elem('orientation_tolerance', 1, 'double', ...
                'Description', 'Orientation error tolerance in degrees', ...
                'Min', 0, 'Max', 180);
            create_elem('articulation_tolerance', 1, 'double', ...
                'Description', 'Articulation error tolerance in degrees', ...
                'Min', 0, 'Max', 90);
        ];
    end
    
    ToleranceBus = create_bus_object('ToleranceBus', tolerance_elems, ...
        'Error tolerance settings for goal achievement checking');
    
    %% 3. VEHICLE CONFIG BUS
    fprintf('3. Creating VehicleConfigBus...\n');
    
    vehicle_config_elems = [
        create_elem('type_code', 1, 'uint8', ...
            'Description', 'Vehicle type code: 1=articulated, 2=car', ...
            'Min', 1, 'Max', 2);
        create_elem('has_articulation', 1, 'boolean', ...
            'Description', 'True if vehicle has articulation capability');
        create_elem('name_length', 1, 'uint8', ...
            'Description', 'Length of vehicle type name string', ...
            'Min', 0, 'Max', 20);
        create_elem('name_chars', [1, 20], 'uint8', ...
            'Description', 'Vehicle type name as character array', ...
            'Min', 0, 'Max', 255);
    ];
    
    VehicleConfigBus = create_bus_object('VehicleConfigBus', vehicle_config_elems, ...
        'Vehicle type configuration and capabilities');
    
    %% 4. GOAL CHECK INPUT BUS
    fprintf('4. Creating GoalCheckInputBus...\n');
    
    input_elems = [
        create_elem('current_state', 1, 'Bus: VehicleStateBus', ...
            'Description', 'Current vehicle state');
        create_elem('goal_state', 1, 'Bus: VehicleStateBus', ...
            'Description', 'Target goal state');
        create_elem('tolerances', 1, 'Bus: ToleranceBus', ...
            'Description', 'Error tolerances for goal checking');
        create_elem('vehicle_config', 1, 'Bus: VehicleConfigBus', ...
            'Description', 'Vehicle type configuration');
    ];
    
    GoalCheckInputBus = create_bus_object('GoalCheckInputBus', input_elems, ...
        'Complete input structure for goal checking function');
    
    %% 5. GOAL CHECK OUTPUT BUS
    fprintf('5. Creating GoalCheckOutputBus...\n');
    
    output_elems = [
        create_elem('should_stop', 1, 'boolean', ...
            'Description', 'True if simulation should stop (goal reached)');
        create_elem('stop_reason_code', 1, 'uint8', ...
            'Description', 'Reason code: 1=goal_reached, 0=continue', ...
            'Min', 0, 'Max', 1);
        create_elem('position_error', 1, 'double', ...
            'Description', 'Current position error in meters', ...
            'Min', 0, 'Max', 1000, 'Units', 'm');
        create_elem('orientation_error', 1, 'double', ...
            'Description', 'Current orientation error in degrees', ...
            'Min', 0, 'Max', 180, 'Units', 'deg');
        create_elem('articulation_error', 1, 'double', ...
            'Description', 'Current articulation error in degrees', ...
            'Min', 0, 'Max', 90, 'Units', 'deg');
        create_elem('position_ok', 1, 'boolean', ...
            'Description', 'True if position error is within tolerance');
        create_elem('orientation_ok', 1, 'boolean', ...
            'Description', 'True if orientation error is within tolerance');
        create_elem('articulation_ok', 1, 'boolean', ...
            'Description', 'True if articulation error is within tolerance');
    ];
    
    GoalCheckOutputBus = create_bus_object('GoalCheckOutputBus', output_elems, ...
        'Complete output structure for goal checking function');
    
    %% 6. SAVE TO WORKSPACE
    fprintf('\n6. Saving bus structures to workspace...\n');
    
    % Save all bus structures to base workspace
    assignin('base', 'VehicleStateBus', VehicleStateBus);
    assignin('base', 'ToleranceBus', ToleranceBus);
    assignin('base', 'VehicleConfigBus', VehicleConfigBus);
    assignin('base', 'GoalCheckInputBus', GoalCheckInputBus);
    assignin('base', 'GoalCheckOutputBus', GoalCheckOutputBus);
    
    %% SUMMARY
    fprintf('\nBus structure creation completed successfully!\n');
    fprintf('===============================================\n');
    fprintf('Created bus structures:\n');
    fprintf('  ✓ VehicleStateBus     - Vehicle state (x, y, theta, gamma)\n');
    fprintf('  ✓ ToleranceBus        - Error tolerances (position, orientation, articulation)\n');
    fprintf('  ✓ VehicleConfigBus    - Vehicle type configuration\n');
    fprintf('  ✓ GoalCheckInputBus   - Combined input structure\n');
    fprintf('  ✓ GoalCheckOutputBus  - Combined output structure\n\n');
    
    fprintf('All bus structures are now available in the MATLAB workspace.\n');
    fprintf('Use them with Simulink models or the goal checking functions.\n\n');
    
    % Display quick usage example
    display_usage_example();
end

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

function display_usage_example()
    % DISPLAY_USAGE_EXAMPLE - Show example usage of the created bus structures
    
    fprintf('QUICK USAGE EXAMPLE:\n');
    fprintf('===================\n');
    fprintf('%% Create goal checker component\n');
    fprintf('gc = components.goal_checker.GoalChecker();\n\n');
    fprintf('%% Create vehicle states using helper functions\n');
    fprintf('current = gc.create_state(60.1, 64.2, 268, 1, ''articulated'');\n');
    fprintf('goal = gc.create_state(60, 64, 270, 0, ''articulated'');\n');
    fprintf('tolerances = gc.create_tolerances(1.0, 10.0, 5.0);\n\n');
    fprintf('%% Create input bus and check goal\n');
    fprintf('input_bus = gc.create_input_bus(current, goal, tolerances, ''articulated'');\n');
    fprintf('[should_stop, reason, errors] = gc.check_goal(input_bus);\n\n');
    fprintf('For more examples, run: gc.demo(''all'')\n\n');
end
