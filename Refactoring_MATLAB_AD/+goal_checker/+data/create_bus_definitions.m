function create_bus_definitions()
    % CREATE_BUS_DEFINITIONS - Creates all vehicle goal checking bus structures
    %
    % This function creates standardized bus structures using the centralized
    % bus element creation system for consistent and validated bus definitions.
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
    %   % Bus structures will be available in workspace
    
    fprintf('Creating vehicle goal checking bus structures...\n\n');
    
    % Import the centralized bus element creator
    create_elem = @utils.create_bus_elem;
    create_bus_object = @utils.create_bus_object;
    
    %% 1. VEHICLE STATE BUS
    fprintf('1. Creating VehicleStateBus...\n');
    
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
    
    VehicleStateBus = create_bus_object('VehicleStateBus', vehicle_state_elems, ...
        'Vehicle state containing position, orientation, and articulation angle');
    
    %% 2. TOLERANCE BUS
    fprintf('2. Creating ToleranceBus...\n');
    
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
    
    ToleranceBus = create_bus_object('ToleranceBus', tolerance_elems, ...
        'Error tolerance settings for goal achievement checking');
    
    %% 3. VEHICLE CONFIG BUS
    fprintf('3. Creating VehicleConfigBus...\n');
    
    vehicle_config_elems = [
        create_elem('name', [1, 20], 'uint8', ...
            'Description', 'Vehicle type name: "articulated" or "car"', ...
            'Min', 0, 'Max', 255);
    ];
    
    VehicleConfigBus = create_bus_object('VehicleConfigBus', vehicle_config_elems, ...
        'Vehicle type configuration with simple name field');
    
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
            'Min', 0, 'Max', 1000);
        create_elem('orientation_error', 1, 'double', ...
            'Description', 'Current orientation error in degrees', ...
            'Min', 0, 'Max', 180);
        create_elem('articulation_error', 1, 'double', ...
            'Description', 'Current articulation error in degrees', ...
            'Min', 0, 'Max', 90);
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

end