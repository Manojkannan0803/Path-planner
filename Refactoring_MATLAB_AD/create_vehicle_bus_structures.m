function create_vehicle_bus_structures()
    % CREATE_VEHICLE_BUS_STRUCTURES - Creates MATLAB bus structures for vehicle goal checking
    %
    % This script creates standardized bus structures for use with the goal checking
    % system, making it easier to work with different vehicle types and parameters.
    %
    % CREATED BUS STRUCTURES:
    %   1. VehicleStateBus - For current_state and goal_state
    %   2. ToleranceBus - For position, orientation, and articulation tolerances
    %   3. VehicleTypeBus - For vehicle type configuration
    %   4. GoalCheckInputBus - Combined input structure
    %   5. GoalCheckOutputBus - Combined output structure
    %
    % USAGE:
    %   run('create_vehicle_bus_structures.m');cr
    %   % Bus structures will be available in workspace
    
    fprintf('Creating vehicle bus structures for goal checking system...\n\n');
    
    %% 1. VEHICLE STATE BUS - For both current_state and goal_state
    fprintf('1. Creating VehicleStateBus...\n');
    
    % Create bus elements for vehicle state
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'x';
    elems(1).DataType = 'double';
    elems(1).Description = 'Vehicle X position in meters';
    elems(1).Min = -1000;
    elems(1).Max = 1000;
    
    elems(2) = Simulink.BusElement;
    elems(2).Name = 'y';
    elems(2).DataType = 'double';
    elems(2).Description = 'Vehicle Y position in meters';
    elems(2).Min = -1000;
    elems(2).Max = 1000;
    
    elems(3) = Simulink.BusElement;
    elems(3).Name = 'theta';
    elems(3).DataType = 'double';
    elems(3).Description = 'Vehicle heading angle in degrees';
    elems(3).Min = -360;
    elems(3).Max = 360;
    
    elems(4) = Simulink.BusElement;
    elems(4).Name = 'gamma';
    elems(4).DataType = 'double';
    elems(4).Description = 'Articulation angle in degrees (0 for car vehicles)';
    elems(4).Min = -90;
    elems(4).Max = 90;
    
    % Create the VehicleStateBus
    VehicleStateBus = Simulink.Bus;
    VehicleStateBus.Elements = elems;
    VehicleStateBus.Description = 'Vehicle state containing position, orientation, and articulation';
    
    % Save to base workspace
    assignin('base', 'VehicleStateBus', VehicleStateBus);
    
    %% 2. TOLERANCE BUS - For position, orientation, and articulation tolerances
    fprintf('2. Creating ToleranceBus...\n');
    
    clear elems;
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'position_tolerance';
    elems(1).DataType = 'double';
    elems(1).Description = 'Position error tolerance in meters';
    elems(1).Min = 0;
    elems(1).Max = 100;
    
    elems(2) = Simulink.BusElement;
    elems(2).Name = 'orientation_tolerance';
    elems(2).DataType = 'double';
    elems(2).Description = 'Orientation error tolerance in degrees';
    elems(2).Min = 0;
    elems(2).Max = 180;
    
    elems(3) = Simulink.BusElement;
    elems(3).Name = 'articulation_tolerance';
    elems(3).DataType = 'double';
    elems(3).Description = 'Articulation error tolerance in degrees (ignored for car)';
    elems(3).Min = 0;
    elems(3).Max = 90;
    
    % Create the ToleranceBus
    ToleranceBus = Simulink.Bus;
    ToleranceBus.Elements = elems;
    ToleranceBus.Description = 'Tolerance settings for goal achievement checking';
    
    % Save to base workspace
    assignin('base', 'ToleranceBus', ToleranceBus);
    
    %% 3. VEHICLE TYPE BUS - For vehicle type configuration
    fprintf('3. Creating VehicleTypeBus...\n');
    
    clear elems;
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'type';
    elems(1).DataType = 'double';
    elems(1).Description = 'Vehicle type: 1=articulated, 2=car';
    elems(1).Min = 1;
    elems(1).Max = 2;
    
    elems(2) = Simulink.BusElement;
    elems(2).Name = 'name';
    elems(2).DataType = 'double';
    elems(2).Description = 'Vehicle type name as string (for logging)';
    
    elems(3) = Simulink.BusElement;
    elems(3).Name = 'has_articulation';
    elems(3).DataType = 'boolean';
    elems(3).Description = 'True if vehicle has articulation (gamma) capability';
    
    % Create the VehicleTypeBus
    VehicleTypeBus = Simulink.Bus;
    VehicleTypeBus.Elements = elems;
    VehicleTypeBus.Description = 'Vehicle type configuration and capabilities';
    
    % Save to base workspace
    assignin('base', 'VehicleTypeBus', VehicleTypeBus);
    
    %% 4. GOAL CHECK INPUT BUS - Combined input structure
    fprintf('4. Creating GoalCheckInputBus...\n');
    
    clear elems;
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'current_state';
    elems(1).DataType = 'Bus: VehicleStateBus';
    elems(1).Description = 'Current vehicle state';
    
    elems(2) = Simulink.BusElement;
    elems(2).Name = 'goal_state';
    elems(2).DataType = 'Bus: VehicleStateBus';
    elems(2).Description = 'Target goal state';
    
    elems(3) = Simulink.BusElement;
    elems(3).Name = 'tolerances';
    elems(3).DataType = 'Bus: ToleranceBus';
    elems(3).Description = 'Error tolerances for goal checking';
    
    elems(4) = Simulink.BusElement;
    elems(4).Name = 'vehicle_config';
    elems(4).DataType = 'Bus: VehicleTypeBus';
    elems(4).Description = 'Vehicle type configuration';
    
    % Create the GoalCheckInputBus
    GoalCheckInputBus = Simulink.Bus;
    GoalCheckInputBus.Elements = elems;
    GoalCheckInputBus.Description = 'Complete input structure for goal checking function';
    
    % Save to base workspace
    assignin('base', 'GoalCheckInputBus', GoalCheckInputBus);
    
    %% 5. GOAL CHECK OUTPUT BUS - Combined output structure
    fprintf('5. Creating GoalCheckOutputBus...\n');
    
    clear elems;
    elems(1) = Simulink.BusElement;
    elems(1).Name = 'should_stop';
    elems(1).DataType = 'boolean';
    elems(1).Description = 'True if simulation should stop (goal reached)';
    
    elems(2) = Simulink.BusElement;
    elems(2).Name = 'stop_reason';
    elems(2).DataType = 'double';
    elems(2).Description = 'Reason code: 1=goal_reached, 0=continue';
    
    elems(3) = Simulink.BusElement;
    elems(3).Name = 'position_error';
    elems(3).DataType = 'double';
    elems(3).Description = 'Current position error in meters';
    elems(3).Min = 0;
    elems(3).Max = 1000;
    
    elems(4) = Simulink.BusElement;
    elems(4).Name = 'orientation_error';
    elems(4).DataType = 'double';
    elems(4).Description = 'Current orientation error in degrees';
    elems(4).Min = 0;
    elems(4).Max = 180;
    
    elems(5) = Simulink.BusElement;
    elems(5).Name = 'articulation_error';
    elems(5).DataType = 'double';
    elems(5).Description = 'Current articulation error in degrees';
    elems(5).Min = 0;
    elems(5).Max = 90;
    
    elems(6) = Simulink.BusElement;
    elems(6).Name = 'position_ok';
    elems(6).DataType = 'boolean';
    elems(6).Description = 'True if position error is within tolerance';
    
    elems(7) = Simulink.BusElement;
    elems(7).Name = 'orientation_ok';
    elems(7).DataType = 'boolean';
    elems(7).Description = 'True if orientation error is within tolerance';
    
    elems(8) = Simulink.BusElement;
    elems(8).Name = 'articulation_ok';
    elems(8).DataType = 'boolean';
    elems(8).Description = 'True if articulation error is within tolerance';
    
    % Create the GoalCheckOutputBus
    GoalCheckOutputBus = Simulink.Bus;
    GoalCheckOutputBus.Elements = elems;
    GoalCheckOutputBus.Description = 'Complete output structure for goal checking function';
    
    % Save to base workspace
    assignin('base', 'GoalCheckOutputBus', GoalCheckOutputBus);
    
    %% Summary
    fprintf('\nBus structure creation completed!\n');
    fprintf('====================================\n');
    fprintf('Created bus structures:\n');
    fprintf('  - VehicleStateBus     (x, y, theta, gamma)\n');
    fprintf('  - ToleranceBus        (position, orientation, articulation tolerances)\n');
    fprintf('  - VehicleTypeBus      (type, name, has_articulation)\n');
    fprintf('  - GoalCheckInputBus   (combined input structure)\n');
    fprintf('  - GoalCheckOutputBus  (combined output structure)\n\n');
    
    fprintf('All bus structures are now available in the workspace.\n');
    fprintf('Use them with the updated goal checking functions.\n\n');
    
    % Display example usage
    fprintf('EXAMPLE USAGE:\n');
    fprintf('=============\n');
    fprintf('Create structures using helper functions:\n');
    fprintf('current = create_vehicle_state(60.1, 64.2, 268, 1, ''articulated'');\n');
    fprintf('goal = create_vehicle_state(60, 64, 270, 0, ''articulated'');\n');
    fprintf('tolerances = create_tolerance_config(1.0, 10.0, 5.0);\n');
    fprintf('vehicle_config = create_vehicle_config(''articulated'');\n\n');
    
end
