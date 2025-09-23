classdef TestCheckGoalReached < matlab.unittest.TestCase
    % TESTCHECKGOALREACHED - Unit tests for check_goal_reached function
    %
    % This test class follows ISTQB testing principles and includes:
    % - Equivalence partitioning
    % - Boundary value analysis  
    % - Decision table testing
    % - State transition testing
    % - Error guessing
    % - Negative testing
    %
    % ISO 26262 Considerations:
    % - Safety-critical path planning requires verified goal detection
    % - ASIL-D level testing for autonomous vehicles
    % - Systematic testing of all safety-relevant scenarios
    %
    % ASPICE Considerations:
    % - SWE.4 (Software Unit Testing) compliance
    % - Traceability from requirements to test cases
    % - Coverage analysis and metrics
    
    properties (TestParameter)
        % Test parameters for parameterized tests
        vehicle_types = {'articulated', 'car'}
        tolerance_scenarios = struct(...
            'tight', [0.5, 5.0, 3.0], ...
            'normal', [2.0, 15.0, 10.0], ...
            'relaxed', [5.0, 30.0, 20.0])
    end
    
    properties
        test_tolerance = 1e-6  % Numerical tolerance for floating point comparisons
    end
    
    methods (TestMethodSetup)
        function setupTest(testCase)
            % Setup before each test method
            addpath(genpath(pwd));  % Ensure all paths are available
        end
    end
    
    methods (Test)
        %% EQUIVALENCE PARTITIONING TESTS
        % Test representative values from each equivalence class
        
        function testValidInputs_ArticulatedVehicle(testCase)
            % Test TC-001: Valid inputs for articulated vehicle
            % Requirement: REQ-GC-001 - Support articulated vehicle goal checking
            
            current_state = struct('x', 60.1, 'y', 64.2, 'theta', 268, 'gamma', 1);
            goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            
            % Create input bus for goal checking
            input_bus = struct();
            input_bus.current_state = current_state;
            input_bus.goal_state = goal_state;
            input_bus.tolerances = struct('position_tolerance', 2.0, 'orientation_tolerance', 15.0, 'articulation_tolerance', 10.0);
            
            % Create vehicle configuration structure
            vehicle_name = 'articulated';
            input_bus.vehicle_config = struct();
            input_bus.vehicle_config.name = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
            
            [should_stop, stop_reason, goal_errors] = check_goal_reached_bus(input_bus);
            
            % Verify outputs
            testCase.verifyClass(should_stop, 'logical');
            testCase.verifyClass(stop_reason, 'char');
            testCase.verifyClass(goal_errors, 'struct');
            
            % Verify required fields in error structure
            required_fields = {'position_error', 'orientation_error', 'articulation_error', ...
                'position_ok', 'orientation_ok', 'articulation_ok', 'vehicle_type'};
            for i = 1:length(required_fields)
                testCase.verifyTrue(isfield(goal_errors, required_fields{i}), ...
                    sprintf('Missing field: %s', required_fields{i}));
            end
            
            % Verify vehicle type is correctly stored
            testCase.verifyEqual(goal_errors.debug_info.vehicle_type, 'articulated');
        end
        
        function testValidInputs_CarVehicle(testCase)
            % Test TC-002: Valid inputs for car vehicle
            % Requirement: REQ-GC-002 - Support car vehicle goal checking
            
            current_state = struct('x', 60.1, 'y', 64.2, 'theta', 268);
            goal_state = struct('x', 60, 'y', 64, 'theta', 270);
            
            % Create input bus for goal checking
            input_bus = struct();
            input_bus.current_state = current_state;
            input_bus.goal_state = goal_state;
            input_bus.tolerances = struct('position_tolerance', 2.0, 'orientation_tolerance', 15.0, 'articulation_tolerance', []);
            
            % Create vehicle configuration structure for car
            vehicle_name = 'car';
            input_bus.vehicle_config = struct();
            input_bus.vehicle_config.name = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
            
            [should_stop, stop_reason, goal_errors] = check_goal_reached_bus(input_bus);
            
            % Verify car vehicle specific behavior
            testCase.verifyEqual(goal_errors.debug_info.vehicle_type, 'car');
            testCase.verifyEqual(goal_errors.articulation_error, 0);
            testCase.verifyTrue(goal_errors.articulation_ok);
        end
        
        %% BOUNDARY VALUE ANALYSIS TESTS
        % Test values at boundaries of equivalence classes
        
        function testBoundaryValues_PositionTolerance(testCase)
            % Test TC-003: Position exactly at tolerance boundary
            % Requirement: REQ-GC-003 - Accurate position tolerance checking
            
            tolerance = 2.0;
            
            % Test case 1: Position error exactly at tolerance (should pass)
            current_state = struct('x', 62.0, 'y', 64.0, 'theta', 270, 'gamma', 0);
            goal_state = struct('x', 60.0, 'y', 64.0, 'theta', 270, 'gamma', 0);
            
            % Create input bus for goal checking
            input_bus1 = struct();
            input_bus1.current_state = current_state;
            input_bus1.goal_state = goal_state;
            input_bus1.tolerances = struct('position_tolerance', tolerance, 'orientation_tolerance', 180, 'articulation_tolerance', 180);
            
            % Create vehicle configuration structure
            vehicle_name = 'articulated';
            input_bus1.vehicle_config = struct();
            input_bus1.vehicle_config.name = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
            
            [should_stop1, ~, errors1] = check_goal_reached_bus(input_bus1);
            
            testCase.verifyTrue(errors1.position_ok);
            testCase.verifyAbsoluteTol(errors1.position_error, tolerance, testCase.test_tolerance);
            
            % Test case 2: Position error just over tolerance (should fail)
            current_state.x = 60.0 + tolerance + 0.001;
            
            % Create input bus for second test
            input_bus2 = struct();
            input_bus2.current_state = current_state;
            input_bus2.goal_state = goal_state;
            input_bus2.tolerances = struct('position_tolerance', tolerance, 'orientation_tolerance', 180, 'articulation_tolerance', 180);
            
            % Create vehicle configuration structure
            vehicle_name = 'articulated';
            input_bus2.vehicle_config = struct();
            input_bus2.vehicle_config.name = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
            
            [should_stop2, ~, errors2] = check_goal_reached_bus(input_bus2);
            
            testCase.verifyFalse(errors2.position_ok);
            testCase.verifyTrue(errors2.position_error > tolerance);
        end
        
        function testBoundaryValues_OrientationTolerance(testCase)
            % Test TC-004: Orientation at tolerance boundaries including angle wraparound
            % Requirement: REQ-GC-004 - Handle angle wraparound correctly
            
            tolerance = 15.0;
            current_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            
            % Test angle wraparound cases
            test_cases = [
                struct('current_theta', 5, 'goal_theta', 355, 'expected_error', 10);  % Wraparound case
                struct('current_theta', 355, 'goal_theta', 5, 'expected_error', 10);   % Reverse wraparound
                struct('current_theta', 180, 'goal_theta', -180, 'expected_error', 0); % Equivalent angles
                struct('current_theta', 270, 'goal_theta', 270+tolerance, 'expected_error', tolerance); % Boundary
            ];
            
            for i = 1:length(test_cases)
                tc = test_cases(i);
                current_state.theta = tc.current_theta;
                goal_state.theta = tc.goal_theta;
                
                % Create input bus for angle testing
                input_bus = struct();
                input_bus.current_state = current_state;
                input_bus.goal_state = goal_state;
                input_bus.tolerances = struct('position', 100, 'orientation', tolerance, 'articulation', 180);
                
                % Create vehicle configuration structure
                vehicle_name = 'articulated';
                input_bus.vehicle_config = struct();
                input_bus.vehicle_config.type_code = uint8(1);  % 1=articulated
                input_bus.vehicle_config.has_articulation = true;
                input_bus.vehicle_config.name_length = uint8(length(vehicle_name));
                input_bus.vehicle_config.name_chars = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
                
                [~, ~, errors] = check_goal_reached_bus(input_bus);
                
                testCase.verifyAbsoluteTol(errors.orientation_error, tc.expected_error, ...
                    testCase.test_tolerance, ...
                    sprintf('Angle wraparound test %d failed: current=%.1f, goal=%.1f', ...
                    i, tc.current_theta, tc.goal_theta));
            end
        end
        
        %% DECISION TABLE TESTING
        % Test all combinations of tolerance satisfaction
        
        function testDecisionTable_ToleranceCombinations(testCase)
            % Test TC-005: All combinations of tolerance satisfaction
            % Requirement: REQ-GC-005 - Goal reached only when all tolerances satisfied
            
            base_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            
            % Decision table: [pos_ok, orient_ok, artic_ok] -> should_stop
            test_cases = [
                % Case 1: All tolerances satisfied -> should stop
                struct('pos_offset', 0.5, 'angle_offset', 5, 'gamma_offset', 2, ...
                       'tolerances', [1.0, 10, 5], 'expected_stop', true);
                
                % Case 2: Position fails -> should not stop
                struct('pos_offset', 2.0, 'angle_offset', 5, 'gamma_offset', 2, ...
                       'tolerances', [1.0, 10, 5], 'expected_stop', false);
                
                % Case 3: Orientation fails -> should not stop
                struct('pos_offset', 0.5, 'angle_offset', 15, 'gamma_offset', 2, ...
                       'tolerances', [1.0, 10, 5], 'expected_stop', false);
                
                % Case 4: Articulation fails -> should not stop
                struct('pos_offset', 0.5, 'angle_offset', 5, 'gamma_offset', 10, ...
                       'tolerances', [1.0, 10, 5], 'expected_stop', false);
                
                % Case 5: Multiple failures -> should not stop
                struct('pos_offset', 2.0, 'angle_offset', 15, 'gamma_offset', 10, ...
                       'tolerances', [1.0, 10, 5], 'expected_stop', false);
            ];
            
            for i = 1:length(test_cases)
                tc = test_cases(i);
                
                % Create test state with offsets
                current_state = base_state;
                current_state.x = current_state.x + tc.pos_offset;
                current_state.theta = current_state.theta + tc.angle_offset;
                current_state.gamma = current_state.gamma + tc.gamma_offset;
                
                % Create input bus for decision table testing
                input_bus = struct();
                input_bus.current_state = current_state;
                input_bus.goal_state = goal_state;
                input_bus.tolerances = struct('position', tc.tolerances(1), 'orientation', tc.tolerances(2), 'articulation', tc.tolerances(3));
                
                % Create vehicle configuration structure
                vehicle_name = 'articulated';
                input_bus.vehicle_config = struct();
                input_bus.vehicle_config.type_code = uint8(1);  % 1=articulated
                input_bus.vehicle_config.has_articulation = true;
                input_bus.vehicle_config.name_length = uint8(length(vehicle_name));
                input_bus.vehicle_config.name_chars = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
                
                [should_stop, ~, ~] = check_goal_reached_bus(input_bus);
                
                testCase.verifyEqual(should_stop, tc.expected_stop, ...
                    sprintf('Decision table test %d failed', i));
            end
        end
        
        %% STATE TRANSITION TESTING
        % Test vehicle approaching goal through different states
        
        function testStateTransition_ApproachingGoal(testCase)
            % Test TC-006: Vehicle state transitions from far to goal
            % Requirement: REQ-GC-006 - Continuous goal monitoring during approach
            
            start_state = struct('x', 70, 'y', 50, 'theta', 180, 'gamma', 10);
            goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            tolerances = [1.5, 10.0, 8.0];
            
            % Simulate approach with multiple steps
            steps = 20;
            goal_reached_step = -1;
            
            for step = 1:steps
                % Linear interpolation toward goal
                alpha = step / steps;
                current_state = struct();
                current_state.x = start_state.x + alpha * (goal_state.x - start_state.x);
                current_state.y = start_state.y + alpha * (goal_state.y - start_state.y);
                current_state.theta = start_state.theta + alpha * (goal_state.theta - start_state.theta);
                current_state.gamma = start_state.gamma + alpha * (goal_state.gamma - start_state.gamma);
                
                % Create input bus for state transition testing
                input_bus = struct();
                input_bus.current_state = current_state;
                input_bus.goal_state = goal_state;
                input_bus.tolerances = struct('position', tolerances(1), 'orientation', tolerances(2), 'articulation', tolerances(3));
                
                % Create vehicle configuration structure
                vehicle_name = 'articulated';
                input_bus.vehicle_config = struct();
                input_bus.vehicle_config.type_code = uint8(1);  % 1=articulated
                input_bus.vehicle_config.has_articulation = true;
                input_bus.vehicle_config.name_length = uint8(length(vehicle_name));
                input_bus.vehicle_config.name_chars = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
                
                [should_stop, ~, errors] = check_goal_reached_bus(input_bus);
                
                % Verify error decreases over time
                if step > 1
                    testCase.verifyTrue(errors.position_error <= prev_pos_error + testCase.test_tolerance, ...
                        'Position error should decrease or remain constant');
                end
                prev_pos_error = errors.position_error;
                
                % Record when goal is first reached
                if should_stop && goal_reached_step == -1
                    goal_reached_step = step;
                end
                
                % Once goal is reached, it should remain reached
                if goal_reached_step > 0 && step > goal_reached_step
                    testCase.verifyTrue(should_stop, ...
                        'Goal should remain reached once achieved');
                end
            end
            
            % Verify goal was eventually reached
            testCase.verifyTrue(goal_reached_step > 0, 'Goal should be reached during approach');
        end
        
        %% ERROR GUESSING AND NEGATIVE TESTING
        % Test error conditions and edge cases
        
        function testErrorConditions_InvalidInputs(testCase)
            % Test TC-007: Invalid input handling
            % Requirement: REQ-GC-007 - Robust error handling
            
            valid_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            
            % Create valid vehicle configuration
            vehicle_name = 'articulated';
            valid_vehicle_config = struct();
            valid_vehicle_config.type_code = uint8(1);  % 1=articulated
            valid_vehicle_config.has_articulation = true;
            valid_vehicle_config.name_length = uint8(length(vehicle_name));
            valid_vehicle_config.name_chars = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
            
            % Test invalid current_state
            testCase.verifyError(@() check_goal_reached_bus(struct('current_state', 'invalid', 'goal_state', valid_state, 'tolerances', struct('position', 2, 'orientation', 15, 'articulation', 10), 'vehicle_config', valid_vehicle_config)), ...
                'MATLAB:InputParser:ArgumentFailedValidation');
            
            % Test invalid goal_state
            testCase.verifyError(@() check_goal_reached_bus(struct('current_state', valid_state, 'goal_state', 'invalid', 'tolerances', struct('position', 2, 'orientation', 15, 'articulation', 10), 'vehicle_config', valid_vehicle_config)), ...
                'MATLAB:InputParser:ArgumentFailedValidation');
            
            % Test invalid vehicle type
            invalid_vehicle_config = valid_vehicle_config;
            invalid_vehicle_config.type_code = uint8(99);  % Invalid type code
            testCase.verifyError(@() check_goal_reached_bus(struct('current_state', valid_state, 'goal_state', valid_state, 'tolerances', struct('position', 2, 'orientation', 15, 'articulation', 10), 'vehicle_config', invalid_vehicle_config)), ...
                'MATLAB:error');
        end
        
        function testErrorConditions_MissingFields(testCase)
            % Test TC-008: Missing required fields
            % Requirement: REQ-GC-008 - Validate state structure fields
            
            goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            
            % Test missing fields for articulated vehicle
            incomplete_states = {
                struct('y', 64, 'theta', 270, 'gamma', 0);  % Missing x
                struct('x', 60, 'theta', 270, 'gamma', 0);  % Missing y
                struct('x', 60, 'y', 64, 'gamma', 0);       % Missing theta
                struct('x', 60, 'y', 64, 'theta', 270);     % Missing gamma
            };
            
            for i = 1:length(incomplete_states)
                % Create input bus for missing field testing
                input_bus = struct();
                input_bus.current_state = incomplete_states{i};
                input_bus.goal_state = goal_state;
                input_bus.tolerances = struct('position', 2, 'orientation', 15, 'articulation', 10);
                
                % Create vehicle configuration structure
                vehicle_name = 'articulated';
                input_bus.vehicle_config = struct();
                input_bus.vehicle_config.type_code = uint8(1);  % 1=articulated
                input_bus.vehicle_config.has_articulation = true;
                input_bus.vehicle_config.name_length = uint8(length(vehicle_name));
                input_bus.vehicle_config.name_chars = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
                
                testCase.verifyError(@() check_goal_reached_bus(input_bus), ...
                    'MATLAB:error', sprintf('Should error for missing field case %d', i));
            end
        end
        
        function testEdgeCases_ExtremeValues(testCase)
            % Test TC-009: Extreme input values
            % Requirement: REQ-GC-009 - Handle extreme numerical values
            
            % Test very large coordinates
            large_state = struct('x', 1e6, 'y', 1e6, 'theta', 270, 'gamma', 0);
            goal_state = struct('x', 0, 'y', 0, 'theta', 270, 'gamma', 0);
            
            % Create input bus for large value testing
            input_bus1 = struct();
            input_bus1.current_state = large_state;
            input_bus1.goal_state = goal_state;
            input_bus1.tolerances = struct('position', 1e7, 'orientation', 15, 'articulation', 10);
            
            % Create vehicle configuration structure
            vehicle_name = 'articulated';
            input_bus1.vehicle_config = struct();
            input_bus1.vehicle_config.type_code = uint8(1);  % 1=articulated
            input_bus1.vehicle_config.has_articulation = true;
            input_bus1.vehicle_config.name_length = uint8(length(vehicle_name));
            input_bus1.vehicle_config.name_chars = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
            
            [should_stop, ~, errors] = check_goal_reached_bus(input_bus1);
            
            testCase.verifyClass(errors.position_error, 'double');
            testCase.verifyTrue(isfinite(errors.position_error));
            
            % Test very small tolerances
            close_state = struct('x', 60.001, 'y', 64.001, 'theta', 270.1, 'gamma', 0.1);
            exact_goal = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            
            % Create input bus for small tolerance testing
            input_bus2 = struct();
            input_bus2.current_state = close_state;
            input_bus2.goal_state = exact_goal;
            input_bus2.tolerances = struct('position', 1e-6, 'orientation', 1e-3, 'articulation', 1e-3);
            
            % Create vehicle configuration structure
            vehicle_name = 'articulated';
            input_bus2.vehicle_config = struct();
            input_bus2.vehicle_config.type_code = uint8(1);  % 1=articulated
            input_bus2.vehicle_config.has_articulation = true;
            input_bus2.vehicle_config.name_length = uint8(length(vehicle_name));
            input_bus2.vehicle_config.name_chars = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
            
            [should_stop, ~, errors] = check_goal_reached_bus(input_bus2);
            
            testCase.verifyFalse(should_stop);  % Should not reach with very tight tolerances
        end
        
        %% PARAMETERIZED TESTS
        % Test different vehicle types and tolerance scenarios
        
        function testParameterized_VehicleTypes(testCase, vehicle_types)
            % Test TC-010: Parameterized test for different vehicle types
            % Requirement: REQ-GC-010 - Support multiple vehicle types
            
            % Create appropriate state structures for vehicle type
            if strcmp(vehicle_types, 'articulated')
                current_state = struct('x', 60.1, 'y', 64.2, 'theta', 268, 'gamma', 1);
                goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            else  % car
                current_state = struct('x', 60.1, 'y', 64.2, 'theta', 268);
                goal_state = struct('x', 60, 'y', 64, 'theta', 270);
            end
            
            % Create input bus for parameterized vehicle testing
            input_bus = struct();
            input_bus.current_state = current_state;
            input_bus.goal_state = goal_state;
            input_bus.tolerances = struct('position', 2.0, 'orientation', 15.0, 'articulation', 10.0);
            
            % Create vehicle configuration structure based on vehicle type
            input_bus.vehicle_config = struct();
            if strcmp(vehicle_types, 'articulated')
                input_bus.vehicle_config.type_code = uint8(1);  % 1=articulated
                input_bus.vehicle_config.has_articulation = true;
            else  % car
                input_bus.vehicle_config.type_code = uint8(2);  % 2=car
                input_bus.vehicle_config.has_articulation = false;
            end
            input_bus.vehicle_config.name_length = uint8(length(vehicle_types));
            input_bus.vehicle_config.name_chars = uint8([double(vehicle_types), zeros(1, 20-length(vehicle_types))]);
            
            [should_stop, stop_reason, goal_errors] = check_goal_reached_bus(input_bus);
            
            % Verify basic functionality for each vehicle type
            testCase.verifyClass(should_stop, 'logical');
            testCase.verifyEqual(goal_errors.vehicle_type, vehicle_types);
            
            % Vehicle-specific verifications
            if strcmp(vehicle_types, 'car')
                testCase.verifyEqual(goal_errors.articulation_error, 0);
                testCase.verifyTrue(goal_errors.articulation_ok);
            end
        end
        
        function testParameterized_ToleranceScenarios(testCase, tolerance_scenarios)
            % Test TC-011: Different tolerance scenarios
            % Requirement: REQ-GC-011 - Configurable tolerance settings
            
            current_state = struct('x', 61, 'y', 65, 'theta', 275, 'gamma', 5);
            goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            
            tol = tolerance_scenarios;
            
            % Create input bus for tolerance scenario testing
            input_bus = struct();
            input_bus.current_state = current_state;
            input_bus.goal_state = goal_state;
            input_bus.tolerances = struct('position', tol(1), 'orientation', tol(2), 'articulation', tol(3));
            
            % Create vehicle configuration structure
            vehicle_name = 'articulated';
            input_bus.vehicle_config = struct();
            input_bus.vehicle_config.type_code = uint8(1);  % 1=articulated
            input_bus.vehicle_config.has_articulation = true;
            input_bus.vehicle_config.name_length = uint8(length(vehicle_name));
            input_bus.vehicle_config.name_chars = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
            
            [should_stop, ~, errors] = check_goal_reached_bus(input_bus);
            
            % Verify tolerance values are correctly stored
            testCase.verifyEqual(errors.position_tolerance, tol(1));
            testCase.verifyEqual(errors.orientation_tolerance, tol(2));
            testCase.verifyEqual(errors.articulation_tolerance, tol(3));
            
            % Verify behavior is consistent with tolerance level
            if strcmp(testCase.TestParameterName, 'tight')
                % With tight tolerances, goal likely not reached
                testCase.verifyFalse(should_stop);
            end
        end
        
        %% PERFORMANCE AND NUMERICAL STABILITY TESTS
        
        function testNumericalStability_AngleNormalization(testCase)
            % Test TC-012: Angle normalization numerical stability
            % Requirement: REQ-GC-012 - Robust angle calculations
            
            % Test angles that could cause numerical issues
            test_angles = [
                struct('current', 359.9999, 'goal', 0.0001, 'expected_error', 0.0002);
                struct('current', -179.9999, 'goal', 180.0001, 'expected_error', 0.0002);
                struct('current', 720, 'goal', 0, 'expected_error', 0);  % Multiple rotations
                struct('current', -360, 'goal', 0, 'expected_error', 0);
            ];
            
            base_state = struct('x', 60, 'y', 64, 'theta', 0, 'gamma', 0);
            
            for i = 1:length(test_angles)
                ta = test_angles(i);
                current_state = base_state;
                goal_state = base_state;
                current_state.theta = ta.current;
                goal_state.theta = ta.goal;
                
                % Create input bus for angle normalization testing
                input_bus = struct();
                input_bus.current_state = current_state;
                input_bus.goal_state = goal_state;
                input_bus.tolerances = struct('position', 100, 'orientation', 180, 'articulation', 180);
                
                % Create vehicle configuration structure
                vehicle_name = 'articulated';
                input_bus.vehicle_config = struct();
                input_bus.vehicle_config.type_code = uint8(1);  % 1=articulated
                input_bus.vehicle_config.has_articulation = true;
                input_bus.vehicle_config.name_length = uint8(length(vehicle_name));
                input_bus.vehicle_config.name_chars = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
                
                [~, ~, errors] = check_goal_reached_bus(input_bus);
                
                testCase.verifyAbsoluteTol(errors.orientation_error, ta.expected_error, ...
                    0.001, sprintf('Angle normalization test %d failed', i));
            end
        end
        
        %% INTEGRATION AND SYSTEM TESTS
        
        function testIntegration_DefaultParameters(testCase)
            % Test TC-013: Default parameter behavior
            % Requirement: REQ-GC-013 - Sensible default values
            
            current_state = struct('x', 60.1, 'y', 64.2, 'theta', 268, 'gamma', 1);
            goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            
            % Test with minimal parameters (should use defaults)
            % Create input bus with default tolerances
            input_bus = struct();
            input_bus.current_state = current_state;
            input_bus.goal_state = goal_state;
            input_bus.tolerances = struct('position', 2.0, 'orientation', 15.0, 'articulation', 10.0);
            
            % Create vehicle configuration structure
            vehicle_name = 'articulated';
            input_bus.vehicle_config = struct();
            input_bus.vehicle_config.type_code = uint8(1);  % 1=articulated
            input_bus.vehicle_config.has_articulation = true;
            input_bus.vehicle_config.name_length = uint8(length(vehicle_name));
            input_bus.vehicle_config.name_chars = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
            
            [should_stop, stop_reason, goal_errors] = check_goal_reached_bus(input_bus);
            
            % Verify default values are used
            testCase.verifyEqual(goal_errors.position_tolerance, 2.0);
            testCase.verifyEqual(goal_errors.orientation_tolerance, 15.0);
            testCase.verifyEqual(goal_errors.articulation_tolerance, 10.0);
            testCase.verifyEqual(goal_errors.vehicle_type, 'articulated');
        end
    end
    
    methods (Test, TestTags = {'Performance'})
        function testPerformance_LargeDataset(testCase)
            % Test TC-014: Performance with multiple goal checks
            % Requirement: REQ-GC-014 - Efficient computation
            
            % Generate test data
            n_tests = 1000;
            current_states = cell(n_tests, 1);
            goal_state = struct('x', 60, 'y', 64, 'theta', 270, 'gamma', 0);
            
            for i = 1:n_tests
                current_states{i} = struct('x', 60 + randn(), 'y', 64 + randn(), ...
                    'theta', 270 + 10*randn(), 'gamma', 5*randn());
            end
            
            % Measure execution time
            tic;
            for i = 1:n_tests
                % Create input bus for performance testing
                input_bus = struct();
                input_bus.current_state = current_states{i};
                input_bus.goal_state = goal_state;
                input_bus.tolerances = struct('position', 2.0, 'orientation', 15.0, 'articulation', 10.0);
                
                % Create vehicle configuration structure
                vehicle_name = 'articulated';
                input_bus.vehicle_config = struct();
                input_bus.vehicle_config.type_code = uint8(1);  % 1=articulated
                input_bus.vehicle_config.has_articulation = true;
                input_bus.vehicle_config.name_length = uint8(length(vehicle_name));
                input_bus.vehicle_config.name_chars = uint8([double(vehicle_name), zeros(1, 20-length(vehicle_name))]);
                
                [~, ~, ~] = check_goal_reached_bus(input_bus);
            end
            elapsed_time = toc;
            
            % Verify reasonable performance (should complete in reasonable time)
            max_time_per_call = 0.001;  % 1ms per call
            avg_time_per_call = elapsed_time / n_tests;
            
            testCase.verifyLessThan(avg_time_per_call, max_time_per_call, ...
                sprintf('Performance test failed: %.6f ms/call (max: %.6f ms/call)', ...
                avg_time_per_call*1000, max_time_per_call*1000));
        end
    end
end
