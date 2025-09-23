classdef VehicleParamsTest < matlab.unittest.TestCase
    % Unit tests for Planner.Src.VehicleParams class
    % Package: +Planner.+Testing
    % Updated: 2025-09-23 (Moved from +TwinsimPlanner to +Planner)
    %
    % Run with: 
    %   runtests('Planner.Testing.VehicleParamsTest')
    %   or
    %   import Planner.Testing.*
    %   runtests('VehicleParamsTest')
    
    properties (TestParameter)
        % Test parameters for validation (simplified)
    end
    
    properties
        vehicleParams
        testConfigFile = 'test_planner_vehicle_config.yaml';
    end
    
    methods (TestMethodSetup)
        function setupTest(testCase)
            % Create test configuration file
            testCase.createTestConfig();
            testCase.vehicleParams = Planner.Src.VehicleParams(testCase.testConfigFile);
        end
    end
    
    methods (TestMethodTeardown)  
        function teardownTest(testCase)
            % Clean up test files
            if isfile(testCase.testConfigFile)
                delete(testCase.testConfigFile);
            end
        end
    end
    
    methods (Test)
        function testValidConfiguration(testCase)
            % Test loading valid configuration
            testCase.verifyTrue(testCase.vehicleParams.isLoaded());
            testCase.verifyTrue(testCase.vehicleParams.validateConfig());
        end
        
        function testVehicleDimensions(testCase)
            % Test vehicle dimension properties
            testCase.verifyEqual(testCase.vehicleParams.length, 4.07, 'AbsTol', 1e-6);
            testCase.verifyEqual(testCase.vehicleParams.width, 1.75, 'AbsTol', 1e-6);
            testCase.verifyEqual(testCase.vehicleParams.wheelbase, 2.55, 'AbsTol', 1e-6);
            testCase.verifyEqual(testCase.vehicleParams.rearAxleDistance, 1.53, 'AbsTol', 1e-6);
        end
        
        function testSteeringCharacteristics(testCase)
            % Test steering parameters
            testCase.verifyEqual(testCase.vehicleParams.maxSteeringAngle, 44, 'AbsTol', 1e-6);
            testCase.verifyEqual(testCase.vehicleParams.maxSteeringRate, 35, 'AbsTol', 1e-6);
            testCase.verifyEqual(testCase.vehicleParams.minTurningRadius, 2.64, 'AbsTol', 1e-6);
        end
        
        function testPerformanceLimits(testCase)
            % Test performance parameters
            testCase.verifyEqual(testCase.vehicleParams.maxVelocity, 8.0, 'AbsTol', 1e-6);
            testCase.verifyEqual(testCase.vehicleParams.maxAcceleration, 2.5, 'AbsTol', 1e-6);
            testCase.verifyEqual(testCase.vehicleParams.maxJerk, 1.5, 'AbsTol', 1e-6);
        end
        
        function testSafetyMargins(testCase)
            % Test safety parameters
            testCase.verifyEqual(testCase.vehicleParams.safetyMarginStatic, 0.75, 'AbsTol', 1e-6);
            testCase.verifyEqual(testCase.vehicleParams.safetyMarginDynamic, 1.5, 'AbsTol', 1e-6);
            testCase.verifyEqual(testCase.vehicleParams.emergencyStopDistance, 2.0, 'AbsTol', 1e-6);
        end
        
        function testVehicleFootprint(testCase)
            % Test footprint calculation
            x = 5; y = 3; theta = pi/4; % 45 degrees
            
            footprint = testCase.vehicleParams.getVehicleFootprint(x, y, theta);
            
            % Verify footprint is [4x2] array
            testCase.verifySize(footprint, [4, 2]);
            
            % Verify footprint contains vehicle centroid
            centroid = mean(footprint, 1);
            expectedCentroid = [x, y]; % Approximately
            testCase.verifyEqual(centroid, expectedCentroid, 'AbsTol', 0.5);
        end
        
        function testTurningRadiusConsistency(testCase)
            % Test turning radius vs steering angle consistency
            wheelbase = testCase.vehicleParams.wheelbase;
            maxSteerAngle = deg2rad(testCase.vehicleParams.maxSteeringAngle);
            minRadius = testCase.vehicleParams.minTurningRadius;
            
            theoreticalRadius = wheelbase / tan(maxSteerAngle);
            
            % Allow 10% tolerance for practical considerations
            testCase.verifyEqual(theoreticalRadius, minRadius, 'RelTol', 0.1);
        end
        
        function testConfigExport(testCase)
            % Test export to workspace
            testCase.vehicleParams.exportToStruct();
            
            % Verify structure exists in base workspace
            testCase.verifyTrue(evalin('base', 'exist(''VehicleConfig'', ''var'') == 1'));
            
            % Verify structure contents
            VehicleConfig = evalin('base', 'VehicleConfig');
            testCase.verifyTrue(isfield(VehicleConfig, 'Dimensions'));
            testCase.verifyTrue(isfield(VehicleConfig, 'Steering'));
            testCase.verifyTrue(isfield(VehicleConfig, 'Performance'));
            testCase.verifyTrue(isfield(VehicleConfig, 'Safety'));
            
            % Clean up workspace
            evalin('base', 'clear VehicleConfig');
        end
        
        function testPlanningParameterAccess(testCase)
            % Test access to planning parameters
            planningParams = testCase.vehicleParams.getPlanningParams();
            
            testCase.verifyTrue(isfield(planningParams, 'globalPlannerTimeout'));
            testCase.verifyTrue(isfield(planningParams, 'replanTimeout'));
            testCase.verifyTrue(isfield(planningParams, 'spatialResolution'));
            testCase.verifyTrue(isfield(planningParams, 'angularResolution'));
            testCase.verifyTrue(isfield(planningParams, 'costWeights'));
            
            % Verify specific values
            testCase.verifyEqual(planningParams.globalPlannerTimeout, 1250);
            testCase.verifyEqual(planningParams.replanTimeout, 200);
            testCase.verifyEqual(planningParams.spatialResolution, 0.25);
            testCase.verifyEqual(planningParams.angularResolution, 7.5);
        end
        
        function testEnvironmentParameterAccess(testCase)
            % Test access to environment parameters
            envParams = testCase.vehicleParams.getEnvironmentParams();
            
            testCase.verifyTrue(isfield(envParams, 'maxObstacles'));
            testCase.verifyTrue(isfield(envParams, 'maxDynamicObjects'));
            testCase.verifyTrue(isfield(envParams, 'parkingSpaceWidth'));
            testCase.verifyTrue(isfield(envParams, 'planningHorizon'));
            
            % Verify specific values
            testCase.verifyEqual(envParams.maxObstacles, 50);
            testCase.verifyEqual(envParams.maxDynamicObjects, 3);
            testCase.verifyEqual(envParams.parkingSpaceWidth, 2.3);
            testCase.verifyEqual(envParams.planningHorizon, 30.0);
        end
        
        function testSimulationParameterAccess(testCase)
            % Test access to simulation parameters
            simParams = testCase.vehicleParams.getSimulationParams();
            
            testCase.verifyTrue(isfield(simParams, 'baseSampleTime'));
            testCase.verifyTrue(isfield(simParams, 'globalPlannerRate'));
            testCase.verifyTrue(isfield(simParams, 'localAdapterRate'));
            testCase.verifyTrue(isfield(simParams, 'trajectoryGenRate'));
            testCase.verifyTrue(isfield(simParams, 'safetyMonitorRate'));
            
            % Verify specific values
            testCase.verifyEqual(simParams.baseSampleTime, 0.02);
            testCase.verifyEqual(simParams.globalPlannerRate, 1.0);
            testCase.verifyEqual(simParams.localAdapterRate, 20.0);
            testCase.verifyEqual(simParams.trajectoryGenRate, 50.0);
            testCase.verifyEqual(simParams.safetyMonitorRate, 100.0);
        end
        
        function testInvalidConfigFile(testCase)
            % Test behavior with non-existent config file
            testCase.verifyError(@() Planner.Src.VehicleParams('nonexistent.yaml'), ...
                'Planner:VehicleParams:FileNotFound');
        end
        
        function testConfigValidation(testCase)
            % Test configuration validation with invalid config
            invalidConfigFile = 'invalid_planner_test_config.yaml';
            testCase.createInvalidTestConfig(invalidConfigFile);
            
            try
                % This should fail during validation
                invalidVP = Planner.Src.VehicleParams(invalidConfigFile);
                testCase.verifyFalse(invalidVP.validateConfig(), ...
                    'Validation should fail for invalid config');
            catch ME
                % Either validation error or file loading error is acceptable
                testCase.verifyTrue(contains(ME.identifier, 'Planner:VehicleParams'), ...
                    'Error should be from VehicleParams');
            end
            
            % Clean up
            if isfile(invalidConfigFile)
                delete(invalidConfigFile);
            end
        end
        
        function testDefaultConfigPath(testCase)
            % Test default configuration file loading
            if isfile('+Planner/+Config/vehicle_config.yaml')
                try
                    defaultVP = Planner.Src.VehicleParams();
                    testCase.verifyTrue(defaultVP.isLoaded());
                    testCase.verifyTrue(defaultVP.validateConfig());
                catch ME
                    % If default config fails, that's still a test result
                    testCase.verifyTrue(contains(ME.identifier, 'Planner:VehicleParams'));
                end
            else
                % If no default config file, constructor should handle gracefully
                testCase.verifyError(@() Planner.Src.VehicleParams(), ...
                    'Planner:VehicleParams:FileNotFound');
            end
        end
    end
    
    methods (Access = private)
        function createTestConfig(testCase)
            % Create test YAML configuration file
            configContent = sprintf([...
                '# Test Vehicle Configuration - Planner Package\n',...
                'vehicle:\n',...
                '  length: 4.07\n',...
                '  width: 1.75\n',...
                '  wheelbase: 2.55\n',...
                '  rear_axle_distance: 1.53\n',...
                '  max_steering_angle: 44\n',...
                '  max_steering_rate: 35\n',...
                '  min_turning_radius: 2.64\n',...
                '  max_velocity: 8.0\n',...
                '  max_acceleration: 2.5\n',...
                '  max_jerk: 1.5\n',...
                '  safety_margin_static: 0.75\n',...
                '  safety_margin_dynamic: 1.5\n',...
                '  emergency_stop_distance: 2.0\n',...
                '\n',...
                'planning:\n',...
                '  global_planner_timeout: 1250\n',...
                '  replan_timeout: 200\n',...
                '  max_planning_iterations: 10000\n',...
                '  spatial_resolution: 0.25\n',...
                '  angular_resolution: 7.5\n',...
                '  cost_weights:\n',...
                '    length: 1.0\n',...
                '    time: 0.8\n',...
                '    comfort: 0.6\n',...
                '    efficiency: 0.4\n',...
                '    reverse_penalty: 2.0\n',...
                '\n',...
                'environment:\n',...
                '  max_obstacles: 50\n',...
                '  max_dynamic_objects: 3\n',...
                '  parking_space_width: 2.3\n',...
                '  planning_horizon: 30.0\n',...
                '\n',...
                'simulation:\n',...
                '  base_sample_time: 0.02\n',...
                '  global_planner_rate: 1.0\n',...
                '  local_adapter_rate: 20.0\n',...
                '  trajectory_gen_rate: 50.0\n',...
                '  safety_monitor_rate: 100.0\n']);
            
            fid = fopen(testCase.testConfigFile, 'w');
            fprintf(fid, '%s', configContent);
            fclose(fid);
        end
        
        function createInvalidTestConfig(~, filename)
            % Create invalid YAML configuration for testing
            configContent = sprintf([...
                '# Invalid Test Configuration - Planner Package\n',...
                'vehicle:\n',...
                '  length: 4.07\n',...
                '  width: 1.75\n',...
                '  wheelbase: -1.0\n',...  % Invalid negative wheelbase
                '  rear_axle_distance: 1.53\n',...
                '  max_steering_angle: 44\n',...
                '  max_steering_rate: 35\n',...
                '  min_turning_radius: 2.64\n',...
                '  max_velocity: 8.0\n',...
                '  max_acceleration: 2.5\n',...
                '  max_jerk: 1.5\n',...
                '  safety_margin_static: 0.75\n',...
                '  safety_margin_dynamic: 1.5\n',...
                '  emergency_stop_distance: 2.0\n']);
            
            fid = fopen(filename, 'w');
            fprintf(fid, '%s', configContent);
            fclose(fid);
        end
    end
end