classdef PlannerTestSuite < matlab.unittest.TestCase
    % PlannerTestSuite - Comprehensive regression test suite for +Planner package
    % 
    % This test suite provides comprehensive testing for all components in the
    % +Planner package, including regression tests, integration tests, and
    % performance benchmarks.
    %
    % Package: +Planner.+Testing
    % 
    % Usage:
    %   % Run all tests
    %   results = runtests('Planner.Testing.PlannerTestSuite')
    %   
    %   % Run with detailed output
    %   results = runtests('Planner.Testing.PlannerTestSuite', 'Verbosity', 3)
    %   
    %   % Run specific test group
    %   results = runtests('Planner.Testing.PlannerTestSuite', 'Tag', 'VehicleParams')
    %
    % Test Categories:
    %   - VehicleParams: Configuration management and validation
    %   - BusStructures: Simulink bus definitions and validation (Future)
    %   - MotionPrimitives: ACADO primitive loading and validation (Future)
    %   - Integration: End-to-end component integration (Future)
    %   - Performance: Benchmarking and performance regression (Future)
    
    properties (TestParameter)
        % Test configuration parameters
        validConfigFiles = {'+Planner/+Config/vehicle_config.yaml'};
        invalidConfigScenarios = {'nonexistent.yaml', 'invalid_structure.yaml'};
    end
    
    properties
        testDataDir = '+Planner/+Testing/TestData'
        tempFiles = {}  % Track temporary files for cleanup
    end
    
    methods (TestClassSetup)
        function classSetup(testCase)
            % Setup for entire test class
            fprintf('\n=== Planner Package Regression Test Suite ===\n');
            fprintf('Setting up test environment...\n');
            
            % Create test data directory if it doesn't exist
            if ~isfolder(testCase.testDataDir)
                mkdir(testCase.testDataDir);
            end
            
            % Display package info
            fprintf('Package: +Planner (Autonomous Valet Parking)\n');
            fprintf('Test Date: %s\n', datestr(now));
            fprintf('MATLAB Version: %s\n', version);
        end
    end
    
    methods (TestClassTeardown)
        function classTeardown(testCase)
            % Cleanup after all tests
            fprintf('\nCleaning up test environment...\n');
            
            % Clean up temporary files
            for i = 1:numel(testCase.tempFiles)
                if isfile(testCase.tempFiles{i})
                    delete(testCase.tempFiles{i});
                end
            end
            
            fprintf('=== Test Suite Complete ===\n\n');
        end
    end
    
    methods (TestMethodSetup)
        function methodSetup(testCase)
            % Setup for each test method
            % Add any per-test setup here
        end
    end
    
    methods (TestMethodTeardown)
        function methodTeardown(testCase)
            % Cleanup after each test method
            % Clear workspace variables that might interfere with other tests
            evalin('base', 'clear VehicleConfig');
        end
    end
    
    %% VehicleParams Component Tests
    methods (Test, TestTags = {'VehicleParams', 'Core', 'Regression'})
        
        function testVehicleParamsBasicFunctionality(testCase)
            % Test basic VehicleParams functionality
            vp = Planner.Src.VehicleParams();
            
            % Basic property access
            testCase.verifyTrue(vp.isLoaded(), 'Configuration should be loaded');
            testCase.verifyTrue(vp.validateConfig(), 'Configuration should be valid');
            
            % Test all dependent properties
            testCase.verifyGreaterThan(vp.length, 0, 'Length must be positive');
            testCase.verifyGreaterThan(vp.width, 0, 'Width must be positive');
            testCase.verifyGreaterThan(vp.wheelbase, 0, 'Wheelbase must be positive');
            testCase.verifyLessThan(vp.wheelbase, vp.length, 'Wheelbase must be less than length');
        end
        
        function testVehicleParamsConfigurationIntegrity(testCase)
            % Test configuration integrity and consistency
            vp = Planner.Src.VehicleParams();
            
            % Physics consistency checks
            expectedRadius = vp.wheelbase / tan(deg2rad(vp.maxSteeringAngle));
            testCase.verifyEqual(vp.minTurningRadius, expectedRadius, 'RelTol', 0.1, ...
                'Turning radius should match theoretical calculation');
            
            % Safety parameter consistency
            testCase.verifyLessThanOrEqual(vp.safetyMarginStatic, vp.safetyMarginDynamic, ...
                'Static safety margin should be <= dynamic margin');
        end
        
        function testVehicleParamsEnhancedAPI(testCase)
            % Test enhanced API methods
            vp = Planner.Src.VehicleParams();
            
            % Planning parameters
            planningParams = vp.getPlanningParams();
            testCase.verifyTrue(isstruct(planningParams), 'Planning params should be struct');
            testCase.verifyTrue(isfield(planningParams, 'globalPlannerTimeout'));
            testCase.verifyTrue(isfield(planningParams, 'costWeights'));
            testCase.verifyTrue(isstruct(planningParams.costWeights), 'Cost weights should be struct');
            
            % Environment parameters
            envParams = vp.getEnvironmentParams();
            testCase.verifyTrue(isstruct(envParams), 'Environment params should be struct');
            testCase.verifyTrue(isfield(envParams, 'maxObstacles'));
            
            % Simulation parameters
            simParams = vp.getSimulationParams();
            testCase.verifyTrue(isstruct(simParams), 'Simulation params should be struct');
            testCase.verifyTrue(isfield(simParams, 'baseSampleTime'));
        end
        
        function testVehicleParamsFootprintCalculation(testCase)
            % Test vehicle footprint calculation
            vp = Planner.Src.VehicleParams();
            
            % Test at origin
            footprint = vp.getVehicleFootprint(0, 0, 0);
            testCase.verifySize(footprint, [4, 2], 'Footprint should be 4x2 array');
            
            % Test at different pose
            footprint = vp.getVehicleFootprint(5, 3, pi/4);
            testCase.verifySize(footprint, [4, 2], 'Footprint should be 4x2 array');
            
            % Verify footprint is reasonable (within bounds)
            maxDim = max(vp.length, vp.width) * 2;  % Allow for rotation
            testCase.verifyLessThan(max(footprint(:)), maxDim, ...
                'Footprint coordinates should be reasonable');
        end
        
        function testVehicleParamsYAMLParser(testCase)
            % Test YAML parser with various configurations
            
            % Create test YAML with nested structures
            testFile = fullfile(testCase.testDataDir, 'test_nested.yaml');
            testCase.tempFiles{end+1} = testFile;
            
            yamlContent = sprintf([...
                'vehicle:\n',...
                '  length: 4.5\n',...
                '  width: 2.0\n',...
                '\n',...
                'planning:\n',...
                '  timeout: 1000\n',...
                '  weights:\n',...
                '    length: 1.0\n',...
                '    time: 0.8\n',...
                '    nested:\n',...
                '      deep_value: 42\n']);
            
            fid = fopen(testFile, 'w');
            fprintf(fid, '%s', yamlContent);
            fclose(fid);
            
            % Test parser
            vp = Planner.Src.VehicleParams(testFile);
            testCase.verifyEqual(vp.length, 4.5, 'Should parse vehicle length');
            testCase.verifyEqual(vp.width, 2.0, 'Should parse vehicle width');
        end
    end
    
    %% Error Handling and Edge Cases
    methods (Test, TestTags = {'ErrorHandling', 'EdgeCases', 'Regression'})
        
        function testVehicleParamsInvalidConfiguration(testCase)
            % Test behavior with invalid configurations
            
            % Test with nonexistent file
            testCase.verifyError(@() Planner.Src.VehicleParams('nonexistent.yaml'), ...
                'Planner:VehicleParams:FileNotFound');
            
            % Test with invalid structure
            invalidFile = fullfile(testCase.testDataDir, 'invalid_test.yaml');
            testCase.tempFiles{end+1} = invalidFile;
            
            invalidContent = sprintf([...
                'vehicle:\n',...
                '  length: -1.0\n',...  % Invalid negative length
                '  width: 2.0\n',...
                '  wheelbase: 3.0\n',...  % Invalid: wheelbase > length
                '  rear_axle_distance: 1.5\n',...
                '  max_steering_angle: 45\n',...
                '  max_steering_rate: 30\n',...
                '  min_turning_radius: 5.0\n',...
                '  max_velocity: 10.0\n',...
                '  max_acceleration: 3.0\n',...
                '  max_jerk: 2.0\n',...
                '  safety_margin_static: 0.5\n',...
                '  safety_margin_dynamic: 1.0\n',...
                '  emergency_stop_distance: 2.0\n']);
            
            fid = fopen(invalidFile, 'w');
            fprintf(fid, '%s', invalidContent);
            fclose(fid);
            
            % Should fail validation
            testCase.verifyError(@() Planner.Src.VehicleParams(invalidFile), ...
                'Planner:VehicleParams:ValidationFailed');
        end
        
        function testVehicleParamsEdgeCaseValues(testCase)
            % Test with edge case values
            
            edgeFile = fullfile(testCase.testDataDir, 'edge_case.yaml');
            testCase.tempFiles{end+1} = edgeFile;
            
            % Minimal valid configuration
            edgeContent = sprintf([...
                'vehicle:\n',...
                '  length: 3.0\n',...
                '  width: 1.5\n',...
                '  wheelbase: 2.0\n',...
                '  rear_axle_distance: 1.0\n',...
                '  max_steering_angle: 30\n',...  % Lower limit
                '  max_steering_rate: 20\n',...
                '  min_turning_radius: 3.46\n',...  % Calculated for 30 degrees
                '  max_velocity: 5.0\n',...
                '  max_acceleration: 1.5\n',...
                '  max_jerk: 1.0\n',...
                '  safety_margin_static: 0.1\n',...  % Minimal margin
                '  safety_margin_dynamic: 0.2\n',...
                '  emergency_stop_distance: 1.0\n',...
                '\n',...
                'planning:\n',...
                '  global_planner_timeout: 500\n',...
                '  replan_timeout: 100\n',...
                '  max_planning_iterations: 1000\n',...
                '  spatial_resolution: 0.5\n',...
                '  angular_resolution: 15.0\n',...
                '  cost_weights:\n',...
                '    length: 1.0\n',...
                '    time: 0.5\n',...
                '    comfort: 0.3\n',...
                '    efficiency: 0.2\n',...
                '    reverse_penalty: 1.5\n',...
                '\n',...
                'environment:\n',...
                '  max_obstacles: 10\n',...
                '  max_dynamic_objects: 1\n',...
                '  parking_space_width: 2.0\n',...
                '  planning_horizon: 15.0\n',...
                '\n',...
                'simulation:\n',...
                '  base_sample_time: 0.05\n',...
                '  global_planner_rate: 0.5\n',...
                '  local_adapter_rate: 10.0\n',...
                '  trajectory_gen_rate: 20.0\n',...
                '  safety_monitor_rate: 50.0\n']);
            
            fid = fopen(edgeFile, 'w');
            fprintf(fid, '%s', edgeContent);
            fclose(fid);
            
            % Should work with minimal values
            vp = Planner.Src.VehicleParams(edgeFile);
            testCase.verifyTrue(vp.isLoaded(), 'Should load edge case configuration');
            testCase.verifyTrue(vp.validateConfig(), 'Should validate edge case configuration');
        end
    end
    
    %% Integration Tests (Placeholder for future components)
    methods (Test, TestTags = {'Integration', 'Future'})
        
        function testPlaceholderBusStructures(testCase)
            % Placeholder for future bus structure tests
            testCase.assumeFail('Bus structures not yet implemented');
        end
        
        function testPlaceholderMotionPrimitives(testCase)
            % Placeholder for future motion primitive tests
            testCase.assumeFail('Motion primitives not yet implemented');
        end
        
        function testPlaceholderSimulinkIntegration(testCase)
            % Placeholder for future Simulink integration tests
            testCase.assumeFail('Simulink integration not yet implemented');
        end
    end
    
    %% Performance Tests
    methods (Test, TestTags = {'Performance', 'Benchmark'})
        
        function testVehicleParamsPerformance(testCase)
            % Performance benchmarks for VehicleParams
            
            % Benchmark instantiation time
            tic;
            for i = 1:10
                vp = Planner.Src.VehicleParams();
                clear vp;  % Force cleanup
            end
            instantiationTime = toc / 10;  % Average time
            
            testCase.verifyLessThan(instantiationTime, 1.0, ...
                'VehicleParams instantiation should be < 1 second');
            
            fprintf('Average VehicleParams instantiation time: %.3f seconds\n', instantiationTime);
            
            % Benchmark property access
            vp = Planner.Src.VehicleParams();
            tic;
            for i = 1:1000
                length = vp.length; %#ok<NASGU>
                width = vp.width; %#ok<NASGU>
                wheelbase = vp.wheelbase; %#ok<NASGU>
            end
            propertyTime = toc / 1000;  % Average time per access
            
            testCase.verifyLessThan(propertyTime, 0.001, ...
                'Property access should be < 1ms');
            
            fprintf('Average property access time: %.6f seconds\n', propertyTime);
        end
    end
    
    %% Utility Methods
    methods (Access = private)
        function createTestConfigFile(testCase, filename, content)
            % Helper to create test configuration files
            fid = fopen(filename, 'w');
            fprintf(fid, '%s', content);
            fclose(fid);
            testCase.tempFiles{end+1} = filename;
        end
    end
end