% GOAL CHECKER COMPONENT - COMPLETE ORGANIZATION GUIDE
% ===================================================
%
% This document provides a complete overview of the organized Goal Checker
% component structure, including all files, their purposes, and usage patterns.

%% COMPONENT ARCHITECTURE
% ======================
%
% +components/+goal_checker/               % Main component package
%   │
%   ├── GoalChecker.m                      % Main component interface class
%   ├── README.m                           % Component documentation
%   │
%   ├── +core/                            % Core functionality
%   │   ├── check_goal_reached.m          % Original parameter-based function
%   │   └── check_goal_reached_bus.m      % Bus structure-based function
%   │
%   ├── +bus_structures/                  % Bus definitions and utilities
%   │   ├── create_bus_elem.m             % Centralized bus element creator
%   │   └── create_bus_definitions.m      % Complete bus structure definitions
%   │
%   ├── +helpers/                         % Helper utilities
%   │   ├── create_vehicle_state.m        % Vehicle state constructor
%   │   ├── create_tolerance_config.m     % Tolerance configuration
%   │   └── create_vehicle_config.m       % Vehicle type configuration
%   │
%   └── +examples/                        % Usage examples and demos
%       ├── basic_usage_demo.m            % Simple usage examples
%       └── bus_integration_demo.m        % Bus structure and Simulink examples

%% QUICK START GUIDE
% ==================

% 1. CREATE COMPONENT INSTANCE
% gc = components.goal_checker.GoalChecker();

% 2. BASIC USAGE (Parameter Interface)
% current = gc.create_state(60.1, 64.2, 268, 1, 'articulated');
% goal = gc.create_state(60, 64, 270, 0, 'articulated');
% [should_stop, reason, errors] = gc.check_goal(current, goal);

% 3. BUS STRUCTURE USAGE (Simulink Ready)
% gc.create_bus_structures();  % Create Simulink bus definitions
% input_bus = gc.create_input_bus(current, goal, tolerances, 'articulated');
% [should_stop, reason, errors] = gc.check_goal(input_bus);

% 4. RUN EXAMPLES
% gc.demo('basic');    % Basic usage examples
% gc.demo('bus');      % Bus structure examples
% gc.demo('all');      % All examples

%% FILE DESCRIPTIONS
% ==================

% MAIN INTERFACE
% --------------
% GoalChecker.m
%   - Main component class providing unified interface
%   - Handles both parameter-based and bus-based goal checking
%   - Provides helper methods for state/tolerance creation
%   - Manages bus structure creation and examples

% CORE FUNCTIONS
% --------------
% +core/check_goal_reached.m
%   - Original goal checking function with parameter interface
%   - Supports both articulated and car vehicle types
%   - Provides detailed error information and validation
%   - Backward compatible with existing code

% +core/check_goal_reached_bus.m
%   - Enhanced goal checking using bus structures
%   - Optimized for Simulink integration
%   - Structured input/output for better maintainability
%   - Code generation compatible

% BUS STRUCTURES
% --------------
% +bus_structures/create_bus_elem.m
%   - Centralized bus element creation with validation
%   - Code generation compatibility checking
%   - Consistent element properties and constraints
%   - Based on ASML best practices for bus element creation

% +bus_structures/create_bus_definitions.m
%   - Complete bus structure definitions for the system
%   - Creates: VehicleStateBus, ToleranceBus, VehicleConfigBus
%   - Creates: GoalCheckInputBus, GoalCheckOutputBus
%   - Simulink-ready with proper data types and constraints

% HELPER FUNCTIONS
% ----------------
% +helpers/create_vehicle_state.m
%   - Creates standardized vehicle state structures
%   - Handles both articulated (x,y,theta,gamma) and car (x,y,theta) types
%   - Input validation and type conversion

% +helpers/create_tolerance_config.m
%   - Creates tolerance configuration structures
%   - Default values for different scenarios (parking, highway)
%   - Validation of positive tolerance values

% +helpers/create_vehicle_config.m
%   - Creates vehicle type configuration structures
%   - Defines capabilities (has_articulation flag)
%   - Simulink-compatible numeric codes and string arrays

% EXAMPLES AND DEMOS
% ------------------
% +examples/basic_usage_demo.m
%   - Demonstrates basic goal checking functionality
%   - Shows both articulated and car vehicle usage
%   - Custom tolerance scenarios and simulation loops

% +examples/bus_integration_demo.m
%   - Demonstrates bus structure usage
%   - Simulink integration guidance
%   - Bus structure validation and testing

%% USAGE PATTERNS
% ===============

% PATTERN 1: Simple Goal Checking
% --------------------------------
% gc = components.goal_checker.GoalChecker();
% current = gc.create_state(x, y, theta, gamma, 'articulated');
% goal = gc.create_state(x_goal, y_goal, theta_goal, gamma_goal, 'articulated');
% [should_stop, reason, errors] = gc.check_goal(current, goal);

% PATTERN 2: Custom Tolerances
% -----------------------------
% tolerances = gc.create_tolerances(1.0, 10.0, 5.0);  % tight tolerances
% [should_stop, reason, errors] = gc.check_goal(current, goal, tolerances);

% PATTERN 3: Vehicle Type Specific
% ---------------------------------
% % For car vehicles (no articulation)
% current_car = gc.create_state(x, y, theta, 0, 'car');
% goal_car = gc.create_state(x_goal, y_goal, theta_goal, 0, 'car');
% [should_stop, reason, errors] = gc.check_goal(current_car, goal_car);

% PATTERN 4: Simulink Integration
% --------------------------------
% gc.create_bus_structures();  % Create bus definitions
% input_bus = gc.create_input_bus(current, goal, tolerances, vehicle_type);
% [should_stop, reason, errors] = gc.check_goal(input_bus);

% PATTERN 5: Simulation Loop
% ---------------------------
% for step = 1:max_steps
%     % Update vehicle state
%     current_state = update_vehicle_state(current_state);
%     
%     % Check goal
%     [should_stop, reason, errors] = gc.check_goal(current_state, goal_state);
%     
%     if should_stop
%         fprintf('Goal reached at step %d\n', step);
%         break;
%     end
% end

%% SIMULINK INTEGRATION
% =====================

% STEP 1: Setup Bus Structures
% gc = components.goal_checker.GoalChecker();
% gc.create_bus_structures();

% STEP 2: Create MATLAB Function Block
% function [should_stop, goal_errors] = goal_check_fcn(input_bus)
%     gc = components.goal_checker.GoalChecker();
%     [should_stop, ~, goal_errors] = gc.check_goal(input_bus);
% end

% STEP 3: Configure Block Ports
% Input: Bus: GoalCheckInputBus
% Outputs: boolean (should_stop), Bus: GoalCheckOutputBus (goal_errors)

% STEP 4: Connect Signals
% Use Bus Creator blocks to construct input_bus from:
% - current_state (from vehicle model)
% - goal_state (from path planner)
% - tolerances (from configuration)
% - vehicle_config (from vehicle type selector)

%% VEHICLE TYPES SUPPORTED
% ========================

% ARTICULATED VEHICLE
% -------------------
% State fields: x, y, theta, gamma
% Description: Tractor-trailer with articulation angle
% Use case: Long vehicles with trailer articulation
% Example: gc.create_state(60, 64, 270, 5, 'articulated');

% CAR VEHICLE
% -----------
% State fields: x, y, theta (gamma ignored/set to 0)
% Description: Standard car without articulation
% Use case: Regular passenger cars, simple vehicles
% Example: gc.create_state(60, 64, 270, 0, 'car');

%% TOLERANCE SCENARIOS
% ====================

% PARKING SCENARIO (Tight Tolerances)
% tolerances = gc.create_tolerances(0.5, 5.0, 3.0);
% Position: 0.5m, Orientation: 5°, Articulation: 3°

% HIGHWAY SCENARIO (Relaxed Tolerances)
% tolerances = gc.create_tolerances(3.0, 20.0, 15.0);
% Position: 3.0m, Orientation: 20°, Articulation: 15°

% DEFAULT SCENARIO (Balanced Tolerances)
% tolerances = gc.create_tolerances();  % Uses defaults
% Position: 2.0m, Orientation: 15°, Articulation: 10°

%% MIGRATION FROM OLD CODE
% ========================

% OLD CODE (scattered files in root):
% check_goal_reached.m
% create_vehicle_state.m
% create_tolerance_config.m
% bus_structure_demo.m
% etc.

% NEW CODE (organized component):
% gc = components.goal_checker.GoalChecker();
% [should_stop, reason, errors] = gc.check_goal(current, goal);

% BACKWARD COMPATIBILITY:
% The original check_goal_reached function is still available at:
% components.goal_checker.core.check_goal_reached()

%% BEST PRACTICES
% ===============

% 1. USE COMPONENT INTERFACE
%    - Always use GoalChecker class for new code
%    - Provides consistent interface and validation

% 2. CHOOSE APPROPRIATE TOLERANCES
%    - Parking: Tight tolerances (0.5m, 5°, 3°)
%    - Highway: Relaxed tolerances (3m, 20°, 15°)
%    - Default: Balanced tolerances (2m, 15°, 10°)

% 3. VEHICLE TYPE CONSISTENCY
%    - Use 'articulated' for vehicles with gamma angle
%    - Use 'car' for simple vehicles without articulation
%    - Ensure state structures match vehicle type

% 4. BUS STRUCTURES FOR SIMULINK
%    - Create bus structures once per session
%    - Use bus interface for Simulink integration
%    - Validate bus compatibility before deployment

% 5. ERROR HANDLING
%    - Check should_stop flag for goal achievement
%    - Use error structure for debugging information
%    - Monitor individual tolerance satisfaction

%% COMPONENT BENEFITS
% ===================

% ✓ ORGANIZED STRUCTURE
%   - Clear separation of concerns
%   - Logical file organization
%   - Easy to find and maintain

% ✓ UNIFIED INTERFACE
%   - Single entry point through GoalChecker class
%   - Consistent API across all functionality
%   - Simplified usage patterns

% ✓ MODULARITY
%   - Centralized bus element creation
%   - Reusable helper functions
%   - Independent core functions

% ✓ SIMULINK READY
%   - Code generation compatible
%   - Proper bus structure definitions
%   - Integration guidance and examples

% ✓ VALIDATION & ROBUSTNESS
%   - Input validation at multiple levels
%   - Error checking and meaningful messages
%   - Type safety and bounds checking

% ✓ EXTENSIBILITY
%   - Easy to add new vehicle types
%   - Flexible tolerance configurations
%   - Pluggable architecture

%% SUPPORT AND EXAMPLES
% =====================

% RUN EXAMPLES:
% components.goal_checker.examples.basic_usage_demo()
% components.goal_checker.examples.bus_integration_demo()

% OR USING COMPONENT:
% gc = components.goal_checker.GoalChecker();
% gc.demo('all');

% READ DOCUMENTATION:
% help components.goal_checker.GoalChecker
% help components.goal_checker.README
