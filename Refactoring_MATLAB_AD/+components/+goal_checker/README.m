% GOAL CHECKER COMPONENT
% =====================
%
% This component provides comprehensive goal checking functionality for vehicle 
% path planning systems, supporting both articulated and car vehicle types.
%
% COMPONENT STRUCTURE:
% -------------------
% +components/+goal_checker/
%   ├── +core/                    % Core goal checking functions
%   │   ├── check_goal_reached.m      % Original function interface
%   │   └── check_goal_reached_bus.m  % Bus-based interface
%   │
%   ├── +bus_structures/          % Bus structure definitions
%   │   └── create_bus_definitions.m  % Creates all Simulink bus structures
%   │
%   ├── +helpers/                 % Helper utilities
%   │   ├── create_vehicle_state.m    % Vehicle state constructor
%   │   ├── create_tolerance_config.m % Tolerance configuration
%   │   └── create_vehicle_config.m   % Vehicle type configuration
%   │
%   ├── +examples/                % Usage examples and demos
%   │   ├── basic_usage_demo.m        % Simple usage examples
%   │   └── bus_integration_demo.m    % Bus structure examples
%   │
%   └── GoalChecker.m             % Main component interface class
%
% QUICK START:
% -----------
% 1. Initialize the component:
%    gc = components.goal_checker.GoalChecker();
%
% 2. Set up vehicle and goal states:
%    current = gc.create_state(60.1, 64.2, 268, 1, 'articulated');
%    goal = gc.create_state(60, 64, 270, 0, 'articulated');
%
% 3. Check goal achievement:
%    [should_stop, reason, errors] = gc.check_goal(current, goal);
%
% FEATURES:
% --------
% ✓ Support for articulated and car vehicle types
% ✓ Configurable tolerance settings for different scenarios
% ✓ Clean bus structure interface for Simulink integration
% ✓ Comprehensive validation and error checking
% ✓ Helper functions for easy state creation
% ✓ Detailed error reporting and debugging information
%
% VEHICLE TYPES SUPPORTED:
% -----------------------
% • Articulated Vehicle: x, y, theta, gamma (tractor-trailer)
% • Car Vehicle: x, y, theta (standard car)
%
% INTEGRATION:
% -----------
% • MATLAB Scripts: Use GoalChecker class or direct function calls
% • Simulink Models: Use bus structures with MATLAB Function blocks
% • Planning Systems: Integrate with existing path planning architecture
%
% SEE ALSO:
% --------
% components.goal_checker.GoalChecker - Main component interface
% components.goal_checker.examples.basic_usage_demo - Usage examples
% components.goal_checker.examples.bus_integration_demo - Simulink integration
