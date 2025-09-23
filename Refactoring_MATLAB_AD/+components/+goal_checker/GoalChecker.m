classdef GoalChecker < handle
    % GOALCHECKER - Main interface class for vehicle goal checking
    %
    % This class provides a clean, object-oriented interface to the goal checking
    % functionality, supporting both articulated and car vehicle types.
    %
    % PROPERTIES:
    %   default_tolerances - Default tolerance configuration
    %   vehicle_configs - Available vehicle type configurations
    %
    % METHODS:
    %   GoalChecker() - Constructor
    %   check_goal() - Check if goal is reached (multiple signatures)
    %   create_state() - Create vehicle state structure
    %   create_tolerances() - Create tolerance configuration
    %   set_default_tolerances() - Set default tolerance values
    %   get_vehicle_types() - Get list of supported vehicle types
    %
    % EXAMPLES:
    %   % Basic usage
    %   gc = components.goal_checker.GoalChecker();
    %   current = gc.create_state(60.1, 64.2, 268, 1, 'articulated');
    %   goal = gc.create_state(60, 64, 270, 0, 'articulated');
    %   [should_stop, reason, errors] = gc.check_goal(current, goal);
    %
    %   % With custom tolerances
    %   tolerances = gc.create_tolerances(1.0, 10.0, 5.0);
    %   [should_stop, reason, errors] = gc.check_goal(current, goal, tolerances);
    %
    %   % Bus structure interface
    %   input_bus = gc.create_input_bus(current, goal, tolerances, 'articulated');
    %   [should_stop, reason, errors] = gc.check_goal_bus(input_bus);
    
    properties (Access = private)
        default_tolerances
        vehicle_configs
        bus_structures_created
    end
    
    methods
        function obj = GoalChecker()
            % Constructor - Initialize the goal checker component
            obj.initialize_defaults();
            obj.bus_structures_created = false;
        end
        
        function [should_stop, stop_reason, goal_errors] = check_goal(obj, varargin)
            % CHECK_GOAL - Check if vehicle has reached goal (multiple signatures)
            %
            % Signatures:
            %   [stop, reason, errors] = check_goal(current_state, goal_state)
            %   [stop, reason, errors] = check_goal(current_state, goal_state, tolerances)
            %   [stop, reason, errors] = check_goal(current_state, goal_state, tolerances, vehicle_type)
            %   [stop, reason, errors] = check_goal(input_bus)
            
            if nargin == 2 && isstruct(varargin{1}) && isfield(varargin{1}, 'current_state')
                % Bus structure interface
                [should_stop, stop_reason, goal_errors] = obj.check_goal_bus(varargin{1});
            else
                % Parameter interface
                [should_stop, stop_reason, goal_errors] = obj.check_goal_params(varargin{:});
            end
        end
        
        function vehicle_state = create_state(obj, x, y, theta, gamma, vehicle_type)
            % CREATE_STATE - Create vehicle state structure
            if nargin < 6
                vehicle_type = 'articulated';
            end
            vehicle_state = components.goal_checker.helpers.create_vehicle_state(x, y, theta, gamma, vehicle_type);
        end
        
        function tolerances = create_tolerances(obj, position_tol, orientation_tol, articulation_tol)
            % CREATE_TOLERANCES - Create tolerance configuration
            if nargin < 2
                position_tol = obj.default_tolerances.position_tolerance;
            end
            if nargin < 3
                orientation_tol = obj.default_tolerances.orientation_tolerance;
            end
            if nargin < 4
                articulation_tol = obj.default_tolerances.articulation_tolerance;
            end
            tolerances = components.goal_checker.helpers.create_tolerance_config(position_tol, orientation_tol, articulation_tol);
        end
        
        function input_bus = create_input_bus(obj, current_state, goal_state, tolerances, vehicle_type)
            % CREATE_INPUT_BUS - Create complete input bus structure
            if nargin < 5
                vehicle_type = 'articulated';
            end
            if nargin < 4 || isempty(tolerances)
                tolerances = obj.create_tolerances();
            end
            
            input_bus = struct();
            input_bus.current_state = current_state;
            input_bus.goal_state = goal_state;
            input_bus.tolerances = tolerances;
            input_bus.vehicle_config = components.goal_checker.helpers.create_vehicle_config(vehicle_type);
        end
        
        function set_default_tolerances(obj, position_tol, orientation_tol, articulation_tol)
            % SET_DEFAULT_TOLERANCES - Update default tolerance values
            obj.default_tolerances.position_tolerance = position_tol;
            obj.default_tolerances.orientation_tolerance = orientation_tol;
            obj.default_tolerances.articulation_tolerance = articulation_tol;
        end
        
        function vehicle_types = get_vehicle_types(obj)
            % GET_VEHICLE_TYPES - Get list of supported vehicle types
            vehicle_types = fieldnames(obj.vehicle_configs);
        end
        
        function create_bus_structures(obj)
            % CREATE_BUS_STRUCTURES - Create Simulink bus definitions
            if ~obj.bus_structures_created
                components.goal_checker.bus_structures.create_bus_definitions();
                obj.bus_structures_created = true;
                fprintf('Bus structures created successfully.\n');
            else
                fprintf('Bus structures already exist in workspace.\n');
            end
        end
        
        function demo(obj, demo_type)
            % DEMO - Run demonstration examples
            if nargin < 2
                demo_type = 'basic';
            end
            
            switch lower(demo_type)
                case 'basic'
                    components.goal_checker.examples.basic_usage_demo();
                case 'bus'
                    components.goal_checker.examples.bus_integration_demo();
                case 'all'
                    components.goal_checker.examples.basic_usage_demo();
                    components.goal_checker.examples.bus_integration_demo();
                otherwise
                    error('Unknown demo type. Use ''basic'', ''bus'', or ''all''');
            end
        end
    end
    
    methods (Access = private)
        function initialize_defaults(obj)
            % Initialize default configurations
            obj.default_tolerances = struct(...
                'position_tolerance', 2.0, ...
                'orientation_tolerance', 15.0, ...
                'articulation_tolerance', 10.0);
            
            obj.vehicle_configs = struct(...
                'articulated', struct('has_articulation', true, 'fields', {{'x', 'y', 'theta', 'gamma'}}), ...
                'car', struct('has_articulation', false, 'fields', {{'x', 'y', 'theta'}}));
        end
        
        function [should_stop, stop_reason, goal_errors] = check_goal_params(obj, current_state, goal_state, tolerances, vehicle_type)
            % Parameter-based goal checking
            if nargin < 4 || isempty(tolerances)
                tolerances = obj.create_tolerances();
            end
            if nargin < 5
                vehicle_type = 'articulated';
            end
            
            % Extract tolerance values
            if isstruct(tolerances)
                pos_tol = tolerances.position_tolerance;
                orient_tol = tolerances.orientation_tolerance;
                artic_tol = tolerances.articulation_tolerance;
            else
                % Assume tolerances is a vector [pos, orient, artic]
                pos_tol = tolerances(1);
                orient_tol = tolerances(2);
                if length(tolerances) > 2
                    artic_tol = tolerances(3);
                else
                    artic_tol = obj.default_tolerances.articulation_tolerance;
                end
            end
            
            % Call the core function
            [should_stop, stop_reason, goal_errors] = components.goal_checker.core.check_goal_reached(...
                current_state, goal_state, pos_tol, orient_tol, artic_tol, vehicle_type);
        end
        
        function [should_stop, stop_reason, goal_errors] = check_goal_bus(obj, input_bus)
            % Bus-based goal checking
            [should_stop, stop_reason, goal_errors] = components.goal_checker.core.check_goal_reached_bus(input_bus);
        end
    end
end
