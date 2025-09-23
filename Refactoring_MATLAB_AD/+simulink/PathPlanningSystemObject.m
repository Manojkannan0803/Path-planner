classdef PathPlanningSystemObject < matlab.System
    % PATHPLANNINGSYSTEMOBJECT - Main Simulink System Object for Path Planning
    % Integrates the entire refactored path planning system into Simulink
    % Supports real-time path planning with dynamic inputs and scenario updates
    
    properties (Nontunable)
        ConfigurationFile = 'scenarios/DPDScenario.json'    % Configuration file path
        EnableVisualization = false                          % Enable real-time visualization
        PlanningAlgorithm = 'astar'                         % Default algorithm
        UpdateRate = 0.1                                    % Planning update rate (seconds)
    end
    
    properties (Access = private)
        planning_session        % PlanningSession instance
        last_planning_time     % Last planning execution time
        current_path           % Current planned path
        planning_stats         % Last planning statistics
        is_initialized         % Initialization flag
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            % Initialize the path planning system
            try
                % Initialize planning session
                obj.planning_session = PlanningSession(obj.ConfigurationFile);
                
                % Set algorithm
                obj.planning_session.set_algorithm(obj.PlanningAlgorithm);
                
                % Initialize timing
                obj.last_planning_time = 0;
                obj.current_path = [];
                obj.planning_stats = struct();
                obj.is_initialized = true;
                
                fprintf('[Simulink] Path Planning System initialized successfully\n');
                fprintf('[Simulink] Algorithm: %s, Config: %s\n', ...
                    obj.PlanningAlgorithm, obj.ConfigurationFile);
                
            catch ME
                obj.is_initialized = false;
                warning('PathPlanning:InitError', '[Simulink] Failed to initialize Path Planning System: %s', ME.message);
            end
        end
        
        function [path_x, path_y, path_theta, path_gamma, success, stats_out] = stepImpl(obj, ...
                current_x, current_y, current_theta, current_gamma, ...
                goal_x, goal_y, goal_theta, goal_gamma, ...
                trigger_planning, simulation_time)
            % Main step function - executes path planning
            
            % Initialize outputs
            path_x = zeros(100, 1);      % Pre-allocate path arrays
            path_y = zeros(100, 1);
            path_theta = zeros(100, 1);
            path_gamma = zeros(100, 1);
            success = false;
            stats_out = [0, 0, 0, 0];    % [iterations, nodes_explored, computation_time, path_length]
            
            if ~obj.is_initialized
                return;
            end
            
            % Check if planning should be triggered
            should_plan = trigger_planning > 0.5 || ...
                         (simulation_time - obj.last_planning_time) > obj.UpdateRate;
            
            if should_plan
                try
                    % Create state structures
                    start_state = struct('x', current_x, 'y', current_y, ...
                                       'theta', current_theta, 'gamma', current_gamma);
                    goal_state = struct('x', goal_x, 'y', goal_y, ...
                                      'theta', goal_theta, 'gamma', goal_gamma);
                    
                    % Execute path planning
                    [path, success, stats] = obj.planning_session.execute_single_planning(...
                        start_state, goal_state);
                    
                    % Store results
                    obj.current_path = path;
                    obj.planning_stats = stats;
                    obj.last_planning_time = simulation_time;
                    
                    % Convert path to output format
                    if success && ~isempty(path)
                        path_length = min(length(path.x), 100);
                        path_x(1:path_length) = path.x(1:path_length);
                        path_y(1:path_length) = path.y(1:path_length);
                        path_theta(1:path_length) = path.theta(1:path_length);
                        path_gamma(1:path_length) = path.gamma(1:path_length);
                        
                        % Extract statistics
                        stats_out(1) = stats.iterations;
                        stats_out(2) = stats.nodes_explored;
                        stats_out(3) = stats.computation_time;
                        stats_out(4) = path_length;
                    end
                    
                    fprintf('[Simulink] Planning completed: Success=%d, Path length=%d\n', ...
                        success, sum(path_x ~= 0));
                    
                catch ME
                    warning('PathPlanning:PlanningError', '[Simulink] Planning failed: %s', ME.message);
                    success = false;
                end
            else
                % Return last computed path
                if ~isempty(obj.current_path)
                    path_length = min(length(obj.current_path.x), 100);
                    path_x(1:path_length) = obj.current_path.x(1:path_length);
                    path_y(1:path_length) = obj.current_path.y(1:path_length);
                    path_theta(1:path_length) = obj.current_path.theta(1:path_length);
                    path_gamma(1:path_length) = obj.current_path.gamma(1:path_length);
                    success = true;
                end
            end
        end
        
        function resetImpl(obj)
            % Reset the system
            obj.last_planning_time = 0;
            obj.current_path = [];
            obj.planning_stats = struct();
            if obj.is_initialized
                fprintf('[Simulink] Path Planning System reset\n');
            end
        end
        
        function releaseImpl(obj)
            % Release resources
            obj.planning_session = [];
            obj.is_initialized = false;
            fprintf('[Simulink] Path Planning System released\n');
        end
        
        function flag = isInactivePropertyImpl(~, ~)
            % Define inactive properties
            flag = false;
        end
    end
    
    methods (Access = protected, Static)
        function header = getHeaderImpl()
            header = matlab.system.display.Header('PathPlanningSystemObject', ...
                'Title', 'ADAS Path Planning System', ...
                'Text', ['Real-time path planning system for ADAS applications. ' ...
                        'Supports multiple algorithms (A*, Dijkstra, RRT) with ' ...
                        'collision detection for tractor-trailer vehicles.']);
        end
        
        function groups = getPropertyGroupsImpl()
            % Define property groups for System block dialog
            main_group = matlab.system.display.Section(...
                'Title', 'Main Configuration', ...
                'PropertyList', {'ConfigurationFile', 'PlanningAlgorithm'});
            
            execution_group = matlab.system.display.Section(...
                'Title', 'Execution Settings', ...
                'PropertyList', {'UpdateRate', 'EnableVisualization'});
            
            groups = [main_group, execution_group];
        end
        
        function simMode = getSimulateUsingImpl()
            simMode = 'Interpreted execution';
        end
        
        function flag = showSimulateUsingImpl()
            flag = false;
        end
    end
    
    methods (Static, Access = protected)
        function groups = getPropertyGroupsForMaskImpl()
            % Property groups for mask dialog
            main_props = matlab.system.display.Section(...
                'Title', 'Path Planning Configuration', ...
                'PropertyList', {'ConfigurationFile', 'PlanningAlgorithm', 'UpdateRate'});
            
            viz_props = matlab.system.display.Section(...
                'Title', 'Visualization', ...
                'PropertyList', {'EnableVisualization'});
            
            groups = [main_props, viz_props];
        end
    end
    
    methods (Access = public)
        function set_algorithm(obj, algorithm_name)
            % Change planning algorithm at runtime
            if obj.is_initialized
                obj.planning_session.set_algorithm(algorithm_name);
                obj.PlanningAlgorithm = algorithm_name;
                fprintf('[Simulink] Algorithm changed to: %s\n', algorithm_name);
            end
        end
        
        function update_configuration(obj, config_file)
            % Update configuration file
            if obj.is_initialized
                obj.ConfigurationFile = config_file;
                obj.setupImpl();  % Reinitialize with new config
            end
        end
        
        function stats = get_last_stats(obj)
            % Get last planning statistics
            stats = obj.planning_stats;
        end
    end
end
