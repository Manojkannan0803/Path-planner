classdef AlgorithmInterface < handle
    % ALGORITHMINTERFACE - Common interface for all planning algorithms
    % All planning algorithms must implement this interface
    
    methods (Abstract)
        % Main planning execution method
        [path, success, stats] = execute_planning(obj, planning_context)
        
        % Get algorithm information
        info = get_algorithm_info(obj)
        
        % Update services (when services are changed at runtime)
        update_services(obj, services)
        
        % Configure algorithm parameters
        configure(obj, config)
        
        % Validate inputs before planning
        is_valid = validate_inputs(obj, start_state, goal_state)
        
        % Clean up resources
        cleanup(obj)
    end
    
    methods
        function obj = AlgorithmInterface(config, services)
            % Base constructor
            if nargin > 0
                obj.configure(config);
            end
            if nargin > 1
                obj.update_services(services);
            end
        end
        
        function is_supported = supports_feature(obj, feature_name)
            % Check if algorithm supports specific feature
            % Override in subclasses to specify supported features
            is_supported = false;
            
            % Common features that might be supported:
            % - 'realtime_planning'
            % - 'dynamic_obstacles' 
            % - 'path_smoothing'
            % - 'anytime_planning'
            % - 'bidirectional_search'
        end
        
        function params = get_configurable_parameters(obj)
            % Get list of configurable parameters
            % Override in subclasses to specify parameters
            params = {};
        end
        
        function set_parameter(obj, param_name, value)
            % Set individual parameter
            % Override in subclasses for parameter-specific validation
            warning('AlgorithmInterface:NotImplemented', ...
                'Parameter setting not implemented for %s', class(obj));
        end
        
        function value = get_parameter(obj, param_name)
            % Get individual parameter value
            % Override in subclasses
            value = [];
            warning('AlgorithmInterface:NotImplemented', ...
                'Parameter getting not implemented for %s', class(obj));
        end
    end
    
    methods (Static)
        function interface_version = get_interface_version()
            % Interface version for compatibility checking
            interface_version = '1.0.0';
        end
        
        function requirements = get_service_requirements()
            % Define required services for all algorithms
            requirements = {
                'cost_calculator', ...
                'heuristic_calculator', ...
                'obstacle_checker', ...
                'motion_primitive_engine', ...
                'state_space'
            };
        end
    end
end
