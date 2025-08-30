classdef PlanningEngine < handle
    % PLANNINGENGINE - Main orchestrator that accepts algorithm type as input
    % This is the core of Layer 4 - modular algorithm selection
    
    properties (Access = private)
        algorithm_factory   % Factory for creating algorithm instances
        current_algorithm   % Currently active algorithm
        cost_calculator     % Cost calculation service
        heuristic_calculator % Heuristic calculation service
        obstacle_checker    % Obstacle checking service
        motion_primitive_engine % Motion primitive service
        state_space         % State space definition
        config              % Configuration object
    end
    
    methods
        function obj = PlanningEngine(config)
            % Constructor - Initialize with configuration
            obj.config = config;
            
            % Initialize core services (Layer 3)
            obj.cost_calculator = CostCalculator(config.planning.cost_function);
            obj.heuristic_calculator = HeuristicCalculator(config.planning.heuristic);
            obj.obstacle_checker = ObstacleChecker(config.planning.obstacle_checker);
            obj.motion_primitive_engine = MotionPrimitiveEngine(config.motion_primitives);
            
            % Initialize framework components (Layer 2)
            obj.state_space = StateSpace(config);
            
            % Initialize algorithm factory and set initial algorithm
            obj.algorithm_factory = AlgorithmFactory();
            obj.set_algorithm(config.planning.algorithm.name, config.planning.algorithm);
        end
        
        function [path, success, stats] = plan_path(obj, start_state, goal_state, scenario_data)
            % Main path planning execution - algorithm agnostic
            
            % Validate inputs
            if ~obj.state_space.is_state_valid(start_state)
                error('PlanningEngine:InvalidStart', 'Invalid start state');
            end
            if ~obj.state_space.is_state_valid(goal_state)
                error('PlanningEngine:InvalidGoal', 'Invalid goal state');
            end
            
            % Load scenario obstacles if provided
            if nargin > 3 && ~isempty(scenario_data)
                obj.obstacle_checker.load_obstacles(scenario_data);
            end
            
            % Create planning context
            planning_context = obj.create_planning_context(start_state, goal_state);
            
            % Execute planning using current algorithm
            [path, success, stats] = obj.current_algorithm.execute_planning(planning_context);
            
            % Post-process results
            if success
                path = obj.post_process_path(path);
                stats = obj.enhance_statistics(stats, start_state, goal_state);
            end
        end
        
        function set_algorithm(obj, algorithm_name, algorithm_config)
            % Set/change algorithm dynamically - KEY FEATURE
            obj.current_algorithm = obj.algorithm_factory.create_algorithm(...
                algorithm_name, algorithm_config, obj.get_services());
        end
        
        function algorithm_info = get_current_algorithm_info(obj)
            % Get information about current algorithm
            if ~isempty(obj.current_algorithm)
                algorithm_info = obj.current_algorithm.get_algorithm_info();
            else
                algorithm_info = struct('name', 'none', 'status', 'not_set');
            end
        end
        
        function available_algorithms = get_available_algorithms(obj)
            % Get list of available algorithms
            available_algorithms = obj.algorithm_factory.get_available_algorithms();
        end
        
        function set_cost_function(obj, cost_config)
            % Update cost function configuration
            obj.cost_calculator = CostCalculator(cost_config);
            if ~isempty(obj.current_algorithm)
                obj.current_algorithm.update_services(obj.get_services());
            end
        end
        
        function set_heuristic_function(obj, heuristic_config)
            % Update heuristic function configuration
            obj.heuristic_calculator = HeuristicCalculator(heuristic_config);
            if ~isempty(obj.current_algorithm)
                obj.current_algorithm.update_services(obj.get_services());
            end
        end
        
        function compare_algorithms(obj, algorithm_list, start_state, goal_state, scenario_data)
            % Compare multiple algorithms on same problem
            results = struct();
            
            for i = 1:length(algorithm_list)
                alg_name = algorithm_list{i};
                
                % Set algorithm
                obj.set_algorithm(alg_name, obj.config.planning.algorithm);
                
                % Plan path
                tic;
                [path, success, stats] = obj.plan_path(start_state, goal_state, scenario_data);
                computation_time = toc;
                
                % Store results
                results.(alg_name) = struct(...
                    'path', path, ...
                    'success', success, ...
                    'stats', stats, ...
                    'computation_time', computation_time);
                
                fprintf('Algorithm %s: Success=%d, Time=%.3fs\n', ...
                    alg_name, success, computation_time);
            end
        end
        
        function benchmark_performance(obj, test_cases)
            % Benchmark current algorithm on multiple test cases
            performance_data = [];
            
            for i = 1:length(test_cases)
                test_case = test_cases(i);
                
                tic;
                [path, success, stats] = obj.plan_path(...
                    test_case.start_state, test_case.goal_state, test_case.scenario_data);
                computation_time = toc;
                
                performance_data = [performance_data; struct(...
                    'test_id', i, ...
                    'success', success, ...
                    'computation_time', computation_time, ...
                    'path_length', length(path), ...
                    'nodes_explored', stats.nodes_explored)];
            end
            
            % Display summary
            obj.display_benchmark_results(performance_data);
        end
    end
    
    methods (Access = private)
        function services = get_services(obj)
            % Package all services for algorithm use
            services = struct(...
                'cost_calculator', obj.cost_calculator, ...
                'heuristic_calculator', obj.heuristic_calculator, ...
                'obstacle_checker', obj.obstacle_checker, ...
                'motion_primitive_engine', obj.motion_primitive_engine, ...
                'state_space', obj.state_space);
        end
        
        function context = create_planning_context(obj, start_state, goal_state)
            % Create comprehensive planning context
            context = struct(...
                'start_state', start_state, ...
                'goal_state', goal_state, ...
                'services', obj.get_services(), ...
                'config', obj.config);
        end
        
        function processed_path = post_process_path(obj, raw_path)
            % Post-process path (smoothing, validation, etc.)
            processed_path = raw_path;
            
            % Add any post-processing logic here
            % - Path smoothing
            % - Feasibility validation
            % - Trajectory generation
        end
        
        function enhanced_stats = enhance_statistics(obj, basic_stats, start_state, goal_state)
            % Enhance statistics with additional information
            enhanced_stats = basic_stats;
            enhanced_stats.algorithm_name = obj.current_algorithm.get_algorithm_info().name;
            enhanced_stats.start_state = start_state;
            enhanced_stats.goal_state = goal_state;
            enhanced_stats.timestamp = datetime('now');
        end
        
        function display_benchmark_results(obj, performance_data)
            % Display benchmark results
            fprintf('\n=== BENCHMARK RESULTS ===\n');
            fprintf('Algorithm: %s\n', obj.get_current_algorithm_info().name);
            fprintf('Total test cases: %d\n', length(performance_data));
            
            success_rate = sum([performance_data.success]) / length(performance_data) * 100;
            avg_time = mean([performance_data.computation_time]);
            avg_nodes = mean([performance_data.nodes_explored]);
            
            fprintf('Success rate: %.1f%%\n', success_rate);
            fprintf('Average computation time: %.3f seconds\n', avg_time);
            fprintf('Average nodes explored: %.0f\n', avg_nodes);
            fprintf('========================\n\n');
        end
    end
end
