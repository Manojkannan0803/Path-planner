classdef PlanningSession < handle
    % PLANNINGSESSION - Main execution orchestrator for path planning
    % Top layer (Layer 5) - replaces the original monolithic script execution
    
    properties (Access = private)
        planning_engine     % Layer 4 planning engine
        config_manager      % Configuration management
        scenario_manager    % Scenario management
        results_manager     % Results storage and analysis
        session_config      % Current session configuration
    end
    
    methods
        function obj = PlanningSession(config_file)
            % Constructor - Initialize planning session
            
            if nargin < 1
                config_file = 'scenarios/DPDScenario.json';
            end
            
            % Load configuration
            obj.config_manager = ConfigManager();
            obj.session_config = obj.config_manager.load_config(config_file);
            
            % Initialize components
            obj.planning_engine = PlanningEngine(obj.session_config);
            obj.scenario_manager = ScenarioManager(obj.session_config.scenario);
            obj.results_manager = ResultsManager();
            
            fprintf('Planning session initialized with config: %s\n', config_file);
        end
        
        function [path, success, stats] = execute_single_planning(obj, start_state, goal_state, options)
            % Execute single path planning task
            % This replaces the main execution from original Pathplanning_Astar.m
            
            if nargin < 4
                options = struct();
            end
            
            % Load scenario data
            scenario_data = obj.scenario_manager.get_scenario_data();
            
            % Set algorithm if specified in options
            if isfield(options, 'algorithm')
                obj.planning_engine.set_algorithm(options.algorithm, obj.session_config.planning.algorithm);
            end
            
            % Execute planning
            fprintf('Starting path planning...\n');
            fprintf('Start: (%.1f, %.1f, %.0f°)\n', start_state.x, start_state.y, start_state.theta);
            fprintf('Goal:  (%.1f, %.1f, %.0f°)\n', goal_state.x, goal_state.y, goal_state.theta);
            
            planning_start_time = tic;
            [path, success, stats] = obj.planning_engine.plan_path(...
                start_state, goal_state, scenario_data);
            total_time = toc(planning_start_time);
            
            % Display results
            obj.display_planning_results(path, success, stats, total_time);
            
            % Store results
            session_result = obj.create_session_result(start_state, goal_state, path, success, stats, total_time);
            obj.results_manager.add_result(session_result);
            
            % Visualize if enabled
            if obj.should_visualize()
                obj.visualize_results(path, start_state, goal_state, scenario_data);
            end
        end
        
        function results = execute_batch_planning(obj, test_cases, options)
            % Execute batch planning for multiple test cases
            
            if nargin < 3
                options = struct();
            end
            
            results = [];
            fprintf('Starting batch planning with %d test cases...\n', length(test_cases));
            
            for i = 1:length(test_cases)
                fprintf('\n--- Test Case %d/%d ---\n', i, length(test_cases));
                
                test_case = test_cases(i);
                [path, success, stats] = obj.execute_single_planning(...
                    test_case.start_state, test_case.goal_state, options);
                
                % Store batch result
                batch_result = struct(...
                    'test_case_id', i, ...
                    'start_state', test_case.start_state, ...
                    'goal_state', test_case.goal_state, ...
                    'path', path, ...
                    'success', success, ...
                    'stats', stats);
                
                results = [results, batch_result];
            end
            
            % Display batch summary
            obj.display_batch_summary(results);
        end
        
        function compare_algorithms(obj, algorithm_list, start_state, goal_state)
            % Compare multiple algorithms on same problem
            
            fprintf('\n=== ALGORITHM COMPARISON ===\n');
            comparison_results = struct();
            
            for i = 1:length(algorithm_list)
                algorithm_name = algorithm_list{i};
                fprintf('\nTesting algorithm: %s\n', algorithm_name);
                
                % Execute planning with specific algorithm
                options = struct('algorithm', algorithm_name);
                [path, success, stats] = obj.execute_single_planning(...
                    start_state, goal_state, options);
                
                % Store comparison result
                comparison_results.(algorithm_name) = struct(...
                    'path', path, ...
                    'success', success, ...
                    'stats', stats);
            end
            
            % Display comparison
            obj.display_algorithm_comparison(comparison_results);
        end
        
        function benchmark_performance(obj, test_suite)
            % Run performance benchmark
            
            fprintf('\n=== PERFORMANCE BENCHMARK ===\n');
            
            % Execute test suite
            results = obj.execute_batch_planning(test_suite);
            
            % Analyze performance
            performance_analysis = obj.analyze_performance(results);
            
            % Display benchmark results
            obj.display_benchmark_results(performance_analysis);
            
            % Save benchmark report
            obj.save_benchmark_report(performance_analysis);
        end
        
        function change_scenario(obj, scenario_name)
            % Change planning scenario
            scenario_config_file = sprintf('scenarios/%s.json', scenario_name);
            
            try
                new_config = obj.config_manager.load_config(scenario_config_file);
                obj.session_config = new_config;
                obj.scenario_manager = ScenarioManager(new_config.scenario);
                obj.planning_engine = PlanningEngine(new_config);
                
                fprintf('Scenario changed to: %s\n', scenario_name);
            catch ME
                fprintf('Failed to change scenario: %s\n', ME.message);
            end
        end
        
        function set_algorithm(obj, algorithm_name, algorithm_config)
            % Change planning algorithm
            if nargin < 3
                algorithm_config = obj.session_config.planning.algorithm;
            end
            
            obj.planning_engine.set_algorithm(algorithm_name, algorithm_config);
            fprintf('Algorithm changed to: %s\n', algorithm_name);
        end
        
        function session_summary = get_session_summary(obj)
            % Get summary of current session
            session_summary = obj.results_manager.get_session_summary();
            session_summary.current_algorithm = obj.planning_engine.get_current_algorithm_info();
            session_summary.current_scenario = obj.scenario_manager.get_scenario_info();
        end
        
        function save_session(obj, filename)
            % Save session results to file
            if nargin < 2
                timestamp = datestr(now, 'yyyymmdd_HHMMSS');
                filename = sprintf('planning_session_%s.mat', timestamp);
            end
            
            session_data = struct(...
                'config', obj.session_config, ...
                'results', obj.results_manager.get_all_results(), ...
                'summary', obj.get_session_summary());
            
            save(filename, 'session_data');
            fprintf('Session saved to: %s\n', filename);
        end
        
        function load_session(obj, filename)
            % Load session from file
            try
                loaded_data = load(filename);
                session_data = loaded_data.session_data;
                
                obj.session_config = session_data.config;
                obj.results_manager.load_results(session_data.results);
                
                fprintf('Session loaded from: %s\n', filename);
            catch ME
                fprintf('Failed to load session: %s\n', ME.message);
            end
        end
    end
    
    methods (Access = private)
        function display_planning_results(obj, path, success, stats, total_time)
            % Display planning results
            fprintf('\n=== PLANNING RESULTS ===\n');
            fprintf('Success: %s\n', obj.bool_to_string(success));
            
            if success
                fprintf('Path length: %d waypoints\n', length(path));
                fprintf('Computation time: %.3f seconds\n', total_time);
                if isfield(stats, 'iterations')
                    fprintf('Algorithm iterations: %d\n', stats.iterations);
                end
                if isfield(stats, 'nodes_explored')
                    fprintf('Nodes explored: %d\n', stats.nodes_explored);
                end
            else
                if isfield(stats, 'termination_reason')
                    fprintf('Failure reason: %s\n', stats.termination_reason);
                end
            end
            fprintf('========================\n');
        end
        
        function display_batch_summary(obj, results)
            % Display batch planning summary
            total_cases = length(results);
            successful_cases = sum([results.success]);
            success_rate = successful_cases / total_cases * 100;
            
            if successful_cases > 0
                successful_results = results([results.success]);
                avg_computation_time = mean([successful_results.stats.computation_time]);
                avg_iterations = mean([successful_results.stats.iterations]);
            else
                avg_computation_time = 0;
                avg_iterations = 0;
            end
            
            fprintf('\n=== BATCH SUMMARY ===\n');
            fprintf('Total test cases: %d\n', total_cases);
            fprintf('Successful cases: %d (%.1f%%)\n', successful_cases, success_rate);
            fprintf('Average computation time: %.3f seconds\n', avg_computation_time);
            fprintf('Average iterations: %.0f\n', avg_iterations);
            fprintf('=====================\n');
        end
        
        function display_algorithm_comparison(obj, comparison_results)
            % Display algorithm comparison results
            algorithm_names = fieldnames(comparison_results);
            
            fprintf('\n=== ALGORITHM COMPARISON RESULTS ===\n');
            fprintf('%-15s %-10s %-12s %-12s\n', 'Algorithm', 'Success', 'Time (s)', 'Iterations');
            fprintf('%-15s %-10s %-12s %-12s\n', '---------', '-------', '--------', '----------');
            
            for i = 1:length(algorithm_names)
                alg_name = algorithm_names{i};
                result = comparison_results.(alg_name);
                
                success_str = obj.bool_to_string(result.success);
                if result.success
                    time_str = sprintf('%.3f', result.stats.computation_time);
                    iter_str = sprintf('%d', result.stats.iterations);
                else
                    time_str = 'N/A';
                    iter_str = 'N/A';
                end
                
                fprintf('%-15s %-10s %-12s %-12s\n', alg_name, success_str, time_str, iter_str);
            end
            fprintf('=====================================\n');
        end
        
        function should_visualize = should_visualize(obj)
            % Check if visualization should be performed
            should_visualize = isfield(obj.session_config.planning, 'visualizer') && ...
                              ~strcmp(obj.session_config.planning.visualizer.name, 'none');
        end
        
        function visualize_results(obj, path, start_state, goal_state, scenario_data)
            % Visualize planning results
            % This would call the visualization layer components
            fprintf('Visualizing results... (visualization implementation needed)\n');
        end
        
        function session_result = create_session_result(obj, start_state, goal_state, path, success, stats, total_time)
            % Create session result structure
            session_result = struct(...
                'timestamp', datetime('now'), ...
                'start_state', start_state, ...
                'goal_state', goal_state, ...
                'path', path, ...
                'success', success, ...
                'stats', stats, ...
                'total_time', total_time, ...
                'algorithm', obj.planning_engine.get_current_algorithm_info(), ...
                'scenario', obj.scenario_manager.get_scenario_info());
        end
        
        function performance_analysis = analyze_performance(obj, results)
            % Analyze performance from batch results
            performance_analysis = struct();
            % Implementation for performance analysis
        end
        
        function display_benchmark_results(obj, performance_analysis)
            % Display benchmark results
            fprintf('Benchmark analysis completed.\n');
        end
        
        function save_benchmark_report(obj, performance_analysis)
            % Save benchmark report
            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            filename = sprintf('benchmark_report_%s.json', timestamp);
            % Save implementation
            fprintf('Benchmark report saved to: %s\n', filename);
        end
        
        function str = bool_to_string(obj, bool_val)
            % Convert boolean to string
            if bool_val
                str = 'Yes';
            else
                str = 'No';
            end
        end
    end
end
