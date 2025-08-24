classdef AStarAlgorithm < AlgorithmInterface
    % ASTARALGORITHM - A* algorithm implementation 
    % Refactored from original monolithic Pathplanning_Astar.m
    
    properties (Access = private)
        services            % Planning services (cost, heuristic, obstacles, etc.)
        max_iterations      % Maximum planning iterations
        visualization       % Visualization settings
        open_list          % Open list data structure
        closed_list        % Closed list data structure
        search_tree        % Search tree for path reconstruction
    end
    
    methods
        function obj = AStarAlgorithm(config, services)
            % Constructor
            obj@AlgorithmInterface(config, services);
            
            % Default configuration
            obj.max_iterations = 10000;
            obj.visualization = struct('enabled', false, 'frequency', 100);
            
            if nargin > 0 && ~isempty(config)
                obj.configure(config);
            end
            if nargin > 1 && ~isempty(services)
                obj.update_services(services);
            end
        end
        
        function [path, success, stats] = execute_planning(obj, planning_context)
            % Main A* planning execution
            
            % Extract context
            start_state = planning_context.start_state;
            goal_state = planning_context.goal_state;
            services = planning_context.services;
            
            % Update services if provided in context
            if ~isempty(services)
                obj.update_services(services);
            end
            
            % Validate inputs
            if ~obj.validate_inputs(start_state, goal_state)
                path = [];
                success = false;
                stats = struct('error', 'Invalid inputs');
                return;
            end
            
            % Initialize algorithm
            obj.initialize_search();
            
            % Initialize statistics
            stats = struct(...
                'iterations', 0, ...
                'nodes_explored', 0, ...
                'nodes_generated', 0, ...
                'start_time', tic);
            
            % Create start node
            start_node = StateNode(start_state);
            start_node.update_costs(0, obj.services.heuristic_calculator.calculate_h_cost(...
                start_state, goal_state, start_state));
            
            obj.open_list.push(start_node, start_node.f_cost);
            obj.search_tree.add_node(start_node);
            stats.nodes_generated = 1;
            
            % Main A* search loop
            while ~obj.open_list.is_empty() && stats.iterations < obj.max_iterations
                stats.iterations = stats.iterations + 1;
                
                % Get node with minimum f-cost
                current_node = obj.open_list.pop();
                obj.closed_list.add(current_node, obj.services.state_space);
                stats.nodes_explored = stats.nodes_explored + 1;
                
                % Check if goal reached
                if obj.is_goal_reached(current_node.state, goal_state)
                    [path, success] = obj.reconstruct_path(current_node);
                    stats.computation_time = toc(stats.start_time);
                    stats.success = success;
                    return;
                end
                
                % Generate and process successors
                [new_nodes, successor_stats] = obj.generate_successors(current_node, goal_state, start_state);
                stats.nodes_generated = stats.nodes_generated + successor_stats.nodes_generated;
                
                % Visualization
                if obj.visualization.enabled && mod(stats.iterations, obj.visualization.frequency) == 0
                    obj.visualize_search_progress(current_node, stats);
                end
            end
            
            % Planning failed
            path = [];
            success = false;
            stats.computation_time = toc(stats.start_time);
            stats.success = false;
            
            if stats.iterations >= obj.max_iterations
                stats.termination_reason = 'max_iterations_reached';
            else
                stats.termination_reason = 'open_list_exhausted';
            end
        end
        
        function info = get_algorithm_info(obj)
            % Get algorithm information
            info = struct(...
                'name', 'A* Search', ...
                'version', '2.0', ...
                'description', 'A* algorithm with motion primitives', ...
                'optimal', true, ...
                'complete', true, ...
                'time_complexity', 'O(b^d)', ...
                'space_complexity', 'O(b^d)', ...
                'supports_heuristic', true, ...
                'supports_dynamic_obstacles', false);
        end
        
        function update_services(obj, services)
            % Update planning services
            obj.services = services;
        end
        
        function configure(obj, config)
            % Configure algorithm parameters
            if isfield(config, 'max_iterations')
                obj.max_iterations = config.max_iterations;
            end
            if isfield(config, 'visualization')
                obj.visualization = config.visualization;
            end
        end
        
        function is_valid = validate_inputs(obj, start_state, goal_state)
            % Validate planning inputs
            is_valid = ~isempty(start_state) && ~isempty(goal_state) && ...
                      obj.services.state_space.is_state_valid(start_state) && ...
                      obj.services.state_space.is_state_valid(goal_state);
        end
        
        function cleanup(obj)
            % Clean up resources
            if ~isempty(obj.open_list)
                obj.open_list.clear();
            end
            if ~isempty(obj.closed_list)
                obj.closed_list.clear();
            end
            if ~isempty(obj.search_tree)
                obj.search_tree.clear();
            end
        end
        
        function is_supported = supports_feature(obj, feature_name)
            % Check supported features
            supported_features = {
                'heuristic_search', ...
                'optimal_path', ...
                'complete_search', ...
                'motion_primitives', ...
                'visualization'
            };
            is_supported = any(strcmp(feature_name, supported_features));
        end
        
        function params = get_configurable_parameters(obj)
            % Get configurable parameters
            params = {
                'max_iterations', ...
                'visualization.enabled', ...
                'visualization.frequency'
            };
        end
    end
    
    methods (Access = private)
        function initialize_search(obj)
            % Initialize search data structures
            obj.open_list = OpenList();
            obj.closed_list = ClosedList();
            obj.search_tree = SearchTree();
        end
        
        function [successor_nodes, stats] = generate_successors(obj, current_node, goal_state, start_state)
            % Generate successor nodes using motion primitives
            
            successor_nodes = [];
            stats = struct('nodes_generated', 0);
            
            % Get available motion primitives for current state
            motion_primitives = obj.services.motion_primitive_engine.get_primitives(...
                current_node.state.theta, current_node.state.gamma);
            
            for i = 1:length(motion_primitives)
                mp = motion_primitives(i);
                
                % Apply motion primitive
                successor_state = obj.apply_motion_primitive(current_node.state, mp);
                
                % Check state validity
                if ~obj.services.state_space.is_state_valid(successor_state)
                    continue;
                end
                
                % Check collision
                if ~obj.check_collision_free(current_node.state, successor_state, mp)
                    continue;
                end
                
                % Check if state already in closed list
                if obj.closed_list.contains_state(successor_state, obj.services.state_space)
                    continue;
                end
                
                % Create successor node
                successor_node = StateNode(successor_state);
                
                % Calculate costs
                g_cost = current_node.g_cost + obj.services.cost_calculator.calculate_g_cost(...
                    current_node, successor_state, mp);
                h_cost = obj.services.heuristic_calculator.calculate_h_cost(...
                    successor_state, goal_state, start_state);
                
                successor_node.update_costs(g_cost, h_cost);
                successor_node.set_parent(current_node, i, mp.trajectory);
                
                % Check if better path to existing node in open list
                if obj.open_list.contains(successor_node.id)
                    existing_node = obj.open_list.get_node(successor_node.id);
                    if successor_node.g_cost < existing_node.g_cost
                        obj.open_list.update_priority(successor_node.id, successor_node.f_cost);
                        existing_node.update_costs(g_cost, h_cost);
                        existing_node.set_parent(current_node, i, mp.trajectory);
                    end
                else
                    % Add new node to open list and search tree
                    obj.open_list.push(successor_node, successor_node.f_cost);
                    obj.search_tree.add_node(successor_node);
                    successor_nodes = [successor_nodes, successor_node];
                    stats.nodes_generated = stats.nodes_generated + 1;
                end
            end
        end
        
        function successor_state = apply_motion_primitive(obj, current_state, motion_primitive)
            % Apply motion primitive to get successor state
            successor_state = current_state;
            successor_state.x = current_state.x + motion_primitive.dx;
            successor_state.y = current_state.y + motion_primitive.dy;
            successor_state.theta = motion_primitive.final_theta;
            successor_state.gamma = motion_primitive.final_gamma;
        end
        
        function is_collision_free = check_collision_free(obj, current_state, successor_state, motion_primitive)
            % Check if motion from current to successor state is collision-free
            is_collision_free = obj.services.obstacle_checker.check_static_obstacles(...
                current_state, motion_primitive.trajectory, ...
                motion_primitive.theta_array, motion_primitive.gamma_array, ...
                motion_primitive.direction, [], [], []);
        end
        
        function is_goal = is_goal_reached(obj, current_state, goal_state)
            % Check if current state is close enough to goal
            position_tolerance = 2.0;  % meters
            orientation_tolerance = 5.0;  % degrees
            
            position_distance = sqrt((current_state.x - goal_state.x)^2 + ...
                                   (current_state.y - goal_state.y)^2);
            orientation_distance = abs(current_state.theta - goal_state.theta);
            if orientation_distance > 180
                orientation_distance = 360 - orientation_distance;
            end
            
            is_goal = position_distance <= position_tolerance && ...
                     orientation_distance <= orientation_tolerance;
        end
        
        function [path, success] = reconstruct_path(obj, goal_node)
            % Reconstruct path from goal to start
            path = [];
            success = false;
            
            if isempty(goal_node)
                return;
            end
            
            % Trace back from goal to start
            current = goal_node;
            while ~isempty(current)
                path = [current.state, path];
                if isempty(current.parent_id)
                    break;
                end
                current = obj.search_tree.get_node(current.parent_id);
            end
            
            success = ~isempty(path);
        end
        
        function visualize_search_progress(obj, current_node, stats)
            % Visualize search progress (optional)
            if ~obj.visualization.enabled
                return;
            end
            
            % Simple progress display
            fprintf('A* Iteration %d: Exploring (%.1f, %.1f), f=%.2f, Nodes: %d\n', ...
                stats.iterations, current_node.state.x, current_node.state.y, ...
                current_node.f_cost, stats.nodes_explored);
        end
    end
end
