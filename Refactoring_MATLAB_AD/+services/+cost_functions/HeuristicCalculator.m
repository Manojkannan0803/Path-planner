classdef HeuristicCalculator < handle
    % HEURISTICCALCULATOR - H-cost calculation service
    % Refactored from original h_cost.m to services layer
    
    properties (Access = private)
        heuristic_type  % Type of heuristic to use
        goal_zone       % Goal zone definition for enhanced heuristic
        admissible      % Whether to ensure admissible heuristic
    end
    
    methods
        function obj = HeuristicCalculator(config)
            % Constructor with configuration
            if nargin > 0
                obj.heuristic_type = config.heuristic_type;
                obj.goal_zone = config.goal_zone;
                obj.admissible = config.admissible;
            else
                % Default configuration
                obj.heuristic_type = 'euclidean_with_orientation';
                obj.goal_zone = [];
                obj.admissible = true;
            end
        end
        
        function cost = calculate_h_cost(obj, current_state, goal_state, initial_state, goal_zone)
            % Calculate h-cost (heuristic cost to goal)
            % This replaces the original h_cost.m functionality
            
            if nargin > 4 && ~isempty(goal_zone)
                obj.goal_zone = goal_zone;
            end
            
            switch obj.heuristic_type
                case 'euclidean'
                    cost = obj.euclidean_heuristic(current_state, goal_state);
                case 'manhattan'
                    cost = obj.manhattan_heuristic(current_state, goal_state);
                case 'euclidean_with_orientation'
                    cost = obj.euclidean_with_orientation(current_state, goal_state);
                case 'zone_based'
                    cost = obj.zone_based_heuristic(current_state, goal_state, initial_state);
                case 'adaptive'
                    cost = obj.adaptive_heuristic(current_state, goal_state, initial_state);
                otherwise
                    cost = obj.euclidean_heuristic(current_state, goal_state);
            end
            
            % Ensure admissibility if required
            if obj.admissible
                cost = obj.ensure_admissible(cost, current_state, goal_state);
            end
        end
        
        function cost = euclidean_heuristic(obj, current_state, goal_state)
            % Simple Euclidean distance heuristic
            cost = sqrt((goal_state.x - current_state.x)^2 + ...
                       (goal_state.y - current_state.y)^2);
        end
        
        function cost = manhattan_heuristic(obj, current_state, goal_state)
            % Manhattan distance heuristic
            cost = abs(goal_state.x - current_state.x) + ...
                   abs(goal_state.y - current_state.y);
        end
        
        function cost = euclidean_with_orientation(obj, current_state, goal_state)
            % Euclidean distance with orientation penalty
            position_cost = obj.euclidean_heuristic(current_state, goal_state);
            
            % Orientation difference
            theta_diff = abs(goal_state.theta - current_state.theta);
            if theta_diff > 180
                theta_diff = 360 - theta_diff;
            end
            
            % Articulation angle difference
            gamma_diff = abs(goal_state.gamma - current_state.gamma);
            
            % Combined cost
            orientation_cost = 0.1 * theta_diff + 0.05 * gamma_diff;
            cost = position_cost + orientation_cost;
        end
        
        function cost = zone_based_heuristic(obj, current_state, goal_state, initial_state)
            % Zone-based heuristic similar to original h_cost.m
            % This implements the logic from the original rectangular zones
            
            if isempty(obj.goal_zone)
                cost = obj.euclidean_with_orientation(current_state, goal_state);
                return;
            end
            
            % Check if current state is in goal zone
            in_goal_zone = obj.is_in_goal_zone(current_state);
            
            if in_goal_zone
                % If in goal zone, use simple distance to exact goal
                cost = obj.euclidean_heuristic(current_state, goal_state);
            else
                % Distance to goal zone boundary
                zone_distance = obj.distance_to_goal_zone(current_state);
                
                % Additional cost based on orientation and path complexity
                orientation_cost = obj.calculate_orientation_cost(current_state, goal_state);
                
                % Path complexity based on initial and current positions
                complexity_cost = obj.calculate_path_complexity(current_state, initial_state, goal_state);
                
                cost = zone_distance + orientation_cost + complexity_cost;
            end
        end
        
        function cost = adaptive_heuristic(obj, current_state, goal_state, initial_state)
            % Adaptive heuristic that changes based on distance to goal
            distance_to_goal = obj.euclidean_heuristic(current_state, goal_state);
            
            if distance_to_goal > 50  % Far from goal
                cost = obj.euclidean_heuristic(current_state, goal_state);
            elseif distance_to_goal > 20  % Medium distance
                cost = obj.euclidean_with_orientation(current_state, goal_state);
            else  % Close to goal
                cost = obj.zone_based_heuristic(current_state, goal_state, initial_state);
            end
        end
        
        function is_in_zone = is_in_goal_zone(obj, state)
            % Check if state is within goal zone
            if isempty(obj.goal_zone)
                is_in_zone = false;
                return;
            end
            
            % Use polygon check (similar to original InPolygon usage)
            is_in_zone = inpolygon(state.x, state.y, ...
                obj.goal_zone.x_vertices, obj.goal_zone.y_vertices);
        end
        
        function distance = distance_to_goal_zone(obj, state)
            % Calculate minimum distance to goal zone boundary
            if isempty(obj.goal_zone)
                distance = inf;
                return;
            end
            
            % Simple implementation: distance to zone center
            zone_center_x = mean(obj.goal_zone.x_vertices);
            zone_center_y = mean(obj.goal_zone.y_vertices);
            
            distance = sqrt((state.x - zone_center_x)^2 + (state.y - zone_center_y)^2);
            
            % Subtract approximate zone radius
            zone_radius = max(range(obj.goal_zone.x_vertices), range(obj.goal_zone.y_vertices)) / 2;
            distance = max(0, distance - zone_radius);
        end
        
        function cost = calculate_orientation_cost(obj, current_state, goal_state)
            % Calculate cost based on orientation difference
            theta_diff = abs(goal_state.theta - current_state.theta);
            if theta_diff > 180
                theta_diff = 360 - theta_diff;
            end
            cost = 0.1 * theta_diff;
        end
        
        function cost = calculate_path_complexity(obj, current_state, initial_state, goal_state)
            % Calculate additional cost based on path complexity
            % This is a simplified version of the original heuristic logic
            
            % Distance from initial to goal
            total_distance = sqrt((goal_state.x - initial_state.x)^2 + ...
                                (goal_state.y - initial_state.y)^2);
            
            % Distance covered so far
            covered_distance = sqrt((current_state.x - initial_state.x)^2 + ...
                                  (current_state.y - initial_state.y)^2);
            
            % Remaining distance ratio
            if total_distance > 0
                progress_ratio = covered_distance / total_distance;
                cost = (1 - progress_ratio) * 10;  % Penalty decreases as we get closer
            else
                cost = 0;
            end
        end
        
        function cost = ensure_admissible(obj, calculated_cost, current_state, goal_state)
            % Ensure heuristic is admissible (never overestimates)
            min_possible_cost = obj.euclidean_heuristic(current_state, goal_state);
            cost = min(calculated_cost, min_possible_cost);
        end
        
        function set_goal_zone(obj, goal_zone)
            % Set goal zone for zone-based heuristics
            obj.goal_zone = goal_zone;
        end
        
        function set_heuristic_type(obj, heuristic_type)
            % Change heuristic type
            obj.heuristic_type = heuristic_type;
        end
    end
end
