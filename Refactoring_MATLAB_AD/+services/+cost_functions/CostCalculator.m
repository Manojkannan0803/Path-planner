classdef CostCalculator < handle
    % COSTCALCULATOR - G-cost calculation service
    % Refactored from original g_cost.m to services layer
    
    properties (Access = private)
        cost_weights    % Weights for different cost components
    end
    
    methods
        function obj = CostCalculator(config)
            % Constructor with configuration
            if nargin > 0 && isfield(config, 'cost_weights')
                obj.cost_weights = config.cost_weights;
            else
                % Default weights
                obj.cost_weights = struct(...
                    'distance', 1.0, ...
                    'direction_change', 0.1, ...
                    'reverse_penalty', 1.5, ...
                    'articulation_penalty', 0.05);
            end
        end
        
        function cost = calculate_g_cost(obj, current_node, successor_state, motion_primitive)
            % Calculate g-cost (cost so far) from current node to successor
            % This replaces the original g_cost.m functionality
            
            if nargin < 4
                % Fallback to simple distance calculation
                cost = obj.calculate_distance_cost(current_node.state, successor_state);
            else
                cost = obj.calculate_detailed_cost(current_node, successor_state, motion_primitive);
            end
        end
        
        function cost = calculate_distance_cost(obj, current_state, successor_state)
            % Simple Euclidean distance cost
            distance = sqrt((successor_state.x - current_state.x)^2 + ...
                          (successor_state.y - current_state.y)^2);
            cost = obj.cost_weights.distance * distance;
        end
        
        function cost = calculate_detailed_cost(obj, current_node, successor_state, motion_primitive)
            % Detailed cost calculation considering motion primitive
            
            % Base distance cost
            distance_cost = obj.calculate_distance_cost(current_node.state, successor_state);
            
            % Direction change penalty
            direction_change_cost = obj.calculate_direction_change_cost(...
                current_node.state, successor_state);
            
            % Reverse motion penalty
            reverse_cost = obj.calculate_reverse_penalty(motion_primitive);
            
            % Articulation angle change penalty
            articulation_cost = obj.calculate_articulation_penalty(...
                current_node.state, successor_state);
            
            % Time-based cost (if motion primitive has time information)
            time_cost = obj.calculate_time_cost(motion_primitive);
            
            % Total cost
            cost = distance_cost + direction_change_cost + reverse_cost + ...
                   articulation_cost + time_cost;
        end
        
        function cost = calculate_direction_change_cost(obj, current_state, successor_state)
            % Penalty for changing direction
            angle_diff = abs(successor_state.theta - current_state.theta);
            if angle_diff > 180
                angle_diff = 360 - angle_diff;
            end
            cost = obj.cost_weights.direction_change * angle_diff;
        end
        
        function cost = calculate_reverse_penalty(obj, motion_primitive)
            % Penalty for reverse motion
            if isfield(motion_primitive, 'direction') && motion_primitive.direction < 0
                cost = obj.cost_weights.reverse_penalty;
            else
                cost = 0;
            end
        end
        
        function cost = calculate_articulation_penalty(obj, current_state, successor_state)
            % Penalty for articulation angle changes
            gamma_diff = abs(successor_state.gamma - current_state.gamma);
            cost = obj.cost_weights.articulation_penalty * gamma_diff;
        end
        
        function cost = calculate_time_cost(obj, motion_primitive)
            % Time-based cost component
            if isfield(motion_primitive, 'time') && ~isempty(motion_primitive.time)
                cost = motion_primitive.time;
            else
                cost = 0;
            end
        end
        
        function cost = calculate_path_cost(obj, path, motion_primitives)
            % Calculate total cost for entire path
            if isempty(path) || length(path) < 2
                cost = 0;
                return;
            end
            
            total_cost = 0;
            for i = 1:(length(path) - 1)
                if nargin > 2 && i <= length(motion_primitives)
                    mp = motion_primitives(i);
                else
                    mp = [];
                end
                
                % Create dummy node for current state
                current_node = StateNode(path(i));
                segment_cost = obj.calculate_g_cost(current_node, path(i+1), mp);
                total_cost = total_cost + segment_cost;
            end
            
            cost = total_cost;
        end
        
        function update_weights(obj, new_weights)
            % Update cost weights
            fields = fieldnames(new_weights);
            for i = 1:length(fields)
                if isfield(obj.cost_weights, fields{i})
                    obj.cost_weights.(fields{i}) = new_weights.(fields{i});
                end
            end
        end
        
        function weights = get_weights(obj)
            % Get current cost weights
            weights = obj.cost_weights;
        end
    end
end
