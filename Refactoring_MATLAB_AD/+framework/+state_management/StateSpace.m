classdef StateSpace < handle
    % STATESP ACE - State space definition and management
    % Framework layer class for defining and managing the planning state space
    
    properties (Access = private)
        x_bounds
        y_bounds
        theta_discretization
        gamma_discretization
        scale
    end
    
    methods
        function obj = StateSpace(config)
            % Constructor - Initialize state space with configuration
            obj.x_bounds = [0, config.environment.length_dc];
            obj.y_bounds = [0, config.environment.width_dc];
            obj.scale = config.environment.scale;
            
            obj.theta_discretization = config.motion_primitives.discretization.theta_intervals;
            obj.gamma_discretization = config.motion_primitives.discretization.gamma_intervals;
        end
        
        function is_valid = is_state_valid(obj, state)
            % Check if state is within valid bounds
            is_valid = obj.is_position_valid(state.x, state.y) && ...
                      obj.is_orientation_valid(state.theta) && ...
                      obj.is_articulation_valid(state.gamma);
        end
        
        function is_valid = is_position_valid(obj, x, y)
            % Check if position is within bounds
            is_valid = x >= obj.x_bounds(1) && x <= obj.x_bounds(2) && ...
                      y >= obj.y_bounds(1) && y <= obj.y_bounds(2);
        end
        
        function is_valid = is_orientation_valid(obj, theta)
            % Check if orientation is valid (should be in discretized set)
            is_valid = any(abs(obj.theta_discretization - theta) < 1e-6);
        end
        
        function is_valid = is_articulation_valid(obj, gamma)
            % Check if articulation angle is valid
            is_valid = any(abs(obj.gamma_discretization - gamma) < 1e-6);
        end
        
        function discrete_state = discretize_state(obj, continuous_state)
            % Discretize continuous state to valid discrete state
            discrete_state = continuous_state;
            
            % Discretize position
            discrete_state.x = round(continuous_state.x / obj.scale) * obj.scale;
            discrete_state.y = round(continuous_state.y / obj.scale) * obj.scale;
            
            % Snap to nearest valid orientation
            [~, theta_idx] = min(abs(obj.theta_discretization - continuous_state.theta));
            discrete_state.theta = obj.theta_discretization(theta_idx);
            
            % Snap to nearest valid articulation angle
            [~, gamma_idx] = min(abs(obj.gamma_discretization - continuous_state.gamma));
            discrete_state.gamma = obj.gamma_discretization(gamma_idx);
        end
        
        function key = get_state_key(obj, state)
            % Generate unique key for state (for hashing)
            discrete_state = obj.discretize_state(state);
            key = sprintf('%.1f_%.1f_%.0f_%.0f', ...
                discrete_state.x, discrete_state.y, ...
                discrete_state.theta, discrete_state.gamma);
        end
        
        function bounds = get_bounds(obj)
            % Get state space bounds
            bounds = struct(...
                'x_min', obj.x_bounds(1), 'x_max', obj.x_bounds(2), ...
                'y_min', obj.y_bounds(1), 'y_max', obj.y_bounds(2), ...
                'theta_values', obj.theta_discretization, ...
                'gamma_values', obj.gamma_discretization);
        end
        
        function distance = state_distance(obj, state1, state2)
            % Calculate distance between two states
            pos_dist = sqrt((state1.x - state2.x)^2 + (state1.y - state2.y)^2);
            angle_dist = min(abs(state1.theta - state2.theta), ...
                           360 - abs(state1.theta - state2.theta));
            
            % Weighted combination of position and orientation distance
            distance = pos_dist + 0.1 * angle_dist;
        end
    end
end
