classdef PathFollowingControllerSystemObject < matlab.System
    % PATHFOLLOWINGCONTROLLERSYSTEMOBJECT - Path following controller for Simulink
    % Implements path tracking controller for tractor-trailer vehicle
    
    properties (Nontunable)
        LookaheadDistance = 5.0    % Lookahead distance for path following (m)
        MaxVelocity = 5.0          % Maximum velocity (m/s)
        MaxSteeringAngle = 30      % Maximum steering angle (degrees)
        PositionTolerance = 1.0    % Position tolerance for goal reaching (m)
        VelocityGain = 1.0         % Velocity control gain
        SteeringGain = 2.0         % Steering control gain
        PathUpdateRate = 0.5       % Path update rate (s)
    end
    
    properties (Access = private)
        current_path              % Current path to follow
        path_index               % Current path point index
        last_path_update        % Last path update time
        controller_state        % Controller internal state
        is_initialized         % Initialization flag
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            % Initialize path following controller
            obj.current_path = [];
            obj.path_index = 1;
            obj.last_path_update = 0;
            obj.controller_state = struct(...
                'at_goal', false, ...
                'path_length', 0, ...
                'cross_track_error', 0, ...
                'heading_error', 0);
            obj.is_initialized = true;
            
            fprintf('[Simulink] Path Following Controller initialized\n');
        end
        
        function [velocity_cmd, steering_cmd, at_goal, path_progress] = stepImpl(obj, ...
                current_x, current_y, current_theta, ~, ...
                path_x, path_y, path_theta, path_gamma, ...
                path_valid, sim_time)
            % Main path following control step
            
            % Initialize outputs
            velocity_cmd = 0;
            steering_cmd = 0;
            at_goal = false;
            path_progress = 0;
            
            if ~obj.is_initialized
                return;
            end
            
            % Update path if new path is available
            if path_valid > 0.5 && (sim_time - obj.last_path_update) > obj.PathUpdateRate
                obj.update_path(path_x, path_y, path_theta, path_gamma);
                obj.last_path_update = sim_time;
            end
            
            % Check if path is available
            if isempty(obj.current_path) || obj.current_path.length == 0
                return;
            end
            
            % Current vehicle position
            current_pos = [current_x, current_y];
            
            % Find closest point on path
            [closest_idx, cross_track_error] = obj.find_closest_path_point(current_pos);
            obj.path_index = max(obj.path_index, closest_idx);
            
            % Check if goal reached
            goal_pos = [obj.current_path.x(end), obj.current_path.y(end)];
            distance_to_goal = norm(current_pos - goal_pos);
            at_goal = distance_to_goal < obj.PositionTolerance;
            obj.controller_state.at_goal = at_goal;
            
            if at_goal
                velocity_cmd = 0;
                steering_cmd = 0;
                path_progress = 1.0;
                return;
            end
            
            % Find lookahead point
            lookahead_idx = obj.find_lookahead_point(current_pos, obj.path_index);
            
            % Calculate control commands
            if lookahead_idx <= obj.current_path.length
                target_pos = [obj.current_path.x(lookahead_idx), obj.current_path.y(lookahead_idx)];
                target_heading = obj.current_path.theta(lookahead_idx);
                
                % Pure pursuit controller for steering
                steering_cmd = obj.calculate_steering_command(current_pos, current_theta, target_pos);
                
                % Velocity control based on path curvature and distance to goal
                velocity_cmd = obj.calculate_velocity_command(distance_to_goal, cross_track_error);
                
                % Update controller state
                obj.controller_state.cross_track_error = cross_track_error;
                obj.controller_state.heading_error = target_heading - current_theta;
            end
            
            % Calculate path progress
            path_progress = min(1.0, obj.path_index / obj.current_path.length);
            
            % Saturate commands
            velocity_cmd = max(0, min(obj.MaxVelocity, velocity_cmd));
            steering_cmd = max(-obj.MaxSteeringAngle, min(obj.MaxSteeringAngle, steering_cmd));
        end
        
        function update_path(obj, path_x, path_y, path_theta, path_gamma)
            % Update current path
            
            % Extract valid path points
            valid_indices = path_x ~= 0 | path_y ~= 0;
            
            if any(valid_indices)
                obj.current_path = struct(...
                    'x', path_x(valid_indices), ...
                    'y', path_y(valid_indices), ...
                    'theta', path_theta(valid_indices), ...
                    'gamma', path_gamma(valid_indices), ...
                    'length', sum(valid_indices));
                
                obj.path_index = 1;  % Reset path following
                obj.controller_state.path_length = obj.current_path.length;
                
                fprintf('[Simulink] Path updated: %d points\n', obj.current_path.length);
            end
        end
        
        function [closest_idx, cross_track_error] = find_closest_path_point(obj, current_pos)
            % Find closest point on path
            
            if isempty(obj.current_path)
                closest_idx = 1;
                cross_track_error = 0;
                return;
            end
            
            % Calculate distances to all path points
            path_points = [obj.current_path.x, obj.current_path.y];
            distances = sqrt(sum((path_points - repmat(current_pos, size(path_points, 1), 1)).^2, 2));
            
            [cross_track_error, closest_idx] = min(distances);
        end
        
        function lookahead_idx = find_lookahead_point(obj, current_pos, start_idx)
            % Find lookahead point on path
            
            lookahead_idx = start_idx;
            
            if isempty(obj.current_path)
                return;
            end
            
            % Search for point at lookahead distance
            for i = start_idx:obj.current_path.length
                path_point = [obj.current_path.x(i), obj.current_path.y(i)];
                distance = norm(path_point - current_pos);
                
                if distance >= obj.LookaheadDistance
                    lookahead_idx = i;
                    break;
                end
                lookahead_idx = i;
            end
            
            % Ensure we don't go beyond path end
            lookahead_idx = min(lookahead_idx, obj.current_path.length);
        end
        
        function steering_cmd = calculate_steering_command(obj, current_pos, current_heading, target_pos)
            % Calculate steering command using pure pursuit
            
            % Vector to target
            target_vector = target_pos - current_pos;
            distance_to_target = norm(target_vector);
            
            if distance_to_target < 0.1
                steering_cmd = 0;
                return;
            end
            
            % Angle to target
            target_angle = atan2(target_vector(2), target_vector(1));
            heading_error = target_angle - deg2rad(current_heading);
            
            % Wrap angle
            heading_error = atan2(sin(heading_error), cos(heading_error));
            
            % Pure pursuit steering command
            curvature = 2 * sin(heading_error) / distance_to_target;
            steering_cmd = rad2deg(atan(curvature * obj.LookaheadDistance)) * obj.SteeringGain;
        end
        
        function velocity_cmd = calculate_velocity_command(obj, distance_to_goal, cross_track_error)
            % Calculate velocity command
            
            % Base velocity
            velocity_cmd = obj.MaxVelocity * obj.VelocityGain;
            
            % Reduce velocity near goal
            if distance_to_goal < 10.0
                velocity_cmd = velocity_cmd * (distance_to_goal / 10.0);
            end
            
            % Reduce velocity for large cross-track errors
            if abs(cross_track_error) > 1.0
                velocity_cmd = velocity_cmd * 0.5;
            end
            
            % Minimum velocity
            velocity_cmd = max(0.1, velocity_cmd);
        end
        
        function resetImpl(obj)
            % Reset controller
            obj.current_path = [];
            obj.path_index = 1;
            obj.last_path_update = 0;
            obj.controller_state.at_goal = false;
            fprintf('[Simulink] Path Following Controller reset\n');
        end
        
        function releaseImpl(obj)
            % Release resources
            obj.is_initialized = false;
            fprintf('[Simulink] Path Following Controller released\n');
        end
    end
    
    methods (Access = protected, Static)
        function header = getHeaderImpl()
            header = matlab.system.display.Header('PathFollowingControllerSystemObject', ...
                'Title', 'Path Following Controller', ...
                'Text', ['Pure pursuit path following controller for tractor-trailer vehicles. ' ...
                        'Generates velocity and steering commands to follow planned paths.']);
        end
        
        function groups = getPropertyGroupsImpl()
            % Define property groups
            control_group = matlab.system.display.Section(...
                'Title', 'Control Parameters', ...
                'PropertyList', {'LookaheadDistance', 'VelocityGain', 'SteeringGain'});
            
            limits_group = matlab.system.display.Section(...
                'Title', 'Control Limits', ...
                'PropertyList', {'MaxVelocity', 'MaxSteeringAngle', 'PositionTolerance'});
            
            timing_group = matlab.system.display.Section(...
                'Title', 'Timing', ...
                'PropertyList', {'PathUpdateRate'});
            
            groups = [control_group, limits_group, timing_group];
        end
        
        function simMode = getSimulateUsingImpl()
            simMode = 'Interpreted execution';
        end
    end
    
    methods (Access = public)
        function state = get_controller_state(obj)
            % Get controller state
            state = obj.controller_state;
        end
        
        function path = get_current_path(obj)
            % Get current path
            path = obj.current_path;
        end
    end
end
