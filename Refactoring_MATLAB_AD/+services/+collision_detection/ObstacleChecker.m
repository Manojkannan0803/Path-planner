classdef ObstacleChecker < handle
    % OBSTACLECHECKER - Collision detection service
    % Refactored from original staticobs_check.m, virtualobs_check.m to services layer
    
    properties (Access = private)
        static_obstacles    % Static obstacle data
        collision_buffer    % Safety buffer around obstacles
        vehicle_model       % Vehicle dimensions and model
    end
    
    methods
        function obj = ObstacleChecker(config)
            % Constructor with configuration
            obj.collision_buffer = 0.5;  % Default buffer
            obj.static_obstacles = [];
            obj.vehicle_model = [];
            
            if nargin > 0
                if isfield(config, 'collision_buffer')
                    obj.collision_buffer = config.collision_buffer;
                end
                if isfield(config, 'vehicle_model')
                    obj.vehicle_model = config.vehicle_model;
                end
            end
        end
        
        function load_obstacles(obj, obstacle_data)
            % Load obstacle data (replaces loading from DPDscenario.m)
            obj.static_obstacles = obstacle_data;
        end
        
        function is_safe = check_static_obstacles(obj, current_state, trajectory, theta_array, gamma_array, direction, obsc, obsx, obsy)
            % Check collision with static obstacles
            % Refactored from original staticobs_check.m
            
            is_safe = true;
            
            if isempty(obsc) || isempty(trajectory)
                return;
            end
            
            % Check each point along trajectory
            for i = 1:length(trajectory.x)
                % Get vehicle pose at this trajectory point
                vehicle_x = current_state.x + trajectory.x(i);
                vehicle_y = current_state.y + trajectory.y(i);
                vehicle_theta = theta_array(i);
                vehicle_gamma = gamma_array(i);
                
                % Generate vehicle footprint at this pose
                vehicle_footprint = obj.generate_vehicle_footprint(...
                    vehicle_x, vehicle_y, vehicle_theta, vehicle_gamma);
                
                % Check collision with each obstacle
                for obs_idx = 1:length(obsc)
                    if obj.check_vehicle_obstacle_collision(vehicle_footprint, obsx{obs_idx}, obsy{obs_idx})
                        is_safe = false;
                        return;
                    end
                end
            end
        end
        
        function is_safe = check_virtual_obstacles(obj, current_state, obsc, obsx, obsy)
            % Check collision with virtual obstacles
            % Refactored from original virtualobs_check.m
            
            is_safe = true;
            
            if isempty(obsc)
                return;
            end
            
            % Generate current vehicle footprint
            vehicle_footprint = obj.generate_vehicle_footprint(...
                current_state.x, current_state.y, current_state.theta, current_state.gamma);
            
            % Check collision with virtual obstacles
            for obs_idx = 1:length(obsc)
                if obj.check_vehicle_obstacle_collision(vehicle_footprint, obsx{obs_idx}, obsy{obs_idx})
                    is_safe = false;
                    return;
                end
            end
        end
        
        function allowed_primitives = get_allowed_motion_primitives(obj, current_state, all_primitives, obsc, obsx, obsy)
            % Filter motion primitives based on obstacle avoidance
            % This implements the virtual obstacle checking logic from original code
            
            allowed_primitives = [];
            
            if isempty(all_primitives)
                return;
            end
            
            for i = 1:length(all_primitives)
                primitive = all_primitives(i);
                
                % Check if this primitive leads to collision
                end_state = obj.apply_motion_primitive(current_state, primitive);
                
                if obj.check_virtual_obstacles(end_state, obsc, obsx, obsy)
                    allowed_primitives = [allowed_primitives, primitive];
                end
            end
        end
        
        function vehicle_footprint = generate_vehicle_footprint(obj, x, y, theta, gamma)
            % Generate vehicle footprint polygon
            % This creates the vehicle outline for collision checking
            
            if isempty(obj.vehicle_model)
                % Default simplified vehicle model
                obj.vehicle_model = obj.create_default_vehicle_model();
            end
            
            % Generate tractor footprint
            tractor_footprint = obj.generate_tractor_footprint(x, y, theta);
            
            % Generate trailer footprint
            trailer_footprint = obj.generate_trailer_footprint(x, y, theta, gamma);
            
            % Combine footprints
            vehicle_footprint = struct(...
                'tractor', tractor_footprint, ...
                'trailer', trailer_footprint);
        end
        
        function tractor_footprint = generate_tractor_footprint(obj, x, y, theta)
            % Generate tractor (front unit) footprint
            length_t = obj.vehicle_model.tractor_length;
            width_t = obj.vehicle_model.tractor_width;
            
            % Tractor corner points in local coordinates
            corners_local = [
                -length_t/2, -width_t/2;
                length_t/2, -width_t/2;
                length_t/2, width_t/2;
                -length_t/2, width_t/2
            ];
            
            % Transform to global coordinates
            tractor_footprint = obj.transform_points(corners_local, x, y, theta);
        end
        
        function trailer_footprint = generate_trailer_footprint(obj, x, y, theta, gamma)
            % Generate trailer footprint considering articulation
            length_tr = obj.vehicle_model.trailer_length;
            width_tr = obj.vehicle_model.trailer_width;
            hitch_offset = obj.vehicle_model.hitch_offset;
            
            % Trailer reference point (hitch point)
            hitch_x = x - hitch_offset * cosd(theta);
            hitch_y = y - hitch_offset * sind(theta);
            
            % Trailer orientation
            trailer_theta = theta + gamma;
            
            % Trailer corner points in local coordinates
            corners_local = [
                -length_tr/2, -width_tr/2;
                length_tr/2, -width_tr/2;
                length_tr/2, width_tr/2;
                -length_tr/2, width_tr/2
            ];
            
            % Transform to global coordinates
            trailer_footprint = obj.transform_points(corners_local, hitch_x, hitch_y, trailer_theta);
        end
        
        function global_points = transform_points(obj, local_points, x, y, theta)
            % Transform points from local to global coordinates
            theta_rad = deg2rad(theta);
            cos_theta = cos(theta_rad);
            sin_theta = sin(theta_rad);
            
            % Rotation matrix
            R = [cos_theta, -sin_theta; sin_theta, cos_theta];
            
            % Transform each point
            global_points = zeros(size(local_points));
            for i = 1:size(local_points, 1)
                rotated_point = R * local_points(i, :)';
                global_points(i, :) = [x + rotated_point(1), y + rotated_point(2)];
            end
        end
        
        function is_collision = check_vehicle_obstacle_collision(obj, vehicle_footprint, obs_x, obs_y)
            % Check if vehicle footprint collides with obstacle
            
            % Check tractor collision
            tractor_collision = obj.check_polygon_collision(...
                vehicle_footprint.tractor, obs_x, obs_y);
            
            % Check trailer collision
            trailer_collision = obj.check_polygon_collision(...
                vehicle_footprint.trailer, obs_x, obs_y);
            
            is_collision = tractor_collision || trailer_collision;
        end
        
        function is_collision = check_polygon_collision(obj, vehicle_polygon, obs_x, obs_y)
            % Check collision between vehicle polygon and obstacle polygon
            
            % Add collision buffer to obstacle
            buffered_obs = obj.add_buffer_to_polygon(obs_x, obs_y, obj.collision_buffer);
            
            % Check if any vehicle vertex is inside obstacle
            for i = 1:size(vehicle_polygon, 1)
                if inpolygon(vehicle_polygon(i, 1), vehicle_polygon(i, 2), ...
                           buffered_obs.x, buffered_obs.y)
                    is_collision = true;
                    return;
                end
            end
            
            % Check if any obstacle vertex is inside vehicle
            for i = 1:length(buffered_obs.x)
                if inpolygon(buffered_obs.x(i), buffered_obs.y(i), ...
                           vehicle_polygon(:, 1), vehicle_polygon(:, 2))
                    is_collision = true;
                    return;
                end
            end
            
            is_collision = false;
        end
        
        function buffered_polygon = add_buffer_to_polygon(obj, poly_x, poly_y, buffer_size)
            % Add buffer around polygon (simplified implementation)
            % This is a simplified version - more sophisticated buffering could be implemented
            
            % Simple approach: offset each vertex outward
            n = length(poly_x);
            buffered_x = zeros(size(poly_x));
            buffered_y = zeros(size(poly_y));
            
            for i = 1:n
                % Get adjacent vertices
                prev_idx = mod(i - 2, n) + 1;
                next_idx = mod(i, n) + 1;
                
                % Calculate normal vector (simplified)
                dx = poly_x(next_idx) - poly_x(prev_idx);
                dy = poly_y(next_idx) - poly_y(prev_idx);
                
                % Perpendicular vector (normal)
                norm_len = sqrt(dx^2 + dy^2);
                if norm_len > 0
                    nx = -dy / norm_len;
                    ny = dx / norm_len;
                else
                    nx = 0; ny = 0;
                end
                
                % Offset vertex
                buffered_x(i) = poly_x(i) + buffer_size * nx;
                buffered_y(i) = poly_y(i) + buffer_size * ny;
            end
            
            buffered_polygon = struct('x', buffered_x, 'y', buffered_y);
        end
        
        function end_state = apply_motion_primitive(obj, current_state, motion_primitive)
            % Apply motion primitive to get end state
            end_state = current_state;
            end_state.x = current_state.x + motion_primitive.dx;
            end_state.y = current_state.y + motion_primitive.dy;
            end_state.theta = motion_primitive.final_theta;
            end_state.gamma = motion_primitive.final_gamma;
        end
        
        function vehicle_model = create_default_vehicle_model(obj)
            % Create default vehicle model if none provided
            vehicle_model = struct(...
                'tractor_length', 6.0, ...
                'tractor_width', 2.5, ...
                'trailer_length', 10.5, ...
                'trailer_width', 2.5, ...
                'hitch_offset', 3.0);
        end
        
        function set_vehicle_model(obj, vehicle_model)
            % Set vehicle model
            obj.vehicle_model = vehicle_model;
        end
        
        function set_collision_buffer(obj, buffer_size)
            % Set collision buffer size
            obj.collision_buffer = buffer_size;
        end
    end
end
