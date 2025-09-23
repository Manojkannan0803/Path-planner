classdef CollisionDetectionSystemObject < matlab.System
    % COLLISIONDETECTIONSYSTEMOBJECT - Dedicated collision detection for Simulink
    % Real-time collision checking for tractor-trailer vehicles in Simulink
    % Integrates with the ObstacleChecker service from Layer 3
    
    properties (Nontunable)
        VehicleLength = 6.0        % Tractor length (m)
        VehicleWidth = 2.5         % Tractor width (m) 
        TrailerLength = 10.5       % Trailer length (m)
        TrailerWidth = 2.5         % Trailer width (m)
        HitchOffset = 3.0          % Hitch offset from tractor rear (m)
        CollisionBuffer = 0.5      % Safety buffer around obstacles (m)
        MaxObstacles = 50          % Maximum number of obstacles
    end
    
    properties (Access = private)
        obstacle_checker           % ObstacleChecker instance
        vehicle_model             % Vehicle model structure
        is_initialized           % Initialization flag
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            % Initialize collision detection system
            try
                % Create vehicle model
                obj.vehicle_model = struct(...
                    'tractor_length', obj.VehicleLength, ...
                    'tractor_width', obj.VehicleWidth, ...
                    'trailer_length', obj.TrailerLength, ...
                    'trailer_width', obj.TrailerWidth, ...
                    'hitch_offset', obj.HitchOffset);
                
                % Create collision detection configuration
                config = struct(...
                    'collision_buffer', obj.CollisionBuffer, ...
                    'vehicle_model', obj.vehicle_model);
                
                % Initialize ObstacleChecker
                obj.obstacle_checker = ObstacleChecker(config);
                obj.is_initialized = true;
                
                fprintf('[Simulink] Collision Detection System initialized\n');
                
            catch ME
                obj.is_initialized = false;
                warning('CollisionDetection:InitError', ...
                    '[Simulink] Failed to initialize Collision Detection: %s', ME.message);
            end
        end
        
        function [is_safe, tractor_collision, trailer_collision] = stepImpl(obj, ...
                vehicle_x, vehicle_y, vehicle_theta, vehicle_gamma, ...
                obs_x_array, obs_y_array, obs_count)
            % Main collision detection step
            
            % Initialize outputs
            is_safe = true;
            tractor_collision = false;
            trailer_collision = false;
            
            if ~obj.is_initialized
                return;
            end
            
            try
                % Create current vehicle state
                current_state = struct('x', vehicle_x, 'y', vehicle_y, ...
                                     'theta', vehicle_theta, 'gamma', vehicle_gamma);
                
                % Process obstacles
                if obs_count > 0
                    obsc = cell(obs_count, 1);
                    obsx = cell(obs_count, 1);
                    obsy = cell(obs_count, 1);
                    
                    for i = 1:obs_count
                        % Extract obstacle data (assuming rectangular obstacles)
                        start_idx = (i-1) * 4 + 1;
                        end_idx = i * 4;
                        
                        if end_idx <= length(obs_x_array)
                            obsx{i} = obs_x_array(start_idx:end_idx);
                            obsy{i} = obs_y_array(start_idx:end_idx);
                            obsc{i} = 1;  % Obstacle exists
                        end
                    end
                    
                    % Check collision with virtual obstacles (current position)
                    is_safe = obj.obstacle_checker.check_virtual_obstacles(...
                        current_state, obsc, obsx, obsy);
                    
                    % Generate vehicle footprint for detailed collision info
                    if ~is_safe
                        vehicle_footprint = obj.obstacle_checker.generate_vehicle_footprint(...
                            vehicle_x, vehicle_y, vehicle_theta, vehicle_gamma);
                        
                        % Check individual component collisions
                        for i = 1:obs_count
                            if ~isempty(obsx{i})
                                tractor_collision = tractor_collision || ...
                                    obj.obstacle_checker.check_polygon_collision(...
                                    vehicle_footprint.tractor, obsx{i}, obsy{i});
                                
                                trailer_collision = trailer_collision || ...
                                    obj.obstacle_checker.check_polygon_collision(...
                                    vehicle_footprint.trailer, obsx{i}, obsy{i});
                            end
                        end
                    end
                end
                
            catch ME
                warning('CollisionDetection:StepError', ...
                    '[Simulink] Collision detection failed: %s', ME.message);
                is_safe = false;
            end
        end
        
        function resetImpl(obj)
            % Reset collision detection system
            if obj.is_initialized
                fprintf('[Simulink] Collision Detection System reset\n');
            end
        end
        
        function releaseImpl(obj)
            % Release resources
            obj.obstacle_checker = [];
            obj.is_initialized = false;
            fprintf('[Simulink] Collision Detection System released\n');
        end
    end
    
    methods (Access = protected, Static)
        function header = getHeaderImpl()
            header = matlab.system.display.Header('CollisionDetectionSystemObject', ...
                'Title', 'Real-time Collision Detection', ...
                'Text', ['Real-time collision detection for tractor-trailer vehicles. ' ...
                        'Checks collision with static obstacles using polygon-based algorithms.']);
        end
        
        function groups = getPropertyGroupsImpl()
            % Define property groups
            vehicle_group = matlab.system.display.Section(...
                'Title', 'Vehicle Model', ...
                'PropertyList', {'VehicleLength', 'VehicleWidth', 'TrailerLength', ...
                               'TrailerWidth', 'HitchOffset'});
            
            collision_group = matlab.system.display.Section(...
                'Title', 'Collision Settings', ...
                'PropertyList', {'CollisionBuffer', 'MaxObstacles'});
            
            groups = [vehicle_group, collision_group];
        end
        
        function simMode = getSimulateUsingImpl()
            simMode = 'Interpreted execution';
        end
    end
    
    methods (Access = public)
        function footprint = get_vehicle_footprint(obj, x, y, theta, gamma)
            % Get current vehicle footprint for visualization
            if obj.is_initialized
                footprint = obj.obstacle_checker.generate_vehicle_footprint(x, y, theta, gamma);
            else
                footprint = [];
            end
        end
        
        function update_vehicle_model(obj, vehicle_params)
            % Update vehicle model parameters
            if obj.is_initialized
                obj.obstacle_checker.set_vehicle_model(vehicle_params);
                fprintf('[Simulink] Vehicle model updated\n');
            end
        end
    end
end
