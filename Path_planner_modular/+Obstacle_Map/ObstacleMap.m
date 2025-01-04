classdef ObstacleMap < handle
   % ObstacleMap: Class for managing and visualizing obstacle maps for path
   % planning. It handles initial static obstacles, updates with new static
   % obstacles, and provides placeholders for dynamic obstacle handling at
   % the moment.

    properties
        obsx_initial_static; % x-coordinates of predefined static obstacles
        obsy_initial_static; % y-coordinates of predefined static obstacles
        obsx_update_static; % x-coordinates of newly added static obstacles
        obsy_update_static; % y-coordinates of newly added static obstacles
        obsx_dyn; % x-coordinates of dynamic obstacles
        obsy_dyn; % y-coordinates of dynamic obstacles
        vx_dyn; % velocities in X direction for dynamic obstacles
        vy_dyn; % velocities in Y direction for dynamic obstacles
        Initialstatic_count; % Number of static obstacles at initialization
    end

    methods
        % constructor to initialize the obstacle map
        % Inputs: 
        % - obsx_static - predefined x-coordinates of static obstacle [Nx1]
        % - obsy_static - predefined y-coordinates of static obstacle [Nx1]
        function obj = ObstacleMap(obsx_static, obsy_static)
            obj.obsx_initial_static = obsx_static;
            obj.obsy_initial_static = obsy_static;
            obj.obsx_update_static = []; % Initialize as empty for now
            obj.obsy_update_static = []; % Initialize as empty for now
            obj.obsx_dyn = []; % Initialize as empty for now
            obj.obsy_dyn = []; % Initialize as empty for now
            obj.vx_dyn = []; % Initialize as empty for now
            obj.vy_dyn = []; % Initialize as empty for now
            obj.Initialstatic_count = size(obsx_static, 2); % Store initial static count
        end
        
        % function to update the obstacle map with detected obstacles
        function [obsx_out, obsy_out, status] = updateMap(obj, update_obsx, update_obsy, update_flag)
            % Update the map with new static obstacles
            % Placeholder for updating the map with dynamic obstacles in
            % future
            % Inputs:
            % - update_obsx: X-coordinates of new static obstacles [Nx1
            % double]
            % - update_obsy: Y-coordinates of new static obstacles [Nx1
            % double]
            % - update_flag: Boolean flag indicating whether to update the
            % map
            % Outputs:
            % - obsx_out: Combined X-coordinates of all obstacles
            % - obsy_out: Combined Y-coordinates of all obstacles
            % - status: Boolean indicating if the map was successfully
            % updated

            % Initialize output status
            status = false; % Default to failure

            % If update flag is true, update the obstacle map with static
            % obstacles dynamically
            if update_flag
                % Store the new static obstacles separately
                obj.obsx_update_static = [obj.obsx_update_static, update_obsx];
                obj.obsy_update_static = [obj.obsy_update_static, update_obsy];

                % Append new static obstacles to the exisitng map
                combined_obsx_static = [obj.obsx_initial_static, update_obsx];
                combined_obsy_static = [obj.obsy_initial_static, update_obsy];

                % set status to true (indicating the map was updated)
                status = true;
            end

            % Place holder for estimating the moving obstacle position in
            % time

            % Combine static and dynamic obstacles
            obsx_out = [combined_obsx_static, obj.obsx_dyn];
            obsy_out = [combined_obsy_static, obj.obsy_dyn];
        end

        % Add dynamic obstacles (Placeholder)
        function addDynamicObstacles(obj, dyn_obsx, dyn_obsy, dyn_vx, dyn_vy)
            % Add dynamic obstacles to the map with velocity data
            % Inputs:
            % - dyn_obsx: X-coordinates of dynamic obstacles [Nx1 double]
            % - dyn_obsy: Y-coordinates of dynamic obstacles [Nx1 double]
            % - dyn_vx: Velocities in X-direction for dynamic obstacles
            % [Nx1 double]
            % - dyn_vy: Velocities in Y-direction for dynamic obstacles
            % [Nx1 double]

            obj.obsx_dyn = [obj.obsx_dyn; dyn_obsx];
            obj.obsy_dyn = [obj.obsy_dyn; dyn_obsy];
            obj.vx_dyn = [obj.vx_dyn; dyn_vx];
            obj.vy_dyn = [obj.vy_dyn; dyn_vy];

        end

        
        % Method for visualizing the obstacle map
        function visualizeobstaclemap(obj)
            % Visualization of the obstacle map
            figure;
            hold on;
            grid on;

            % Plot Initial Static Obstacles in red
            if ~isempty(obj.obsx_initial_static)
                fill(obj.obsx_initial_static, obj.obsy_initial_static, 'r', 'FaceAlpha', 0.7, 'DisplayName', 'Initial Static Obstacles');
            end
%             plot(obj.obsx_static(:, 1:obj.Initialstatic_count), obj.obsy_static(:,1:obj.Initialstatic_count), 'r', 'LineWidth',1.5,'DisplayName','Initial Static Obstacles');
            
            % Plot updated map with static obstacles in blue
            if ~isempty(obj.obsx_update_static)
                fill(obj.obsx_update_static, obj.obsy_update_static, 'b', 'FaceAlpha', 0.7, 'DisplayName', 'New Static Obstacles');
            end

%             if size(obj.obsx_static, 2) > obj.Initialstatic_count
%                 plot(obj.obsx_static(:, obj.Initialstatic_count+1:end), obj.obsy_static(:, obj.Initialstatic_count+1:end),'b','LineWidth',1.5,'DisplayName','New Static Obstacles');
%             end
   
            % Plot dynamic obstacles in green (placeholder)
            if ~isempty(obj.obsx_dyn)
                plot(obj.obsx_dyn, obj.obsy_dyn, 'go', 'MarkerSize', 8, 'DisplayName', 'Dynamic obstacles');
            end
%             for i = 1:size(obj.obsx_dyn,2)
%                 obsx = obj.obsx_dyn(:,i);
%                 obsy = obj.obsy_dyn(:,i);
%                 fill(obsx,obsy,'g','FaceAlpha',0.7,'DisplayName','Dynamic_Obstacle(s)');
%             end
            
            % Add legned and labels
            xlim([0 286]);
            ylim([0 200]);
            xlabel('X [m]');
            ylabel('Y [m]');
            title('Obstacle Map Visualization');
            hold off;
            legend;
        end
        
    end
end