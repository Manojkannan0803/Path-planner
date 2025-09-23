classdef VehicleDynamicsSystemObject < matlab.System
    % VEHICLEDYNAMICSSYSTEMOBJECT - Tractor-trailer vehicle dynamics for Simulink
    % Implements kinematic model for articulated vehicle with path following
    
    properties (Nontunable)
        TractorLength = 6.0        % Tractor length (m)
        TractorWidth = 2.5         % Tractor width (m)
        TrailerLength = 10.5       % Trailer length (m) 
        HitchOffset = 3.0          % Hitch offset from tractor rear (m)
        MaxSteeringAngle = 30      % Maximum steering angle (degrees)
        MaxArticulationAngle = 60  % Maximum articulation angle (degrees)
        SampleTime = 0.1           % Integration sample time (s)
    end
    
    properties (Access = private)
        vehicle_state              % Current vehicle state
        previous_time             % Previous simulation time
        is_initialized           % Initialization flag
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            % Initialize vehicle dynamics
            obj.vehicle_state = struct(...
                'x', 0, 'y', 0, 'theta', 0, 'gamma', 0, ...
                'velocity', 0, 'steering_angle', 0);
            obj.previous_time = 0;
            obj.is_initialized = true;
            
            fprintf('[Simulink] Vehicle Dynamics System initialized\n');
        end
        
        function [x_out, y_out, theta_out, gamma_out, velocity_out] = stepImpl(obj, ...
                velocity_cmd, steering_cmd, x_init, y_init, theta_init, gamma_init, sim_time)
            % Vehicle dynamics integration step
            
            % Initialize outputs with current state
            x_out = obj.vehicle_state.x;
            y_out = obj.vehicle_state.y;
            theta_out = obj.vehicle_state.theta;
            gamma_out = obj.vehicle_state.gamma;
            velocity_out = obj.vehicle_state.velocity;
            
            if ~obj.is_initialized
                return;
            end
            
            % Initialize state if this is first step or reset
            if obj.previous_time == 0 || sim_time < obj.previous_time
                obj.vehicle_state.x = x_init;
                obj.vehicle_state.y = y_init;
                obj.vehicle_state.theta = theta_init;
                obj.vehicle_state.gamma = gamma_init;
                obj.previous_time = sim_time;
                
                x_out = x_init;
                y_out = y_init;
                theta_out = theta_init;
                gamma_out = gamma_init;
                velocity_out = 0;
                return;
            end
            
            % Calculate time step
            dt = sim_time - obj.previous_time;
            if dt <= 0
                return;
            end
            
            % Saturate inputs
            velocity_cmd = max(-10, min(10, velocity_cmd));  % m/s limits
            steering_cmd = max(-obj.MaxSteeringAngle, min(obj.MaxSteeringAngle, steering_cmd));  % degrees
            
            % Convert to radians
            theta_rad = deg2rad(obj.vehicle_state.theta);
            gamma_rad = deg2rad(obj.vehicle_state.gamma);
            steering_rad = deg2rad(steering_cmd);
            
            % Kinematic model for tractor-trailer
            % Tractor motion
            x_dot = velocity_cmd * cos(theta_rad);
            y_dot = velocity_cmd * sin(theta_rad);
            theta_dot = (velocity_cmd / obj.TractorLength) * tan(steering_rad);
            
            % Trailer articulation dynamics
            % Simplified model: gamma_dot depends on tractor motion and current articulation
            hitch_velocity_x = velocity_cmd * cos(theta_rad) - ...
                              obj.HitchOffset * theta_dot * sin(theta_rad);
            hitch_velocity_y = velocity_cmd * sin(theta_rad) + ...
                              obj.HitchOffset * theta_dot * cos(theta_rad);
            
            trailer_angle = theta_rad + gamma_rad;
            trailer_forward_velocity = hitch_velocity_x * cos(trailer_angle) + ...
                                     hitch_velocity_y * sin(trailer_angle);
            
            gamma_dot = theta_dot - (trailer_forward_velocity / obj.TrailerLength) * sin(gamma_rad);
            
            % Saturate articulation rate
            max_gamma_dot = deg2rad(30);  % 30 deg/s max articulation rate
            gamma_dot = max(-max_gamma_dot, min(max_gamma_dot, gamma_dot));
            
            % Integrate using Euler method
            obj.vehicle_state.x = obj.vehicle_state.x + x_dot * dt;
            obj.vehicle_state.y = obj.vehicle_state.y + y_dot * dt;
            obj.vehicle_state.theta = obj.vehicle_state.theta + rad2deg(theta_dot * dt);
            obj.vehicle_state.gamma = obj.vehicle_state.gamma + rad2deg(gamma_dot * dt);
            obj.vehicle_state.velocity = velocity_cmd;
            obj.vehicle_state.steering_angle = steering_cmd;
            
            % Wrap angles
            obj.vehicle_state.theta = mod(obj.vehicle_state.theta + 180, 360) - 180;
            obj.vehicle_state.gamma = max(-obj.MaxArticulationAngle, ...
                                        min(obj.MaxArticulationAngle, obj.vehicle_state.gamma));
            
            % Update outputs
            x_out = obj.vehicle_state.x;
            y_out = obj.vehicle_state.y;
            theta_out = obj.vehicle_state.theta;
            gamma_out = obj.vehicle_state.gamma;
            velocity_out = obj.vehicle_state.velocity;
            
            % Update time
            obj.previous_time = sim_time;
        end
        
        function resetImpl(obj)
            % Reset vehicle dynamics
            obj.vehicle_state = struct(...
                'x', 0, 'y', 0, 'theta', 0, 'gamma', 0, ...
                'velocity', 0, 'steering_angle', 0);
            obj.previous_time = 0;
            fprintf('[Simulink] Vehicle Dynamics System reset\n');
        end
        
        function releaseImpl(obj)
            % Release resources
            obj.is_initialized = false;
            fprintf('[Simulink] Vehicle Dynamics System released\n');
        end
    end
    
    methods (Access = protected, Static)
        function header = getHeaderImpl()
            header = matlab.system.display.Header('VehicleDynamicsSystemObject', ...
                'Title', 'Tractor-Trailer Vehicle Dynamics', ...
                'Text', ['Kinematic model for articulated tractor-trailer vehicle. ' ...
                        'Integrates vehicle motion with steering and velocity commands.']);
        end
        
        function groups = getPropertyGroupsImpl()
            % Define property groups
            vehicle_group = matlab.system.display.Section(...
                'Title', 'Vehicle Parameters', ...
                'PropertyList', {'TractorLength', 'TractorWidth', 'TrailerLength', 'HitchOffset'});
            
            limits_group = matlab.system.display.Section(...
                'Title', 'Motion Limits', ...
                'PropertyList', {'MaxSteeringAngle', 'MaxArticulationAngle'});
            
            integration_group = matlab.system.display.Section(...
                'Title', 'Integration', ...
                'PropertyList', {'SampleTime'});
            
            groups = [vehicle_group, limits_group, integration_group];
        end
        
        function simMode = getSimulateUsingImpl()
            simMode = 'Interpreted execution';
        end
    end
    
    methods (Access = public)
        function state = get_current_state(obj)
            % Get current vehicle state
            state = obj.vehicle_state;
        end
        
        function set_state(obj, x, y, theta, gamma)
            % Set vehicle state
            if obj.is_initialized
                obj.vehicle_state.x = x;
                obj.vehicle_state.y = y;
                obj.vehicle_state.theta = theta;
                obj.vehicle_state.gamma = gamma;
            end
        end
    end
end
