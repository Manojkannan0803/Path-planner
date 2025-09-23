classdef VehicleParams < handle
    % VehicleParams - Vehicle configuration and parameter management
    % 
    % This class loads and validates vehicle parameters from YAML configuration
    % and provides structured access for the autonomous valet parking system.
    %
    % Package: +Planner.+Src
    % Usage:
    %   vp = Planner.Src.VehicleParams();  % Uses default config
    %   vp = Planner.Src.VehicleParams(configFile);  % Custom config
    %   wheelbase = vp.wheelbase;  % Access via dependent properties
    %   isValid = vp.validateConfig();
    
    properties (Access = private)
        configFile
        config
        isConfigLoaded = false
    end
    
    properties (Access = public, Dependent)
        % Vehicle dimensions
        length
        width  
        wheelbase
        rearAxleDistance
        
        % Steering characteristics
        maxSteeringAngle
        maxSteeringRate
        minTurningRadius
        
        % Performance limits
        maxVelocity
        maxAcceleration
        maxJerk
        
        % Safety parameters
        safetyMarginStatic
        safetyMarginDynamic
        emergencyStopDistance
    end
    
    methods
        function obj = VehicleParams(configFile)
            % Constructor - Load configuration from YAML file
            if nargin < 1
                % Default config path for +Planner package
                configFile = '+Planner/+Config/vehicle_config.yaml';
            end
            obj.configFile = configFile;
            obj.loadConfig();
        end
        
        function loadConfig(obj)
            % Load and parse YAML configuration file
            try
                fprintf('[Planner.VehicleParams] Attempting to load config: %s\n', obj.configFile);
                
                if ~isfile(obj.configFile)
                    error('Planner:VehicleParams:FileNotFound', ...
                        'Configuration file not found: %s', obj.configFile);
                end
                
                fprintf('[Planner.VehicleParams] File exists, parsing YAML...\n');
                
                % Load YAML file (requires YAML toolbox or custom parser)
                obj.config = obj.parseYAML(obj.configFile);
                obj.isConfigLoaded = true;
                
                % Validate configuration
                if ~obj.validateConfig()
                    error('Planner:VehicleParams:ValidationFailed', ...
                        'Configuration validation failed');
                end
                
                fprintf('[Planner.VehicleParams] Configuration loaded successfully: %s\n', obj.configFile);
                
            catch ME
                fprintf('[Planner.VehicleParams] Error loading configuration: %s\n', ME.message);
                obj.isConfigLoaded = false;
                rethrow(ME);
            end
        end
        
        function loaded = isLoaded(obj)
            % Check if configuration is loaded
            loaded = obj.isConfigLoaded;
        end
        
        function isValid = validateConfig(obj)
            % Validate configuration parameters
            isValid = true;
            errorMessages = cell.empty;
            
            if ~obj.isConfigLoaded
                error('Planner:VehicleParams:NotLoaded', 'Configuration not loaded');
            end
            
            % Vehicle dimension validation
            if obj.length <= 0 || obj.length > 10
                isValid = false;
                errorMessages{end+1} = sprintf('Invalid vehicle length: %.2fm', obj.length);
            end
            
            if obj.width <= 0 || obj.width > 3
                isValid = false;
                errorMessages{end+1} = sprintf('Invalid vehicle width: %.2fm', obj.width);
            end
            
            if obj.wheelbase <= 0 || obj.wheelbase >= obj.length
                isValid = false;
                errorMessages{end+1} = sprintf('Invalid wheelbase: %.2fm', obj.wheelbase);
            end
            
            % Steering validation
            if obj.maxSteeringAngle <= 0 || obj.maxSteeringAngle > 60
                isValid = false;
                errorMessages{end+1} = sprintf('Invalid max steering angle: %.1f°', obj.maxSteeringAngle);
            end
            
            % Validate turning radius consistency
            theoreticalRadius = obj.wheelbase / tan(deg2rad(obj.maxSteeringAngle));
            if abs(theoreticalRadius - obj.minTurningRadius) > 0.5
                warning('Planner:VehicleParams:TurningRadiusMismatch', ...
                    'Turning radius mismatch: theoretical=%.2fm, configured=%.2fm', ...
                    theoreticalRadius, obj.minTurningRadius);
            end
            
            % Performance limits validation
            if obj.maxVelocity <= 0 || obj.maxVelocity > 50
                isValid = false;
                errorMessages{end+1} = sprintf('Invalid max velocity: %.1f m/s', obj.maxVelocity);
            end
            
            if obj.maxAcceleration <= 0 || obj.maxAcceleration > 10
                isValid = false;
                errorMessages{end+1} = sprintf('Invalid max acceleration: %.1f m/s²', obj.maxAcceleration);
            end
            
            % Safety margin validation
            if obj.safetyMarginStatic < 0 || obj.safetyMarginStatic > 2
                isValid = false;
                errorMessages{end+1} = sprintf('Invalid static safety margin: %.2fm', obj.safetyMarginStatic);
            end
            
            if obj.safetyMarginDynamic < obj.safetyMarginStatic
                isValid = false;
                errorMessages{end+1} = 'Dynamic safety margin must be >= static margin';
            end
            
            % Report validation results
            if ~isValid
                fprintf('[Planner.VehicleParams] Configuration validation failed:\n');
                numErrors = numel(errorMessages);
                for i = 1:numErrors
                    fprintf('  - %s\n', errorMessages{i});
                end
                error('Planner:VehicleParams:ValidationFailed', 'Configuration validation failed');
            else
                fprintf('[Planner.VehicleParams] Configuration validation passed\n');
            end
        end
        
        function footprint = getVehicleFootprint(obj, x, y, theta)
            % Calculate vehicle footprint corners at given pose
            % Returns [4x2] array of corner coordinates
            
            if ~obj.isConfigLoaded
                error('Planner:VehicleParams:NotLoaded', 'Configuration not loaded');
            end
            
            % Vehicle corners in local coordinate system (rear axle origin)
            half_width = obj.width / 2;
            rear_overhang = obj.rearAxleDistance;
            front_overhang = obj.length - obj.wheelbase - rear_overhang;
            
            % Local corners [x, y] relative to rear axle
            local_corners = [
                -rear_overhang,     -half_width;    % Rear left
                obj.wheelbase + front_overhang, -half_width;    % Front left  
                obj.wheelbase + front_overhang,  half_width;    % Front right
                -rear_overhang,      half_width     % Rear right
            ];
            
            % Rotation matrix
            cos_theta = cos(theta);
            sin_theta = sin(theta);
            R = [cos_theta, -sin_theta; sin_theta, cos_theta];
            
            % Transform to global coordinates
            footprint = (R * local_corners')' + [x, y];
        end
        
        function exportToStruct(obj)
            % Export configuration to base workspace as structured data
            if ~obj.isConfigLoaded
                error('Planner:VehicleParams:NotLoaded', 'Configuration not loaded');
            end
            
            % Create structured parameter set
            VehicleConfig = struct();
            
            % Vehicle dimensions
            VehicleConfig.Dimensions.length = obj.length;
            VehicleConfig.Dimensions.width = obj.width;
            VehicleConfig.Dimensions.wheelbase = obj.wheelbase;
            VehicleConfig.Dimensions.rearAxleDistance = obj.rearAxleDistance;
            
            % Steering characteristics  
            VehicleConfig.Steering.maxAngle = deg2rad(obj.maxSteeringAngle);
            VehicleConfig.Steering.maxRate = deg2rad(obj.maxSteeringRate);
            VehicleConfig.Steering.minTurningRadius = obj.minTurningRadius;
            
            % Performance limits
            VehicleConfig.Performance.maxVelocity = obj.maxVelocity;
            VehicleConfig.Performance.maxAcceleration = obj.maxAcceleration;
            VehicleConfig.Performance.maxJerk = obj.maxJerk;
            
            % Safety parameters
            VehicleConfig.Safety.marginStatic = obj.safetyMarginStatic;
            VehicleConfig.Safety.marginDynamic = obj.safetyMarginDynamic;
            VehicleConfig.Safety.emergencyStopDistance = obj.emergencyStopDistance;
            
            % Export to base workspace
            assignin('base', 'VehicleConfig', VehicleConfig);
            fprintf('[Planner.VehicleParams] Configuration exported to workspace as ''VehicleConfig''\n');
        end
        
        function planningParams = getPlanningParams(obj)
            % Get planning-specific parameters
            if ~obj.isConfigLoaded
                error('Planner:VehicleParams:NotLoaded', 'Configuration not loaded');
            end
            
            planningParams = struct();
            planningParams.globalPlannerTimeout = obj.getConfigValue('planning', 'global_planner_timeout');
            planningParams.replanTimeout = obj.getConfigValue('planning', 'replan_timeout');
            planningParams.maxPlanningIterations = obj.getConfigValue('planning', 'max_planning_iterations');
            planningParams.spatialResolution = obj.getConfigValue('planning', 'spatial_resolution');
            planningParams.angularResolution = obj.getConfigValue('planning', 'angular_resolution');
            planningParams.costWeights = obj.getConfigValue('planning', 'cost_weights');
        end
        
        function envParams = getEnvironmentParams(obj)
            % Get environment constraint parameters
            if ~obj.isConfigLoaded
                error('Planner:VehicleParams:NotLoaded', 'Configuration not loaded');
            end
            
            envParams = struct();
            envParams.maxObstacles = obj.getConfigValue('environment', 'max_obstacles');
            envParams.maxDynamicObjects = obj.getConfigValue('environment', 'max_dynamic_objects');
            envParams.parkingSpaceWidth = obj.getConfigValue('environment', 'parking_space_width');
            envParams.planningHorizon = obj.getConfigValue('environment', 'planning_horizon');
        end
        
        function simParams = getSimulationParams(obj)
            % Get simulation parameter structure
            if ~obj.isConfigLoaded
                error('Planner:VehicleParams:NotLoaded', 'Configuration not loaded');
            end
            
            simParams = struct();
            simParams.baseSampleTime = obj.getConfigValue('simulation', 'base_sample_time');
            simParams.globalPlannerRate = obj.getConfigValue('simulation', 'global_planner_rate');
            simParams.localAdapterRate = obj.getConfigValue('simulation', 'local_adapter_rate');
            simParams.trajectoryGenRate = obj.getConfigValue('simulation', 'trajectory_gen_rate');
            simParams.safetyMonitorRate = obj.getConfigValue('simulation', 'safety_monitor_rate');
        end
    end
    
    % Dependent property getters
    methods
        function val = get.length(obj)
            val = obj.getConfigValue('vehicle', 'length');
        end
        
        function val = get.width(obj)
            val = obj.getConfigValue('vehicle', 'width');
        end
        
        function val = get.wheelbase(obj)
            val = obj.getConfigValue('vehicle', 'wheelbase');
        end
        
        function val = get.rearAxleDistance(obj)
            val = obj.getConfigValue('vehicle', 'rear_axle_distance');
        end
        
        function val = get.maxSteeringAngle(obj)
            val = obj.getConfigValue('vehicle', 'max_steering_angle');
        end
        
        function val = get.maxSteeringRate(obj)
            val = obj.getConfigValue('vehicle', 'max_steering_rate');
        end
        
        function val = get.minTurningRadius(obj)
            val = obj.getConfigValue('vehicle', 'min_turning_radius');
        end
        
        function val = get.maxVelocity(obj)
            val = obj.getConfigValue('vehicle', 'max_velocity');
        end
        
        function val = get.maxAcceleration(obj)
            val = obj.getConfigValue('vehicle', 'max_acceleration');
        end
        
        function val = get.maxJerk(obj)
            val = obj.getConfigValue('vehicle', 'max_jerk');
        end
        
        function val = get.safetyMarginStatic(obj)
            val = obj.getConfigValue('vehicle', 'safety_margin_static');
        end
        
        function val = get.safetyMarginDynamic(obj)
            val = obj.getConfigValue('vehicle', 'safety_margin_dynamic');
        end
        
        function val = get.emergencyStopDistance(obj)
            val = obj.getConfigValue('vehicle', 'emergency_stop_distance');
        end
    end
    
    methods (Access = private)
        function val = getConfigValue(obj, section, field)
            % Safe configuration value retrieval
            if ~obj.isConfigLoaded
                error('Planner:VehicleParams:NotLoaded', 'Configuration not loaded');
            end
            
            try
                val = obj.config.(section).(field);
            catch
                error('Planner:VehicleParams:FieldNotFound', ...
                    'Configuration field not found: %s.%s', section, field);
            end
        end
        
        function config = parseYAML(~, filename)
            % Enhanced YAML parser for vehicle configuration
            % Handles nested structures with proper indentation
            
            config = struct();
            fid = -1;  % Initialize to invalid value
            
            try
                % Read file content
                fprintf('[Planner.VehicleParams] Reading file: %s\n', filename);
                fid = fopen(filename, 'r');
                if fid == -1
                    error('Cannot open file: %s', filename);
                end
                
                % Read file line by line
                lines = {};
                lineCount = 0;
                while ~feof(fid)
                    line = fgetl(fid);
                    if ischar(line)  % fgetl returns -1 at EOF, not a string
                        lineCount = lineCount + 1;
                        lines{lineCount} = line;
                    end
                end
                fclose(fid);
                fid = -1;  % Mark as closed
                
                fprintf('[Planner.VehicleParams] Read %d lines\n', lineCount);
                
                % Parse with nested structure support
                currentPath = {};  % Track current nesting path
                
                for i = 1:lineCount
                    line = lines{i};
                    
                    % Skip comments and empty lines
                    trimmedLineCheck = strtrim(line);
                    if isempty(trimmedLineCheck) || (~isempty(trimmedLineCheck) && trimmedLineCheck(1) == '#')
                        continue;
                    end
                    
                    % Count leading spaces to determine indentation level
                    leadingSpaces = 0;
                    for j = 1:numel(line)
                        if line(j) == ' '
                            leadingSpaces = leadingSpaces + 1;
                        else
                            break;
                        end
                    end
                    
                    indentLevel = floor(leadingSpaces / 2);  % Assuming 2-space indentation
                    trimmedLine = strtrim(line);
                    
                    % Adjust current path based on indentation
                    if indentLevel < numel(currentPath)
                        currentPath = currentPath(1:indentLevel);
                    end
                    
                    % Parse line content
                    if ~isempty(trimmedLine) && trimmedLine(end) == ':'
                        % Section or subsection header
                        sectionName = strtrim(trimmedLine(1:end-1));
                        
                        % Add to current path
                        currentPath{indentLevel + 1} = sectionName;
                        
                        % Create nested structure directly
                        evalStr = 'config';
                        for k = 1:numel(currentPath)
                            evalStr = [evalStr '.(''' currentPath{k} ''')'];
                        end
                        evalStr = [evalStr ' = struct();'];
                        eval(evalStr);
                        
                        fprintf('[Planner.VehicleParams] Level %d: %s\n', indentLevel, ...
                            strjoin(currentPath, '.'));
                        
                    elseif contains(trimmedLine, ':')
                        % Key-value pair
                        colonIdx = strfind(trimmedLine, ':');
                        key = strtrim(trimmedLine(1:colonIdx(1)-1));
                        valueStr = strtrim(trimmedLine(colonIdx(1)+1:end));
                        
                        % Remove comments
                        commentPos = strfind(valueStr, '#');
                        if ~isempty(commentPos)
                            valueStr = strtrim(valueStr(1:commentPos(1)-1));
                        end
                        
                        % Convert value
                        if ~isempty(valueStr)
                            numValue = str2double(valueStr);
                            if ~isnan(numValue)
                                value = numValue;
                            else
                                value = valueStr;
                            end
                            
                            % Set value in nested structure directly
                            evalStr = 'config';
                            for k = 1:numel(currentPath)
                                evalStr = [evalStr '.(''' currentPath{k} ''')'];
                            end
                            evalStr = [evalStr '.(''' key ''') = value;'];
                            eval(evalStr);
                            
                            fullPath = strjoin([currentPath, {key}], '.');
                            if isnumeric(value)
                                fprintf('[Planner.VehicleParams] %s = %.6g\n', fullPath, value);
                            else
                                fprintf('[Planner.VehicleParams] %s = %s\n', fullPath, value);
                            end
                        end
                    end
                end
                
                fprintf('[Planner.VehicleParams] YAML parsing completed successfully\n');
                
            catch ME
                if fid ~= -1
                    fclose(fid);
                end
                error('Planner:VehicleParams:YAMLParseError', ...
                    'Error parsing YAML file %s: %s', filename, ME.message);
            end
        end
    end
end