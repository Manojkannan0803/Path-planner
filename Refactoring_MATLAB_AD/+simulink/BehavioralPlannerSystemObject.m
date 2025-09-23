classdef BehavioralPlannerSystemObject < matlab.System
    % BEHAVIORALPLANNERSYSTEMOBJECT - High-level behavioral planning for ADAS
    % Integrates with the existing 5-layer path planning architecture
    % Acts as Layer 6: Mission Planning above the existing system
    
    properties (Nontunable)
        RoutePlanFile = 'scenarios/route_plan.json'    % Route plan configuration
        SteerLimit = 30                                % Maximum steering angle (degrees)
        SegmentTolerance = 2.0                         % Distance tolerance for segment completion (m)
        PlanningHorizon = 50.0                         % Planning horizon distance (m)
        DefaultSpeed = 5.0                             % Default target speed (m/s)
    end
    
    properties (Access = private)
        route_plan              % Route plan structure
        current_segment_index   % Current segment being executed
        planning_session        % PlanningSession from Layer 5
        segment_goals          % Array of segment goals
        segment_attributes     % Array of segment attributes
        last_planning_trigger  % Last time planning was triggered
        is_initialized        % Initialization flag
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            % Initialize behavioral planner with route plan
            try
                % Load route plan
                obj.route_plan = obj.load_route_plan(obj.RoutePlanFile);
                
                % Extract segments and goals
                obj.extract_route_segments();
                
                % Initialize planning session (Layer 5 integration)
                obj.planning_session = PlanningSession('scenarios/DPDScenario.json');
                
                % Initialize state
                obj.current_segment_index = 1;
                obj.last_planning_trigger = 0;
                obj.is_initialized = true;
                
                fprintf('[Behavioral] Behavioral Planner initialized with %d segments\n', ...
                    length(obj.segment_goals));
                
            catch ME
                obj.is_initialized = false;
                warning('BehavioralPlanner:InitError', ...
                    '[Behavioral] Failed to initialize: %s', ME.message);
            end
        end
        
        function [next_goal, planning_trigger, segment_config, mission_status] = stepImpl(obj, ...
                current_x, current_y, current_theta, current_gamma, ...
                current_speed, planning_complete, sim_time)
            % Main behavioral planning step
            
            % Initialize outputs
            next_goal = [current_x, current_y, current_theta, current_gamma];
            planning_trigger = false;
            segment_config = obj.create_default_config();
            mission_status = [0, 0, 0, 0]; % [segment_index, progress, at_goal, mission_complete]
            
            if ~obj.is_initialized
                return;
            end
            
            % Current vehicle pose
            current_pose = [current_x, current_y, current_theta];
            
            try
                % Check if current segment is completed
                if obj.is_segment_completed(current_pose, planning_complete)
                    obj.advance_to_next_segment();
                end
                
                % Check if mission is completed
                mission_complete = obj.current_segment_index > length(obj.segment_goals);
                
                if mission_complete
                    % Mission completed - stop at current position
                    next_goal = [current_x, current_y, current_theta, current_gamma];
                    mission_status = [obj.current_segment_index, 1.0, true, true];
                    return;
                end
                
                % Get current segment information
                current_segment = obj.segment_goals(obj.current_segment_index);
                current_attributes = obj.segment_attributes(obj.current_segment_index);
                
                % Determine if planning should be triggered
                planning_trigger = obj.should_trigger_planning(current_pose, planning_complete, sim_time);
                
                % Set next goal from current segment
                next_goal = [current_segment.end_pose(1), current_segment.end_pose(2), ...
                           current_segment.end_pose(3), current_segment.end_pose(4)];
                
                % Configure segment-specific parameters
                segment_config = obj.create_segment_config(current_attributes, current_speed);
                
                % Calculate mission status
                segment_progress = obj.calculate_segment_progress(current_pose, current_segment);
                at_segment_goal = obj.is_at_segment_goal(current_pose, current_segment);
                
                mission_status = [obj.current_segment_index, segment_progress, at_segment_goal, false];
                
                fprintf('[Behavioral] Segment %d/%d, Progress: %.1f%%, Next Goal: [%.1f, %.1f, %.0f°]\n', ...
                    obj.current_segment_index, length(obj.segment_goals), ...
                    segment_progress * 100, next_goal(1), next_goal(2), next_goal(3));
                
            catch ME
                warning('BehavioralPlanner:StepError', ...
                    '[Behavioral] Step execution failed: %s', ME.message);
            end
        end
        
        function route_plan = load_route_plan(obj, file_path)
            % Load route plan from JSON file
            
            try
                % Load JSON configuration
                json_text = fileread(file_path);
                route_data = jsondecode(json_text);
                
                % Validate route plan structure
                obj.validate_route_plan(route_data);
                route_plan = route_data;
                
            catch ME
                % Create default route plan if file not found
                warning('BehavioralPlanner:LoadError', ...
                    'Could not load route plan from %s: %s. Using default.', file_path, ME.message);
                route_plan = obj.create_default_route_plan();
            end
        end
        
        function extract_route_segments(obj)
            % Extract route segments into internal format
            
            if ~isfield(obj.route_plan, 'segments')
                error('Route plan must contain segments field');
            end
            
            segments = obj.route_plan.segments;
            num_segments = length(segments);
            
            % Pre-allocate arrays
            obj.segment_goals = struct('start_pose', {}, 'end_pose', {}, 'segment_type', {});
            obj.segment_attributes = struct('max_speed', {}, 'end_speed', {}, 'turn_maneuver', {}, ...
                                          'stop_line', {}, 'algorithm', {});
            
            for i = 1:num_segments
                segment = segments(i);
                
                % Extract goal information
                obj.segment_goals(i).start_pose = segment.start_pose;
                obj.segment_goals(i).end_pose = segment.end_pose;
                obj.segment_goals(i).segment_type = segment.type;
                
                % Extract attributes with defaults
                obj.segment_attributes(i).max_speed = obj.get_field_or_default(segment.attributes, 'max_speed', obj.DefaultSpeed);
                obj.segment_attributes(i).end_speed = obj.get_field_or_default(segment.attributes, 'end_speed', obj.DefaultSpeed);
                obj.segment_attributes(i).turn_maneuver = obj.get_field_or_default(segment.attributes, 'turn_maneuver', false);
                obj.segment_attributes(i).stop_line = obj.get_field_or_default(segment.attributes, 'stop_line', false);
                obj.segment_attributes(i).algorithm = obj.get_field_or_default(segment.attributes, 'algorithm', 'astar');
            end
        end
        
        function value = get_field_or_default(~, struct_data, field_name, default_value)
            % Get field value or return default
            if isfield(struct_data, field_name)
                value = struct_data.(field_name);
            else
                value = default_value;
            end
        end
        
        function completed = is_segment_completed(obj, current_pose, planning_complete)
            % Check if current segment is completed
            
            if obj.current_segment_index > length(obj.segment_goals)
                completed = true;
                return;
            end
            
            current_segment = obj.segment_goals(obj.current_segment_index);
            
            % Check distance to end pose
            distance_to_end = norm(current_pose(1:2) - current_segment.end_pose(1:2));
            
            % Segment completed if close to goal and planning is complete
            completed = (distance_to_end < obj.SegmentTolerance) && planning_complete;
        end
        
        function advance_to_next_segment(obj)
            % Advance to next segment
            obj.current_segment_index = obj.current_segment_index + 1;
            
            if obj.current_segment_index <= length(obj.segment_goals)
                fprintf('[Behavioral] Advanced to segment %d/%d\n', ...
                    obj.current_segment_index, length(obj.segment_goals));
            else
                fprintf('[Behavioral] Mission completed!\n');
            end
        end
        
        function trigger = should_trigger_planning(obj, current_pose, planning_complete, sim_time)
            % Determine if planning should be triggered
            
            % Always trigger on first call
            if obj.last_planning_trigger == 0
                obj.last_planning_trigger = sim_time;
                trigger = true;
                return;
            end
            
            % Trigger if previous planning completed and we're starting new segment
            if planning_complete && obj.current_segment_index <= length(obj.segment_goals)
                current_segment = obj.segment_goals(obj.current_segment_index);
                distance_to_start = norm(current_pose(1:2) - current_segment.start_pose(1:2));
                
                % Trigger if close to segment start
                trigger = distance_to_start < obj.PlanningHorizon;
            else
                trigger = false;
            end
            
            if trigger
                obj.last_planning_trigger = sim_time;
            end
        end
        
        function config = create_segment_config(~, attributes, current_speed)
            % Create configuration for current segment
            
            config = struct();
            config.algorithm = attributes.algorithm;
            config.max_speed = attributes.max_speed;
            config.end_speed = attributes.end_speed;
            config.turn_maneuver = attributes.turn_maneuver;
            config.stop_line = attributes.stop_line;
            config.current_speed = current_speed;
            
            % Adaptive parameters based on segment type
            if attributes.turn_maneuver
                config.planning_tolerance = [0.5, 0.5, 10]; % Tighter tolerance for turns
                config.connection_distance = 5.0;
            else
                config.planning_tolerance = [1.0, 1.0, 15]; % Relaxed for straight segments
                config.connection_distance = 10.0;
            end
        end
        
        function config = create_default_config(~)
            % Create default configuration
            config = struct();
            config.algorithm = 'astar';
            config.max_speed = 5.0;
            config.end_speed = 5.0;
            config.turn_maneuver = false;
            config.stop_line = false;
            config.current_speed = 0.0;
            config.planning_tolerance = [1.0, 1.0, 15];
            config.connection_distance = 10.0;
        end
        
        function progress = calculate_segment_progress(~, current_pose, segment)
            % Calculate progress through current segment
            
            start_pos = segment.start_pose(1:2);
            end_pos = segment.end_pose(1:2);
            current_pos = current_pose(1:2);
            
            % Project current position onto start-end line
            segment_vector = end_pos - start_pos;
            current_vector = current_pos - start_pos;
            
            if norm(segment_vector) < 0.1
                progress = 1.0; % Very short segment
                return;
            end
            
            projection = dot(current_vector, segment_vector) / norm(segment_vector)^2;
            progress = max(0, min(1, projection));
        end
        
        function at_goal = is_at_segment_goal(obj, current_pose, segment)
            % Check if at segment goal
            distance = norm(current_pose(1:2) - segment.end_pose(1:2));
            at_goal = distance < obj.SegmentTolerance;
        end
        
        function validate_route_plan(~, route_data)
            % Validate route plan structure
            
            if ~isfield(route_data, 'segments')
                error('Route plan must contain segments field');
            end
            
            segments = route_data.segments;
            
            for i = 1:length(segments)
                segment = segments(i);
                
                if ~isfield(segment, 'start_pose') || ~isfield(segment, 'end_pose')
                    error('Segment %d must contain start_pose and end_pose', i);
                end
                
                if length(segment.start_pose) ~= 4 || length(segment.end_pose) ~= 4
                    error('Segment %d poses must be [x, y, theta, gamma]', i);
                end
            end
        end
        
        function route_plan = create_default_route_plan(~)
            % Create default route plan for testing
            
            route_plan = struct();
            route_plan.name = 'Default Test Route';
            route_plan.segments = [
                struct('start_pose', [75, 45, 180, 0], 'end_pose', [65, 55, 225, 0], ...
                       'type', 'turn', 'attributes', struct('max_speed', 3, 'end_speed', 2, ...
                       'turn_maneuver', true, 'algorithm', 'astar')), ...
                struct('start_pose', [65, 55, 225, 0], 'end_pose', [60, 64, 270, 0], ...
                       'type', 'straight', 'attributes', struct('max_speed', 5, 'end_speed', 0, ...
                       'turn_maneuver', false, 'stop_line', true, 'algorithm', 'astar'))
            ];
        end
        
        function resetImpl(obj)
            % Reset behavioral planner
            obj.current_segment_index = 1;
            obj.last_planning_trigger = 0;
            fprintf('[Behavioral] Behavioral Planner reset\n');
        end
        
        function releaseImpl(obj)
            % Release resources
            obj.planning_session = [];
            obj.is_initialized = false;
            fprintf('[Behavioral] Behavioral Planner released\n');
        end
    end
    
    methods (Access = protected, Static)
        function header = getHeaderImpl()
            header = matlab.system.display.Header('BehavioralPlannerSystemObject', ...
                'Title', 'Mission-Level Behavioral Planner', ...
                'Text', ['High-level behavioral planning layer that sequences mission ' ...
                        'segments and coordinates with the 5-layer path planning architecture.']);
        end
        
        function groups = getPropertyGroupsImpl()
            % Define property groups
            mission_group = matlab.system.display.Section(...
                'Title', 'Mission Configuration', ...
                'PropertyList', {'RoutePlanFile', 'DefaultSpeed'});
            
            planning_group = matlab.system.display.Section(...
                'Title', 'Planning Parameters', ...
                'PropertyList', {'SteerLimit', 'SegmentTolerance', 'PlanningHorizon'});
            
            groups = [mission_group, planning_group];
        end
        
        function simMode = getSimulateUsingImpl()
            simMode = 'Interpreted execution';
        end
    end
    
    methods (Access = public)
        function segment_info = get_current_segment_info(obj)
            % Get current segment information
            if obj.is_initialized && obj.current_segment_index <= length(obj.segment_goals)
                segment_info = struct();
                segment_info.goal = obj.segment_goals(obj.current_segment_index);
                segment_info.attributes = obj.segment_attributes(obj.current_segment_index);
                segment_info.index = obj.current_segment_index;
                segment_info.total_segments = length(obj.segment_goals);
            else
                segment_info = [];
            end
        end
        
        function set_current_segment(obj, segment_index)
            % Manually set current segment (for testing)
            if segment_index >= 1 && segment_index <= length(obj.segment_goals)
                obj.current_segment_index = segment_index;
                fprintf('[Behavioral] Manually set to segment %d\n', segment_index);
            end
        end
    end
end
