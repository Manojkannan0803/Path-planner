classdef DataLoader
    % DATALOADER - Load and manage data files
    % Foundation layer I/O utility for loading motion primitives and obstacle maps
    
    methods (Static)
        function data = load_motion_primitive_data(file_path)
            % Load motion primitive data from .mat file
            try
                data = load(file_path);
                DataLoader.validate_motion_primitive_data(data);
            catch ME
                error('DataLoader:LoadError', 'Failed to load motion primitive data: %s', ME.message);
            end
        end
        
        function obstacle_map = load_obstacle_map(file_path)
            % Load obstacle map from .mat file
            try
                data = load(file_path);
                obstacle_map = DataLoader.parse_obstacle_map(data);
            catch ME
                error('DataLoader:LoadError', 'Failed to load obstacle map: %s', ME.message);
            end
        end
        
        function config = load_json_config(file_path)
            % Load configuration from JSON file
            try
                json_text = fileread(file_path);
                config = jsondecode(json_text);
            catch ME
                error('DataLoader:ConfigError', 'Failed to load JSON config: %s', ME.message);
            end
        end
        
        function save_data(data, file_path, format)
            % Save data to file in specified format
            if nargin < 3
                format = 'mat';
            end
            
            try
                switch lower(format)
                    case 'mat'
                        save(file_path, '-struct', 'data');
                    case 'json'
                        json_text = jsonencode(data);
                        fid = fopen(file_path, 'w');
                        fprintf(fid, '%s', json_text);
                        fclose(fid);
                    otherwise
                        error('Unsupported format: %s', format);
                end
            catch ME
                error('DataLoader:SaveError', 'Failed to save data: %s', ME.message);
            end
        end
    end
    
    methods (Static, Access = private)
        function validate_motion_primitive_data(data)
            % Validate motion primitive data structure
            required_fields = {'x', 'y', 't', 'g', 'time'};
            for i = 1:length(required_fields)
                if ~isfield(data, required_fields{i})
                    error('Missing required field: %s', required_fields{i});
                end
            end
        end
        
        function obstacle_map = parse_obstacle_map(data)
            % Parse and structure obstacle map data
            if isfield(data, 'obsc')
                obstacle_map.static_obstacles = data.obsc;
            else
                obstacle_map.static_obstacles = [];
            end
            
            if isfield(data, 'obsx') && isfield(data, 'obsy')
                obstacle_map.obstacle_x = data.obsx;
                obstacle_map.obstacle_y = data.obsy;
            else
                obstacle_map.obstacle_x = [];
                obstacle_map.obstacle_y = [];
            end
        end
    end
end
