classdef ClosedList < handle
    % CLOSEDLIST - Hash set implementation for closed list
    % Framework layer data structure - refactored from original closed list arrays
    
    properties (Access = private)
        node_set        % Set of node IDs that have been closed
        state_key_map   % Map from state keys to node IDs for duplicate detection
        nodes_by_id     % Map from node ID to actual node objects
    end
    
    methods
        function obj = ClosedList()
            % Constructor
            obj.node_set = containers.Map('KeyType', 'int32', 'ValueType', 'logical');
            obj.state_key_map = containers.Map('KeyType', 'char', 'ValueType', 'int32');
            obj.nodes_by_id = containers.Map('KeyType', 'int32', 'ValueType', 'any');
        end
        
        function add(obj, node, state_space)
            % Add node to closed list
            obj.node_set(node.id) = true;
            obj.nodes_by_id(node.id) = node;
            
            % Also add state key mapping for duplicate detection
            if nargin > 2
                state_key = state_space.get_state_key(node.state);
                obj.state_key_map(state_key) = node.id;
            end
            
            % Mark node as closed
            node.mark_closed();
        end
        
        function tf = contains_node(obj, node_id)
            % Check if specific node ID is in closed list
            tf = obj.node_set.isKey(node_id);
        end
        
        function tf = contains_state(obj, state, state_space)
            % Check if state is already in closed list (duplicate detection)
            if nargin < 3
                error('ClosedList:MissingArgument', 'StateSpace required for state checking');
            end
            
            state_key = state_space.get_state_key(state);
            tf = obj.state_key_map.isKey(state_key);
        end
        
        function node = get_node_by_state(obj, state, state_space)
            % Get node with given state from closed list
            state_key = state_space.get_state_key(state);
            if obj.state_key_map.isKey(state_key)
                node_id = obj.state_key_map(state_key);
                node = obj.nodes_by_id(node_id);
            else
                node = [];
            end
        end
        
        function node = get_node_by_id(obj, node_id)
            % Get node by ID from closed list
            if obj.node_set.isKey(node_id)
                node = obj.nodes_by_id(node_id);
            else
                node = [];
            end
        end
        
        function node_ids = get_all_node_ids(obj)
            % Get all node IDs in closed list
            node_ids = cell2mat(obj.node_set.keys);
        end
        
        function nodes = get_all_nodes(obj)
            % Get all nodes in closed list
            node_ids = obj.get_all_node_ids();
            nodes = cell(1, length(node_ids));
            for i = 1:length(node_ids)
                nodes{i} = obj.nodes_by_id(node_ids(i));
            end
        end
        
        function remove(obj, node_id, state_space)
            % Remove node from closed list (rarely used)
            if obj.node_set.isKey(node_id)
                node = obj.nodes_by_id(node_id);
                
                % Remove from node set
                obj.node_set.remove(node_id);
                obj.nodes_by_id.remove(node_id);
                
                % Remove from state key map if state_space provided
                if nargin > 2
                    state_key = state_space.get_state_key(node.state);
                    if obj.state_key_map.isKey(state_key)
                        obj.state_key_map.remove(state_key);
                    end
                end
            end
        end
        
        function size_val = size(obj)
            % Get number of nodes in closed list
            size_val = obj.node_set.Count;
        end
        
        function tf = is_empty(obj)
            % Check if closed list is empty
            tf = (obj.node_set.Count == 0);
        end
        
        function clear(obj)
            % Clear all nodes from closed list
            obj.node_set = containers.Map('KeyType', 'int32', 'ValueType', 'logical');
            obj.state_key_map = containers.Map('KeyType', 'char', 'ValueType', 'int32');
            obj.nodes_by_id = containers.Map('KeyType', 'int32', 'ValueType', 'any');
        end
        
        function stats = get_statistics(obj)
            % Get statistics about closed list
            all_nodes = obj.get_all_nodes();
            
            if isempty(all_nodes)
                stats = struct('count', 0, 'avg_g_cost', 0, 'avg_h_cost', 0);
                return;
            end
            
            g_costs = zeros(1, length(all_nodes));
            h_costs = zeros(1, length(all_nodes));
            
            for i = 1:length(all_nodes)
                g_costs(i) = all_nodes{i}.g_cost;
                h_costs(i) = all_nodes{i}.h_cost;
            end
            
            stats = struct(...
                'count', length(all_nodes), ...
                'avg_g_cost', mean(g_costs), ...
                'avg_h_cost', mean(h_costs), ...
                'min_g_cost', min(g_costs), ...
                'max_g_cost', max(g_costs));
        end
    end
end
