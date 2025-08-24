classdef OpenList < handle
    % OPENLIST - Priority queue implementation for open list
    % Framework layer data structure - refactored from original open list arrays
    
    properties (Access = private)
        nodes           % Array of nodes
        priorities      % Array of priorities (f_costs)
        count           % Current number of nodes
        capacity        % Maximum capacity
        node_map        % Map from node ID to index for fast lookup
    end
    
    methods
        function obj = OpenList(initial_capacity)
            % Constructor
            if nargin < 1
                initial_capacity = 1000;
            end
            
            obj.capacity = initial_capacity;
            obj.nodes = cell(1, initial_capacity);
            obj.priorities = inf(1, initial_capacity);
            obj.count = 0;
            obj.node_map = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
        end
        
        function push(obj, node, priority)
            % Add node to open list with given priority
            if obj.count >= obj.capacity
                obj.expand_capacity();
            end
            
            obj.count = obj.count + 1;
            obj.nodes{obj.count} = node;
            obj.priorities(obj.count) = priority;
            obj.node_map(node.id) = obj.count;
            
            % Bubble up to maintain heap property
            obj.bubble_up(obj.count);
        end
        
        function node = pop(obj)
            % Remove and return node with minimum priority
            if obj.count == 0
                error('OpenList:Empty', 'Cannot pop from empty list');
            end
            
            node = obj.nodes{1};
            obj.node_map.remove(node.id);
            
            % Move last element to root and bubble down
            obj.nodes{1} = obj.nodes{obj.count};
            obj.priorities(1) = obj.priorities(obj.count);
            
            if ~isempty(obj.nodes{1})
                obj.node_map(obj.nodes{1}.id) = 1;
            end
            
            obj.count = obj.count - 1;
            
            if obj.count > 0
                obj.bubble_down(1);
            end
        end
        
        function node = peek(obj)
            % Return node with minimum priority without removing it
            if obj.count == 0
                node = [];
            else
                node = obj.nodes{1};
            end
        end
        
        function tf = is_empty(obj)
            % Check if open list is empty
            tf = (obj.count == 0);
        end
        
        function tf = contains(obj, node_id)
            % Check if node is in open list
            tf = obj.node_map.isKey(node_id);
        end
        
        function update_priority(obj, node_id, new_priority)
            % Update priority of existing node
            if ~obj.node_map.isKey(node_id)
                error('OpenList:NotFound', 'Node not found in open list');
            end
            
            index = obj.node_map(node_id);
            old_priority = obj.priorities(index);
            obj.priorities(index) = new_priority;
            
            % Restore heap property
            if new_priority < old_priority
                obj.bubble_up(index);
            else
                obj.bubble_down(index);
            end
        end
        
        function remove(obj, node_id)
            % Remove specific node from open list
            if ~obj.node_map.isKey(node_id)
                return; % Node not in list
            end
            
            index = obj.node_map(node_id);
            obj.node_map.remove(node_id);
            
            % Replace with last element
            obj.nodes{index} = obj.nodes{obj.count};
            obj.priorities(index) = obj.priorities(obj.count);
            
            if ~isempty(obj.nodes{index})
                obj.node_map(obj.nodes{index}.id) = index;
            end
            
            obj.count = obj.count - 1;
            
            % Restore heap property
            if obj.count > 0 && index <= obj.count
                parent_idx = floor(index / 2);
                if parent_idx > 0 && obj.priorities(index) < obj.priorities(parent_idx)
                    obj.bubble_up(index);
                else
                    obj.bubble_down(index);
                end
            end
        end
        
        function size_val = size(obj)
            % Get current size of open list
            size_val = obj.count;
        end
        
        function clear(obj)
            % Clear all nodes from open list
            obj.count = 0;
            obj.node_map = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
        end
    end
    
    methods (Access = private)
        function bubble_up(obj, index)
            % Bubble element up to maintain min-heap property
            while index > 1
                parent_idx = floor(index / 2);
                if obj.priorities(index) >= obj.priorities(parent_idx)
                    break;
                end
                
                obj.swap(index, parent_idx);
                index = parent_idx;
            end
        end
        
        function bubble_down(obj, index)
            % Bubble element down to maintain min-heap property
            while true
                min_idx = index;
                left_child = 2 * index;
                right_child = 2 * index + 1;
                
                if left_child <= obj.count && obj.priorities(left_child) < obj.priorities(min_idx)
                    min_idx = left_child;
                end
                
                if right_child <= obj.count && obj.priorities(right_child) < obj.priorities(min_idx)
                    min_idx = right_child;
                end
                
                if min_idx == index
                    break;
                end
                
                obj.swap(index, min_idx);
                index = min_idx;
            end
        end
        
        function swap(obj, i, j)
            % Swap elements at indices i and j
            % Swap nodes
            temp_node = obj.nodes{i};
            obj.nodes{i} = obj.nodes{j};
            obj.nodes{j} = temp_node;
            
            % Swap priorities
            temp_priority = obj.priorities(i);
            obj.priorities(i) = obj.priorities(j);
            obj.priorities(j) = temp_priority;
            
            % Update node map
            if ~isempty(obj.nodes{i})
                obj.node_map(obj.nodes{i}.id) = i;
            end
            if ~isempty(obj.nodes{j})
                obj.node_map(obj.nodes{j}.id) = j;
            end
        end
        
        function expand_capacity(obj)
            % Double the capacity when needed
            new_capacity = obj.capacity * 2;
            new_nodes = cell(1, new_capacity);
            new_priorities = inf(1, new_capacity);
            
            new_nodes(1:obj.capacity) = obj.nodes;
            new_priorities(1:obj.capacity) = obj.priorities;
            
            obj.nodes = new_nodes;
            obj.priorities = new_priorities;
            obj.capacity = new_capacity;
        end
    end
end
