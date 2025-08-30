classdef StateNode < handle
    % STATENODE - Individual state representation in search tree
    % Framework layer class representing a single node in the search tree
    
    properties
        id              % Unique identifier for this node
        state           % State information (x, y, theta, gamma)
        parent_id       % ID of parent node in search tree
        children_ids    % Array of children node IDs
        
        % Search algorithm specific properties
        g_cost          % Cost from start to this node
        h_cost          % Heuristic cost from this node to goal
        f_cost          % Total cost (g + h)
        
        % Motion primitive information
        motion_primitive_id  % ID of motion primitive used to reach this state
        trajectory      % Detailed trajectory from parent to this state
        
        % Metadata
        creation_time   % When this node was created
        expansion_time  % When this node was expanded (if applicable)
        is_expanded     % Whether this node has been expanded
        is_closed       % Whether this node is in closed list
    end
    
    methods
        function obj = StateNode(state, id)
            % Constructor
            if nargin < 2
                id = StateNode.generate_id();
            end
            
            obj.id = id;
            obj.state = state;
            obj.parent_id = [];
            obj.children_ids = [];
            
            obj.g_cost = inf;
            obj.h_cost = inf;
            obj.f_cost = inf;
            
            obj.motion_primitive_id = [];
            obj.trajectory = [];
            
            obj.creation_time = now;
            obj.expansion_time = [];
            obj.is_expanded = false;
            obj.is_closed = false;
        end
        
        function set_parent(obj, parent_node, motion_primitive_id, trajectory)
            % Set parent node and motion primitive information
            obj.parent_id = parent_node.id;
            if nargin > 2
                obj.motion_primitive_id = motion_primitive_id;
            end
            if nargin > 3
                obj.trajectory = trajectory;
            end
            
            % Add this node as child to parent
            parent_node.add_child(obj.id);
        end
        
        function add_child(obj, child_id)
            % Add child node ID
            if ~any(obj.children_ids == child_id)
                obj.children_ids = [obj.children_ids, child_id];
            end
        end
        
        function mark_expanded(obj)
            % Mark node as expanded
            obj.is_expanded = true;
            obj.expansion_time = now;
        end
        
        function mark_closed(obj)
            % Mark node as closed
            obj.is_closed = true;
        end
        
        function update_costs(obj, g_cost, h_cost)
            % Update cost values
            obj.g_cost = g_cost;
            obj.h_cost = h_cost;
            obj.f_cost = g_cost + h_cost;
        end
        
        function is_better = is_better_than(obj, other_node)
            % Compare this node with another based on f_cost
            is_better = obj.f_cost < other_node.f_cost;
        end
        
        function info = get_info(obj)
            % Get comprehensive node information
            info = struct(...
                'id', obj.id, ...
                'state', obj.state, ...
                'parent_id', obj.parent_id, ...
                'g_cost', obj.g_cost, ...
                'h_cost', obj.h_cost, ...
                'f_cost', obj.f_cost, ...
                'is_expanded', obj.is_expanded, ...
                'is_closed', obj.is_closed, ...
                'num_children', length(obj.children_ids));
        end
    end
    
    methods (Static)
        function id = generate_id()
            % Generate unique node ID
            persistent counter;
            if isempty(counter)
                counter = 0;
            end
            counter = counter + 1;
            id = counter;
        end
        
        function node = create_from_state(state)
            % Factory method to create node from state
            node = StateNode(state);
        end
    end
end
