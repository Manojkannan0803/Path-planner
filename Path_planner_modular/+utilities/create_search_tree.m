function tree = create_search_tree()
% Creating the tree vectors in order to save the explored configurations

tree.x = []; % X config
tree.y = []; % Y config
tree.xa = []; % Actual ending point of MP X
tree.ya = []; % Actual ending point of MP Y
tree.predxy = []; % Index of the config before this config in the tree 
tree.gcost = []; % Cost so far for this config
tree.hcost = []; % Cost to go for this config
tree.pred = []; % Index of the MP used to reach this config
tree.dir = []; % Direction of motion to reach this config
tree.count = 0; % Count of the explored configs