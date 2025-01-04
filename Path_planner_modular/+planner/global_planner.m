function [outputArg1,outputArg2] = global_planner(obstacle_map,i_state, f_state)
% Path planning algorithm - Lattice based path planner using A* search
% algorithm

tic_time = tic; % Computational time calculation

% Defining data structures
state = +utilities.create_motion_primitive_state();
tree = +utilities.create_search_tree();

% TO DO: Include obstacle map as an input to the function
% Replace run DPDscenario; % static obstacle map
obstacle_map = obstacle_map;

% TO DO: Include initial and final state as an inputs to the function
% Replace hard coded stuff in the previous algorithm

intiial_state = i_state;
final_state = f_state;


