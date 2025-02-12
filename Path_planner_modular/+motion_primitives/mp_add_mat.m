function [outputArg1,outputArg2] = mp_add_mat()

% clear the workspace
clearvars;clc;

% Initialize input parameters
input_params = motion_primitives.create_ip_params();
input_params.thetad = 0:6:354;
input_params.gammad = [-30, 0, 30]; 
input_params.motion_primitive_folder = 'FINAL_MP_PP_21';
input_params.straight_lengths = [0.2, 0.3, 5, 8, 10, 30];
input_params.num_samples = 21;

% Generate initial motion primitives
[x_0, y_0, t_0, g_0, d_0, time_0] = motion_primitives.generateInitialPrimitives(input_params);

% Add straight motion primitives
[x_0, y_0, t_0, g_0, d_0, time_0] = motion_primitives.addstraightprimitives(x_0, y_0, t_0, g_0, d_0, time_0, input_params);




