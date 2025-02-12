function consts = create_basic_constants()

% position of tractor drive axle [m]
consts.IC_x0 = double(0); 
consts.IC_y0 = double(0);
% initial yaw angle of tractor [rad]
consts.IC_theta0 = double(0);
% initial yaw angle of semi-trailer [rad]
consts.IC_theta1 = double(0);
% Wheelbase of semitrailer [m]
consts.L_1f = double(8.475); 
% Wheel base of the tractor [m]
consts.L_0f = double(3.8); 
% Distance of 1st king-pin to tractor drive axle [m]
consts.L_0b = double(0.3); 