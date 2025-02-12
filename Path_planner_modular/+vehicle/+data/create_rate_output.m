function KM_SAV_rate = create_rate_output()

% Initializing the output 
KM_SAV_rate.theta_0_dot = 0; % yaw rate of tractor
KM_SAV_rate.theta_1_dot = 0; % yaw rate of trailer
KM_SAV_rate.x_0_dot = 0; % tractor drive axle velocity in global-x-direction
KM_SAV_rate.y_0_dot = 0; % tractor drive axle velocity in global-y-direction