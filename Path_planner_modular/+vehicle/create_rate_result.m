function KM_SAV_rate = create_rate_result(theta_0_dot,theta_1_dot,x_0_dot,y_0_dot)
 
KM_SAV_rate.theta_0_dot = theta_0_dot; % yaw rate of tractor
KM_SAV_rate.theta_1_dot = theta_1_dot; % yaw rate of trailer
KM_SAV_rate.x_0_dot = x_0_dot; % tractor drive axle velocity in global-x-direction
KM_SAV_rate.y_0_dot = y_0_dot; % tractor drive axle velocity in global-y-direction