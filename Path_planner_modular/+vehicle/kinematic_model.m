% The vehicle is described in global coordinates for which ISO system is used.
% Yaw rates can be derived from applying non-holonomic consraint (no lateral
% slip) on each axle.

function [x_0_dot,y_0_dot,theta_0_dot,theta_1_dot,KM_SAV_data_bus] = kinematic_model(x_0,y_0,theta_0,theta_1,L_0b,L_1f,L_0f,v_0,delta)
     
%_______________________ position _________________________________________
% steer axle

        x_0f = x_0+L_0f*cos(theta_0);
        y_0f = y_0+L_0f*sin(theta_0);
        
%king-pin

        x_1f = x_0+L_0b*cos(theta_0);
        y_1f = y_0+L_0b*sin(theta_0);
 
% semitrailer axle

        x_1 = x_1f-L_1f*cos(theta_1);
        y_1 = y_1f-L_1f*sin(theta_1);
        
% articulation angle
        
        gamma_1 = (theta_0-theta_1);                                        % articulation angle
        
% yaw rate        
        
        theta_0_dot = (v_0/L_0f) * tan(delta);                              % tractor 
        theta_1_dot = (v_0/L_1f)*sin(gamma_1) + (L_0b/L_1f)*...
                        (theta_0_dot)*cos(gamma_1);                         % semitrailer                            
                    
                    
% semitrailer longitudinal velocity in its own local body coordinate system

        v_1 = v_0*cos(gamma_1) - L_0b*(theta_0_dot)*sin(gamma_1);  
                    
        
%_________________________next iteration____________________________________
        
        x_0_dot = v_0 * cos(theta_0);                                          % tractor drive axle velocity in global x-direction
        y_0_dot = v_0 * sin(theta_0);                                          % tractor drive axle velocity in global y-direction

 % output bus
SAV_velocity = vehicle.create_velocity_result(v_1,v_0);
SAV_rate = vehicle.create_rate_result(theta_0_dot,theta_1_dot,x_0_dot,y_0_dot);
SAV_pos = vehicle.create_pos_result(x_0f, y_0f, x_0, y_0, x_1f, y_1f, x_1, y_1);
SAV_angle = vehicle.create_angle_result(theta_0, theta_1, gamma_1, delta);
KM_SAV_data_bus = vehicle.create_databus_result(SAV_velocity, SAV_angle, SAV_rate, SAV_pos);