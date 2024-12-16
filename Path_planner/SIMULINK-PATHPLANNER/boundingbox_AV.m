function [checkx_ego,checky_ego] = boundingbox_AV(x_AV,y_AV,t_AV,g_AV)
%UNTITLED2 Summary of this function goes here
% articulated vehicle
L_1f = 8.475; % Wheelbase of semitrailer [m]
L_0f = 3.8; % Wheel base of the tracotr [m]
L_0b = 0.3; % Distance of 1st king-pin to tractor drive axle [m]
oh_1b = 5;       % Longitudinal distance from the trailer axle to the end of the trailer [m] (60% of the WB)
oh_1f = L_1f+1;  % Longitudinal distance from the trailer axle to the front of the trailer [m]
w_1   = 10;     % Width of a trailer [m]
% Now  we will create vectors to each corner of the trailer
% Length of vector 1, 2 is the same and 3,4 is the same
lv12_1 = hypot(oh_1b,(w_1/2)); % Length of Vector 1 and 2
lv34_1 = hypot((w_1/2),oh_1f); % Length of Vector 3 and 4
% Now we have to find the corners of the tractor. The tractor and the
% trailer have one point the same and fixed which is the king pin. So first
% we find the position of the king pin and then the rest of the corners.
oh_0f = 1.5; % Frontal foverhang of the truck [m
oh_0b = 0.94; % Distance from the drive axle to the end of the tractor [m]
lv12_0 = hypot((oh_0b),(w_1/2)); % Length of Vector 1 and 2
lv34_0 = hypot((w_1/2),(L_0f+oh_0f)); % Length of Vector 3 and 4
% For Trailer
% Angle of the vectors
av1 = (180/pi*t_AV)+90+atand(oh_1b/(w_1/2)); % Angle of Vector 1
av2 = (180/pi*t_AV)-90-atand(oh_1b/(w_1/2)); % Angle of Vector 2
av3 = (180/pi*t_AV)-atand((w_1/2)/oh_1f); % Angle of Vector 3
av4 = (180/pi*t_AV)+atand((w_1/2)/oh_1f); % Angle of Vector 4
xv1_1 = x_AV+lv12_1*cosd(av1);
yv1_1 = y_AV+lv12_1*sind(av1);
xv2_1 = x_AV+lv12_1*cosd(av2);
yv2_1 = y_AV+lv12_1*sind(av2);
xv3_1 = x_AV+lv34_1*cosd(av3);
yv3_1 = y_AV+lv34_1*sind(av3);
xv4_1 = x_AV+lv34_1*cosd(av4);
yv4_1 = y_AV+lv34_1*sind(av4);
%For tractor
x_1f = x_AV+L_1f*cos(t_AV); % Position of the king pin
y_1f = y_AV+L_1f*sin(t_AV);
theta_0 = t_AV+g_AV; % Orientation angle of tractor [rad]
x_0 = x_1f-L_0b*cos(theta_0); % Position of center of the driven axle
y_0 = y_1f-L_0b*sin(theta_0);
% Angle of the vectors
av1 = (180/pi*(theta_0))+90+atand(oh_0b/(w_1/2)); % Angle of Vector 1
av2 = (180/pi*(theta_0))-90-atand(oh_0b/(w_1/2)); % Angle of Vector 2
av3 = (180/pi*(theta_0))-atand((w_1/2)/(oh_0f+L_0f)); % Angle of Vector 3
av4 = (180/pi*(theta_0))+atand((w_1/2)/(oh_0f+L_0f)); % Angle of Vector 4
xv1_0 = x_0+lv12_0*cosd(av1);
yv1_0 = y_0+lv12_0*sind(av1);
xv2_0 = x_0+lv12_0*cosd(av2);
yv2_0 = y_0+lv12_0*sind(av2);
xv3_0 = x_0+lv34_0*cosd(av3);
yv3_0 = y_0+lv34_0*sind(av3);
xv4_0 = x_0+lv34_0*cosd(av4);
yv4_0 = y_0+lv34_0*sind(av4);
checkx_ego =[xv1_1;xv2_1;xv3_1;xv4_1;xv1_0;xv2_0;xv3_0;xv4_0];
checky_ego =[yv1_1;yv2_1;yv3_1;yv4_1;yv1_0;yv2_0;yv3_0;yv4_0];
end



