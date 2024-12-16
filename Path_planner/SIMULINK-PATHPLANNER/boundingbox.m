function [checkx_MO,checky_MO] = boundingbox(x_MO,y_MO,t_MO)
%UNTITLED2 Summary of this function goes here
%   Moving obstacle
L_1fMO = 5.600;         % Wheelbase of moving obstacle [m] 
oh_1bMO = 2.440;          % Longitudinal distance from the rear axle to the end of the vehicle [m] 
oh_1fMO = L_1fMO+1.420;   % Longitudinal distance from the rear axle to the front of the vehicle [m]
w_1MO   = 10;         % Width of moving obstacle [m]
% Now  we will create vectors to each corner of the vehicle
% Length of vector 1, 2 is the same and 3,4 is the same
lv12_1MO = hypot(oh_1bMO,(w_1MO/2)); % Length of Vector 1 and 2
lv34_1MO = hypot((w_1MO/2),oh_1fMO); % Length of Vector 3 and 4
% Angle of the vectors
av1MO = (180/pi*t_MO)+90+atand(oh_1bMO/(w_1MO/2)); % Angle of Vector 1
av2MO = (180/pi*t_MO)-90-atand(oh_1bMO/(w_1MO/2)); % Angle of Vector 2
av3MO = (180/pi*t_MO)-atand((w_1MO/2)/oh_1fMO); % Angle of Vector 3
av4MO = (180/pi*t_MO)+atand((w_1MO/2)/oh_1fMO); % Angle of Vector 4

% Finding the actual points
xv1_1MO = x_MO+lv12_1MO*cosd(av1MO);
yv1_1MO = y_MO+lv12_1MO*sind(av1MO);
xv2_1MO = x_MO+lv12_1MO*cosd(av2MO);
yv2_1MO = y_MO+lv12_1MO*sind(av2MO);
xv3_1MO = x_MO+lv34_1MO*cosd(av3MO);
yv3_1MO = y_MO+lv34_1MO*sind(av3MO);
xv4_1MO = x_MO+lv34_1MO*cosd(av4MO);
yv4_1MO = y_MO+lv34_1MO*sind(av4MO);

checkx_MO=[xv1_1MO xv2_1MO xv3_1MO xv4_1MO];
checky_MO=[yv1_1MO yv2_1MO yv3_1MO yv4_1MO];

end

