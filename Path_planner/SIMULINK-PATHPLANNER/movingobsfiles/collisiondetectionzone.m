function movingcollision = collisiondetectionzone(xp,yp,direction,thetap,gammap,xp_MO,yp_MO,thetap_MO)
coder.extrinsic('InPolygon');
% condobssta = false(1,1);    % initialize as logical array
% Consider as if LIDAR placed in the tractor drive axle and with respect to
% that bounding box is created. Then, it is checked with the moving
% obstacle placement. If the moving collision is true, then there is a slow
% moving obstacle in the detection zone otherwise no obstacle.
L_1f = 8.475; % Wheelbase of semitrailer [m]
L_detection = 48.5; % detection zone length [m]
L_0b = 0.3; % Distance of 1st king-pin to tractor drive axle [m]
% oh_1b = 5;       % Longitudinal distance from the trailer axle to the end of the trailer [m] (60% of the WB)
% oh_1f = L_1f+1;  % Longitudinal distance from the trailer axle to the front of the trailer [m]
w_detection   = 25;     % detection zone width [m]

oh_0f = 1.5; % Frontal foverhang of the truck [m
oh_0bdetection = 0; % Distance from the drive axle to the end of the tractor [m]
lv12_0 = hypot((oh_0bdetection),(w_detection/2)); % Length of Vector 1 and 2
lv34_0 = hypot((w_detection/2),(L_detection+oh_0f)); % Length of Vector 3 and 4

% xp = 75;
% yp = 40;
% thetap = deg2rad(180);
% gammap = deg2rad(-30);

%For tractor
x_1f = xp+L_1f*cos(thetap); % Position of the king pin
y_1f = yp+L_1f*sin(thetap);
theta_0 = thetap+gammap; % Orientation angle of tractor [rad]
x_0 = x_1f-L_0b*cos(theta_0); % Position of center of the driven axle
y_0 = y_1f-L_0b*sin(theta_0);

% Angle of the vectors
av1 = (180/pi*(theta_0))+90+atand(oh_0bdetection/(w_detection/2)); % Angle of Vector 1
av2 = (180/pi*(theta_0))-90-atand(oh_0bdetection/(w_detection/2)); % Angle of Vector 2
av3 = (180/pi*(theta_0))-atand((w_detection/2)/(oh_0f+L_detection)); % Angle of Vector 3
av4 = (180/pi*(theta_0))+atand((w_detection/2)/(oh_0f+L_detection)); % Angle of Vector 4

% Finding the actual points
xv1_0 = x_0+lv12_0*cosd(av1);
yv1_0 = y_0+lv12_0*sind(av1);
xv2_0 = x_0+lv12_0*cosd(av2);
yv2_0 = y_0+lv12_0*sind(av2);
xv3_0 = x_0+lv34_0*cosd(av3);
yv3_0 = y_0+lv34_0*sind(av3);
xv4_0 = x_0+lv34_0*cosd(av4);
yv4_0 = y_0+lv34_0*sind(av4);

xc=[xv1_0 xv2_0 xv3_0 xv4_0];
yc=[yv1_0 yv2_0 yv3_0 yv4_0];

% moving obstacle
% xp_MO = 30; % exact x position of MO
% yp_MO = 35; % exact y position of MO
% thetap_MO = deg2rad(90);
%
L_1fMO = 5.600;         % Wheelbase of moving obstacle [m] 
oh_1bMO = 2.440;          % Longitudinal distance from the rear axle to the end of the vehicle [m] 
oh_1fMO = L_1fMO+1.420;   % Longitudinal distance from the rear axle to the front of the vehicle [m]
w_1MO   = 2.854;         % Width of moving obstacle [m]
% Now  we will create vectors to each corner of the vehicle
% Length of vector 1, 2 is the same and 3,4 is the same
lv12_1MO = hypot(oh_1bMO,(w_1MO/2)); % Length of Vector 1 and 2
lv34_1MO = hypot((w_1MO/2),oh_1fMO); % Length of Vector 3 and 4
% Angle of the vectors
av1MO = (180/pi*thetap_MO)+90+atand(oh_1bMO/(w_1MO/2)); % Angle of Vector 1
av2MO = (180/pi*thetap_MO)-90-atand(oh_1bMO/(w_1MO/2)); % Angle of Vector 2
av3MO = (180/pi*thetap_MO)-atand((w_1MO/2)/oh_1fMO); % Angle of Vector 3
av4MO = (180/pi*thetap_MO)+atand((w_1MO/2)/oh_1fMO); % Angle of Vector 4

% Finding the actual points
xv1_1MO = xp_MO+lv12_1MO*cosd(av1MO);
yv1_1MO = yp_MO+lv12_1MO*sind(av1MO);
xv2_1MO = xp_MO+lv12_1MO*cosd(av2MO);
yv2_1MO = yp_MO+lv12_1MO*sind(av2MO);
xv3_1MO = xp_MO+lv34_1MO*cosd(av3MO);
yv3_1MO = yp_MO+lv34_1MO*sind(av3MO);
xv4_1MO = xp_MO+lv34_1MO*cosd(av4MO);
yv4_1MO = yp_MO+lv34_1MO*sind(av4MO);

xc_MO=[xv1_1MO xv2_1MO xv3_1MO xv4_1MO];
yc_MO=[yv1_1MO yv2_1MO yv3_1MO yv4_1MO];

inobs = inpolygon(xc_MO,yc_MO,xc,yc);
inobs = any(inobs);
if inobs==1
    movingcollision = 1; % moving obstacle is detected in zone
else
    movingcollision = 0; % there is no moving obstacle in the zone
end
end
