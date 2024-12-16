function check3 = check3_dyn(xp,yp,thetap,gammap,xp_MO,yp_MO,thetap_MO)
coder.extrinsic('InPolygon');
conddyn = false(1,1);
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
% Creating the Ego vehicle corners 
% We have the position of the trailer axle(xp,yp) and the orientation of the
% trailer(thetap) and the articulation angle (gammap). So from all this
% information we can find all the four corners of the trailer and the four
% corners of the tractor and see if these corners are hitting the obstacle.
L_1f = 8.475; % Wheelbase of semitrailer [m]
L_0f = 3.8; % Wheel base of the tracotr [m]
L_0b = 0.3; % Distance of 1st king-pin to tractor drive axle [m]
oh_1b = 5;       % Longitudinal distance from the trailer axle to the end of the trailer [m] (60% of the WB)
oh_1f = L_1f+1;  % Longitudinal distance from the trailer axle to the front of the trailer [m]
w_1   = 2.5;     % Width of a trailer [m]
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
av1 = (180/pi*thetap)+90+atand(oh_1b/(w_1/2)); % Angle of Vector 1
av2 = (180/pi*thetap)-90-atand(oh_1b/(w_1/2)); % Angle of Vector 2
av3 = (180/pi*thetap)-atand((w_1/2)/oh_1f); % Angle of Vector 3
av4 = (180/pi*thetap)+atand((w_1/2)/oh_1f); % Angle of Vector 4
xv1_1 = xp+lv12_1*cosd(av1);
yv1_1 = yp+lv12_1*sind(av1);
xv2_1 = xp+lv12_1*cosd(av2);
yv2_1 = yp+lv12_1*sind(av2);
xv3_1 = xp+lv34_1*cosd(av3);
yv3_1 = yp+lv34_1*sind(av3);
xv4_1 = xp+lv34_1*cosd(av4);
yv4_1 = yp+lv34_1*sind(av4);
%For tractor
x_1f = xp+L_1f*cos(thetap); % Position of the king pin
y_1f = yp+L_1f*sin(thetap);
theta_0 = thetap+gammap; % Orientation angle of tractor [rad]
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
xc_ego =[xv1_1;xv2_1;xv3_1;xv4_1;xv1_0;xv2_0;xv3_0;xv4_0];
yc_ego =[yv1_1;yv2_1;yv3_1;yv4_1;yv1_0;yv2_0;yv3_0;yv4_0];

% We have the position of the rear axle of moving obstacle(xp_MO,yp_MO) and the orientation of the
% moving obstacle(thetap_MO). So from all this information we can find all the four corners of the 
% moving obstacle and see if these corners are hitting the obstacle. 
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
in2 = InPolygon(xc_ego,yc_ego,xc_MO,yc_MO); % check whether both the vehicles hit each other
conddyn = any(in2);
    if conddyn == 1
        check3 = 1; % intersect
    else
        check3 = 0; % doesn't intersect
    end
    
% cibxmax= any(xc_ego>=286);
% cibymax= any(yc_ego>=200);
% cibxmin= any(xc_ego<=0);
% cibymin= any(yc_ego<=0);
% for j=1:obsc
%         obsxtemp=obsx(j,:);
%         obsytemp=obsy(j,:);
%         obsxtemp=obsxtemp(obsxtemp~=0);
%         obsytemp=obsytemp(obsytemp~=0);
%         in1 = InPolygon(xc_ego,yc_ego,obsxtemp,obsytemp); % check whether cur is hitting obs
%         condobssta = any(in1);
%         if condobssta == 1 ||cibxmax==1||cibymax==1||cibxmin==1||cibymin==1
%          check3 = 1; % intersect
%          break;
%         end
% end    
end

