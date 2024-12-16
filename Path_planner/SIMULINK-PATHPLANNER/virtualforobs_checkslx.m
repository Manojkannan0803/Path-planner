% This virtual obstacle check is done in the beginning of the iteration
% loop inorder to select how the final orientation of semi-trailer be (i.e;
% next candidate MP). Consider as per real life, if there is no obstacle in
% front for example 10[m] then the driver prefers to continue in the same
% path instead of changing it. Same has to be inserted into the algorithm
% as it helps to reduce the computational timing.
function [A1,A2] = virtualforobs_checkslx(cur,obsc,obsx,obsy)
%________________________________________________________________
coder.extrinsic('InPolygon'); % initialize polygon.mex
condobs = false(1,1);    % initialize as logical array
A1 = nan(1,11);         % initialize output array with a size at first. otherwise it is difficult to pull out the output
A2 = nan(1,11);
%_________________________________________________________________
X_VT = cur.xa; % current x-position of semi-trailer [m]
Y_VT = cur.ya; % current y-position of semi-trailer [m]
theta_VT = deg2rad(cur.pred.tf); % current orientation angle of semi-trailer [rad]
gamma_VT = deg2rad(cur.pred.gf); % current articulation angle [rad]
% With the current position of semi-trailer, we could calculate the corners
% of vehicle (tractor alone is done here as from that we will see how much
% distance the obstacle is)
L_1f = 8.475; % Wheelbase of semitrailer [m]
L_0f = 3.8; % Wheel base of the tractor [m]
L_0b = 0.3; % Distance of 1st king-pin to tractor drive axle [m]
oh_1b = 5;       % Longitudinal distance from the trailer axle to the end of the trailer [m] (60% of the WB)
oh_1f = L_1f+1;  % Longitudinal distance from the trailer axle to the front of the trailer [m]
w_1   = 2.5;     % Width of a trailer [m]
oh_0f = 1.5; % Frontal foverhang of the truck [m]
oh_0b = 0.94; % Distance from the drive axle to the end of the tractor [m]
% Now  we will create vectors to each corner of the tractor
% Length of vector 1, 2 is the same and 3,4 is the same
lv12_0 = hypot((oh_0b),(w_1/2)); % Length of Vector 1 and 2
lv34_0 = hypot((w_1/2),(L_0f+oh_0f)); % Length of Vector 3 and 4
x_1f = X_VT+L_1f*cos(theta_VT); % Position of the king pin
y_1f = Y_VT+L_1f*sin(theta_VT);
theta_0VT = theta_VT+gamma_VT; % Orientation angle of tractor [rad]
x_0 = x_1f-L_0b*cos(theta_0VT); % Position of center of the driven axle
y_0 = y_1f-L_0b*sin(theta_0VT);
% Angle of the vectors
av1 = (180/pi*(theta_0VT))+90+atand(oh_0b/(w_1/2)); % Angle of Vector 1
av2 = (180/pi*(theta_0VT))-90-atand(oh_0b/(w_1/2)); % Angle of Vector 2
av3 = (180/pi*(theta_0VT))-atand((w_1/2)/(oh_0f+L_0f)); % Angle of Vector 3
av4 = (180/pi*(theta_0VT))+atand((w_1/2)/(oh_0f+L_0f)); % Angle of Vector 4

% Finding the actual points

xv1_0 = x_0+lv12_0*cosd(av1);
yv1_0 = y_0+lv12_0*sind(av1);
xv2_0 = x_0+lv12_0*cosd(av2);
yv2_0 = y_0+lv12_0*sind(av2);
xv3_0 = x_0+lv34_0*cosd(av3);
yv3_0 = y_0+lv34_0*sind(av3);
xv4_0 = x_0+lv34_0*cosd(av4);
yv4_0 = y_0+lv34_0*sind(av4);

% xc=[xv1_0;xv2_0;xv3_0;xv4_0];
% yc=[yv1_0;yv2_0;yv3_0;yv4_0];
% xv_obs=[xv1_0;xv2_0;xv3_0;xv4_0];
% yv_obs=[yv1_0;yv2_0;yv3_0;yv4_0];

xv_obs = [xv4_0+5;xv4_0+5;xv4_0+5+2;xv4_0+5+2];
yv_obs = [Y_VT+4;Y_VT-4;Y_VT-4;Y_VT+4];

    for j=1:obsc
        obsxtemp=obsx(j,:);
        obsytemp=obsy(j,:);
        obsxtemp=obsxtemp(obsxtemp~=0);
        obsytemp=obsytemp(obsytemp~=0);
        in1 = InPolygon(xv_obs,yv_obs,obsxtemp,obsytemp);
        condobs = any(in1);
        if condobs == 1
            A1 = cur.pred.tf:9:cur.pred.tf+90; % Change the discretization interval as per user-defined cases
            A2 = cur.pred.tf-90:9:cur.pred.tf; 
            break;
        else
            A1 = cur.pred.tf:9:cur.pred.tf+36;
            A2 = cur.pred.tf-36:9:cur.pred.tf;
        end  
    end
end