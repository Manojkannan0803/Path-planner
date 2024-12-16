% This virtual obstacle check is done in the beginning of the iteration
% loop inorder to select how the final orientation of semi-trailer be (i.e;
% next candidate MP). Consider as per real life, if there is no obstacle in
% front for example 10[m] then the driver prefers to continue in the same
% path instead of changing it. Same has to be inserted into the algorithm
% as it helps to reduce the computational timing.
function [A3,A4] = virtualobsreverse_checkCopy(cur,obsc,obsx,obsy,oh_1b,oh_1f,w_1,lv12_1,lv34_1)
%________________________________________________________________
coder.extrinsic('InPolygon'); % initialize polygon.mex
condobs1 = false(1,1);    % initialize as logical array
A3 = nan(1,11);         % initialize output array with a size at first. otherwise it is difficult to pull out the output
A4 = nan(1,11);
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
% % Now  we will create vectors to each corner of the trailer
% % Length of vector 1, 2 is the same and 3,4 is the same
lv12_1 = hypot(oh_1b,(w_1/2)); % Length of Vector 1 and 2
lv34_1 = hypot((w_1/2),oh_1f); % Length of Vector 3 and 4
% For Trailer
% Angle of the vectors
av1 = (180/pi*theta_VT)+90+atand(oh_1b/(w_1/2)); % Angle of Vector 1
av2 = (180/pi*theta_VT)-90-atand(oh_1b/(w_1/2)); % Angle of Vector 2
av3 = (180/pi*theta_VT)-atand((w_1/2)/oh_1f); % Angle of Vector 3
av4 = (180/pi*theta_VT)+atand((w_1/2)/oh_1f); % Angle of Vector 4

% Finding the actual points
xv1_1 = X_VT+lv12_1*cosd(av1);
yv1_1 = Y_VT+lv12_1*sind(av1);
xv2_1 = X_VT+lv12_1*cosd(av2);
yv2_1 = Y_VT+lv12_1*sind(av2);
xv3_1 = X_VT+lv34_1*cosd(av3);
yv3_1 = Y_VT+lv34_1*sind(av3);
xv4_1 = X_VT+lv34_1*cosd(av4);
yv4_1 = Y_VT+lv34_1*sind(av4);

% xc=[xv1_1;xv2_1;xv3_1;xv4_1];
% yc=[yv1_1;yv2_1;yv3_1;yv4_1];
% xv_obsR=[xv1_1;xv2_1;xv3_1;xv4_1];
% yv_obsR=[yv1_1;yv2_1;yv3_1;yv4_1];
xv_obsR = [xv1_1+5;xv1_1+5;xv1_1+5+2;xv1_1+5+2];
yv_obsR = [Y_VT+4;Y_VT-4;Y_VT-4;Y_VT+4];


    for j=1:obsc
        obsxtemp=obsx(j,:);
        obsytemp=obsy(j,:);
        obsxtemp=obsxtemp(obsxtemp~=0);
        obsytemp=obsytemp(obsytemp~=0);
        in1 = InPolygon(xv_obsR,yv_obsR,obsxtemp,obsytemp); % check whether cur is hitting obs
        condobs1 = any(in1);
        if condobs1 == 1
            A3 = cur.pred.tf:9:cur.pred.tf+90; % Change the discretization interval as per user-defined cases
            A4 = cur.pred.tf-90:9:cur.pred.tf; 
            break;
        else
            A3 = cur.pred.tf:9:cur.pred.tf+36;
            A4 = cur.pred.tf-36:9:cur.pred.tf;
        end
    end
%     if in1
%        A3 = cur.pred.tf:9:cur.pred.tf+90; % Change the discretization interval as per user-defined cases
%        A4 = cur.pred.tf-90:9:cur.pred.tf; 
%     else
%        A3 = cur.pred.tf:9:cur.pred.tf+45;
%        A4 = cur.pred.tf-45:9:cur.pred.tf;
%     end
%         
end
