clear variables;
%__________________________________________________________________________

% load the motion primitives
run Add_MP_toworkspace;
% update the static obstacle map
run DPDscenario; 
clearvars -except d g gfin gini obsc obsx obsy t tfin timeAV tini x xfin y yfin
%__________________________________________________________________________
% load Parallelprimitivesnew;
% information
length_dc = 328;    % length of the distribution center [m]
width_dc = 200;     % width of the distribution center [m]
scale = 1;
%__________________________________________________________________________

% initial pose
% changed from struct format to scalar format
i_statex = 200;%200 250
i_statey = 40;%40  45
i_statet = 180;%180 180
i_stateg = 0;
% changed from global to normal variable
f_statex = 20;%20 60 
f_statey = 40;%40 64 
f_statet = 180;%180 270
f_stateg = 0;

%__________________________________________________________________________
% Area definition
% related to h-cost 
lz_lg = 25;
wz_lg= 1;
dt1_lg= f_statet + atand(wz_lg/lz_lg);
dt2_lg= f_statet - atand(wz_lg/lz_lg);
dl_lg= hypot(wz_lg/2,lz_lg/2);
x1_lg=  f_statex + dl_lg*cosd(dt1_lg);
y1_lg=  f_statey + dl_lg*sind(dt1_lg);
x2_lg=  f_statex + dl_lg*cosd(dt2_lg);
y2_lg=  f_statey + dl_lg*sind(dt2_lg);
x3_lg=  f_statex + dl_lg*cosd(dt1_lg+180);
y3_lg=  f_statey + dl_lg*sind(dt1_lg+180);
x4_lg=  f_statex + dl_lg*cosd(dt2_lg+180);
y4_lg=  f_statey + dl_lg*sind(dt2_lg+180);

xv_lg = [x1_lg x2_lg x3_lg x4_lg];
yv_lg = [y1_lg y2_lg y3_lg y4_lg];
% % % related to h-cost1
% xv_dv = [61.4 40 40 61.4];
% yv_dv = [30 30 51 51];
% related to reversemaneuber
xv_rv = [61.4 30 30 61.4];
yv_rv = [20 20 70 70];
% loopcheck
lz_lc= 5; % IN zone
wz_lc= 1.5;
dt1_lc= f_statet + atand(wz_lc/lz_lc);
dt2_lc= f_statet - atand(wz_lc/lz_lc);
dl_lc= hypot(wz_lc/2,lz_lc/2);
x1_lc=  f_statex + dl_lc*cosd(dt1_lc);
y1_lc=  f_statey + dl_lc*sind(dt1_lc);
x2_lc=  f_statex + dl_lc*cosd(dt2_lc);
y2_lc=  f_statey + dl_lc*sind(dt2_lc);
x3_lc=  f_statex + dl_lc*cosd(dt1_lc+180);
y3_lc=  f_statey + dl_lc*sind(dt1_lc+180);
x4_lc=  f_statex + dl_lc*cosd(dt2_lc+180);
y4_lc=  f_statey + dl_lc*sind(dt2_lc+180);
xv_lc = [x1_lc x2_lc x3_lc x4_lc];
yv_lc = [y1_lc y2_lc y3_lc y4_lc];
%__________________________________________________________________________

% discretization intervals
thetad = 0:6:354;
gammad = [-21 0 21];

%__________________________________________________________________________
oc  =   0;  % Open list count
cc  =   0;  % Closed list count
% Adding the inital state in the closed list.
cc          =   cc+1;
c(cc)       =   1;
%__________________________________________________________________________

% defining an articulated vehicle dimensions
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
lv12_1 = hypot(oh_1b,(w_1/2)); % Length of Vector 1 and 2
lv34_1 = hypot((w_1/2),oh_1f); % Length of Vector 3 and 4
%__________________________________________________________________________
% Controller gains
steer_sensforward =4;%7.7                                                   % steersensitivity_forward
lookahead_timeforward = 2;%1.5                                              % lookaheaddistance_forward
steer_ratioforward = 1;                                                     % steerratio_forward
steer_sensreverse = 7.2;                                                    % steersensitivity_reverse
lookahead_timereverse = 1.5;                                                % lookaheaddistance_reverse
steer_ratioreverse = 1;                                                     % steerratio_reverse
IC_x1= i_statex;                                                            % initial x-position of semi axle 
IC_y1 = i_statey;                                                           % initial y-position of semi axle 
IC_theta0 = deg2rad(i_statet)+deg2rad(i_stateg);                            % initial yaw angle of tractor [deg]
IC_theta1 = deg2rad(i_statet);                                              % initial yaw angle of semi-trailer [deg]

x_1f = IC_x1+L_1f*cos(IC_theta1);                                           % Position of the king pin
y_1f = IC_y1+L_1f*sin(IC_theta1);

IC_x0 = x_1f-L_0b*cos(IC_theta0);                                           % Position of tractor drive axle    
IC_y0 = y_1f-L_0b*sin(IC_theta0);
newone = 1;
%__________________________________________________________________________
% moving obstacle initial data
MPSpointx = 160; % start point of moving obstacle (x)
MPSpointy = 25; % start point of moving obstacle (y)
t_MO = deg2rad(90); % orientation of moving obstacle
idMO = 1; % ID of moving obstacle
actfirst = 1; % ID of stop simulation when moving obstacle introduced
%__________________________________________________________________________
% assumption that the movement is rectilinear, so arbitray end point is
% selected
MPFpointx = 160; % final point of moving obstacle (x)
MPFpointy = MPSpointy+50; % final point of moving obstacle (y)
%__________________________________________________________________________
% load Parallelprimitives-theta0&180new;
load Parallelprimitives-theta0&180;

%__________________________________________________________________________
% If you want to run the file for tuning controller paramaters find below
% lines using for loop 
% SSfor = 4:0.1:8;
% LTfor = 2:0.1:3;
% trail = combvec(SSfor,LTfor);
% trailfor = trail';
% SS = trailfor(:,1);
% LT = trailfor(:,2);
% for k = 1:length(SS)
%     steer_sensforward =SS(k);%7.7
%     lookahead_timeforward = LT(k);%1.5
%     disp(steer_sensforward)
%     disp(lookahead_timeforward)
sim('checksimobs');
% end

%__________________________________________________________________________
% Plotting the results
% You can find the output of the controller from "ans" in the workspace
% For example find below lines for plotting results from controller and
% reference path
referencepath = rmmissing(ans.path);
referencepathx = referencepath(:,1);
referencepathy = referencepath(:,2);
figure(2)
plot(ans.SAV_pos(:,7),ans.SAV_pos(:,8),'linewidth',1.5);
hold on;
grid on;
plot(referencepathx,referencepathy,'--b','linewidth',1.5);
legend('Controller output','Reference path');
xlabel('X[m]');
ylabel('Y[m]');
%__________________________________________________________________________
% Plot the result
figure(3);
axis equal;
plot(polyshape([0,286,286,0],[0,0,200,200]));
grid on;
% obsc = 5;
hold on;
for i=1:obsc
    obsxtemp=obsx(i,:);
    obsytemp=obsy(i,:);
    obsxtemp=obsxtemp(obsxtemp~=0);
    obsytemp=obsytemp(obsytemp~=0);
    obsplot= polyshape(obsxtemp,obsytemp);
    plot(obsplot);
end
plot(referencepathx,referencepathy,'k','linewidth',1.5);                    % with respect to semi-trailer axle