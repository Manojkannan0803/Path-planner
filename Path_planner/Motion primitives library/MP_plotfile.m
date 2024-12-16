%% Motion primitives plot
% Author: Manojpriyadharson Kannan (Student number: 638628)
% CATALYST project: Automated docking maneuvering of an articulated
% vehicles in the presence of obstacles
% HAN-AR_HAN university of applied sciences_DPD_TNO
%% Checking the parameters from the workspace
checkti = 0==tini;      % Taking an initial orientation angle of semi-trailer from the workspace
checkgi = 0==gini;      % Taking an initial articulation angle from the workspace
checktif = 0==tfin;    % Taking a final orientation angle of semi-trailer from the workspace 
checkgif = 0==gfin;     % Taking a final articulation angle from the workspace
cond = all([checkti;checkgi;checktif;checkgif]);
index = find(cond);     % Taking index 
% Plotting the forward path
Xf = x_0(:,index);
Yf = y_0(:,index);
figure(1);
plot(Xf,Yf,'linewidth',1.5);
grid on;
hold on;
% Xr = -Xf;
% Yr = -Yf;
% plot(Xr,Yr);
Xa = Xf;
Ya = -Yf;
plot(Xa,Ya,'linewidth',1.5);
xlabel('x-position[m]');
ylabel('y-position[m]');
title('Parallel maneuver - motion primitives library');
Xcheck = xcheck(:,index);
Ycheck = flip(Yf);
plot(Xcheck,Ycheck,'linewidth',1.5);
Xcheck1 = Xcheck;
Ycheck1 = -Ycheck;
plot(Xcheck1,Ycheck1,'linewidth',1.5);
%% For plotting all the MP's
checkti = 0==tini;      % Taking an initial orientation angle of semi-trailer from the workspace
checkgi = -30==gini;      % Taking an initial articulation angle from the workspace
cond = all([checkti;checkgi]);
index = find(cond);     % Taking index 
% Plotting the forward path
Xf = x(:,index);
Yf = y(:,index);
figure(1);
plot(Xf,Yf);
grid on;
hold on;
Xr = -Xf;
Yr = -Yf;
plot(Xr,Yr);