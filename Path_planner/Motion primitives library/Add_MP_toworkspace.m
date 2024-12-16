%% Motion primitives addition to the workspace for the path planner
% Author: Manojpriyadharson Kannan (Student number: 638628)
% CATALYST project: Automated docking maneuvering of an articulated
% vehicles in the presence of obstacles
% HAN-AR_HAN university of applied sciences_DPD_TNO
% This mat.file has to be run before the main path planner.mat file.
%% Clear the workspace
clearvars;
clc;
%% Specifying the discretization intervals
thetad = 0:6:354;                   % Creating discretized orientation angle of semi-trailer
gammad = [-30 0 30];                % Creating discretized articulation angle of vehicle
% gammad = [-30 -24 -21 0 21 24 30];                % Creating discretized articulation angle of vehicle
count = 0;                          % Counting the number of motion primitives created
%% Loading the motion primitives (Initial theta = 0)
for j= 1 : length(gammad)
    folder = strcat('FINAL_MP_PP_21\thetatrailer_',num2str(0),'_gamma_',num2str(gammad(j)));
%     folder = strcat('FINAL_MP_PP_21-Copy\thetatrailer_',num2str(0),'_gamma_',num2str(gammad(j)));
    for k=1:length(thetad)
        for m=1:length(gammad)
            file = strcat(folder,'\','mp_',num2str(thetad(k)),'_',num2str(gammad(m)),'.mat');
            % Check if the motion primitve exists
            if exist(file,'file') 
                load(file);
                count=count+1;
                x_0(:,count)    =   out.STATES(:,2);
                y_0(:,count)    =   out.STATES(:,3);
                t_0(:,count)    =   out.STATES(:,4);
                g_0(:,count)    =   out.STATES(:,5);
                d_0(:,count)    =   out.STATES(:,6);  
                time_0(1,count) =   out.PARAMETERS(1,2);
            end
        end
    end
end
%% Replacing the straight motion primitive
% Finding the straight primitives in the MP bank
checkti     =   0==t_0(1,:);    % In orientation angle of semi-trailer section
checkgi     =   0==g_0(1,:);    % In articulation angle section
checktf     =   0==t_0(21,:);   % In orientation angle of semi-trailer section
checkgf     =   0==g_0(21,:);   % In articulation angle of section
cond        =   all([checkti;checkgi;checktf;checkgf]);
index       =   find(cond);
% Deleting the straight primitives
x_0(:,index)    =   [];
y_0(:,index)    =   [];
t_0(:,index)    =   [];
g_0(:,index)    =   [];
d_0(:,index)    =   [];
time_0(:,index) =   [];
%% Adding more straight motion primitives in the MP library
% Creating straight motion primitive of length 0.2m
xgen    =   linspace(0,0.2,21);
ygen    =   zeros(1,21);
count   =   length(x_0(1,:));
count   =   count+1;
x_0(:,count)    =   xgen(1,:)';
y_0(:,count)    =   ygen';
zer     =   zeros(21,1);
g_0(:,count)    =   zer;
d_0(:,count)    =   zer;
t_0(:,count)    =   zer;
time_0(count)   =   0.2;
% Creating straight motion primitive of length 0.3m
xgen    =   linspace(0,0.3,21);
ygen    =   zeros(1,21);
count   =   length(x_0(1,:));
count   =   count+1;
x_0(:,count)    =   xgen(1,:)';
y_0(:,count)    =   ygen';
zer     =   zeros(21,1);
g_0(:,count)    =   zer;
d_0(:,count)    =   zer;
t_0(:,count)    =   zer;
time_0(count)   =   0.3;
% %% addition extra
% % Creating straight motion primitive of length 5m
% xgen    =   linspace(0,0.6,21);
% ygen    =   zeros(1,21);
% count   =   length(x_0(1,:));
% count   =   count+1;
% x_0(:,count)    =   xgen(1,:)';
% y_0(:,count)    =   ygen';
% zer     =   zeros(21,1);
% g_0(:,count)    =   zer;
% d_0(:,count)    =   zer;
% t_0(:,count)    =   zer;
% time_0(count)   =   0.6;
% % Creating straight motion primitive of length 5m
% xgen    =   linspace(0,1.5,21);
% ygen    =   zeros(1,21);
% count   =   length(x_0(1,:));
% count   =   count+1;
% x_0(:,count)    =   xgen(1,:)';
% y_0(:,count)    =   ygen';
% zer     =   zeros(21,1);
% g_0(:,count)    =   zer;
% d_0(:,count)    =   zer;
% t_0(:,count)    =   zer;
% time_0(count)   =   1.5;
% % Creating straight motion primitive of length 5m
% xgen    =   linspace(0,1.8,21);
% ygen    =   zeros(1,21);
% count   =   length(x_0(1,:));
% count   =   count+1;
% x_0(:,count)    =   xgen(1,:)';
% y_0(:,count)    =   ygen';
% zer     =   zeros(21,1);
% g_0(:,count)    =   zer;
% d_0(:,count)    =   zer;
% t_0(:,count)    =   zer;
% time_0(count)   =   1.8; 
% % Creating straight motion primitive of length 5m
% xgen    =   linspace(0,2,21);
% ygen    =   zeros(1,21);
% count   =   length(x_0(1,:));
% count   =   count+1;
% x_0(:,count)    =   xgen(1,:)';
% y_0(:,count)    =   ygen';
% zer     =   zeros(21,1);
% g_0(:,count)    =   zer;
% d_0(:,count)    =   zer;
% t_0(:,count)    =   zer;
% time_0(count)   =   2; 
%%
% Creating straight motion primitive of length 5m
xgen    =   linspace(0,5,21);
ygen    =   zeros(1,21);
count   =   length(x_0(1,:));
count   =   count+1;
x_0(:,count)    =   xgen(1,:)';
y_0(:,count)    =   ygen';
zer     =   zeros(21,1);
g_0(:,count)    =   zer;
d_0(:,count)    =   zer;
t_0(:,count)    =   zer;
time_0(count)   =   5; 
% Creating straight motion primitive of length 8m
xgen    =   linspace(0,8,21);
ygen    =   zeros(1,21);
count   =   length(x_0(1,:));
count   =   count+1;
x_0(:,count)    =   xgen(1,:)';
y_0(:,count)    =   ygen';
zer     =   zeros(21,1);
g_0(:,count)    =   zer;
d_0(:,count)    =   zer;
t_0(:,count)    =   zer;
time_0(count)   =   8; 
% Creating straight motion primitive of length 10m
xgen    =   linspace(0,10,21);
ygen    =   zeros(1,21);
count   =   length(x_0(1,:));
count   =   count+1;
x_0(:,count)    =   xgen(1,:)';
y_0(:,count)    =   ygen';
zer     =   zeros(21,1);
g_0(:,count)    =   zer;
d_0(:,count)    =   zer;
t_0(:,count)    =   zer;
time_0(count)   =   10; 
% Creating straight motion primitive of length 30m
xgen    =   linspace(0,30,21);
ygen    =   zeros(1,21);
count   =   length(x_0(1,:));
count   =   count+1;
x_0(:,count)    =   xgen(1,:)';
y_0(:,count)    =   ygen';
zer     =   zeros(21,1);
g_0(:,count)    =   zer;
d_0(:,count)    =   zer;
t_0(:,count)    =   zer;
time_0(count)   =   30;
%% Final motion primitives library
% Adding the motion primitives with initial theta=0 into the final motion primitive bank 
x   =   x_0;
y   =   y_0;
t   =   t_0;
g   =   g_0;
d   =   d_0;
timeAV    =   time_0;
% After the creation of the motion primitives from the initial vehicle state of 
% thetad = 0, the motion primitive from other discretized orientation angles have to be created. 
% This can be achieved by rotating the motion primitives generated from thetad = 0 by multiplying 
% it with a rotation matrix and thus, the sets of motion primitives are created for the other initial orientation angles are created.
count_x     =   length(x_0(1,:));   % Taking the length of the x matrix
for j   =   2:length(thetad)        % For all discreet thetad's
    % rot is the rotational matrix used to rotate the motion primitives
    rot     =   [cosd(-thetad(j)) -sind(-thetad(j)); sind(-thetad(j)) cosd(-thetad(j))];   
    for i   =   1:length(x_0(1,:))
        temxy   =   [x_0(:,i) y_0(:,i)]*rot;
        count_x     =   count_x+1;  % Updating Count
        % Adding the rotated MP to the MP bank.
        x(:,count_x)    =   temxy(:,1);
        y(:,count_x)    =   temxy(:,2);
        t(:,count_x)    =   t_0(:,i)+deg2rad(thetad(j));
        g(:,count_x)    =   g_0(:,i);
        d(:,count_x)    =   d_0(:,i);
        timeAV(count_x)   =   time_0(i);        
    end
end
%% Initializing state check vectors (tini,tfin,gini,gfin,xfin,yfin). This 
% done so that we can identify the motion primitives by their initial and 
% final states. 
tini    =   t(1,:);                 % Initial orientation angle of semi-trailer [rad]
tfin    =   t(length(t(:,1)),:);    % Final orientation angle of semi-trailer [rad]    
gini    =   g(1,:);                 % Initial articulation angle [rad]
gfin    =   g(length(g(:,1)),:);    % Final articulation angle [rad]
xfin    =   x(length(x(:,1)),:);    % Initial x-position [m]
yfin    =   y(length(y(:,1)),:);    % Final x-position [m]
tini    =   rad2deg(tini);          % Initial orientation angle of semi-trailer [deg]
tfin    =   rad2deg(tfin);          % Final orientation angle of semi-trailer [deg]    
gini    =   rad2deg(gini);          % Initial articulation angle [deg]
gfin    =   rad2deg(gfin);          % Final articulation angle [deg]
% Rounding off the values as rad to deg conversion will leave decimals 
tini    =   round(tini);            
tfin    =   round(tfin);
gini    =   round(gini);
gfin    =   round(gfin);
% Changing the initial and final theta values of the MP in the 4th
% quadarant in the 360 degrees format.
changef     =   linspace(-90,0,16); % can also be represented as initial:intervalcount:final
changef(length(changef))= 360;
changet     =   linspace(270,360,16);
changet(length(changet))= 0;
for i   =   1:length(changef)
    changer     =   tini==changef(i);
    changer     =~  changer;
    tini        =   times(tini,changer);
    changer     =~  changer;
    changer     =   changer*changet(i);
    tini        =   tini+changer;
    changer     =   tfin==changef(i);
    changer     =~  changer;
    tfin        =   times(tfin,changer);
    changer     =~  changer;
    changer     =   changer*changet(i);
    tfin        =   tfin+changer;
end
% Changing the final theta whose values are above 360; instead it should be
% in 0 to 90 degrees format
cond1 = tfin>360;
indextfin = find(cond1);
tfin(indextfin) = tfin(indextfin)-360;  % Converting it to 0-90 degrees format
cond2 = t>6.28319;
indext = find(cond2);
t(indext) = t(indext)-6.28319;
% Clearing unrequired variables.
clear changef changet changer file folder i j out k m count count1 xgen ygen xygen zer rot;
clear checkgf checkgi checktf checkti cond count_0 count_x d_0 gammad index;
clear temxy thetad time_0 g_0 t_0 cond1 cond2 indext indextfin;