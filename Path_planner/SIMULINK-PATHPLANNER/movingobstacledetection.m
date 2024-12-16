% Moving obstacle function. Main point is to predict the collision time and
% activate new path. Since in my work, vehicle should always be in constant
% speed. Below approach is followed, even for latter use with adjustments
% in the command the mentioned approach can be followed. As time is main
% factor in MO detection, it is calculated based on distance and velocity
% not considering the simulation time. For every point, if vehicle polygon
% check is conducted, it results in more computational cost.
% Defining the start and end point of the moving obstacle along with
% orientation angle. In this work, rectilinear moving obstacle is
% considered. In future, this has to be updated. As of now, straight path
% at a certain angle can be considered. Ego (TST) is travelling at a speed
% of 1m/s and target (shunt vehicle) is travelling at a speed of 0.5m/s

%__________________________________________________________________________
% load AV path
% load Dockingpathnew_MOcheck
pathdir = path(:,3);
pathx = path(:,1); % x - values
pathy = path(:,2); % y - values
paththeta = path(:,4); % theta-values
pathgamma = path(:,5); % gamma-values
checkdir = pathdir==1; % forward path
index = find(checkdir); 
forpathx = pathx(index);
forpathy = pathy(index);
forpaththeta = paththeta(index);
forpathgamma = pathgamma(index);
v_0 = 1; % velocity
x_c1=pchip(linspace(1,length(forpathx),length(forpathx)),forpathx,linspace(1,length(forpathx),length(forpathx)*10)); % interpolate the x- path
y_c1=pchip(linspace(1,length(forpathy),length(forpathy)),forpathy,linspace(1,length(forpathy),length(forpathy)*10)); % interpolate the y- path
t_c1=pchip(linspace(1,length(forpaththeta),length(forpaththeta)),forpaththeta,linspace(1,length(forpaththeta),length(forpaththeta)*10)); % interpolate the theta
g_c1=pchip(linspace(1,length(forpathgamma),length(forpathgamma)),forpathgamma,linspace(1,length(forpathgamma),length(forpathgamma)*10)); % interpolate the gamma

j = 10;
d_tot = 0;

while(d_tot<1.8)
    j=j+1;
    extx=interp1(linspace(1,length(x_c1),length(x_c1)),x_c1,linspace(1,length(x_c1)+(j),length(x_c1)+(j)),'linear','extrap');
    exty=interp1(linspace(1,length(y_c1),length(y_c1)),y_c1,linspace(1,length(y_c1)+(j),length(y_c1)+(j)),'linear','extrap');
    exttheta=interp1(linspace(1,length(t_c1),length(t_c1)),t_c1,linspace(1,length(t_c1)+(j),length(t_c1)+(j)),'linear','extrap');
    extgamma=interp1(linspace(1,length(g_c1),length(g_c1)),g_c1,linspace(1,length(g_c1)+(j),length(g_c1)+(j)),'linear','extrap');
    %     extx=extx';
%     exty=exty';
    d = hypot(diff(extx(length(x_c1):length(extx))),diff(exty(length(y_c1):length(exty))));
    d_tot = sum (d);
end
j=length((~isnan(extx)));

% path_input(:,1) = extx;
% path_input(:,2) = exty;
% path_input(:,3) = zeros(length(path_input(:,1)),1);
path_input((1:j),1)= extx(1,(1:j));
path_input((1:j),2) = exty(1,(1:j));% When directon changes simulate 
path_input(:,3) = zeros(1,length(path_input(:,1)));
path_input((1:j),4) = exttheta(1, (1:j));
path_input((1:j),5) = extgamma(1, (1:j));

clearvars -except path_input obsc obsx obsy
tictime = tic;
%__________________________________________________________________________
MPSpointx = 160; % start point of moving obstacle (x)
MPSpointy = 25; % start point of moving obstacle (y)
t_MO = deg2rad(90); % orientation of moving obstacle

%__________________________________________________________________________
% assumption that the movement is rectilinear, so arbitray end point is
% selected
MPFpointx = 160; % final point of moving obstacle (x)
MPFpointy = 85; % final point of moving obstacle (y)

%__________________________________________________________________________
% 
% Way points are decided based on the time dimension. Since the velocity is
% 0.5m/s, for every 1 sec moving obstacle covers half the distance
% compared to an ego (TST) vehicle. In that manner, data points are
% generated
length_MO = sqrt((MPSpointx-MPFpointx)^2+(MPSpointy-MPFpointy)^2);
waypoints = length_MO/0.5;
pathMO_x = linspace(MPSpointx,MPFpointx,waypoints+1);
pathMO_y = linspace(MPSpointy,MPFpointy,waypoints+1);

%__________________________________________________________________________
% initial check-area. Using the bounding box approach, area of collision is
% checked initially. If yes, next step would be continued or stop the
% simulation.
AV_x = path_input(:,1);
AV_y = path_input(:,2);
AV_theta = path_input(:,4);
AV_gamma = path_input(:,5);
%________________ Bounding box for AV
x_AV = AV_x(1);
y_AV = AV_y(1);
% change as per simulink output
t_AV = AV_theta(1);
g_AV = AV_gamma(1);
[checkx_ego,checky_ego] = boundingbox_AV(x_AV,y_AV,t_AV,g_AV);
x1 = checkx_ego(1);
y1 = checky_ego(1);
x2 = checkx_ego(2);
y2 = checky_ego(2);
x_AV = AV_x(end);
y_AV = AV_y(end);
t_AV = AV_theta(end);
g_AV = AV_gamma(end);
[checkx_ego,checky_ego] = boundingbox_AV(x_AV,y_AV,t_AV,g_AV);
x3 = checkx_ego(7);
y3 = checky_ego(7);
x4 = checkx_ego(8);
y4 = checky_ego(8);
% condition to create a box as path of AV won't be straight always. The
% whole area the vehicle trace has to be taken into an account.
if x3<x4 && t_AV == deg2rad(180)
    x3new=x3;
    x4new=x3new;
else
    x3new=x4;
    x4new=x3new;
end
check_y = y3>y2;
if check_y
    y2new = y3;
    y3new = y2new;
else
    y2new = y2;
    y3new = y2new;
end
check_yadd = y4>y1;
if check_yadd
    y1new = y1;
    y4new = y1new;
else
    y1new = y4;
    y4new = y1new;
end
X_AV = [x1 x2 x3new x4new]; % Bounding box x-dimensions
Y_AV = [y1new y2new y3new y4new]; % Bounding box y-dimensions

%_________________ Bounding box for MO
x_MO = pathMO_x(1);
y_MO = pathMO_y(1);
[checkx_MO,checky_MO] = boundingbox(x_MO,y_MO,t_MO);
x1MO = checkx_MO(1);
y1MO = checky_MO(1);
x2MO = checkx_MO(2);
y2MO = checky_MO(2);
x_MO = pathMO_x(end);
y_MO = pathMO_y(end);
[checkx_MO,checky_MO] = boundingbox(x_MO,y_MO,t_MO);
x3MO = checkx_MO(3);
y3MO = checky_MO(3);
x4MO = checkx_MO(4);
y4MO = checky_MO(4);
X_MO = [x1MO x4MO x3MO x2MO];
Y_MO = [y1MO y4MO y3MO y2MO];
lengthMO_check = sqrt((x1MO-x3MO)^2+(y1MO-y3MO)^2); % to know the 
% interval size
MOcheck = linspace(x1MO,x3MO,lengthMO_check+1);
MOcheck1 = linspace(y1MO,y3MO,lengthMO_check+1);
% check whether the bounding boxes intersect
in_area = InPolygon(MOcheck,MOcheck1,X_AV,Y_AV); 
in_area = any(in_area);
    if in_area == 1
        check = 1; % intersect
    else
        check = 0; % doesn't intersect
    end

%____________________ Plot the check - area
% If want, uncomment the below lines
figure();
plot([X_AV x1],[Y_AV y1new],'linewidth',1);
hold on;
plot([X_MO x1MO],[Y_MO y1MO],'linewidth',1);
plot(path_input(:,1),path_input(:,2),'--black','linewidth',1);
plot(AV_x(1),AV_y(1),'^','MarkerEdgeColor','blue','MarkerFaceColor','blue','MarkerSize',10);
plot(AV_x(end),AV_y(end),'v','MarkerEdgeColor','blue','MarkerFaceColor','blue','MarkerSize',10);
plot(pathMO_x,pathMO_y,'--m','linewidth',1.5)
plot(pathMO_x(1),pathMO_y(1),'^','MarkerEdgeColor','red','MarkerFaceColor','red','MarkerSize',10);
plot(pathMO_x(end),pathMO_y(end),'v','MarkerEdgeColor','red','MarkerFaceColor','red','MarkerSize',10);
grid on;
xlabel('x-position[m]');
ylabel('y-position[m]');
legend('Box for an Articulated Vehicle','Box for a Moving obstacle','path traced by semi-trailer','Start point of AV','End point of AV','path traced by shuntvehicle','Start point of SV','End point of SV');
title('check1 - Area');

%__________________________________________________________________________
% initial check-time. Using the circle approach, time/distance where the
% collision is likely to occur is checked. 
% with time, path has to be separated. As of now, since for a straight path
% discretization interval is calculated. Later stage, a logic to be created
% with respect to every time sec what is the x,y point of AV and MO to be
% decided

% AV_______________________________________________________________________
% Since we interpolate the path, to find for every 1 sec the path x and
% path y point to be found out. I chose this kind of approach, maybe if you
% could find a better logic, it would be nice.
a_AV = diff(AV_x); % check diff of AV
b_AV = diff(AV_y); % check diff of AV
c_AV = sqrt((a_AV).^2+(b_AV).^2); % calculate distance
c1_AV = diff(c_AV); % to find any fluctuations
cc1_AV = round(c1_AV);
condcc1_AV = any(cc1_AV);
if condcc1_AV==0
    discint_AV = c_AV(1); % to find discinterval
end
% find how much total time to reach final point
% finx_AV = AV_x(1)-AV_x(end);
% finy_AV = AV_y(1)-AV_y(end);
% cfin1_AV = sqrt((finx_AV)^2+(finy_AV)^2);
disc_AV = 1/discint_AV;
disc_AV = round(disc_AV);
indexone_AV = 1:disc_AV:length(AV_x);
newpathx_AV = AV_x(indexone_AV);
newpathy_AV = AV_y(indexone_AV);
newpatht_AV = AV_theta(indexone_AV);
newpathg_AV = AV_gamma(indexone_AV);
% Circle check_____________________________________________________________
% thetapAV = deg2rad(180); % theta angle change accordingly
for i = 1:length(newpathx_AV)
centerx = newpathx_AV(i)+4.2375*cos(newpatht_AV(i));  % theta angle change accordingly
centery = newpathy_AV(i)+4.2375*sin(newpatht_AV(i));  % theta angle change accordingly 
posAV = [centerx centery];
viscircles(posAV,9.5,'Color','k','linewidth',1); % radii to be 9.5 based on vehicle dimensions
hold on;
centerxMO = pathMO_x(i)+2.29*cos(t_MO);  % theta angle change accordingly
centeryMO = pathMO_y(i)+2.29*sin(t_MO);  % theta angle change accordingly 
posMO = [centerxMO centeryMO];
viscircles(posMO,4.8,'Color','b','linewidth',1);
[xout,yout] = circcirc(centerx,centery,9.5,centerxMO,centeryMO,4.8);
xcir = any(xout);
        ycir = any(yout);
%         
% t = linspace(0, 2*pi, 100);
% cir = @(r,ctr) [r*cos(t)+ctr(1); r*sin(t)+ctr(2)];                      % Circle Function        
% c1 = cir(9.5, [centerx;centery]);
% c2 = cir(4.8, [centerxMO;centeryMO]);
% in1 = find(inpolygon(c1(1,:), c1(2,:), c2(1,:), c2(2,:)));              % Circle #1 Points Inside Circle #2
% in2 = find(inpolygon(c2(1,:), c2(2,:), c1(1,:), c1(2,:)));              % Circle #2 Points Inside Circle #1
% in1 = any(in1);
% in2 = any(in2);
%         if in1 || in2
%      check1 = 1; % intersect
%             indexcirc = i; % to find index
%             break;
%         else
%             check1 = 0; % didn't intersect
%         end

        if xcir==1 && ycir==1
            check1 = 1; % intersect
            indexcirc = i; % to find index
            break;
        else
            check1 = 0; % didn't intersect
        end
end

%__________________________________________________________________________
% check 3: main collision point check using vehicle dimensions and figure
% out the clash point

indexsuspect = [indexcirc-5 indexcirc-4 indexcirc-3 indexcirc-2 indexcirc-1 indexcirc indexcirc+1 indexcirc+2 indexcirc+3 indexcirc+4 indexcirc+5 indexcirc+6 indexcirc+7 indexcirc+8 indexcirc+9 indexcirc+10 indexcirc+11];
for j = 1:length(indexsuspect)
     xp = newpathx_AV(indexsuspect(j)); % exact x position of Av
    yp = newpathy_AV(indexsuspect(j)); % exact y position of AV
    thetap = newpatht_AV(indexsuspect(j));
    gammap = newpathg_AV(indexsuspect(j));
     xp_MO = pathMO_x(indexsuspect(j)); % exact x position of MO
    yp_MO = pathMO_y(indexsuspect(j)); % exact y position of MO
    thetap_MO = t_MO; % as of now change the orientation of moving obstacle as per defined cases
    check3 = check3_dyn(xp,yp,thetap,gammap,xp_MO,yp_MO,thetap_MO);
    if check3==1
        check3a = 1; % indexsuspect intersect
        indexsuspect_final = j;
        break;
    else
        check3a = 0; % indexsuspect doesn't intersect
    end
end

% PLot the vehicle check __________________________________________________
figure(2);
plot(path_input(:,1),path_input(:,2),'black','linewidth',1.5);
hold on;
plot(AV_x(1),AV_y(1),'^','MarkerEdgeColor','blue','MarkerFaceColor','blue','MarkerSize',5);
plot(AV_x(end),AV_y(end),'v','MarkerEdgeColor','blue','MarkerFaceColor','blue','MarkerSize',5);
plot(pathMO_x,pathMO_y,'black-','linewidth',1.5)
plot(pathMO_x(1),pathMO_y(1),'^','MarkerEdgeColor','red','MarkerFaceColor','red','MarkerSize',5);
plot(pathMO_x(end),pathMO_y(end),'v','MarkerEdgeColor','red','MarkerFaceColor','red','MarkerSize',5);
grid on;
xlabel('x-position[m]');
ylabel('y-position[m]');
legend('path traced by semi-trailer','Start point of AV','End point of AV','path traced by shuntvehicle','Start point of SV','End point of SV');
title('check3 - Vehicle dimensions');
% If want, uncomment the below lines
xp = newpathx_AV(indexsuspect(indexsuspect_final));
yp = newpathy_AV(indexsuspect(indexsuspect_final));
thetap = newpatht_AV(indexsuspect(indexsuspect_final)); % as of now we know orientation is 180 but it has to be changed
gammap = newpathg_AV(indexsuspect(indexsuspect_final));
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
plot([xv1_1 xv2_1 xv3_1 xv4_1 xv1_1],[yv1_1 yv2_1 yv3_1 yv4_1 yv1_1],'k');
plot([xv1_0 xv2_0 xv3_0 xv4_0 xv1_0],[yv1_0 yv2_0 yv3_0 yv4_0 yv1_0],'b');
% for polyshape
asd = polyshape([xv1_1 xv2_1 xv3_1 xv4_1 xv1_1],[yv1_1 yv2_1 yv3_1 yv4_1 yv1_1]);
sad = polyshape([xv1_0 xv2_0 xv3_0 xv4_0 xv1_0],[yv1_0 yv2_0 yv3_0 yv4_0 yv1_0]);
plot(asd,'EdgeColor','black','FaceColor','yellow');
plot(sad,'EdgeColor','black','FaceColor','green');
%_____________________ plot_Moving obstacle
xp_MO = pathMO_x(indexsuspect(indexsuspect_final)); % exact x position of MO
yp_MO = pathMO_y(indexsuspect(indexsuspect_final)); % exact y position of MO
thetap_MO = t_MO;
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
plot([xc_MO xv1_1MO],[yc_MO yv1_1MO],'m');
moasd = polyshape([xc_MO xv1_1MO],[yc_MO yv1_1MO]);
plot(moasd,'EdgeColor','black','FaceColor','blue');

% finding the clash point
indexfinalAV = indexsuspect(indexsuspect_final); % find exact x and y point
clashx_AV = newpathx_AV(indexfinalAV);
clashy_AV = newpathy_AV(indexfinalAV);
clasht_AV = newpatht_AV(indexfinalAV);
indexfinalMO = indexsuspect(indexsuspect_final); % find exact x and y point
clashx_MO = pathMO_x(indexfinalMO);
clashy_MO = pathMO_y(indexfinalMO);
plot(clashx_AV,clashy_AV,'o','MarkerEdgeColor','r','MarkerFaceColor','r','MarkerSize',5);
plot(clashx_MO,clashy_MO,'o','MarkerEdgeColor','magenta','MarkerFaceColor','magenta','MarkerSize',5);
text(clashx_AV,47.5,'(193.93,40)');
text(180,clashy_MO,'(180,35.5)');
text(165,45,'Collision time = 56.0700 sec','Color','red','FontSize',12);
xlim([20 165]);
ylim([25 65]);
% find new start point of AV
startcheckx_AV = clashx_AV-15*cos(clasht_AV); % 15m gap is chosen for smooth transition
startchecky_AV = clashy_AV-15*sin(clasht_AV);
[~, indexAV]= min(abs(startcheckx_AV-AV_x));
startx_AV = AV_x(indexAV);
starty_AV = AV_y(indexAV);
starttheta_AV = AV_theta(indexAV); % change it as per theta value
startgamma_AV = AV_gamma(indexAV);
% for MO
startx_MO = clashx_MO-15*cos(thetap_MO);
starty_MO = clashy_MO-15*sin(thetap_MO);
length_MO1 = sqrt((startx_MO-MPFpointx)^2+(starty_MO-MPFpointy)^2);
waypoints1 = length_MO1/0.5;
pathMO_x1 = linspace(startx_MO,MPFpointx,waypoints1+1);
pathMO_y1 = linspace(starty_MO,MPFpointy,waypoints1+1);

% parallel primitives check
% for check purpose, load parallel primitives is done here
load Parallelprimitives-theta0&180new
PPfinaltini = rad2deg(PP_T(1,:));
% checkPP = rad2deg(starttheta_AV) ==PPfinaltini;
checkPP = 180 ==PPfinaltini;

indexPP = find(checkPP);
% for my convienence splitting the array and then combining it
A_PP = indexPP(1:10);
B_PP = indexPP(11:20);
C_PP = [A_PP(1) B_PP(1) A_PP(2) B_PP(2) A_PP(3) B_PP(3) A_PP(4) B_PP(4) A_PP(5) B_PP(5) A_PP(6) B_PP(6) A_PP(7) B_PP(7) A_PP(8) B_PP(8) A_PP(9) B_PP(9) A_PP(10) B_PP(10)];

for k = 1:length(C_PP)
    xp_clash = startx_AV+PP_X(:,C_PP(k));
    yp_clash = starty_AV+PP_Y(:,C_PP(k));
    thetap_clash = PP_T(:,C_PP(k));
    gammap_clash = PP_G(:,C_PP(k));
    for j = 1:length(xp_clash)
        xp = xp_clash(j);
        yp = yp_clash(j);
        thetap = thetap_clash(j);
        gammap = gammap_clash(j);
        xp_MO = pathMO_x1(j);
        yp_MO = pathMO_y1(j);
        thetap_MO = t_MO;
        checkfin = checkfinal_dyn(xp,yp,thetap,gammap,xp_MO,yp_MO,thetap_MO,obsc,obsx,obsy);
        if checkfin==1
            check_k = 1; % intersect
            break;
        else
            check_k = 0; % do not intersect
        end
    end
    if check_k ==0
        indexfree = k
        break;
    end
end

% plot the path and see
finalPATH_XPP = startx_AV+PP_X(:,C_PP(indexfree+2));
finalPATH_YPP = starty_AV+PP_Y(:,C_PP(indexfree+2));
finalPATH_THETAPP = PP_T(:,C_PP(indexfree+2));
finalPATH_GAMMAPP = PP_G(:,C_PP(indexfree+2));
% interpolate PP
finalPATH_XPP=(pchip(linspace(1,length(finalPATH_XPP),length(finalPATH_XPP)),finalPATH_XPP,linspace(1,length(finalPATH_XPP),length(finalPATH_XPP)*10)))'; % interpolate the x- path
finalPATH_YPP=(pchip(linspace(1,length(finalPATH_YPP),length(finalPATH_YPP)),finalPATH_YPP,linspace(1,length(finalPATH_YPP),length(finalPATH_YPP)*10)))'; % interpolate the y- path
finalPATH_THETAPP=(pchip(linspace(1,length(finalPATH_THETAPP),length(finalPATH_THETAPP)),finalPATH_THETAPP,linspace(1,length(finalPATH_THETAPP),length(finalPATH_THETAPP)*10)))'; % interpolate the theta
finalPATH_GAMMAPP=(pchip(linspace(1,length(finalPATH_GAMMAPP),length(finalPATH_GAMMAPP)),finalPATH_GAMMAPP,linspace(1,length(finalPATH_GAMMAPP),length(finalPATH_GAMMAPP)*10)))'; % interpolate the gamma

% Check
checkXPP = finalPATH_XPP(end)==AV_x;
checkyPP = ismember(round(finalPATH_YPP(end)),AV_y);
[num, indexPP]= min(abs(finalPATH_XPP(end)-AV_x));
PPend_X = AV_x(indexPP);
if PPend_X<finalPATH_XPP(end)
    finalPATH_X = [finalPATH_XPP;AV_x(indexPP+1:length(AV_x))];
    finalPATH_Y = [finalPATH_YPP;AV_y(indexPP+1:length(AV_y))];
    finalPATH_THETA = [finalPATH_THETAPP;AV_theta(indexPP+1:length(AV_theta))];
    finalPATH_GAMMA = [finalPATH_GAMMAPP;AV_gamma(indexPP+1:length(AV_gamma))];
else
    finalPATH_X = [finalPATH_XPP;AV_x(indexPP:length(AV_x))];
    finalPATH_Y = [finalPATH_YPP;AV_y(indexPP:length(AV_y))];
    finalPATH_THETA = [finalPATH_THETAPP;AV_theta(indexPP:length(AV_theta))];
    finalPATH_GAMMA = [finalPATH_GAMMAPP;AV_gamma(indexPP:length(AV_gamma))];
end
toctime=toc(tictime);
clearvars -except finalPATH_X finalPATH_Y finalPATH_THETA finalPATH_GAMMA pathMO_x1 pathMO_y1 thetap_MO toctime

% a_AV = diff(finalPATH_X); % check diff of AV
% b_AV = diff(finalPATH_Y); % check diff of AV
% c_AV = sqrt((a_AV).^2+(b_AV).^2); % calculate distance
% c1_AV = diff(c_AV); % to find any fluctuations
% cc1_AV = round(c1_AV);
% condcc1_AV = any(cc1_AV);
% if condcc1_AV==0
%     discint_AV = c_AV(1); % to find discinterval
% end
% % find how much total time to reach final point
% % finx_AV = AV_x(1)-AV_x(end);
% % finy_AV = AV_y(1)-AV_y(end);
% % cfin1_AV = sqrt((finx_AV)^2+(finy_AV)^2);
% disc_AV = 1/discint_AV;
% disc_AV = round(disc_AV);
% indexone_AV = 1:disc_AV:length(finalPATH_X);
% finalPATH_X = finalPATH_X(indexone_AV);
% finalPATH_Y = finalPATH_Y(indexone_AV);
% finalPATH_THETA = finalPATH_THETA(indexone_AV);
% finalPATH_GAMMA = finalPATH_GAMMA(indexone_AV);


disMO = length(finalPATH_X);
pathMO_x2 = pathMO_x1';
pathMO_y2= pathMO_y1';
% disMO = (sqrt((pathMO_x1(1)-pathMO_x1(end))^2+(pathMO_y1(1)-pathMO_y1(end))^2))/disMO;
as = pathMO_x2(1); as1 = pathMO_x2(end);
as3 = pathMO_y2(1); as4 = pathMO_y2(end);
pathMO_x1 = (linspace(as,as1,disMO));
pathMO_y1 = (linspace(as3,as4,disMO));
% Calculation for the motion of the truck
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
    
     oh_0f = 1.5; % Frontal foverhang of the truck [m
    oh_0b = 0.94; % Distance from the drive axle to the end of the tractor [m]
    lv12_0 = hypot((oh_0b),(w_1/2)); % Length of Vector 1 and 2
    lv34_0 = hypot((w_1/2),(L_0f+oh_0f)); % Length of Vector 3 and 4
    
    av1 = (180/pi*finalPATH_THETA)+90+atand(oh_1b/(w_1/2)); % Angle of Vector 1
    av2 = (180/pi*finalPATH_THETA)-90-atand(oh_1b/(w_1/2)); % Angle of Vector 2
    av3 = (180/pi*finalPATH_THETA)-atand((w_1/2)/oh_1f); % Angle of Vector 3
    av4 = (180/pi*finalPATH_THETA)+atand((w_1/2)/oh_1f); % Angle of Vector 4

% Finding the actual points
    xv1_1 = finalPATH_X+lv12_1*cosd(av1);
    yv1_1 = finalPATH_Y+lv12_1*sind(av1);
    xv2_1 = finalPATH_X+lv12_1*cosd(av2);
    yv2_1 = finalPATH_Y+lv12_1*sind(av2);
    xv3_1 = finalPATH_X+lv34_1*cosd(av3);
    yv3_1 = finalPATH_Y+lv34_1*sind(av3);
    xv4_1 = finalPATH_X+lv34_1*cosd(av4);
    yv4_1 = finalPATH_Y+lv34_1*sind(av4);
x_1f = finalPATH_X+L_1f*cos(finalPATH_THETA); % Position of the king pin
    y_1f = finalPATH_Y+L_1f*sin(finalPATH_THETA);
    theta_0 = finalPATH_THETA+finalPATH_GAMMA; % Orientation angle of tractor [rad]
    x_0 = x_1f-L_0b*cos(theta_0); % Position of center of the driven axle
    y_0 = y_1f-L_0b*sin(theta_0);
    
    x_0f = x_0+L_0f*cos(theta_0);   % position of tractor front axle
    y_0f = y_0+L_0f*sin(theta_0);
    
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
    
      xc=[xv1_1 xv2_1 xv3_1 xv4_1 xv1_0 xv2_0 xv3_0 xv4_0];
    yc=[yv1_1 yv2_1 yv3_1 yv4_1 yv1_0 yv2_0 yv3_0 yv4_0];
    
    %% MO
    
    L_1fMO = 5.600;         % Wheelbase of moving obstacle [m] 
oh_1bMO = 2.440;          % Longitudinal distance from the rear axle to the end of the vehicle [m] 
oh_1fMO = L_1fMO+1.420;   % Longitudinal distance from the rear axle to the front of the vehicle [m]
w_1MO   = 2.854;         % Width of moving obstacle [m]
% Now  we will create vectors to each corner of the vehicle
% Length of vector 1, 2 is the same and 3,4 is the same
lv12_1MO = hypot(oh_1bMO,(w_1MO/2)); % Length of Vector 1 and 2
lv34_1MO = hypot((w_1MO/2),oh_1fMO); % Length of Vector 3 and 4
av1MO = (180/pi*thetap_MO)+90+atand(oh_1bMO/(w_1MO/2)); % Angle of Vector 1
av2MO = (180/pi*thetap_MO)-90-atand(oh_1bMO/(w_1MO/2)); % Angle of Vector 2
av3MO = (180/pi*thetap_MO)-atand((w_1MO/2)/oh_1fMO); % Angle of Vector 3
av4MO = (180/pi*thetap_MO)+atand((w_1MO/2)/oh_1fMO); % Angle of Vector 4

xv1_1MO = pathMO_x1'+lv12_1MO*cosd(av1MO);
yv1_1MO = pathMO_y1'+lv12_1MO*sind(av1MO);
xv2_1MO = pathMO_x1'+lv12_1MO*cosd(av2MO);
yv2_1MO = pathMO_y1'+lv12_1MO*sind(av2MO);
xv3_1MO = pathMO_x1'+lv34_1MO*cosd(av3MO);
yv3_1MO = pathMO_y1'+lv34_1MO*sind(av3MO);
xv4_1MO = pathMO_x1'+lv34_1MO*cosd(av4MO);
yv4_1MO = pathMO_y1'+lv34_1MO*sind(av4MO);

xcMO=[xv1_1MO xv2_1MO xv3_1MO xv4_1MO];
ycMO=[yv1_1MO yv2_1MO yv3_1MO yv4_1MO];

figure(3);
axis equal;
myVideo = VideoWriter('myVideoFile_case1'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)

for i=1:length(xc(:,1))
    clf
        trailer= polyshape([xc(i,1);xc(i,2);xc(i,3);xc(i,4)],[yc(i,1);yc(i,2);yc(i,3);yc(i,4)]);
        tractor= polyshape([xc(i,5);xc(i,6);xc(i,7);xc(i,8)],[yc(i,5);yc(i,6);yc(i,7);yc(i,8)]);
        MO = polyshape([xcMO(i,1);xcMO(i,2);xcMO(i,3);xcMO(i,4)],[ycMO(i,1);ycMO(i,2);ycMO(i,3);ycMO(i,4)]);
        hold on
        plot(tractor,'EdgeColor','black','FaceColor','yellow');
        plot(trailer,'EdgeColor','black','FaceColor','green');
        plot(MO,'EdgeColor','black','FaceColor','red');
        hold on
        xlabel('x-position [m]');
        ylabel('y-position [m]');
        xlim([30 180]);
        ylim([25 70]);
        title('Simulation');
        drawnow();
%         delete(a)
%         delete(b)
%         delete(c)
%         Cdad(i) = getframe(figure(2));
pause(0.01) %Pause and grab frame
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
close(myVideo)