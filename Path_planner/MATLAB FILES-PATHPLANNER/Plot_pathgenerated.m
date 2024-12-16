%% Path planning algorithm - plot results
% Author: Manojpriyadharson Kannan (Student number: 638628)
% CATALYST project: Automated docking maneuvering of an articulated
% vehicles in the presence of obstacles
% HAN-AR_HAN university of applied sciences_DPD_TNO
%% Obstacle map
figure(2);
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

figure(3);
plot(polyshape([0,286,286,0],[0,0,200,200]));
grid on;
hold on;
for i=1:obsc
    obsxtemp=obsx(i,:);
    obsytemp=obsy(i,:);
    obsxtemp=obsxtemp(obsxtemp~=0);
    obsytemp=obsytemp(obsytemp~=0);
    obsplot= polyshape(obsxtemp,obsytemp);
    plot(obsplot);
end
%% Plotting the results
i=0;
xc=[]; % creating an x - array
yc=[]; % creating an y - array
pathx=[]; % storing x-path in an array
pathy=[]; % storing y-path in an array
indexp = c(cc); % closed list count index
indexmp =   t_pred(indexp); % By this we will get the MP index that is used to get the needed configuration
thetapvector1 = []; % To visualize the change in orientation angle of semi-trailer
gammapvector1 = []; % To visualize the change in articulation angle
while (t_x(indexp)~= i_state.x || t_y(indexp) ~= i_state.y)&& ~isnan(indexmp)
    % Extract information from the tree
    xp=x(:,indexmp);
    yp=y(:,indexmp);
    thetap=t(:,indexmp);
    gammap=g(:,indexmp);
    dirp=t_dir(indexp);
    pred_index= t_predxy(indexp);
    if dirp==1
        xp= xp+t_xa(pred_index);
        yp= yp+t_ya(pred_index);
        
        figure(2);
        plot(xp,yp,'r');
        drawnow
   
    else
        xp=xp-xp(length(xp))+t_xa(pred_index);
        yp=yp-yp(length(yp))+t_ya(pred_index);
        xp=flip(xp);
        yp=flip(yp);
        thetap=flip(thetap);
        gammap=flip(gammap);
        
        figure(2);
        plot(xp,yp,'-r');
        drawnow
    
    end
    % Plot the path
    figure(2);
    plot(xp,yp,'r','linewidth',1.5);
    drawnow
    
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

    % Finding the actual points
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
    plot(x_0,y_0,'k','linewidth',1.5);
%     plot(x_1f,y_1f,'green','linewidth',1.5);
    x_0f = x_0+L_0f*cos(theta_0);   % position of tractor front axle
    y_0f = y_0+L_0f*sin(theta_0);
%     plot(x_0f,y_0f,'b','linewidth',1.5);
%     legend('Tractor path');
    % Angle of the vectors
    av1 = (180/pi*(theta_0))+90+atand(oh_0b/(w_1/2)); % Angle of Vector 1
    av2 = (180/pi*(theta_0))-90-atand(oh_0b/(w_1/2)); % Angle of Vector 2
    av3 = (180/pi*(theta_0))-atand((w_1/2)/(oh_0f+L_0f)); % Angle of Vector 3
    av4 = (180/pi*(theta_0))+atand((w_1/2)/(oh_0f+L_0f)); % Angle of Vector 4

    % Finding the actual points
    xv1_0 = x_0+lv12_0*cosd(av1);
    yv1_0 = y_0+lv12_0*sind(av1);
    xv2_0 = x_0+lv12_0*cosd(av2);
    yv2_0 = y_0+lv12_0*sind(av2);
    xv3_0 = x_0+lv34_0*cosd(av3);
    yv3_0 = y_0+lv34_0*sind(av3);
    xv4_0 = x_0+lv34_0*cosd(av4);
    yv4_0 = y_0+lv34_0*sind(av4);
        
    xc=[[xv1_1 xv2_1 xv3_1 xv4_1 xv1_0 xv2_0 xv3_0 xv4_0];xc];
    yc=[[yv1_1 yv2_1 yv3_1 yv4_1 yv1_0 yv2_0 yv3_0 yv4_0];yc];

    indexp=t_predxy(indexp);
    indexmp      =   t_pred(indexp);
    thetapvector1 = [thetapvector1; thetap];
    gammapvector1 = [gammapvector1; gammap];
end

scatter(i_state.x,i_state.y,'r');
scatter(f_state.x,f_state.y,'b');
xlabel('x-position [m]');
ylabel('y-position [m]');
xlim([0 285]);
ylim([0 200]);
% legend('Semi-trailer path','Tractor path');
title(strcat('X_i=',num2str(i_state.x),', Y_i=',num2str(i_state.y),', \theta_i=',num2str(i_state.t),', X_f=',num2str(f_state.x),', Y_f=',num2str(f_state.y),', \theta_f=',num2str(f_state.t),', XError=',num2str(abs(f_state.x-cur.xa)),', YError=',num2str(abs(f_state.y-cur.ya)),', Comp Time=',num2str(toctime)));
% % Moving obstacle plot
% IM_X = 50;  % Initial x_position of the moving obstacle [m]
% IM_Y = 40;  % Initial y_position of the moving obstacle [m]
% IM_T = 0;   % Initial orientation angle of the moving obstacle [m]
% FM_X = 80;  % Final x_position of the moving obstacle [m]
% FM_Y = 40;  % Final y_position of the moving obstacle [m]
% FM_T = 0;   % Final orientation angle of the moving obstacle [m]
% % Pre-defined path (reference path)
% xpMO = linspace(IM_X,FM_X,404);
% ypMO = linspace(IM_Y,FM_Y,404);
% xpMO = xpMO';
% ypMO = ypMO';
% thetapMO = linspace(IM_T,FM_T,404);  
% thetapMO = thetapMO';
% thetapMO = deg2rad(thetapMO);
% % figure(3)
% % plot(xpMO,ypMO);
% % drawnow;
% % hold on;
% % vehicle dimensions
% xcMO=[]; % creating an x - array
% ycMO=[]; % creating an y - array
% % Calculation for the motion of the streetdrone
% L_1fMO = 1.685; % Wheelbase of streetdrone 
% oh_1bMO = 0.3;       % Longitudinal distance from the rear axle to the end of the vehicle [m] 
% oh_1fMO = L_1fMO+0.3;  % Longitudinal distance from the rear axle to the front of the vehicle [m]
% w_1MO   = 1.19;     % Width of streetdrone [m]
% % Now  we will create vectors to each corner of the vehicle
% % Length of vector 1, 2 is the same and 3,4 is the same
% lv12_1MO = hypot(oh_1bMO,(w_1MO/2)); % Length of Vector 1 and 2
% lv34_1MO = hypot((w_1MO/2),oh_1fMO); % Length of Vector 3 and 4
% 
% % For streetdrone
% % Angle of the vectors
% av1MO = (180/pi*thetapMO)+90+atand(oh_1bMO/(w_1MO/2)); % Angle of Vector 1
% av2MO = (180/pi*thetapMO)-90-atand(oh_1bMO/(w_1MO/2)); % Angle of Vector 2
% av3MO = (180/pi*thetapMO)-atand((w_1MO/2)/oh_1fMO); % Angle of Vector 3
% av4MO = (180/pi*thetapMO)+atand((w_1MO/2)/oh_1fMO); % Angle of Vector 4
% 
% % Finding the actual points
% xv1_1MO = xpMO+lv12_1MO*cosd(av1MO);
% yv1_1MO = ypMO+lv12_1MO*sind(av1MO);
% xv2_1MO = xpMO+lv12_1MO*cosd(av2MO);
% yv2_1MO = ypMO+lv12_1MO*sind(av2MO);
% xv3_1MO = xpMO+lv34_1MO*cosd(av3MO);
% yv3_1MO = ypMO+lv34_1MO*sind(av3MO);
% xv4_1MO = xpMO+lv34_1MO*cosd(av4MO);
% yv4_1MO = ypMO+lv34_1MO*sind(av4MO);
% 
% xcMO=[[xv1_1MO xv2_1MO xv3_1MO xv4_1MO];xcMO];
% ycMO=[[yv1_1MO yv2_1MO yv3_1MO yv4_1MO];ycMO];
%% Plotting the truck movement
figure(3);
axis equal;
for i=1:length(xc(:,1))
%     clf
        trailer= polyshape([xc(i,1);xc(i,2);xc(i,3);xc(i,4)],[yc(i,1);yc(i,2);yc(i,3);yc(i,4)]);
        tractor= polyshape([xc(i,5);xc(i,6);xc(i,7);xc(i,8)],[yc(i,5);yc(i,6);yc(i,7);yc(i,8)]);
%         MO = polyshape([xcMO(i,1);xcMO(i,2);xcMO(i,3);xcMO(i,4)],[ycMO(i,1);ycMO(i,2);ycMO(i,3);ycMO(i,4)]);
        hold on
        plot(tractor,'EdgeColor','b','FaceColor','blue','FaceAlpha',0.02);
        plot(trailer,'EdgeColor','g','FaceColor', 'g','FaceAlpha',0.02);
        
%         plot(tractor,'EdgeColor','green','linewidth',1);
%         plot(trailer,'EdgeColor',[17 17 17]/255,'linewidth',1);
%           plot(tractor,'EdgeColor','green','linewidth',1);
%         plot(trailer,'EdgeColor','k','linewidth',1);
%         plot(MO,'EdgeColor','black','FaceColor','red');
        hold on
        xlabel('x-position [m]');
        ylabel('y-position [m]');
        xlim([0 286]);
        ylim([0 200]);
        title('Simulation');
        drawnow();
%         delete(a)
%         delete(b)
%         delete(c)
%         Cdad(i) = getframe(figure(2));
end

% video = VideoWriter('Simulation6.avi','Uncompressed AVI');
% video.FrameRate = 60; 
% % video.Quality = 95;
% open(video);
% for i=1:length(Cdad)
%     frame = Cdad(i);
%     writeVideo(video,Cdad);
% end
% close(video);