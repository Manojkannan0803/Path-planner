function check_area = checkareafirststep(pathMO_x,pathMO_y,t_MO,AV_x,AV_y,AV_theta,AV_gamma)
coder.extrinsic('InPolygon');
in_areacheck = false(1,1);
%__________________________________________________________________________
% initial check-area. Using the bounding box approach, area of collision is
% checked initially. If yes, next step would be continued or stop the
% detection module. The code has been written as per my convienence, if the
% reader feels he/she can shorten it "always WELCOME" without altering the
% concept.
%________________ Bounding box for articulated vehicle with respect to
%generated reference path 
x_AV = AV_x(1);
y_AV = AV_y(1);
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
in_areacheck = any(in_area);
    if in_areacheck == 1
        check_area = 1; % intersect
    else
        check_area = 0; % doesn't intersect
    end
end

