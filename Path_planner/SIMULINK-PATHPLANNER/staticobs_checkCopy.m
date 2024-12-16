% To check whether the vehicle hits the static obstacles
function  check = staticobs_checkCopy(cur,xp,yp,thetap,gammap,dirp,obsc,obsx,obsy,L_0b,L_0f,L_1f,lv12_0,lv34_0,oh_0b,oh_0f,oh_1b,oh_1f,w_1,lv12_1,lv34_1)
check=1;     % Collision check variable
%_______________________________________________________________________
coder.extrinsic('InPolygon');
condobssta = false(1,1);    % initialize as logical array
%_________________________________________________________________________
% global obsc;
% We have the position of the trailer axle(xp,yp) and the orientation of the
% trailer(thetap) and the articulation angle (gammap). So from all this
% information we can find all the four corners of the trailer and the four
% corners of the tractor and see if these corners are hitting the obstacle.
% L_1f = 8.475; % Wheelbase of semitrailer [m]
% L_0f = 3.8; % Wheel base of the tracotr [m]
% L_0b = 0.3; % Distance of 1st king-pin to tractor drive axle [m]
% oh_1b = 5;       % Longitudinal distance from the trailer axle to the end of the trailer [m] (60% of the WB)
% oh_1f = L_1f+1;  % Longitudinal distance from the trailer axle to the front of the trailer [m]
% w_1   = 2.5;     % Width of a trailer [m]
% Now  we will create vectors to each corner of the trailer
% Length of vector 1, 2 is the same and 3,4 is the same
% lv12_1 = hypot(oh_1b,(w_1/2)); % Length of Vector 1 and 2
% lv34_1 = hypot((w_1/2),oh_1f); % Length of Vector 3 and 4
% if dirp == 0
%     thetap=thetap+deg2rad(180);
%     thetap=wrapTo360(thetap);
%     gammap= -gammap;
% end

% Now we have to find the corners of the tractor. The tractor and the
% trailer have one point the same and fixed which is the king pin. So first
% we find the position of the king pin and then the rest of the corners.

% oh_0f = 1.5; % Frontal foverhang of the truck [m
% oh_0b = 0.94; % Distance from the drive axle to the end of the tractor [m]
% lv12_0 = hypot((oh_0b),(w_1/2)); % Length of Vector 1 and 2
% lv34_0 = hypot((w_1/2),(L_0f+oh_0f)); % Length of Vector 3 and 4

% For Trailer
% Angle of the vectors
av1 = (180/pi*thetap)+90+atand(oh_1b/(w_1/2)); % Angle of Vector 1
av2 = (180/pi*thetap)-90-atand(oh_1b/(w_1/2)); % Angle of Vector 2
av3 = (180/pi*thetap)-atand((w_1/2)/oh_1f); % Angle of Vector 3
av4 = (180/pi*thetap)+atand((w_1/2)/oh_1f); % Angle of Vector 4

% Finding the actual points
xv1_1 = xp+lv12_1*cosd(av1)+cur.xa;
yv1_1 = yp+lv12_1*sind(av1)+cur.ya;
xv2_1 = xp+lv12_1*cosd(av2)+cur.xa;
yv2_1 = yp+lv12_1*sind(av2)+cur.ya;
xv3_1 = xp+lv34_1*cosd(av3)+cur.xa;
yv3_1 = yp+lv34_1*sind(av3)+cur.ya;
xv4_1 = xp+lv34_1*cosd(av4)+cur.xa;
yv4_1 = yp+lv34_1*sind(av4)+cur.ya;

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

% Finding the actual points
xv1_0 = x_0+lv12_0*cosd(av1)+cur.xa;
yv1_0 = y_0+lv12_0*sind(av1)+cur.ya;
xv2_0 = x_0+lv12_0*cosd(av2)+cur.xa;
yv2_0 = y_0+lv12_0*sind(av2)+cur.ya;
xv3_0 = x_0+lv34_0*cosd(av3)+cur.xa;
yv3_0 = y_0+lv34_0*sind(av3)+cur.ya;
xv4_0 = x_0+lv34_0*cosd(av4)+cur.xa;
yv4_0 = y_0+lv34_0*sind(av4)+cur.ya;

xc=[xv1_1;xv2_1;xv3_1;xv4_1;xv1_0;xv2_0;xv3_0;xv4_0];
yc=[yv1_1;yv2_1;yv3_1;yv4_1;yv1_0;yv2_0;yv3_0;yv4_0];

cibxmax= any(xc>=286);
cibymax= any(yc>=200);
cibxmin= any(xc<=0);
cibymin= any(yc<=0);

    for j=1:obsc
        obsxtemp=obsx(j,:);
        obsytemp=obsy(j,:);
        obsxtemp=obsxtemp(obsxtemp~=0);
        obsytemp=obsytemp(obsytemp~=0);
        in1 = InPolygon(xc,yc,obsxtemp,obsytemp); % check whether cur is hitting obs
        condobssta = any(in1);
        if condobssta == 1 ||cibxmax==1||cibymax==1||cibxmin==1||cibymin==1
         check = 0;
         break;
        end
    end
end
    


