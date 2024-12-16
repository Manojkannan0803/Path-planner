function [checktime,indexcirc] = checktimesecondstep(AV_x,AV_y,AV_theta,AV_gamma,pathMO_x,pathMO_y,t_MO,newpathx_AV,newpathy_AV,newpatht_AV)
coder.extrinsic('InPolygon');
checktime = nan(1,1);
indexcirc = zeros(1,1);
% indexcirc = zeros(1,1);
% in2time = false(1,1);
% in1time = false(1,1);
%__________________________________________________________________________
% initial check-time. Using the circle approach, time/distance where the
% collision is likely to occur is checked. 
% with time, path has to be separated. As of now, since for a straight path
% discretization interval is calculated. Later stage, a logic to be created
% with respect to every time sec what is the x,y point of AV and MO to be
% decided

% % AV_______________________________________________________________________
% % Since we interpolate the path, to find for every 1 sec the path x and
% % path y point to be found out. I chose this kind of approach, maybe if you
% % could find a better logic, it would be nice.
% a_AV = diff(AV_x); % check diff of AV
% b_AV = diff(AV_y); % check diff of AV
% c_AV = sqrt((a_AV).^2+(b_AV).^2); % calculate distance
% c1_AV = diff(c_AV); % to find any fluctuations
% cc1_AV = round(c1_AV);
% condcc1_AV = any(cc1_AV);
% if condcc1_AV==0
%     discint_AV = c_AV(1) % to find discinterval
% end
% disc_AV = 1/discint_AV;
% disc_AV = round(disc_AV);
% indexone_AV = 1:disc_AV:length(AV_x);
% newpathx_AV = AV_x(indexone_AV);
% newpathy_AV = AV_y(indexone_AV);
% newpatht_AV = AV_theta(indexone_AV);
% newpathg_AV = AV_gamma(indexone_AV);
% Circle check_____________________________________________________________
% thetapAV = deg2rad(180); % theta angle change accordingly
for i = 1:length(newpathx_AV)
centerx = newpathx_AV(i)+4.2375*cos(newpatht_AV(i));  % theta angle change accordingly
centery = newpathy_AV(i)+4.2375*sin(newpatht_AV(i));  % theta angle change accordingly 
% posAV = [centerx centery];
% viscircles(posAV,9.5,'Color','k','linewidth',1); % radii to be 9.5 based on vehicle dimensions
% hold on;
centerxMO = pathMO_x(i)+2.29*cos(t_MO);  % theta angle change accordingly
centeryMO = pathMO_y(i)+2.29*sin(t_MO);  % theta angle change accordingly 
% posMO = [centerxMO centeryMO];
% viscircles(posMO,4.8,'Color','b','linewidth',1);
% [xout,yout] = circcirc(centerx,centery,9.5,centerxMO,centeryMO,4.8);
% xcir = any(xout);
%         ycir = any(yout);
%         
t = linspace(0, 2*pi, 100);
cir = @(r,ctr) [r*cos(t)+ctr(1); r*sin(t)+ctr(2)];                      % Circle Function        
c1 = cir(9.5, [centerx;centery]);
c2 = cir(4.8, [centerxMO;centeryMO]);
in1 = find(inpolygon(c1(1,:), c1(2,:), c2(1,:), c2(2,:)));              % Circle #1 Points Inside Circle #2
in2 = find(inpolygon(c2(1,:), c2(2,:), c1(1,:), c1(2,:)));              % Circle #2 Points Inside Circle #1
in1time = any(in1);
in2time = any(in2);
        if in1time || in2time
            checktime = 1; % intersect
            indexcirc = i; % to find index
            break;
        else
            checktime = 0; % didn't intersect
            indexcirc = 0;
        end

%         if xcir==1 && ycir==1
%             checktime = 1; % intersect
%             indexcirc = i; % to find index
%             break;
%         else
%             checktime = 0; % didn't intersect
%             indexcirc = 0;
%         end
end
end

