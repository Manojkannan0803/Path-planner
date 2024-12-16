function [startx_AV,starty_AV,starttheta_AV,startgamma_AV,startx_MO,starty_MO] = clashpointfinder(indexsuspect,pathMO_x,pathMO_y,t_MO,newpathx_AV,newpathy_AV,newpatht_AV,newpathg_AV,AV_x,AV_y,AV_theta,AV_gamma,obsc,obsx,obsy)
coder.extrinsic('InPolygon');
indexsuspect_final = nan(1,1);
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
%     discint_AV = c_AV(1); % to find discinterval
% end
% % find how much total time to reach final point
% % finx_AV = AV_x(1)-AV_x(end);
% % finy_AV = AV_y(1)-AV_y(end);
% % cfin1_AV = sqrt((finx_AV)^2+(finy_AV)^2);
% disc_AV = 1/discint_AV;
% disc_AV = round(disc_AV);
% indexone_AV = 1:disc_AV:length(AV_x);
% newpathx_AV = AV_x(indexone_AV);
% newpathy_AV = AV_y(indexone_AV);
% newpatht_AV = AV_theta(indexone_AV);
% newpathg_AV = AV_gamma(indexone_AV);
% check 3: main collision point check using vehicle dimensions and figure
% out the clash point
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
        checkvehicle = 1; % indexsuspect intersect
        indexsuspect_final = j;
        break;
%     else
%         checkvehicle = 0; % indexsuspect doesn't intersect
    end
end

% finding the clash point
indexfinalAV = indexsuspect(indexsuspect_final); % find exact x and y point
clashx_AV = newpathx_AV(indexfinalAV);
clashy_AV = newpathy_AV(indexfinalAV);
clasht_AV = newpatht_AV(indexfinalAV);
indexfinalMO = indexsuspect(indexsuspect_final); % find exact x and y point
clashx_MO = pathMO_x(indexfinalMO);
clashy_MO = pathMO_y(indexfinalMO);

% find new start point of AV
startcheckx_AV = clashx_AV-15*cos(clasht_AV); % 15m gap is chosen for smooth transition
startchecky_AV = clashy_AV-15*sin(clasht_AV);
[~, indexAV]= min(abs(startcheckx_AV-AV_x));

startx_AV = AV_x(indexAV);
starty_AV = AV_y(indexAV);
starttheta_AV = AV_theta(indexAV); 
startgamma_AV = AV_gamma(indexAV);

% for MO
startx_MO = clashx_MO-15*cos(thetap_MO);
starty_MO = clashy_MO-15*sin(thetap_MO);
end

