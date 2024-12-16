% This function is to check if the SAV has reached the final goal
% configuration.
function check = loop_checkfinal(cur,xv_lc,yv_lc)
global f_state;

% Check the orientation of the final state and make the zone accordingly
% Creating zone
% check = 1;
% lz= 5; % IN zone
% wz= 1.5;
% dt1= f_statet + atand(wz/lz);
% dt2= f_statet - atand(wz/lz);
% dl= hypot(wz/2,lz/2);
% x1=  f_statex + dl*cosd(dt1);
% y1=  f_statey + dl*sind(dt1);
% x2=  f_statex + dl*cosd(dt2);
% y2=  f_statey + dl*sind(dt2);
% x3=  f_statex + dl*cosd(dt1+180);
% y3=  f_statey + dl*sind(dt1+180);
% x4=  f_statex + dl*cosd(dt2+180);
% y4=  f_statey + dl*sind(dt2+180);
% xv = [x1 x2 x3 x4];
% yv = [y1 y2 y3 y4];
% in = InPolygon(cur.xa,cur.ya,xv,yv); % check whether cur is in the zone
% % Check weather the final conditions are reached.
%     if in && cur.pred.tf == f_statet && cur.pred.gf == f_stateg 
% %     if cur.x == f_state.x && cur.y == f_state.y && cur.pred.tf == f_state.t && cur.pred.gf == f_state.g
%     check = 0;  % this will jump out of the while loop in Path_planner.m
%     end
    % Check the orientation of the final state and make the zone accordingly
% Creating zone
check = 1;
% lz= 5; % IN zone
% wz= 1.5;
% dt1= f_state.t + atand(wz/lz);
% dt2= f_state.t - atand(wz/lz);
% dl= hypot(wz/2,lz/2);
% x1=  f_state.x + dl*cosd(dt1);
% y1=  f_state.y + dl*sind(dt1);
% x2=  f_state.x + dl*cosd(dt2);
% y2=  f_state.y + dl*sind(dt2);
% x3=  f_state.x + dl*cosd(dt1+180);
% y3=  f_state.y + dl*sind(dt1+180);
% x4=  f_state.x + dl*cosd(dt2+180);
% y4=  f_state.y + dl*sind(dt2+180);
% xv_lc = [x1_lc x2_lc x3_lc x4_lc];
% yv_lc = [y1_lc y2_lc y3_lc y4_lc];
in = InPolygon(cur.xa,cur.ya,xv_lc,yv_lc); % check whether cur is in the zone
% Check weather the final conditions are reached.
    if in && cur.pred.tf == f_state.t && cur.pred.gf == f_state.g 
%     if cur.x == f_state.x && cur.y == f_state.y && cur.pred.tf == f_state.t && cur.pred.gf == f_state.g
    check = 0;  % this will jump out of the while loop in Path_planner.m
    end
end