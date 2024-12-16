% This function is to check if the SAV has reached the final goal
% configuration.
function check = loop_checkfinalslx(cur,xv_lc,yv_lc,f_statet,f_stateg)
coder.extrinsic('InPolygon');
check = 1;
in = InPolygon(cur.xa,cur.ya,xv_lc,yv_lc); % check whether cur is in the zone
% Check weather the final conditions are reached.
    if in && cur.pred.tf == f_statet && cur.pred.gf == f_stateg 
%     if cur.x == f_state.x && cur.y == f_state.y && cur.pred.tf == f_state.t && cur.pred.gf == f_state.g
    check = 0;  % this will jump out of the while loop in Path_planner.m
    end
end