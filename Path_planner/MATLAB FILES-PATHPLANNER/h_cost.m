% Function to calculate the heuristics i.e; h-cost (cost to reach the final
% node from the current node). In this work, two kinds of heuristics is
% being utilized. One is Euclidean distance which is employed for solving
% the shorter path planning problem. For solving the longer path planning
% problem, Rectangular heuristics (newly proposed) is being used instead of
% Euclidean distance as it didn't satisfy the admissibility requirements
function hc = h_cost(ip,cur,i_state,xv_lg,yv_lg)
global f_state;

if f_state.y<=80 && i_state.y>=70
    hc_d = rectheur(ip);
elseif f_state.y>=80 && i_state.y<=70
    hc_d = rectheur1(ip);
else
a = (ip.xa-f_state.x)^2;
b = (ip.ya-f_state.y)^2;
dist = sqrt(a + b); % Euclidean distance
% dist = sqrt((ip.xa-f_state.x)^2 + (ip.ya-f_state.y)^2); % Euclidean distance
hc_d= dist*2.5; % Heuristic cost times the heuristic coefficient..
end

inlg = InPolygon(ip.xa,ip.ya,xv_lg,yv_lg);
xv_lg = ppround_1(xv_lg);
yv_lg = ppround_1(yv_lg);
in2 = InPolygon(ip.xa,ip.ya,xv_lg,yv_lg); % check whether cur is in the zone

% If in the final zone and the theta is not the required theta then the
% cost is heavily penalized. 
if in2 && ~inlg
    hc_d= hc_d*1000;
end
if inlg && f_state.t ~= ip.pred.tf
    hc_d= hc_d*1000;
end

hc = hc_d;
end