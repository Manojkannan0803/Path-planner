
function hc = h_costnewslx(ip,xv_lg,yv_lg,f_statet,f_statex,f_statey,i_statey)
coder.extrinsic('InPolygon');
if f_statey<=80 && i_statey>=70
    hc_d = rectheurslx(ip,f_statex,f_statey);
elseif f_statey>=80 && i_statey<=70
    hc_d = rectheur1slx(ip,f_statex,f_statey);
else
a = (ip.xa-f_statex)^2;
b = (ip.ya-f_statey)^2;
dist = sqrt(a + b); % Euclidean distance
hc_d= dist*2.2; % Heuristic cost times the heuristic coefficient.
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
if inlg && f_statet ~= ip.pred.tf
    hc_d= hc_d*1000;
end

hc = hc_d;
end

