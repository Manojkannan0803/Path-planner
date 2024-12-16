% This function calculates if an input leads to a state which is there 
% already in the closed list.If it is it skips the input.
function check = closed_check(t_x,t_y,c,ip)
check = 1; % not in closed list
checkx= t_x(c)==ip.x;
checky= t_y(c)==ip.y;

cond = all([checkx;checky]);
cond = any(cond);
if cond
    check=0;
end
end