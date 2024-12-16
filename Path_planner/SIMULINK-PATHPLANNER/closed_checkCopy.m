% This function calculates if an input leads to a state which is there 
%already in the closed list.If it is it skips the input.
function check = closed_checkCopy(t_x,t_y,c,ip)
check = 1; % not in closed list.
ccopy = nonzeros(c);
ccopy1 = ccopy';
checkx= t_x(ccopy1)==ip.x;
checky= t_y(ccopy1)==ip.y;

cond = all([checkx;checky]);
cond = any(cond);
if cond
    check=0;
end
end