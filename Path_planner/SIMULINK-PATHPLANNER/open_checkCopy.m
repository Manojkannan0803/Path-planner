% This function finds out if the current input state is already in the open list
% and finds if the cuttent way to it is better that what already exsists.
% If so we proceed to add this option to the state space.
function check = open_checkCopy(t_x,t_y,t_gcost,ip,o,oc)
check = 1;
ocopycheck = nonzeros(o);
ocopycheck1 = ocopycheck';
% ocopycheck2 = rmmissing(ocopycheck1);
% ocopy = rmmissing(ocopy);
% Finding if the state is already present.
checkx= t_x(ocopycheck1)==ip.x;
checky= t_y(ocopycheck1)==ip.y;
cond = all([checkx;checky]);
cond1 = any(cond); % checks if the previous copy of the state is in the open list
if cond1 && ~isempty(checkx)    
    oindex=find(cond);  % Finds the indexes of the previous copies. 
    tempcost= t_gcost(ocopycheck1(oindex)); % Extracts gcost
    if   tempcost <  ip.pred.gcost  % Compares gcost
        check = 0;  % Blocks the state
    else
        o(oindex)=zeros;   % Continues with the state removing the previous copies. 
%         o = rmmissing(o);
%         o = getback';
%         oc=oc-1;
    end
end
end
