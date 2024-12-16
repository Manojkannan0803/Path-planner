function m = ppround_1(x_r)
% This format should be used to find the discreet states if other resolution is used. 

% temp    =   rem(x_r,0.001);
% temp2   =   x_r-temp;
% fdp     =   rem(temp2,1);   %first 3 decimal places.
% u_l=[0.125 0.375 0.625 0.875 1];
% l_l=[0 0.125 0.375 0.625 0.875];
% check1=l_l<fdp;
% check2=u_l>fdp;
% ind=all([check1;check2]);
% ind=find(ind);
% m=(x_r-rem(x_r,1))+((u_l(2)-l_l(2))/2)+l_l(2);

m=round(x_r); % Since the resolution is 1m, simple rounding is sufficient. Or a swith has to be used.
% m=round((x_r*2)/2);
end