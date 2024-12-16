% Function to calculate the g-cost (cost so far travelled). For this cost
% function, we could get the length of the each motion primitive from the
% ACADO output file itself. Extract the data from that
function gc = g_costCopy(cur,pathlength)
% Calculation of the path length of motion primitive. 
% d       =   hypot(diff(xp),diff(yp));
% d_tot   =   sum (d);        % total distance coverd by motion primitives
d_tot   =   pathlength;        % total distance coverd by motion primitives
% d_tot   =   round(d_tot);   % d_tot= length of motion primitive;

if isempty(cur.pred.gcost) % For the first iteration.

gc      =   abs(0+0+d_tot);
                                        
                                        
else                                   
                                        
gc      =   abs(0+cur.pred.gcost+d_tot);    % For subsequent iterations. 

            
end