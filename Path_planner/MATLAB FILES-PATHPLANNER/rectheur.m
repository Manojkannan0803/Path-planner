function hc_d = rectheur(ip)
% This function is to give the heurstics cost considering the obstacle
% constraint (main DC building). Certain boundaries are set and with
% respect to that length is calculated
global f_state;
xlim1 = 25;
ylim1 = 146;
ylim2 = 50;
xv_1 = [58.6 28 28 58.6];
yv_1 = [53 53 144 144];
in5 = InPolygon(ip.xa,ip.ya,xv_1,yv_1);
a = ip.ya>=144 && ip.xa>=28;
b = ip.ya>=119.5 && ip.ya<=144 && ip.xa>=87;
c = ip.xa<28 && ip.ya>54;
if a
    length = (abs(ip.xa-xlim1)+abs(ip.ya-ylim2)+abs(xlim1-f_state.x)+abs(ylim2-f_state.y))*3;%*3
elseif b
    length = (abs(ip.ya-ylim1)+abs(ip.xa-xlim1)+abs(ylim1-ylim2)+abs(xlim1-f_state.x)+abs(ylim2-f_state.y))*3;%*3
elseif c
    length = (abs(ip.ya-ylim2)+abs(ylim2-f_state.y)+abs(ip.xa-f_state.x))*2.8;%*2.8
elseif in5
    length = 0;
else
    length = (abs(ip.ya-f_state.y)+abs(ip.xa-f_state.x))*2.5;%*2.5
end
hc_d = length;
end

