function hc_d = rectheur1slx(ip,f_statex,f_statey)
coder.extrinsic('InPolygon');
xlim1 = 25;
ylim1 = 146;
ylim2 = 50;
% xv_1 = [58.6 28 28 58.6];
% yv_1 = [53 53 144 144];
% in5 = InPolygon(ip.xa,ip.ya,xv_1,yv_1);
a = ip.ya<=53 && ip.xa>=28;
b = ip.ya>53 && ip.ya<144 && ip.xa<28;
c = ip.ya<=61 && ip.ya>=53 && ip.xa>108;
if a
    length = (abs(ip.xa-xlim1)+abs(ip.ya-ylim1)+abs(xlim1-f_statex)+abs(ylim1-f_statey))*2.5;%*3
elseif b
    length = (abs(ip.xa-f_statex)+abs(ip.ya-ylim1)+abs(ylim1-f_statey))*2.8;
%     length = (abs(ip.ya-ylim1)+abs(ip.xa-xlim1)+abs(ylim1-ylim2)+abs(xlim1-f_state.x)+abs(ylim2-f_state.y))*3;%*3
elseif c
    length = (abs(ip.ya-ylim2)+abs(ip.xa-xlim1)+abs(ylim1-ylim2)+abs(xlim1-f_statex)+abs(ylim1-f_statey))*3;%*3
% elseif in5
%     length = 0;
else
    length = (abs(ip.ya-f_statey)+abs(ip.xa-f_statex))*2.5;%*2.5
end
hc_d = length;
end
