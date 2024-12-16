%% Path planning algorithm
% Author: Manojpriyadharson Kannan (Student number: 638628)
% CATALYST project: Automated docking maneuvering of an articulated
% vehicles in the presence of obstacles
% HAN-AR_HAN university of applied sciences_DPD_TNO
% As an initial step, MP_addition.m file has to be run inorder to load the
% motion primitives library to the workspace
% Function files associated with this path planner
% closed_check.m - To check the closed lists
% g_cost.m - To calculate the g-cost values
% h_cost.m - To calculate the h-cost values
% loop_checkfinal.m - To check whether the vehicle reached the final
% configuration zone
% open_check.m - To check the open lists
% staticobs_check.m - To check whether the vehicle hits the static
% obstacles present
%%
tictime = tic;    % Used to calculate the computational time
clc;
time = timeAV;
%%If you want to visualize the path planning process live plese uncomment
%%the following lines.(WARNING: INCREASES COMPUTATIONAL TIME)
% figure(1);
% hold on;
%% Initializing parameters
global length_dc;   length_dc   =   328;     % Length of distribution center [m]
global width_dc;    width_dc    =   200;     % Width of distribution center [m]
global scale;       scale       =   1;       % Scale of the grid [m]
%% Defining data structures
% Creating motion primitive structure. Each motion primitive can be
% uniquely identified by 4 tags:
% ti(theta initial), gi(gamma initial), tf (theta final), gf (gamma final)
mp      =   struct('ti',[],'gi',[],'tf',[],'gf',[],'gcost',[],'hcost',[],'dir',[]);  
% The state should contain the info of gcost and hcost of moving to that 
% by the motion primitive pred (predecessor).
state   =   struct('x',[],'y',[],'xa',[],'ya',[],'predxy',[],'pred',mp);
% Creating the tree vectors in order to save the explored configurations
t_x     =   [];     % X config
t_y     =   [];     % Y config
t_xa    =   [];     % Actual ending point of MP X
t_ya    =   [];     % Actual ending point of MP Y
t_predxy    =   []; % Index of the config before this config in the tree 
t_gcost     =   []; % Cost so far for this config
t_hcost     =   []; % Cost to go for this config
t_pred  =   [];     % Index of the MP used to reach this config
t_dir   =   [];     % Direction of motion to reach this config
t_count     =   0;  % Count of the explored configs. 
%% Load the files
run DPDscenario; % static obstacle map
%% A star algorithm
% Initial state
i_state.x   =   75;        %[m]
i_state.y   =   45;         %[m]
i_state.t   =   180;          %[degrees] have to be a discreet theta config [orientation angle of semi-trailer]
i_state.g   =   0;          %[degrees]have to be a discreet gamma config [articulation angle]
i_time = 0;
% Final state
global f_state;
f_state.x   =   60;        %[m]
f_state.y   =   64;         %[m]
f_state.t   =   270;         %[degrees] have to be a discreet theta config [orientation angle of semi-trailer]
f_state.g   =   0;          %[degrees]have to be a discreet gamma config  [articulation angle]
%% Discretization intervals
% thetad = 0:9:351;               % Creating discretized orientation angle of semi-trailer
% gammad = [-21 0 21];              % Creating discretized articulation angle of vehicle
%%If you want to visualize the path planning process live plese uncomment
%%the following lines.(WARNING: INCREASES COMPUTATIONAL TIME)
% figure(1);
% scatter(i_state.x,i_state.y,'y');
% scatter(f_state.x,f_state.y,'b');
%% Area definition
% related to h-cost
lz_lg = 25;
wz_lg= 1;
dt1_lg= f_state.t + atand(wz_lg/lz_lg);
dt2_lg= f_state.t - atand(wz_lg/lz_lg);
dl_lg= hypot(wz_lg/2,lz_lg/2);
x1_lg=  f_state.x + dl_lg*cosd(dt1_lg);
y1_lg=  f_state.y + dl_lg*sind(dt1_lg);
x2_lg=  f_state.x + dl_lg*cosd(dt2_lg);
y2_lg=  f_state.y + dl_lg*sind(dt2_lg);
x3_lg=  f_state.x + dl_lg*cosd(dt1_lg+180);
y3_lg=  f_state.y + dl_lg*sind(dt1_lg+180);
x4_lg=  f_state.x + dl_lg*cosd(dt2_lg+180);
y4_lg=  f_state.y + dl_lg*sind(dt2_lg+180);

xv_lg = [x1_lg x2_lg x3_lg x4_lg];
yv_lg = [y1_lg y2_lg y3_lg y4_lg];
% % % related to h-cost1
% xv_dv = [61.4 40 40 61.4];
% yv_dv = [30 30 51 51];
% loopcheck
lz_lc= 5; % IN zone
wz_lc= 1.5;
dt1_lc= f_state.t + atand(wz_lc/lz_lc);
dt2_lc= f_state.t - atand(wz_lc/lz_lc);
dl_lc= hypot(wz_lc/2,lz_lc/2);
x1_lc=  f_state.x + dl_lc*cosd(dt1_lc);
y1_lc=  f_state.y + dl_lc*sind(dt1_lc);
x2_lc=  f_state.x + dl_lc*cosd(dt2_lc);
y2_lc=  f_state.y + dl_lc*sind(dt2_lc);
x3_lc=  f_state.x + dl_lc*cosd(dt1_lc+180);
y3_lc=  f_state.y + dl_lc*sind(dt1_lc+180);
x4_lc=  f_state.x + dl_lc*cosd(dt2_lc+180);
y4_lc=  f_state.y + dl_lc*sind(dt2_lc+180);
xv_lc = [x1_lc x2_lc x3_lc x4_lc];
yv_lc = [y1_lc y2_lc y3_lc y4_lc];
%% A star algorithm lists
o   =   []; % Open list
oc  =   0;  % Open list count
c   =   []; % Closed list
cc  =   0;  % Closed list count

cur             =   state;          % Current state in consideration
cur.x           =   i_state.x;      % Initializing cur to initial state 
cur.y           =   i_state.y;
cur.xa          =   i_state.x;      % keeping same as cur.x (but actually it is ending point of MP)
cur.ya          =   i_state.y;
cur.pred.tf     =   i_state.t;
cur.pred.gf     =   i_state.g;
cur.pred.dir    =   1;              % giving preference to forward motion
% Adding the initail state in the tree.
t_count     =   t_count+1;
t_x(t_count)=   cur.x;
t_y(t_count)=   cur.y;
t_xa(t_count)   =   cur.xa;
t_ya(t_count)   =   cur.ya;
t_predxy(t_count)   =   nan;
t_gcost(t_count)    =   nan;
t_hcost(t_count)    =   nan;
t_pred(t_count) =   nan;
t_dir(t_count)  =   nan;
% Adding the inital state in the closed list.
cc          =   cc+1;
c(cc)       =   1;
%% Algorithm iteration loop starts
loopcheck = 1;  %Checks the final condition. Updated by function loop_check.m 
% to have straight path in the upper sector of DPD
xv_rv = [286 30 30 286]; 
yv_rv = [144 144 200 200];

xv_rv1 = [28 0 0 28]; 
yv_rv1 = [60 60 140 140];

xv_rv2 = [286 0 0 286];
yv_rv2 = [0 0 50 50];
while loopcheck 
%         inrv = InPolygon(cur.xa,cur.ya,xv_rv,yv_rv);
%         inrv1 = InPolygon(cur.xa,cur.ya,xv_rv1,yv_rv1);
% % % || inrv1==1
% %     if inrv==1 || inrv1==1
% %         A = [180,270];
% %         B = 0;
% %     else
%         inrv2 = InPolygon(cur.xa,cur.ya,xv_rv2,yv_rv2);
%     if inrv2==1 && i_state.y<70 && f_state.y>70
%         A = [180,90];
%         B = 0;
%     elseif inrv==1 && i_state.y>70 && f_state.y<70
%         A = [180,270];
%         B = 0;
%     else
%         [A1,A2] = virtualobs_check(cur,obsc,obsx,obsy);                         % For selection of motion primitives
%         indextoreplace = A2<0;                                                  % For converting it in 360 degree format
%         A2(indextoreplace) = 360+A2(indextoreplace);
%         indextoreplace1 = A1>=360;
%         A1(indextoreplace1) = A1(indextoreplace1)-360;
%         A6 = [270 342 351 0];
%         A = horzcat(A1,A2,A6);                                                     % Theta_final orientation angle of semi-trailer candidates
% %     A = horzcat(A1,A2);
%         B = gammad;                                                         % gamma_final articulation angle candidates
%     end
    checkti     = cur.pred.tf==tini;                                        % Selecting MP with initial states equal to the final state of the previous config
    checkgi     = cur.pred.gf==gini;
%     checktif    = ismember(tfin,A);                                         % This is done because the matrix dimensions didn't match if we use previous logical condition
%     checkgif    = ismember(gfin,B);                                         % This is done because the matrix dimensions didn't match if we use previous logical condition
%     cond        =   all([checkti;checkgi;checktif;checkgif]);               % Finding indexes of those MP in the MP bank
    cond        =   all([checkti;checkgi]);               % Finding indexes of those MP in the MP bank
    index       =   find(cond);                                             % Finding the coordinates of the required MP
    ifcheck     =   ~isempty(index);
    if ifcheck
        for k   =   1:length(index) 
            ip  =   state;                                                  % Utilizing all of those MP
            ip.pred.ti  =   cur.pred.tf;                                    % Creating the input state, initial theta
            ip.pred.gi  =   cur.pred.gf;                                    % Creating the input state, initial gamma
            ip.predxy   =   c(cc);
            if cur.pred.dir     ==  1                                       % Setting the direction of motion
               ip.pred.dir=1;
            end

            if cur.pred.dir     ==  0
               ip.pred.dir=1;
            end 
            xp   = x(:,index(k));                                           % Extracting x and y data from mp bank.
            yp   = y(:,index(k));
            thetap = t(:,index(k));                                         % Extracting theta data from mp bank.
            gammap = g(:,index(k));                                         % Extracting gamma data from mp bank.
            dirp   = ip.pred.dir;                                           % Setting the direction for sending to functions
            ip.xa = cur.xa+xp(length(xp));                                  % Finding the end coordinates of the MP to find new state.
            ip.ya = cur.ya+yp(length(yp));
            ip.pred.tf  =   tfin(index(k));                                 % Creating the input state, final theta 
            ip.pred.gf  =   gfin(index(k));                                 % Creating the input state, final gamma
            pathlength = time(index(k));
            ip.x = ppround_1(ip.xa);                                        % Rounding the values to fit the scale of the state space(if u wish to chang the resolution this has to be changed)
            ip.y = ppround_1(ip.ya);
            check_c       = closed_check(t_x,t_y,c,ip);                     % Check if the input is in the closed list. 
            check_obstacle = staticobs_check(cur,xp,yp,thetap,gammap,dirp,obsc,obsx,obsy);
            if check_c && check_obstacle
%                 ip.pred.gcost = g_cost(cur,xp,yp);                           % Function to calculate the gcost of ip                
                ip.pred.gcost = g_cost(cur,pathlength);                           % Function to calculate the gcost of ip
                ip.pred.hcost = h_cost(ip,cur,i_state,xv_lg,yv_lg);                             % Function to calculate the hcost of ip
                check_o       = open_check (t_x,t_y,t_gcost,ip,o,oc);          % Check if the state is already in open list
                if check_o
                    % Adding the state to the tree
                    t_count=t_count+1;
                    t_x(t_count)= ip.x;
                    t_y(t_count)= ip.y;
                    t_predxy(t_count)= ip.predxy;
                    t_xa(t_count)= ip.xa;
                    t_ya(t_count)= ip.ya;
                    t_pred(t_count)= index(k);
                    t_dir(t_count)= ip.pred.dir;
                    t_gcost(t_count)= ip.pred.gcost;
                    t_hcost(t_count)= ip.pred.hcost;
                    oc=oc+1;                                                % Updating open count
                    o(oc)=t_count;                                          % Adding the index of the state to the open list
                    
                end
            end
        end
    end

    %% Exploring reverse paths  
%     [A3,A4] = virtualobsreverse_check(cur,obsc,obsx,obsy);                  % For selection of motion primitives
%     indextoreplace = A4<0;                                                  % For converting it in 360 degree format
%     A4(indextoreplace) = 360+A4(indextoreplace);
%     indextoreplace1 = A3>=360;
%     A3(indextoreplace1) = A3(indextoreplace1)-360;
% %     A5 = [270];
%     A5 = [270 90];
%     AR = horzcat(A3,A4,A5);                                                    % Theta_final orientation angle of semi-trailer candidates
%     B = gammad;                                                           % gamma_final articulation angle candidates
    checkti= cur.pred.tf==tfin;                                             % Selecting MP with initial states equal to the final state of the previous config but with tfin and gfin (reverse)
    checkgi= cur.pred.gf==gfin;
%     checktif    = ismember(tini,AR);                                        % This is done because the matrix dimensions didn't match if we use previous logical condition
%     checkgif    = ismember(gini,B);                                         % This is done because the matrix dimensions didn't match if we use previous logical condition
%     cond        = all([checkti;checkgi;checktif;checkgif]);                 % Finding indexes of those MP in the MP bank
    cond = all([checkti;checkgi]);                                        % Finding indexes of those MP in the MP bank
    index= find(cond);                                                      % Finding the coordinates of the required MP
    ifcheck = ~isempty(index);
    if ifcheck
        for k=1:length(index)
            ip  =   state;
            ip.pred.ti  =   cur.pred.tf;
            ip.pred.gi  =   cur.pred.gf;
            ip.predxy   =   c(cc);
            if cur.pred.dir==1
               ip.pred.dir=0;
            end
            if cur.pred.dir==0
               ip.pred.dir=0;
            end
            xp=x(:,index(k));
            yp=y(:,index(k));
            xfinr=xfin(index(k));
            yfinr=yfin(index(k));
            xp=xp-xfinr;
            yp=yp-yfinr; 
            thetap = t(:,index(k));
            gammap = g(:,index(k));
            dirp   = ip.pred.dir;
            pathlength = time(index(k));
            ip.xa = cur.xa+xp(1);                                           % Finding the end coordinates of the MP to find the new state.
            ip.ya = cur.ya+yp(1); 
            ip.pred.tf  =   tini(index(k));
            ip.pred.gf  =   gini(index(k));
            ip.x = ppround_1(ip.xa);                                          % Rounding the values to fit the scale of the state space
            ip.y = ppround_1(ip.ya);
            check_c       = closed_check(t_x,t_y,c,ip);                          % Check if the input is in the closed list.
            check_obstacle = staticobs_check(cur,xp,yp,thetap,gammap,dirp,obsc,obsx,obsy);
            if check_c && check_obstacle
%                 ip.pred.gcost = g_cost(cur,xp,yp);                           % Function to calculate the gcost of ip
                ip.pred.gcost = g_cost(cur,pathlength);                           % Function to calculate the gcost of ip
                ip.pred.hcost = h_cost(ip,cur,i_state,xv_lg,yv_lg);                             % Function to calculate the hcost of ip
                check_o       = open_check(t_x,t_y,t_gcost,ip,o,oc);           % Check if the state is already reached
                if check_o
                    t_count=t_count+1;
                    t_x(t_count)= ip.x;
                    t_y(t_count)= ip.y;
                    t_predxy(t_count)= ip.predxy;
                    t_xa(t_count)= ip.xa;
                    t_ya(t_count)= ip.ya;
                    t_pred(t_count)= index(k);
                    t_dir(t_count)= ip.pred.dir;
                    t_gcost(t_count)= ip.pred.gcost;
                    t_hcost(t_count)= ip.pred.hcost;
                    oc=oc+1;
                    o(oc)=t_count;
                    
                end
            end
        end
    
    end


    % Now from the open list that we have created we have to find the motion
    % option with the least cost and that will be transfered into the closed
    % list and become the current state.
    
   
    tcostall= t_gcost(o)+t_hcost(o);
    [min_cost,min_ind]    = min(tcostall);                                  % Finding the state with the min cost
    cc         =   cc+1;                                                    % Updating close list count 
    c(cc)      =   o(min_ind);                                              % Puting it in closed list
    o(min_ind) =   [];                                                      % Removing from open list
    oc         =   oc-1;

    %% Plot the explored graph real-time (to be deleted)
    index        =   t_pred(c(cc));
    if t_dir(c(cc))==1
        xp=x(:,index);
        yp   = y(:,index);
    else
        xp=x(:,index)-xfin(index);
        yp=y(:,index)-yfin(index);
    end
    thetap = t(:,index);
    gammap = g(:,index);
    dirp   = t_dir(c(cc));
    pred_index= t_predxy(c(cc));
    xp= xp+t_xa(pred_index);
    yp= yp+t_ya(pred_index);
    plot(xp,yp);
    drawnow
    hold on;
        
    cur.x           =   t_x(c(cc));                                         % Initializing cur to minimum cost state
    cur.y           =   t_y(c(cc));
    cur.xa          =   t_xa(c(cc));
    cur.ya          =   t_ya(c(cc));
    cur.predxy      =   t_predxy(c(cc));
    index           =   t_pred(c(cc));
    cur.pred.dir    =   t_dir(c(cc)); 
    cur.pred.hcost  =   t_hcost(c(cc));
    cur.pred.gcost  =   t_gcost(c(cc));
    if cur.pred.dir == 1
        cur.pred.ti     =   tini(index);
        cur.pred.gi     =   gini(index);
        cur.pred.tf     =   tfin(index);
        cur.pred.gf     =   gfin(index);
    else
        cur.pred.ti     =   tfin(index);
        cur.pred.gi     =   gfin(index);
        cur.pred.tf     =   tini(index);
        cur.pred.gf     =   gini(index);   
    end
   
    loopcheck   = loop_checkfinal(cur,xv_lc,yv_lc);
    
end
toctime=toc(tictime);   % Finding the computational time
% clearvars -except dum hc_d index indexvector t_gcost t_hcost toctime dumx obsx obsy obsc lol x1 y1 d g gfin gini t tfin time tini x xfin y yfin c cc cur i_state f_state t_x t_y t_xa t_ya t_predxy t_pred t_dir t_count=0 i_state.x i_state.y i_state.t i_state.g t_time cur.time timevector 
run Plot_pathgenerated