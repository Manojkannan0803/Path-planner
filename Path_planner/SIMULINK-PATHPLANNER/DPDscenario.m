%% The DPD scenario.
% Creating static obstacles - 2D representation
% We have to define the vertices which when joined forms the obstacle. 
obsc= 6; % Number of static obstacles

% Please put the obstacle with most number of vertices as the first obstacle. 
obsx(1,:) = [28,58.6,58.6,61.4,61.4,108,108,200.5,200.5,204.25,204.25,228,228,285,285,228,228,111.4,111.4,108.6,108.6,108,108,91.4,91.4,88.6,88.6,28];
obsy(1,:) = [53,53,69.5,69.5,53,53,61,61,77.5,77.5,61,61,57,57,140,140,136,136,119.5,119.5,136,136,144,144,127.5,127.5,144,144];

% Adding 2nd obstacle
temp1   =   zeros(1,length(obsx(1,:)));                                     % Matching the number of elements
% assind1     =   find([72,240,255,202,190,185,195,87,72]);            % Assigning the indexes     
% temp1(assind1)  =   [72,240,255,202,190,185,195,87,72];              % Putting the obstacle coordinates
assind1     =   find([72,240,255,122,110,105,115,87,72]);            % Assigning the indexes     
temp1(assind1)  =   [72,240,255,122,110,105,115,87,72]; 
obsx(2,:)   =   temp1;                                                      % Putting in the obstacle matrix. 

temp1   =   zeros(1,length(obsy(1,:)));
% assind2    =   find([13,13,33,33,21.33,18.94,33,33,13]);
% temp1(assind2)  =   [13,13,33,33,21.33,23.3099,33,33,13];
assind2    =   find([13,13,33,33,21.33,18.94,33,33,13]);
temp1(assind2)  =   [13,13,33,33,21.33,23.3099,33,33,13];
obsy(2,:)   =   temp1;

% Adding 3rd obstacle
temp1   =   zeros(1,length(obsx(1,:)));
assind3     =   find([27,243,260,44]);
temp1(assind3)  =   [27,243,260,44];
obsx(3,:)   =   temp1;

temp1   =   zeros(1,length(obsy(1,:)));
assind4     =   find([168,168,178,178]);
temp1(assind4)  =   [168,168,178,178];
obsy(3,:)   =   temp1;
% Adding 4rd obstacle
temp1   =   zeros(1,length(obsx(1,:)));
assind5     =   find([100,150,150,100]);
temp1(assind5)  =   [100,150,150,100];
obsx(4,:)   =   temp1;

temp1   =   zeros(1,length(obsy(1,:)));
assind6     =   find([195,195,192.5,192.5]);
temp1(assind6)  =   [195,195,190,190];
obsy(4,:)   =   temp1;
% Adding 5rd obstacle
temp1   =   zeros(1,length(obsx(1,:)));
assind7     =   find([162.6,180,180,162.6]);
temp1(assind7)  =   [162.6,180,180,162.6];
obsx(5,:)   =   temp1;

temp1   =   zeros(1,length(obsy(1,:)));
assind8     =   find(([195,195,192.5,192.5]));
temp1(assind8)  =   [195,195,190,190];
obsy(5,:)   =   temp1;

% Adding 6th obstacle
temp1   =   zeros(1,length(obsx(1,:)));
assind9     =   find([28,50,50,28]);
temp1(assind9)  =   [28,50,50,28];
obsx(6,:)   =   temp1;

temp1   =   zeros(1,length(obsy(1,:)));
assind10     =   find(([53,53,51,51]));
temp1(assind10)  =   [53,53,51,51];
obsy(6,:)   =   temp1;

% % Adding 7th obstacle
% temp1   =   zeros(1,length(obsx(1,:)));
% assind11     =   find([28,50,50,28]);
% temp1(assind11)  =   [28,50,50,28];
% obsx(7,:)   =   temp1;
% 
% temp1   =   zeros(1,length(obsy(1,:)));
% assind12     =   find(([144,144,147,147]));
% temp1(assind12)  =   [144,144,147,147];
% obsy(7,:)   =   temp1;

% % Adding 8th obstacle
% temp1   =   zeros(1,length(obsx(1,:)));
% assind13     =   find([150,145,145,150]);
% temp1(assind13)  =   [150,145,145,150];
% obsx(8,:)   =   temp1;
% 
% temp1   =   zeros(1,length(obsy(1,:)));
% assind14     =   find(([35,35,40,40]));
% temp1(assind14)  =   [35,35,40,40];
% obsy(8,:)   =   temp1;
% If there are more static obstacles, add them in a similar fashion and
% update the obstacle count "obsc".
%If you want to visualize the path planning process live plese uncomment
%the following lines.(WARNING: INCREASES COMPUTATIONAL TIME)
% figure(1);
% fill([0,286,286,0],[0,0,200,200],'c');
% hold on;
% for i=1:obsc
%     obsxtemp=obsx(i,:);
%     obsytemp=obsy(i,:);
%     obsxtemp=obsxtemp(obsxtemp~=0);
%     obsytemp=obsytemp(obsytemp~=0);
%     fill(obsxtemp,obsytemp,'r');
% end