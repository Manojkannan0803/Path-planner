%% ACADO Toolkit - Motion primitives generation
% Author: Manojpriyadharson Kannan (Student number: 638628)
% CATALYST project: Automated docking maneuvering of an articulated
% vehicles in the presence of obstacles
% HAN-AR_HAN university of applied sciences_DPD_TNO
%% Clear the workspace
clc;
clear all;
%% Initializing the primary conditions
% Discretizing the orientation angle of the semi-trailer
thetad = -90:3:90;                                                          % final orientation angle of semi-trailer [deg]
initialtheta = 0;                                                           % defining initial orientation angle of semi-trailer [deg]
initialgamma = 0;                                                           % defining final vehicle articulation angle [deg]
folder=strcat('Motion_primitives\thetatrailer_',num2str(wrapTo360(initialtheta)),'_gamma_',num2str(wrapTo360(initialgamma))); 
mkdir (folder);                                                             % Making folder
for i= 1: length(thetad)
        tval= thetad(i);                                                    % Specifying end theta   
        gval = 0;                                                           % Specifying end gamma  
        file=strcat(folder,'\','mp_',num2str(wrapTo360(tval)),'_',num2str(gval),'.mat');
        chk= exist (file);                                                  % Check if MP already exists.
        if chk == 0
%% This is the actual ACADO code which is necessary. 
            BEGIN_ACADO;                                                    % Always start with "BEGIN_ACADO". 

                acadoSet('problemname', 'Motionprimitives_acado');          % Set your problemname. If you 
                                                                            % skip this, all files will
                                                                            % be named "myAcadoProblem"


                DifferentialState       x1 y1 theta1 gamma delta omega ;    % The differential states 
                Control                 u;                                  % The controls
                Parameter               T;                                  % We would like to minize the T, so T is a parameter



                %% Differential Equation
                f = acado.DifferentialEquation(0,T);                        % Set the differential equation object

                % Initialize vehicle model
                % 'L0b'  = +ve, indicates that the respective kingpin is ahead of tractor
                % drive axle (Fifth-wheel coupling)
                % 'L0b'  = -ve, indicates that the respective kingpin is behind the tractor
                % drive axle (Draw-bar coupling)
                v0 = 1;                                                     % Velocity of driven axle= +1/-1 forward/reverse [m/sec] 
                L0f = 3.8;                                                  % Wheelbase of tractor [m]
                L0b = 0.3;                                                  % Distance of 1st king-pin to tractor drive axle [m]               
                L1f = 8.475;                                                %Wheelbase of trailer [m]
%               Kinematic equations are derived for tractor-semitrailer
%               [Derivations can be found in the report]
%                 f.add(dot(x0) == v0*cos(theta0));
%                 f.add(dot(y0) == v0*sin(theta0));
%                 f.add(dot(theta0) == (v0/L0f)*tan(delta));
                A11 = L0b/L0f;
                A1 = v0*cos(theta1)*cos(gamma);
                A2 = v0*sin(theta1)*cos(gamma);
                B1 = A11*tan(delta)*tan(gamma);
                A3 = v0/L1f;
                B3 = A11*tan(delta)*cos(gamma);
                A4 = v0/L0f;
                B4 = A4*tan(delta);
                B5 = A3*sin(gamma);
                B6 = A11*A3*tan(delta)*cos(gamma);
%               
                f.add(dot(x1) == A1*(1-B1));
                f.add(dot(y1) == A2*(1-B1));
                f.add(dot(theta1) == A3*(sin(gamma)+B3));
                f.add(dot(gamma) == B4-B5-B6);
                f.add(dot(delta) == omega);
                f.add(dot(omega) == u);

                %% Optimal Control Problem
                ocp = acado.OCP(0.0,T);                                     % here you can specify the time horizon default it contains 21 points
%                 ocp = acado.OCP(0.0,T,100);                               % here you can specify the time horizon

                ocp.minimizeMayerTerm(T);                                   % Minimize a Lagrange term

                ocp.subjectTo( f );                                         % Your OCP is always subject to your 
                % differential equation  
%                 ocp.subjectTo( 'AT_START', x0 == 8.175 );  % Initial condition formula x0 = x1+(L1f*cos(theta1))-(L0b*cos(theta0));
%                 ocp.subjectTo( 'AT_START', y0 == 0.0 );  % Initial condition           y0 = y1+(L1f*sin(theta1))-(L0b*sin(theta0));         
%                 ocp.subjectTo( 'AT_START', theta0 == deg2rad(0) );  % Initial condition  theta0 = gamma+theta1;                        
                ocp.subjectTo( 'AT_START', x1 == 0.0 );                     % Initial condition
                ocp.subjectTo( 'AT_START', y1 == 0.0);                      % Initial condition
                ocp.subjectTo( 'AT_START', theta1 == deg2rad(0) );          % Initial condition 
                ocp.subjectTo( 'AT_START', gamma == deg2rad(0) );           % Initial condition
                ocp.subjectTo( 'AT_START', delta == deg2rad(0) );           % Initial condition
                ocp.subjectTo( 'AT_START', omega == 0.0 );                  % Initial condition
                ocp.subjectTo( 'AT_START', u == 0.0 );                      % Initial condition


%                 ocp.subjectTo( 'AT_END'  , x1 == xval );
%                 ocp.subjectTo( 'AT_END'  , y1 == yval );
                ocp.subjectTo( 'AT_END'  , theta1 == deg2rad(tval) );
                ocp.subjectTo( 'AT_END'  , gamma == deg2rad(gval) );
                ocp.subjectTo( 'AT_END'  , delta == deg2rad(0) );
                ocp.subjectTo( 'AT_END'  , omega == 0.0 );

% % 
%                 ocp.subjectTo( 0 <= x1 <= 30 );
%                 ocp.subjectTo( 0 <= y1 <= 30 );
                ocp.subjectTo( -deg2rad(70) <= gamma <= deg2rad(70) );
                ocp.subjectTo( -deg2rad(23) <= delta <= deg2rad(23) );
                % Conservative: 0.0765, Short: 0.13 
                ocp.subjectTo( -0.13 <= omega <= 0.13 );                    % For timihorizon alone
%                 ocp.subjectTo( -0.0765 <= omega <= 0.0765);               % For timehorizon_increasingtimeconstraint
                ocp.subjectTo( -4 <= u <= 4 );
                ocp.subjectTo( 10 <= T <= 45);                              % For timihorizon alone
%                 ocp.subjectTo( 10 <= T <=100);                            % For timehorizon_increasingtimeconstraint

                %% Optimization Algorithm
                algo = acado.OptimizationAlgorithm(ocp);                    % Set up the optimization algorithm

                algo.set('DISCRETIZATION_TYPE','COLLOCATION');
                algo.set('MAX_NUM_ITERATIONS' , 300) ; 
                algo.set( 'KKT_TOLERANCE', 1);
                algo.set( 'INTEGRATOR_TYPE' , 'INT_BDF' ) ;
                algo.set( 'INTEGRATOR_TOLERANCE' , 1e-1) ;
                algo.set( 'ABSOLUTE_TOLERANCE' , 1e-1) ;
                algo.set( 'HESSIAN_APPROXIMATION' , 'EXACT_HESSIAN' );
                
%                 algo.set('DYNAMIC_SENSITIVITY','BACKWARD_SENSITIVITY');
%                 algo.set('OBJECTIVE_SENSITIVITY','BACKWARD_SENSITIVITY');
%                 algo.set('CONSTRAINT_SENSITIVITY','BACKWARD_SENSITIVITY');
                


            END_ACADO;           % Always end with "END_ACADO".  
                                 % This will generate a file problemname_ACADO.m. 
                                 % Run this file to get your results. You can

                                 % run the file problemname_ACADO.m as many
                                 % times as you want without having to compile again.



            out = Motionprimitives_acado_RUN();% Run the test
% ACADO code ends here.
            file=strcat(folder,'\','mp_',num2str(wrapTo360(tval)),'_',num2str(gval),'.mat'); 
            save (file,'out'); % Save file of MP
            

        
        end

    end