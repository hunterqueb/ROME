function qSaved = poseTrackingOpen(path_AR2_J,Stepper1)
%path is the position of the arm that you want to initially drive the arm
%to and hold for the duration of the demonstration
%path_AR2_J is of length (1,1:6)


%Stepper1 is the stepper motor object that defines the stepper motors in
%matlab using the custom library

%% Robot Definition
d1 = 169.77;    a1 = 64.2;  alpha1 = -90*pi/180;    
d2 = 0;         a2 = 305;   alpha2 = 0;             
d3 = 0;         a3 = 0;     alpha3 = 90*pi/180;     
d4 = -222.63;   a4 = 0;     alpha4 = -90*pi/180;    
d5 = 0;         a5 = 0;     alpha5 = 90*pi/180;     
d6 = -36.25;    a6 = 0;     alpha6 = 0; 

L(1) = Link([0 d1 a1 alpha1], 'R');
L(2) = Link([0 d2 a2 alpha2], 'R');
L(3) = Link([0 d3 a3 alpha3], 'R');
L(4) = Link([0 d4 a4 alpha4], 'R');
L(5) = Link([0 d5 a5 alpha5], 'R');
L(6) = Link([0 d6 a6 alpha6], 'R');

global Robot
Robot = SerialLink(L);

%% Arm initialization

% move the arm to the inital joint configuration 
statesArray_AR2_J = [path_AR2_J(1),path_AR2_J(2),path_AR2_J(3)...
               path_AR2_J(4),path_AR2_J(5),path_AR2_J(6)...
               .25,.25,.25,.25,.25,.25];
Stepper1.updateStates(statesArray_AR2_J);

command0_AR2_J = path_AR2_J(1:6);

[command0_AR2_W_pos, command0_AR2_W_ori] = AR2FKZYZ(command0_AR2_J);
global initPos
global initOri

initPos = command0_AR2_W_pos;
initOri = command0_AR2_W_ori;

% %Gains
% Kp = eye(3);
% Ko = eye(3);

%% Path Definitons - updated per experiment, need a better implementation for this


%% NatNet Connection
% not needed for open loop

% natnetclient = natnet;
% natnetclient.HostIP = '127.0.0.1';
% natnetclient.ClientIP = '127.0.0.1';
% natnetclient.ConnectionType = 'Multicast';
% natnetclient.connect;
% 
% if ( natnetclient.IsConnected == 0 )
% 	fprintf( 'Client failed to connect\n' )
% 	fprintf( '\tMake sure the host is connected to the network\n' )
% 	fprintf( '\tand that the host and client IP addresses are correct\n\n' )
% 	return
% end
% 
% model = natnetclient.getModelDescription;
% if ( model.RigidBodyCount < 1 )
% 	return
% end

%% Notes
%Ri- inital orentation
%Rf - final orentation

%Rotiation matrix  Rif=[Rix, Riy, Riz]'*[Rfx, Rfy, Rfz] = [1 2 3;
%                                                          4 5 6;
%                                                          7 8 9;];
%for Ri=Rf, Rif=I
% Rif=eye(3);

% nuef=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2);     %angle of rotation about axis r
% r=1/(2*sin(nuef) * [Rif(3,2)-Rif(2,3);Rif(1,3)-Rif(3,1);Rif(2,1)-Rif(1,2);]);      %unit vector of the axis of rotation

%eul vectors are defined by the orentation of the end effector in terms of zyz rotations
    %reference eul angle defined by PHI, THETA, PSI.


%% Final Checks - Before proceeding, make sure you don't break anything 
disp('Final Checks Before Experiment')
pause(2)
disp(' ')
disp('Press Any Key to Continue')
disp(' ')
disp('Rigid Body Defined in Motive?')
pause
disp('Weights In Place?')
pause
disp('Are you Ready? Experiment Will Start Immediately After Key Press.')
pause
disp('Executing!')

%% Main
% set main iterator and initiate time for the while loop to start
i=1;
tic
while(toc<10)  
    %on first iteration, initialize variables that constantly update
    if (i == 1)
        AR2_error_W = getError_init(command0_AR2_W_pos, command0_AR2_W_ori, command0_AR2_J);
        errorPostition_W=AR2_error_W(1:3);
        errorOrientation_W=AR2_error_W(4:6);
        configuration=[command0_AR2_J; errorPostition_W; errorOrientation_W];
        
        %timestep for RK4
        h = 0.0155;
%     else
%         
%         h = qSaved(:,i-1) - tTime;
    end
%   during calculations of the RK4 solver, there needs to be a universal
%   time value used for each element of the RK4 variables k1, k2, k3, and
%   k4

   tTime=toc;

%     calculate xdot_global theta_global(orientaiton) thetadot_(global) through forward kinematics
%     this is where the loop will be closed


% RK4 Solver

    k_1 = AR2KinDE(configuration,xdot_ref(tTime),theta_ref(tTime),thetadot_ref(tTime));
    k_2 = AR2KinDE((configuration+0.5*h*k_1),xdot_ref(tTime+0.5*h),theta_ref(tTime+0.5*h),thetadot_ref(tTime+0.5*h));
    k_3 = AR2KinDE((configuration+0.5*h*k_2),xdot_ref(tTime+0.5*h),theta_ref(tTime+0.5*h),thetadot_ref(tTime+0.5*h));
    k_4 = AR2KinDE((configuration+k_3*h),xdot_ref(tTime+h),theta_ref(tTime+h),thetadot_ref(tTime+h));
    configuration = configuration + ((1/6)*(k_1+2*k_2+2*k_3+k_4)*h); 
    % output configuration is a large vector that solves for q, and error with AR2KinDE
    % having the output of q dot and error dot

    command_AR2_J=configuration(1:6);
    command_dot_AR2_J=k_1(1:6);
    AR2_error_W=configuration(7:12);
    
    % saving the commands given to the arm at each timestep for analysis
    qSaved(i,:)= [tTime, command_AR2_J', command_dot_AR2_J',AR2_error_W'];
    
    statesArray_AR2_J = [command_AR2_J(1),command_AR2_J(2),command_AR2_J(3),command_AR2_J(4),command_AR2_J(5),command_AR2_J(6),command_dot_AR2_J(1),command_dot_AR2_J(2),command_dot_AR2_J(3),command_dot_AR2_J(4),command_dot_AR2_J(5),command_dot_AR2_J(6)];
    Stepper1.updateStates(statesArray_AR2_J);

    
%   should we close the loop here?
    %update after
%     J = JacobionAR2(q);

%     J = JacobionAR2tool(q);
    %these will be read from the camera system
%     xdot_ref=J*qdot;
%     [~, theta_ref] = AR2fk(q);
%     [~, thetadot_ref] = AR2fk(qdot);

%     [~, theta_ref] = AR2fkine(q);
%     [~, thetadot_ref] = AR2fkine(qdot);
    
    i=i+1;
end
% at the end of the manuever, rest the arm and plot the output
Stepper1.default("REST");

figure
plot(qSaved(:,1),qSaved(:,2:7)*180/pi)
xlabel('time [sec]')
ylabel('Configuration variables [deg]')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6')


end
