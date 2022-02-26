%AR2 RNE Dynamics Model
% http://www.petercorke.com/RTB/r9/html/Link.html
% Bm: link viscous friction- Link.B
% Tc: link Coulomb friction use the function- Link.Tc
% G: gear ratio- Link.G
% m- Link mass
% r- link centre of gravity (3x1)

%DH First, then inertia matrices for each 


rest = [0,-110*pi/180,141*pi/180,0,0,0];
theta0 = pi/180*[0,-70,90,0,12,0];
home = pi/180*[0,-90,90,0,0,0];



%% NatNet Connection
natnetclient = natnet;
natnetclient.HostIP = '127.0.0.1';
natnetclient.ClientIP = '127.0.0.1';
natnetclient.ConnectionType = 'Multicast';
natnetclient.connect;

if ( natnetclient.IsConnected == 0 )
	fprintf( 'Client failed to connect\n' )
	fprintf( '\tMake sure the host is connected to the network\n' )
	fprintf( '\tand that the host and client IP addresses are correct\n\n' )
	return
end

%% Move to initial position
pause(5);
statesArray = [theta0(1),theta0(2),theta0(3),...
               theta0(4),theta0(5),theta0(6),...
               .2,.2,.2,.2,.2,.2];
AR3.updateStates(statesArray);
pause(10);

%%

L(1) = Link([0 169.77/1000 64.2/1000 -1.5707], 'R');
L(2) = Link([0 0 305/1000 0], 'R');
L(3) = Link([-1.5707 0 0 1.5707], 'R');
L(4) = Link([0 -222.63/1000 0 -1.5707], 'R');
L(5) = Link([0 0 0 1.5707], 'R');
L(6) = Link([pi -36.25/1000 0 0], 'R');


%load inertia matrices
inertias

%set inertia matrices for each link
L(1).I=I1;
L(2).I=I2;
L(3).I=I3;
L(4).I=I4;
L(5).I=I5;
L(6).I=I6;

%link masses taken from urdf

L(1).m=0.88065;
L(2).m=0.57738;
L(3).m=0.1787;
L(4).m=0.34936;
L(5).m=0.11562;
L(6).m=0.013863;

% center of mass of all links taken from urdf
L(1).r=[-0.022706 0.04294 -0.12205];
L(2).r=[0.064818 -0.11189 -0.038671];
L(3).r=[-0.00029765 -0.023661 -0.0019125];
L(4).r=[-0.0016798 -0.00057319 -0.074404];
L(5).r=[0.0015066 -1.3102E-05 -0.012585];
L(6).r=[2.9287E-10 -1.6472E-09 0.0091432];


% assuming friction to be zero in this model
L(1).Jm=0;
L(2).Jm=0;
L(3).Jm=0;
L(4).Jm=0;
L(5).Jm=0;
L(6).Jm=0;

Robot = SerialLink(L);
Robot.name = 'AR2';

Robot.nofriction('all');
Robot.gravity = [0 0 9.81]';



% initialize the time variable
tSim = 30;

% Control Parameters
KpInner = 4 * eye(6);
KdInner = 40 * eye(6);

% initial arm pos in joint
% theta0 = [0,-110*pi/180,141*pi/180,0,0,0];

% initial arm pos in task space
[state0,ori0] = AR2FKZYZ(theta0);
% 
% offsetTot = getTransformationMatrix(theta0,0);
% 
% offset = offsetTot(1:3);
% offsetAng = offsetTot(4:6);


maxSinAmount = 200;

index = 1;
tic

while toc < tSim
    t = toc;
    
%     calculate xdot_global theta_global(orientaiton) thetadot_(global) through forward kinematics
%     this is where the loop will be closed
    data = natnetclient.getFrame;
    if (isempty(data.LabeledMarker(1)))
        fprintf( '\tPacket is empty/stale\n' )
        fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
        return
    end
    yaw = data.RigidBody(1).qy;
    pitch = data.RigidBody(1).qz;
    roll = data.RigidBody(1).qx;
    scalar = data.RigidBody(1).qw;
    quat = quaternion(roll,yaw,pitch,scalar);
    qRot = quaternion(0,0,0,1);
    quat = mtimes(quat,qRot);
    anglesFromCamera = EulerAngles(quat,'zyz');
    positionFromCamera = 1000*[-data.RigidBody(1).x;data.RigidBody(1).z;data.RigidBody(1).y;];

%     xSim,xdotSim
%     actualWorkPos = positionFromCamera - offset';
%     actualWorkOri = anglesFromCamera - offsetAng';
%   consider the angles not matter right now for first tests
    actualWorkOri = [0 0 0]';
    
%   desired equations of motion

    xd(index,:) = [state0(1),0,maxSinAmount*sin(2*pi*t/tSim)+state0(3),0,0,0];
    xdotd(index,:) = [0,0,maxSinAmount*cos(2*pi*t/tSim) * 2 * pi / tSim,0,0,0];
    xddotd(index,:) = [0,0,maxSinAmount*-sin(2*pi*t/tSim) * (2 * pi / tSim)^2,0,0,0];
    
%   inital conditions
    
%   on first pass, set the IC to first array index for all the simulated
%   configurations
    if index == 1
        xIC(index,:) = [state0',ori0'];
        xdotIC(index,:) = [0, 0, 0, 0, 0, 0];
        xddotIC(index,:) = [0, 0, 0, 0 ,0 ,0];
    
        xSim(index,:) = xIC(index,:);
        xdotSim(index,:) = xdotIC(index,:);
        xddotSim(index,:) = xddotIC(index,:);
        
        qSim(index,:) = theta0;
        qdotSim(index,:) = pinv(Jacobian0_analytical(qSim(index,:))) * xdotSim(index,:)';
        qddotSim(index,:) = [0 0 0 0 0 0];
        qddotCommand(index,:) = qddotSim(index,:);
        qdotCommand(index,:) = [0 0 0 0 0 0];
        qCommand(index,:) = theta0;
        h = t;
    else
        h = t - qSaved(index-1,1);
%         xSim(index,:) = [actualWorkPos,actualWorkOri];
        
%       remove for closed loop
        [state0,ori0] = AR2FKZYZ(qCommand(index-1,:));
        xSim(index,:) = [state0',ori0'];
        
        xdotSim(index,:) =  (xSim(index,:) - xSim(index-1,:))/h;
%       using task space motions, get the joint space motions 
        qdotSim(index,:) = pinv(Jacobian0_analytical(qSim(index-1,:))) * xdotSim(index,:)';

            % this calculation for the qdotSim is based on the previous time step calculation of the joint angles. this 
%             WILL cause issues for calculation accuracy, but i hope the
%             controller is robust enough to couteract these effects
        qSim(index,:) = h*qdotSim(index,:) + qSim(index-1,:);
        
%       recalc the task space motions based on the new joint angle
%       configuration estimations
        xdotSimG(index,:) = Jacobian0_analytical(qSim(index,:)) * qdotSim(index,:)';
        
        [xSimG(index,1:3),xSimG(index,4:6)] = AR2FKZYZ(qSim(index,:));
        
        % inner control loop is here
        xddotSim(index,:) = (xddotd(index,:)' + KdInner * (xdotd(index,:)'-xdotSimG(index,:)') + KpInner * (xd(index,:)' - xSimG(index,:)'))';  
        
%       take xddotSim to q and qdot
        qddotCommand(index,:) = tAccel2jAccelRobot(qSim(index,:)',qdotSim(index,:)',xddotSim(index,:)',Robot);
        qdotCommand(index,:) = h*qddotCommand(index,:) + qddotCommand(index-1,:);
        qCommand(index,:) = h*qdotCommand(index,:) + qCommand(index-1,:);

        statesArray_AR2_J = [qCommand(1),qCommand(2),qCommand(3),qCommand(4),qCommand(5),qCommand(6),...
            qdotCommand(1),qdotCommand(2),qdotCommand(3),qdotCommand(4),qdotCommand(5),qdotCommand(6)];
        AR3.updateStates(statesArray_AR2_J);
        
        
    end


    
% update loop
qSaved(index,:)= [t, qCommand(index,:), qdotCommand(index,:), xSim(index,:), xdotSim(index,:), xddotSim(index,:)];
index = index + 1;
t = t + h;

end
% tGraph = h:h:tSim;
% tGraph = tGraph';
% 
% dimToView = 3;
% 
% figure
% plot(tGraph,xd(:,dimToView));
% hold on;
% plot(tGraph,xSimG(:,dimToView));
% % title('Corrected Inner Loop Position')
% grid on
% xlabel('Time [s]','FontSize',12)
% ylabel('Position [mm]','FontSize',12)
% legend('Desired Position','Simulated Position')
% 
% figure
% plot(tGraph,xdotd(:,dimToView));
% hold on;
% plot(tGraph,xdotSimG(:,dimToView));
% % title('Corrected Inner Loop Velocity')
% grid on
% 
% xlabel('Time [s]','FontSize',12)
% ylabel('Velocity [mm/s]','FontSize',12)
% legend('Desired Velocity','Simulated Velocity')
% 
% 
% figure
% plot(tGraph,xddotd(:,dimToView));
% hold on;
% plot(tGraph,xddotSim(:,dimToView));
% % title('Corrected Inner Loop Acceleration')
% grid on
% 
% xlabel('Time [s]','FontSize',12)
% ylabel('Acceleration [mm/s^2]','FontSize',12)
% legend('Desired Acceleration','Simulated Acceleration')
% 
% 
% errorInnerPos = xSimG(:,dimToView) - xd(:,dimToView);
% errorInnerVel = xdotSimG(:,dimToView) - xdotd(:,dimToView);
% errorInnerAcc = xddotSim(:,dimToView) - xddotd(:,dimToView);
% 
% meanErrPos = mean(errorInnerPos);
% meanErrPosPercent = max(meanErrPos)/maxSinAmount * 100
% meanErrVel = mean(errorInnerVel);
% meanErrVelPercent = max(meanErrVel)/maxSinAmount * 100
% meanErrAcc = mean(errorInnerAcc);
% meanErrAccPercent = max(meanErrAcc)/maxSinAmount * 100
% % units in mm