%AR2 RNE Dynamics Model
% http://www.petercorke.com/RTB/r9/html/Link.html
% Bm: link viscous friction- Link.B
% Tc: link Coulomb friction use the function- Link.Tc
% G: gear ratio- Link.G
% m- Link mass
% r- link centre of gravity (3x1)

%DH First, then inertia matrices for each 

clear
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

% timestep for sim
h = 0.01;

% initialize the time variable
t = 0;
tSim = 10;

% Control Parameters
KpInner = 1.2 * eye(6);
KdInner = 10 * eye(6);

% initial arm pos in joint
theta0 = [0,-110*pi/180,141*pi/180,0,0,0];

% initial arm pos in task space
[state0,ori0] = AR2FKZYZ(theta0);

maxSinAmount = 200;

index = 1;
while t < tSim
%   desired equations of motion

    xd(index,:) = [state0(1),0,maxSinAmount*sin(2*pi*t/tSim)+state0(3),0,0,0];
    xdotd(index,:) = [0,0,maxSinAmount*cos(2*pi*t/tSim) * 2 * pi / tSim,0,0,0];
    xddotd(index,:) = [0,0,maxSinAmount*-sin(2*pi*t/tSim) * (2 * pi / tSim)^2,0,0,0];
    
%   inital conditions
    xIC(index,:) = [state0',ori0'];
    xdotIC(index,:) = [0, 0, 0, 0, 0, 0];
	xddotIC(index,:) = [0, 0, 0, 0 ,0 ,0];
    
%   on first pass, set the IC to first array index for all the simulated
%   configurations
    if index == 1
        xSim(index,:) = xIC(index,:);
        xdotSim(index,:) = xdotIC(index,:);
        xddotSim(index,:) = xddotIC(index,:);
        
        qSim(index,:) = theta0;
        qdotSim(index,:) = pinv(Jacobian0_analytical(qSim(index,:))) * xdotSim(index,:)';
    else
%       forward integrate to get the task space motions
        xdotSim(index,:) = h*xddotSim(index-1,:) + xdotSim(index-1,:);
        xSim(index,:) = h*xdotSim(index,:) + xSim(index-1,:);
        
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
        xddotSim(index,:) = (xddotd(index,:)' + KdInner * (xdotd(index,:)'-xdotSim(index,:)') + KpInner * (xd(index,:)' - xSim(index,:)'))';
    end
    
    
% update loop
index = index + 1;
t = t + h;

end

dimToView = 3;

figure
plot(xd(:,dimToView));
hold on;
plot(xSimG(:,dimToView));
title('Corrected Inner Loop Position')
grid on

xlabel('Iterations','FontSize',12)
ylabel('Position [mm]','FontSize',12)
legend('Desired Position','Simulated Position')

figure
plot(xdotd(:,dimToView));
hold on;
plot(xdotSimG(:,dimToView));
title('Corrected Inner Loop Velocity')
grid on

xlabel('Iterations','FontSize',12)
ylabel('Velocity [mm/s]','FontSize',12)
legend('Desired Velocity','Simulated Velocity')


figure
plot(xddotd(:,dimToView));
hold on;
plot(xddotSim(:,dimToView));
title('Corrected Inner Loop Acceleration')
grid on

xlabel('Iterations','FontSize',12)
ylabel('Acceleration [mm/s^2]','FontSize',12)
legend('Desired Acceleration','Simulated Acceleration')


errorInnerPos = xSimG - xd;
errorInnerVel = xdotSimG - xdotd;
errorInnerAcc = xddotSim - xddotd;

meanErrPos = mean(errorInnerPos);
meanErrPosPercent = max(meanErrPos)/maxSinAmount * 100
meanErrVel = mean(errorInnerVel);
meanErrVelPercent = max(meanErrVel)/maxSinAmount * 100
meanErrAcc = mean(errorInnerAcc);
meanErrAccPercent = max(meanErrAcc)/maxSinAmount * 100
% units in mm
