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
h = 0.005;

% initialize the time variable
t = 0;

% Control Parameters
KpInner = 1.2 * eye(6);
KdInner = 10 * eye(6);

% this would be the outer loop variables
KOuter = 1 * eye(6);

index = 1;

xd = zeros(6,1);
xSim = zeros(6,1);
while t < 10
%     inverse dynamics starts
qd(index,:) = [      10*sin(t)   10*sin(t)   10*sin(t)         10*sin(t)       10*sin(t)  10*sin(t)  ]; % rad
qdotd(index,:) = [   10*cos(t)   10*cos(t)  10*cos(t)         10*cos(t)       10*cos(t)   10*cos(t)];
qddotd(index,:) = [  -10*sin(t)  -10*sin(t)  -10*sin(t)        -10*sin(t)           -10*sin(t)    -10*sin(t) ];


qIC(index,:) = pi/180 * [20, 20, 20, 20, 20, 20];
qdotIC(index,:) = [0, 0, 0, 0, 0, 0];
qddotIC(index,:) = [0, 0, 0, 0 ,0 ,0];


if index == 1
    qSim(index,:) = qIC(index,:);
    qdotSim(index,:) = qdotIC(index,:);
    qddotSim(index,:) = qddotIC(index,:);
else
    qdotSim(index,:) = h*qddotSim(index-1,:) + qdotSim(index-1,:);
	qSim(index,:) = h*qdotSim(index,:) + qSim(index-1,:);
    % inner control loop is here
    qddotSim(index,:) = (qddotd(index,:)' + KdInner * (qdotd(index,:)'-qdotSim(index,:)') + KpInner * (qd(index,:)' - qSim(index,:)'))';
end

Jd = Jacobian0_analytical(qd);
JSim = Jacobian0_analytical(qSim);

[xd(1:3),xd(4:6)] = AR2FKZYZ(qd);
[xSim(1:3),xSim(4:6)] = AR2FKZYZ(qSim);

xdotd(index,:) = Jd * qdotd(index,:)';

taud(index,:) = Robot.rne(qd(index,:),qdotd(index,:),qddotd(index,:));
tauSim(index,:) = Robot.rne(qSim(index,:),qdotSim(index,:),qddotSim(index,:));
    % we can specify the wrench using this, by calling a fourth parameter
    % 'fext',W where W is a 6x1 vector [fx,fy,fz,rx,ry,rz]

% inverse ends


% this is where the outer loop is added

% xdotd is the desired cartesian space 1st derivative
% ePos is the error e = xd - xactual;

ePos(index,:) = xd - xSim;

qdotCorrected(index,:) = pinv(JSim) * (xdotd(index,:)' + KOuter * ePos(index,:)'); % + (eye(n) - pinv(J)*J*qdot0; 
                                     % this term is added for a redundant
                                     % manipulator
                        
if (index == 1)
    qCorrected(index,:) = qIC(index,:);
else
    qCorrected(index,:) = h*qdotCorrected(index,:) + qCorrected(index-1,:);
end
% % forward starts -- this is here just for checking the accuracy of the
% % forward and inverse dynamics functions
% 
% CandGVectors(index,:) = Robot.rne(qd(index,:),qdotd(index,:),zeros(1,6));
% 
% Robot.gravity = [0 0 0]';
% 
% for i = 1:6
%     qddotMass = zeros(1,6);
%     qddotMass(i) = 1;
%     M(index,:,i) = Robot.rne(qd(index,:),zeros(1,6),qddotMass);
% end
% 
% MMat = squeeze(M(index,:,:));
% qddotdForw(index,:) = pinv(MMat)*(taud(index,:)' - CandGVectors(index,:)');
% here the wrench term (J(q)' * W) is also subtracted
% qddotSim(index+1,:) = pinv(MMat)*(tauCorr' - CandGVectors(index,:)');
% 
% % reset gravity from forward dynamics
% Robot.gravity = [0 0 9.81]';
% % forward ends


% update loop counters and time variable
index = index + 1;
t = t + h;

end

angleToView = 1;

% graph to check the acceleration results without the outer loop
figure
plot(qddotSim(:,1));
hold on
plot(qddotd(:,1));
title('Corrected Acceleration')
grid on

xlabel('Iterations','FontSize',12)
ylabel('Acceleration [rad/s^2]','FontSize',12)

% graph to check the velocity results without the outer loop
figure
plot(qdotSim(:,1));
hold on
plot(qdotd(:,1));
title('Corrected Inner Loop Velocity')
grid on

xlabel('Iterations','FontSize',12)
ylabel('Velocity [rad/s]','FontSize',12)

% graph to check the position results without the outer loop
figure
plot(qSim(:,1));
hold on
plot(qd(:,1));
title('Corrected Inner Loop Position')

grid on

xlabel('Iterations','FontSize',12)
ylabel('Position [rad]','FontSize',12)

% calculate the error of the acceleration using just the inner loop
errorInnerAcc = qddotSim - qddotd;
meanErrorInnerAcc(1) = mean(errorInnerAcc(:,1));
meanErrorInnerAcc(2) = mean(errorInnerAcc(:,2));
meanErrorInnerAcc(3) = mean(errorInnerAcc(:,3));
meanErrorInnerAcc(4) = mean(errorInnerAcc(:,4));
meanErrorInnerAcc(5) = mean(errorInnerAcc(:,5));
meanErrorInnerAcc(6) = mean(errorInnerAcc(:,6));

meanErrorInnerAcc

% this graph is to check the velocity results with the outer loop
figure
plot(qdotCorrected(:,angleToView));
hold on
plot(qdotd(:,angleToView));
title('Corrected Outer Loop Velocity')

grid on

xlabel('Iterations','FontSize',12)
ylabel('Velocity [rad/s]','FontSize',12)


errorOuterVel = qdotCorrected - qdotd;
meanErrorOuterVel(1) = mean(errorOuterVel(:,1));
meanErrorOuterVel(2) = mean(errorOuterVel(:,2));
meanErrorOuterVel(3) = mean(errorOuterVel(:,3));
meanErrorOuterVel(4) = mean(errorOuterVel(:,4));
meanErrorOuterVel(5) = mean(errorOuterVel(:,5));
meanErrorOuterVel(6) = mean(errorOuterVel(:,6));

errorInnerVel = qdotSim - qdotd;
meanErrorInnerVel(1) = mean(errorInnerVel(:,1));
meanErrorInnerVel(2) = mean(errorInnerVel(:,2));
meanErrorInnerVel(3) = mean(errorInnerVel(:,3));
meanErrorInnerVel(4) = mean(errorInnerVel(:,4));
meanErrorInnerVel(5) = mean(errorInnerVel(:,5));
meanErrorInnerVel(6) = mean(errorInnerVel(:,6));

meanErrorInnerVel
meanErrorOuterVel


% this graph is to check the position results with the outer loop

figure
plot(qCorrected(:,angleToView));
hold on
plot(qd(:,angleToView));
title('Corrected Outer Loop Position')

grid on

xlabel('Iterations','FontSize',12)
ylabel('Position [rad]','FontSize',12)


errorOuterPos = qCorrected - qd;
meanErrorOuterPos(1) = mean(errorOuterPos(:,1));
meanErrorOuterPos(2) = mean(errorOuterPos(:,2));
meanErrorOuterPos(3) = mean(errorOuterPos(:,3));
meanErrorOuterPos(4) = mean(errorOuterPos(:,4));
meanErrorOuterPos(5) = mean(errorOuterPos(:,5));
meanErrorOuterPos(6) = mean(errorOuterPos(:,6));

errorInnerPos = qSim - qd;
meanErrorInnerPos(1) = mean(errorInnerPos(:,1));
meanErrorInnerPos(2) = mean(errorInnerPos(:,2));
meanErrorInnerPos(3) = mean(errorInnerPos(:,3));
meanErrorInnerPos(4) = mean(errorInnerPos(:,4));
meanErrorInnerPos(5) = mean(errorInnerPos(:,5));
meanErrorInnerPos(6) = mean(errorInnerPos(:,6));

meanErrorInnerPos
meanErrorOuterPos


