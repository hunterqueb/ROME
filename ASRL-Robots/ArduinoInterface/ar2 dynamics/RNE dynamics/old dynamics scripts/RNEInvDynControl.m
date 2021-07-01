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

h = 0.1;
t = 0;

Kpval = 0.003;
Kdval = 2*Kpval;
Kival = 1;

% Kpval = 0;
% Kdval = 0;
% Kival = 0;

Kp = [Kpval 0 0 0 0 0;0 Kpval 0 0 0 0;0 0 Kpval 0 0 0;0 0 0 Kpval 0 0;0 0 0 0 Kpval 0;0 0 0 0 0 Kpval;];
Kd = [Kdval 0 0 0 0 0;0 Kdval 0 0 0 0;0 0 Kdval 0 0 0;0 0 0 Kdval 0 0;0 0 0 0 Kdval 0;0 0 0 0 0 Kdval;];
Ki = [Kival 0 0 0 0 0;0 Kival 0 0 0 0;0 0 Kival 0 0 0;0 0 0 Kival 0 0;0 0 0 0 Kival 0;0 0 0 0 0 Kival;];
index = 1;

qIC = [180, 180, 180, 180, 180, 180];
qdotIC = [-20, -20, -20, -20, -20, -20];
qddotIC = [0, 0, 0, 0, 0, 0];


time = 20;


while t < time
%     inverse starts
% qR(index,:) = [      10*sin(t)   10*sin(t)   10*sin(t)         10*sin(t)       10*sin(t)  10*sin(t)  ]; % rad
% qdotR(index,:) = [   10*cos(t)   10*cos(t)  10*cos(t)         10*cos(t)       10*cos(t)   10*cos(t)];
% qddotR(index,:) = [  -10*sin(t)  -10*sin(t)  -10*sin(t)        -10*sin(t)           -10*sin(t)    -10*sin(t) ];

qR(index,:) = [      10*sin(t) .5*t^3 .5*t^3 .5*t^3 .5*t^3 .5*t^3  ]; % rad
qdotR(index,:) = [   10*cos(t)   1.5*t^2 1.5*t^2 1.5*t^2 1.5*t^2 1.5*t^2];
qddotR(index,:) = [  -10*sin(t)  3*t  3*t        3*t           3*t    3*t ];

if index == 1
    qSim(index,:) = qIC;
    qdotSim(index,:) = qdotIC;
    qddotSim(index,:) = qddotIC;  
end

y = Ki*qddotR(index,:)'- Kd*((qdotR(index,:)')-(qdotSim(index,:)')) - Kp*(qR(index,:)'-qSim(index,:)');
% r = qddotR(index,:)' + Kd*qdotR(index,:)' + Kp*qR(index,:)';
% forward starts
CandGVectors(index,:) = Robot.rne(qSim(index,:),qdotSim(index,:),zeros(1,6));

Robot.gravity = [0 0 0]';

for i = 1:6
    qddotMass = zeros(1,6);
    qddotMass(i) = 1;
    M(index,:,i) = Robot.rne(qSim(index,:),zeros(1,6),qddotMass);
end

MMat = squeeze(M(index,:,:));

tau = (MMat*y)+CandGVectors(index,:)';

qddotSim(index+1,:) = pinv(MMat)*(tau - CandGVectors(index,:)');

qdotSim(index+1,:) = h*qddotSim(index+1,:) + qdotSim(index,:);
qSim(index+1,:) = h*qdotSim(index+1,:) + qSim(index,:);


% forward ends

% reset 
Robot.gravity = [0 0 9.81]';

index = index + 1;
t = t + h;

end

tGraph = 0.1:0.1:time;
tGraph = tGraph';

qSimResults = qSim(1:200,:);
qdotSimResults = qdotSim(1:200,:);
angleToView = 1;

figure
plot(tGraph,qR(:,angleToView))
hold on;
plot(tGraph,qSimResults(:,angleToView))
l = legend('Reference Trajectory $\theta_1$','Simulated Trajectory $\theta_1$');

set(l, 'Interpreter', 'latex','FontSize',12);
grid on

xlabel('Time [sec]','FontSize',12)
ylabel('Joint Angle Position [rads]','FontSize',12)

figure
plot(tGraph,qdotR(:,angleToView))
hold on;
plot(tGraph,qdotSimResults(:,angleToView))
l = legend('Reference Trajectory $\theta_1$','Simulated Trajectory $\theta_1$');

set(l, 'Interpreter', 'latex','FontSize',12);
grid on

xlabel('Time [sec]','FontSize',12)
ylabel('Joint Angle Velocity [rads]','FontSize',12)

qError = qR - qSimResults;
% figure
% 
% plot(tGraph,qError)
% hold on;
% l = legend('Reference Trajectory $\theta_1$','Simulated Trajectory $\theta_1$');
% 
% set(l, 'Interpreter', 'latex','FontSize',12);
% grid on
% 
% xlabel('Time [sec]','FontSize',12)
% ylabel('Joint Angle Position Error [rads]','FontSize',12)




