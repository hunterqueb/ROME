%AR2 RNE Dynamics Model
% http://www.petercorke.com/RTB/r9/html/Link.html
% Bm: link viscous friction- Link.B
% Tc: link Coulomb friction use the function- Link.Tc
% G: gear ratio- Link.G
% m- Link mass
% r- link centre of gravity (3x1)

%DH First, then inertia matrices for each 


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

Kp = 1;
Ki = 0;
Kd = 0;

index = 1;
while t < 10
%     inverse starts
t = t + h;
qR(index,:) = [      10*sin(t)   10*sin(t)   10*sin(t)         10*sin(t)       10*sin(t)  10*sin(t)  ]; % rad
qdotR(index,:) = [   10*cos(t)   10*cos(t)  10*cos(t)         10*cos(t)       10*cos(t)   10*cos(t)];
qddotR(index,:) = [  -10*sin(t)  -10*sin(t)  -10*sin(t)        -10*sin(t)           -10*sin(t)    -10*sin(t) ];

qSim(index,:) = [0, 0, 0, 0, 0, 0];
qdotSim(index,:) = [10, 10, 10, 10, 10, 10];
qddotSim(index,:) = [14, 14, 14, 14 ,14 ,14];


qddot(index,:) = qddotSim(index,:) - qddotR(index,:);    


if index == 1
    qddotCorr = Kp*qddot(index,:) + Ki*(qddot(index,:)*h);
else
    qddotCorr = Kp*qddot(index,:) + Ki*(qddot(index,:)*h + qddot(index-1,:) + Kd/h*(qddot(index,:)-qddot(index-1,:)));
end



% tau(index,:) = Robot.rne(q(index,:),qdot(index,:),qddot(index,:));
tau(index,:) = Robot.rne(qSim(index,:),qdotSim(index,:),qddotCorr);
% inverse ends

% forward starts
CandGVectors(index,:) = Robot.rne(qR(index,:),qdotR(index,:),zeros(1,6));
% CandGVectors(index,:) = Robot.rne(qCorr,qdotCorr,zeros(1,6));

Robot.gravity = [0 0 0]';

for i = 1:6
    qddotMass = zeros(1,6);
    qddotMass(i) = 1;
    M(index,:,i) = Robot.rne(qR(index,:),zeros(1,6),qddotMass);
%     M(index,:,i) = Robot.rne(qCorr,zeros(1,6),qddotMass);
end

MMat = squeeze(M(index,:,:));
qddotSim(index,:) = pinv(MMat)*(tau(index,:)' - CandGVectors(index,:)');

if index == 1
    qdotSim(index,:) = h*qddotSim(index,:);
	qSim(index,:) = h*qdotSim(index,:);

else
    qdotSim(index,:) = h*qddotSim(index,:) + qdotSim(index-1,:);
	qSim(index,:) = h*qdotSim(index,:) + qSim(index-1,:);

end

% forward ends





% reset 
Robot.gravity = [0 0 9.81]';

index = index + 1;
    
end


tGraph = 0:0.1:10;
tGraph = tGraph';

angleToView = 6;

figure
plot(qddotR(:,angleToView))
hold on;
plot(qddotSim(:,angleToView))
% l = legend('Analytical Generation $\ddot{\theta_1}$','Analytical Generation $\ddot{\theta_2}$','Analytical Generation $\ddot{\theta_3}$','Analytical Generation $\ddot{\theta_4}$','Analytical Generation $\ddot{\theta_5}$','Analytical Generation $\ddot{\theta_6}$',...
%     'Simulation $\ddot{\theta_1}$','Simulation $\ddot{\theta_2}$','Simulation $\ddot{\theta_3}$','Simulation $\ddot{\theta_4}$','Simulation $\ddot{\theta_5}$','Simulation $\ddot{\theta_6}$');
% 
% set(l, 'Interpreter', 'latex','FontSize',12);
grid on

xlabel('Time [sec]','FontSize',12)
ylabel('Acceleration [rad/s^2]','FontSize',12)




