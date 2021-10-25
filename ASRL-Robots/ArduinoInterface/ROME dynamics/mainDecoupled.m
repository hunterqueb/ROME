% The order for computing the dynamics is as follows. Starting with the desired Cartesian equations of motion, the closed loop task space dynamics for the robotic manipulator and ground vehicle are computed separately. Once calculated, the task space motion is converted to joint space, the torque via inverse dynamics are calculated using the transfer of forces by equation \eqref{eq:dynamicCoupling}, the joint space accelerations are calculated via the forward dynamics, and the joint accelerations are converted back to task space.

%AR2 RNE Dynamics Model
% http://www.petercorke.com/RTB/r9/html/Link.html
% Bm: link viscous friction- Link.B
% Tc: link Coulomb friction use the function- Link.Tc
% G: gear ratio- Link.G
% m- Link mass
% r- link centre of gravity (3x1)

%DH First, then inertia matrices for each 

clear
tic
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

% calculate the maxRadPerSec for each joint
STEPPER_CONSTANT(1) = 1/(.022368421*(pi/180));
STEPPER_CONSTANT(2) = 1/(.018082192*(pi/180));
STEPPER_CONSTANT(3) = 1/(.017834395*(pi/180));
STEPPER_CONSTANT(4) = 1/(.021710526*(pi/180));
STEPPER_CONSTANT(5) = 1/(.045901639*(pi/180));
STEPPER_CONSTANT(6) = 1/(.046792453*(pi/180));

maxStepsPerSec = 1000;
maxRadPerSec = zeros(6,1);

for i = 1:6
    maxRadPerSec(i) = maxStepsPerSec / STEPPER_CONSTANT(i);
end

% timestep for sim
h = 0.01;

% initialize the time variable
tSim = 30;

% Control Parameters
KpInner = 4 * eye(6);
KdInner = 40 * eye(6);

% initial arm pos in joint
theta0 = [0,-110*pi/180,141*pi/180,0,0,0];

% initial arm pos in task space
[state0,ori0] = AR2FKZYZ(theta0);

maxSinAmount = 200;

planarOrbitRadius = 1000;

circle = Circle(tSim,1000,0);
omni = omni();
[t,result] = omni.trajectory(circle);

pathW = circle.getPathW(t);
pathX = pathW(:,1);
pathY = pathW(:,2);
pathTheta = pathW(:,3);

pathXdot = pathW(:,4);
pathYdot = pathW(:,5);

xi = linspace(0, tSim, tSim/h);

pathX = interp1(t, pathX, xi, 'linear','extrap');
pathY = interp1(t, pathY, xi, 'linear','extrap');
pathXdot = interp1(t, pathXdot, xi, 'linear','extrap');
pathYdot = interp1(t, pathYdot, xi, 'linear','extrap');

t = 0;
index = 1;

% preallocation
sizeOfArray = tSim/h;
GVx = zeros(sizeOfArray,1);
GVy = zeros(sizeOfArray,1);
GVxdot = zeros(sizeOfArray,1);
GVydot = zeros(sizeOfArray,1);
GVxddot = zeros(sizeOfArray,1);
GVyddot = zeros(sizeOfArray,1);
xd = zeros(sizeOfArray,6);
xdotd = zeros(sizeOfArray,6);
xddotd = zeros(sizeOfArray,6);
    
xdTot = zeros(sizeOfArray,6);
xdotdTot = zeros(sizeOfArray,6);
xddotdTot = zeros(sizeOfArray,6);

xSim = zeros(sizeOfArray,6);
xdotSim = zeros(sizeOfArray,6);
xddotSim = zeros(sizeOfArray,6);

qSim = zeros(sizeOfArray,6);
qdotSim = zeros(sizeOfArray,6);

qdotCoupled = zeros(sizeOfArray,6);
qCoupled = zeros(sizeOfArray,6);
xdotSimG = zeros(sizeOfArray,6);
xdotSimTot = zeros(sizeOfArray,6);
xSimG = zeros(sizeOfArray,6);
xSimTot = zeros(sizeOfArray,6);
qddotSim = zeros(sizeOfArray,6);

tau = zeros(sizeOfArray,6);
CandGVectors = zeros(sizeOfArray,6);
qddotCoupled = zeros(sizeOfArray,6);
xddotCoupled = zeros(sizeOfArray,6);

while t < tSim
% 
% 
% 
    GVx(index) = planarOrbitRadius*cos(2*pi*t/tSim);
    GVy(index) = planarOrbitRadius*sin(2*pi*t/tSim);
    
    GVxdot(index) = planarOrbitRadius*2*pi/tSim*-sin(2*pi*t/tSim);
    GVydot(index) = planarOrbitRadius*2*pi/tSim*cos(2*pi*t/tSim);
    
    GVxddot(index) = planarOrbitRadius*(2*pi/tSim)^2*-cos(2*pi*t/tSim);
    GVyddot(index) = planarOrbitRadius*(2*pi/tSim)^2*-sin(2*pi*t/tSim);


    %   desired equations of motion

    xd(index,:) = [state0(1),0,maxSinAmount*sin(2*pi*t/tSim)+state0(3),0,0,0];
    xdotd(index,:) = [0,0,maxSinAmount*cos(2*pi*t/tSim) * 2 * pi / tSim,0,0,0];
    xddotd(index,:) = [0,0,maxSinAmount*-sin(2*pi*t/tSim) * (2 * pi / tSim)^2,0,0,0];
    
    xdTot(index,:) = xd(index,:) + [GVx(index),GVy(index),0,0,0,0];
    xdotdTot(index,:) = xdotd(index,:) + [GVxdot(index),GVydot(index),0,0,0,0];
    xddotdTot(index,:) = xddotd(index,:) + [GVxddot(index),GVyddot(index),0,0,0,0];
%   inital conditions
    xIC = [state0',ori0'];
    xdotIC = [0, 0, 0, 0, 0, 0];
	xddotIC = [0, 0, 0, 0 ,0 ,0];
    
%   on first pass, set the IC to first array index for all the simulated
%   configurations
    if index == 1
        xSim(index,:) = xIC;
        xdotSim(index,:) = xdotIC;
        xddotSim(index,:) = xddotIC;
        
        qSim(index,:) = theta0;
        qdotSim(index,:) = pinv(Jacobian0_analytical(qSim(index,:))) * xdotSim(index,:)';
        
        qdotCoupled(index,:) = [0,0,0,0,0,0];
        qCoupled(index,:) = theta0;

    else
%       forward integrate to get the task space motions
        xdotSim(index,:) = h*xddotSim(index-1,:) + xdotSim(index-1,:);
        xSim(index,:) = h*xdotSim(index,:) + xSim(index-1,:);
        
%       using task space motions, get the joint space motions 
        qdotSim(index,:) = pinv(Jacobian0_analytical(qSim(index-1,:))) * xdotSim(index,:)';
        for i = 1:6
           if qdotSim(index,i) > maxRadPerSec(i)
               qdotSim(index,i) =  maxRadPerSec(i);
           elseif qdotSim(index,i) < -maxRadPerSec(i)
               qdotSim(index,i) =  -maxRadPerSec(i);
           end
        end
            % this calculation for the qdotSim is based on the previous time step calculation of the joint angles. this 
%             WILL cause issues for calculation accuracy, but i hope the
%             controller is robust enough to couteract these effects
        qSim(index,:) = h*qdotSim(index,:) + qSim(index-1,:);
        
%       recalc the task space motions based on the new joint angle
%       configuration estimations
        xdotSimG(index,:) = Jacobian0_analytical(qSim(index,:)) * qdotSim(index,:)';
        xdotSimTot(index,:) = xdotSimG(index,:) + [pathXdot(index),pathYdot(index),0,0,0,0];

        
        [xSimG(index,1:3),xSimG(index,4:6)] = AR2FKZYZ(qSim(index,:));
        
        
        % inner control loop is here
        xddotSim(index,:) = (xddotd(index,:)' + KdInner * (xdotd(index,:)'-xdotSimG(index,:)') + KpInner * (xd(index,:)' - xSimG(index,:)'))';
        
        xSimTot(index,:) = xSimG(index,:) + [pathX(index),pathY(index),0,0,0,0];
        
        %       convert to qddot using qsim and qddot sim,
        
        
        qddotSim(index,:) = ((pinv(Jacobian0_analytical(qSim(index,:)))*(xddotSim(index,:)') - Robot.jacob_dot(qSim(index,:)',qdotSim(index,:)')))';
        
        
        GVForces = [GVxddot(index),GVyddot(index),9.81]';
        Robot.gravity = GVForces;
        tau(index,:) = Robot.rne(qSim(index,:),qdotSim(index,:),qddotSim(index,:)); %,'gravity',GVForces
        CandGVectors(index,:) = Robot.rne(qSim(index,:),qdotSim(index,:),zeros(1,6));

        Robot.gravity = [0 0 0]';

        for i = 1:6
            qddotMass = zeros(1,6);
            qddotMass(i) = 1;
            M(index,:,i) = Robot.rne(qSim(index,:),zeros(1,6),qddotMass);

        end

        MMat = squeeze(M(index,:,:));
        qddotCoupled(index,:) = pinv(MMat)*(tau(index,:)' - CandGVectors(index,:)');
        qdotCoupled(index,:) = h*qddotCoupled(index,:) + qdotCoupled(index-1,:);
        qCoupled(index,:) = h*qdotCoupled(index,:) + qCoupled(index-1,:);

        xddotCoupled(index,:) = ((Jacobian0_analytical(qSim(index,:)) * qddotSim(index,:)') + Robot.jacob_dot(qSim(index,:),qdotSim(index,:)))';
% take qddot convert back to task space -- coupled

    end
    
    
% update loop
index = index + 1;
t = t + h;

end
tGraph = h:h:tSim;
tGraph = tGraph';

dimToView = 3;

figure
plot(tGraph,xd(:,dimToView));
hold on;
plot(tGraph,xSimG(:,dimToView));
% title('Corrected Inner Loop Position')
grid on
xlabel('Time [s]','FontSize',12)
ylabel('Position [mm]','FontSize',12)
legend('Desired Position','Simulated Position')

figure
plot(tGraph,xdotd(:,dimToView));
hold on;
plot(tGraph,xdotSimG(:,dimToView));
% title('Corrected Inner Loop Velocity')
grid on

xlabel('Time [s]','FontSize',12)
ylabel('Velocity [mm/s]','FontSize',12)
legend('Desired Velocity','Simulated Velocity')


figure
plot(tGraph,xddotd(:,dimToView));
hold on;
plot(tGraph,xddotSim(:,dimToView));
% title('Corrected Inner Loop Acceleration')
grid on

xlabel('Time [s]','FontSize',12)
ylabel('Acceleration [mm/s^2]','FontSize',12)
legend('Desired Acceleration','Simulated Acceleration')


errorInnerPos = xSimG(:,dimToView) - xd(:,dimToView);
errorInnerVel = xdotSimG(:,dimToView) - xdotd(:,dimToView);
errorInnerAcc = xddotSim(:,dimToView) - xddotd(:,dimToView);

meanErrPos = mean(errorInnerPos);
meanErrPosPercent = max(meanErrPos)/maxSinAmount * 100
meanErrVel = mean(errorInnerVel);
meanErrVelPercent = max(meanErrVel)/maxSinAmount * 100
meanErrAcc = mean(errorInnerAcc);
meanErrAccPercent = max(meanErrAcc)/maxSinAmount * 100
% units in mm

figure; hold on;
plot3(xdTot(:,1),xdTot(:,2),xdTot(:,3))
plot3(xSimTot(:,1),xSimTot(:,2),xSimTot(:,3))

error = xSimTot - xdTot;
figure; hold on;
plot(tGraph(2:end),error(2:end,1))
plot(tGraph(2:end),error(2:end,2))
plot(tGraph(2:end),error(2:end,3))


figure
plot(xddotd(:,3))
hold on
plot(xddotCoupled(:,3))
toc

xddotCoupledError = xddotdTot - xddotCoupled;

xdotCoupled(1,:) = xdotSimTot(1,:);
xCoupled(1,:) = xSimTot(1,:);

for index = 2:length(xddotCoupled(:,1))
    xdotCoupled(index,:) = h*xddotCoupled(index,:) + xdotCoupled(index-1,:);
    xCoupled(index,:) = h*xdotCoupled(index,:) + xCoupled(index-1,:);
end



figure;hold on;grid on
plot(tGraph,xddotCoupledError(:,1));
plot(tGraph,xddotCoupledError(:,2));
plot(tGraph,xddotCoupledError(:,3));
