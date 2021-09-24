% initialize the time vector
tSim = 30; % in sec - time for total sim time
h = 0.01;
t = (h:h:tSim)';

% initial arm pos in joint - rest
theta0Arm = [0,-110*pi/180,141*pi/180,0,0,0];
% inital gv pos in joint - at zero zero
gvInitx = 0; % in mm
gvInity = 0;
gvYaw = 0;
theta0GV = [gvInitx,gvInity,gvYaw];

% inital vehicle pos
theta0 = [theta0GV,theta0Arm];

% initial arm pos in task space
[state0,ori0] = ROMEFK(theta0);

% constants to define the scale of sinisoid
radOfCircle = 1000; %in mm
verticalReach = 80;

% define the vectors for the x,xdot,and xddot for the desired motion
x_des = zeros(length(t),6);
xdot_des = zeros(length(t),6);
xddot_des = zeros(length(t),6);

x_sim = x_des;
xdot_sim = xdot_des;
xddot_sim = xddot_des;

for i = 1:length(t)
    x_des(i,:) = [radOfCircle*cos(2*pi*t(i)/tSim) radOfCircle*sin(2*pi*t(i)/tSim) verticalReach*sin(2*pi*t(i)/tSim) 0 0 0] + [state0',ori0'];
    xdot_des(i,:) = [radOfCircle*-sin(2*pi*t(i)/tSim)*2*pi/tSim radOfCircle*cos(2*pi*t(i)/tSim)*2*pi/tSim verticalReach*cos(2*pi*t(i)/tSim)*2*pi/tSim 0 0 0];
    xddot_des(i,:) = [radOfCircle*-cos(2*pi*t(i)/tSim)*(2*pi/tSim)^2 radOfCircle*-sin(2*pi*t(i)/tSim)*(2*pi/tSim)^2 verticalReach*cos(2*pi*t(i)/tSim)*(2*pi/tSim)^2 0 0 0];
end

figure
plot3(x_des(:,1),x_des(:,2),x_des(:,3))
grid on
title("Desired Trajectory")

for i = 1:length(t)
%     main script to calc forces
    if i == 1
    %   for initial conditions - assuming it starts at rest
        x_sim(i,:) = [state0',ori0'];
        xdot_sim(i,:) = zeros(1,6);
        xddot_sim(i,:) = zeros(1,6);
    else
        
    end
end


