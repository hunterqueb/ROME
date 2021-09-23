
tSim = 30; % in sec

h = 0.01;

t = (h:h:tSim)';

% initial arm pos in joint
theta0Arm = [0,-110*pi/180,141*pi/180,0,0,0];

% inital gv pos in joint
theta0GV = [0,0,0];

theta0 = [theta0GV,theta0Arm];

% initial arm pos in task space
[state0,ori0] = ROMEFK(theta0);


radOfCircle = 1000; %in mm
verticalReach = 80;
for i = l:length(t)
    x = [radOfCircle*cos(2*pi*t(i)/tSim) radOfCircle*sin(2*pi*t(i)/tSim verticalReach*sin(2*pi*t(i)/tSim) 0 0 0] + [state0,ori0];
    xdot = [];
    xddot = [];
end



