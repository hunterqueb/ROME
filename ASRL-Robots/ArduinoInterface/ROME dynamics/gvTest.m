clear
close all
% timestep for sim
h = 0.01;

% initialize the time variable
tSim = 30;

planarOrbitRadius = 1; % in m

circle = Circle(tSim,planarOrbitRadius,0);

gv = omni();

tSimulation = h:h:tSim;
tSimulation = tSimulation';

pathW = circle.getPathW(tSimulation);
pathX = pathW(:,1);
pathY = pathW(:,2);
pathTheta = pathW(:,3);
pathXdot = pathW(:,4);
pathYdot = pathW(:,5);
pathThetadot = pathW(:,6);
pathXddot = pathW(:,7);
pathYddot = pathW(:,8);
pathThetaddot = pathW(:,9);

% preallocation
sizeOfArray = tSim/h;
GVx = zeros(sizeOfArray,1);
GVy = zeros(sizeOfArray,1);
GVtheta = zeros(sizeOfArray,1);
GVxdot = zeros(sizeOfArray,1);
GVydot = zeros(sizeOfArray,1);
GVxddot = zeros(sizeOfArray,1);
GVyddot = zeros(sizeOfArray,1);
GVthetadot = zeros(sizeOfArray,1);
gvIC = [pathX(1);pathY(1);pathTheta(1);pathXdot(1);pathYdot(1);pathThetadot(1);];
gvStates(1,:) = [gvIC]';

% initialize progress bar
f = waitbar(0,'Sim Progress');


t = 0;
index = 1;

GVx(1) = gvIC(1);
GVy(1) = gvIC(2);
GVtheta(1) = gvIC(3);
GVxdot(1) = gvIC(4);
GVydot(1) = gvIC(5);
GVthetadot(1) = gvIC(6);
while t < tSim
% 
% 
%  
    waitbar(index/sizeOfArray,f,'Sim Progress');
    results = gv.nextState(circle,gvStates(index,:),t);
    
    GVx(index+1) = results(end,1);
    GVy(index+1) = results(end,2);
    GVtheta(index+1) = results(end,3);
    GVxdot(index+1) = results(end,4);
    GVydot(index+1) = results(end,5);
    GVthetadot(index+1) = results(end,6);

%     GVxddot(index) = pathXddot(index);
%     GVyddot(index) = pathYddot(index);
    gvStates(index+1,:) = results(end,1:6);
    
    index = index + 1;
    t = t + h;
end