% initialize the time vector
tSim = 30; % in sec - time for total sim time
h = 0.01;
tVect = (h:h:tSim)';

% initial arm pos in joint - rest
theta0Arm = [0,-110*pi/180,141*pi/180,0,0,0];
% inital gv pos in joint - at zero zero
gvInitx = 0; % in mm
gvInity = 0;
gvYaw = 0;
theta0GV = [gvInitx,gvInity,gvYaw];

% inital vehicle pos
theta0 = [theta0Arm,theta0GV];

% initial arm pos in task space
[state0,ori0] = ROMEFK(theta0);

% constants to define the scale of sinisoid
radOfCircle = 1000; %in mm
verticalReach = 80;

% define the vectors for the x,xdot,and xddot for the desired motion
x_des = zeros(length(tVect),6);
xdot_des = zeros(length(tVect),6);
xddot_des = zeros(length(tVect),6);

x_sim = x_des;
xdot_sim = xdot_des;
xddot_sim = xddot_des;

for i = 1:length(tVect)
    x_des(i,:) = [radOfCircle*cos(2*pi*tVect(i)/tSim) radOfCircle*sin(2*pi*tVect(i)/tSim) verticalReach*sin(2*pi*tVect(i)/tSim) 0 0 0] + [state0',ori0'];
    xdot_des(i,:) = [radOfCircle*-sin(2*pi*tVect(i)/tSim)*2*pi/tSim radOfCircle*cos(2*pi*tVect(i)/tSim)*2*pi/tSim verticalReach*cos(2*pi*tVect(i)/tSim)*2*pi/tSim 0 0 0];
    xddot_des(i,:) = [radOfCircle*-cos(2*pi*tVect(i)/tSim)*(2*pi/tSim)^2 radOfCircle*-sin(2*pi*tVect(i)/tSim)*(2*pi/tSim)^2 verticalReach*cos(2*pi*tVect(i)/tSim)*(2*pi/tSim)^2 0 0 0];
end

figure
plot3(x_des(:,1),x_des(:,2),x_des(:,3))
grid on
title("Desired Trajectory")

figure
plot3(xddot_des(:,1),xddot_des(:,2),xddot_des(:,3))
grid on
title("Desired Acceleration")



KdInner = 1;
KpInner = 1;
t=0;
index = 1;
while t < tSim
%     main script to calc forces
    if index == 1
    %   for initial conditions - assuming it starts at rest
        x_sim(index,:) = [state0',ori0'];
        xdot_sim(index,:) = zeros(1,6);
        xddot_sim(index,:) = zeros(1,6);
        
        q_sim(index,:) = theta0;
        qdot_sim(index,:) = pinv(J_ROME(q_sim(index,:))) * xdot_sim(index,:)';

    else
        %       forward integrate to get the task space motions
        xdot_sim(index,:) = h*xddot_sim(index-1,:) + xdot_sim(index-1,:);
        x_sim(index,:) = h*xdot_sim(index,:) + x_sim(index-1,:);
        
%       using task space motions, get the joint space motions 
        qdot_sim(index,:) = pinv(J_ROME(q_sim(index-1,:))) * xdot_sim(index,:)';
        
            % this calculation for the qdotSim is based on the previous time step calculation of the joint angles. this 
%             WILL cause issues for calculation accuracy, but i hope the
%             controller is robust enough to couteract these effects
        q_sim(index,:) = h*qdot_sim(index,:) + q_sim(index-1,:);
        
%       recalc the task space motions based on the new joint angle
%       configuration estimations
        xdot_simG(index,:) = J_ROME(q_sim(index,:)) * qdot_sim(index,:)';
        
        [x_simG(index,1:3),x_simG(index,4:6)] = ROMEFK(q_sim(index,:));
        
        % inner control loop is here
        xddot_sim(index,:) = (xddot_des(index,:)' + KdInner * (xdot_des(index,:)'-xdot_simG(index,:)') + KpInner * (x_des(index,:)' - x_simG(index,:)'))';
        
        torques = J_ROME(q_sim(index,:))' * xddot_sim(index,:)';
        
    end
index = index + 1;
t = t + h;
end

figure
plot3(xddot_des(:,1),xddot_des(:,2),xddot_des(:,3))
grid on
title("Desired Acceleration")

figure
plot3(xddot_sim(:,1),xddot_sim(:,2),xddot_sim(:,3))
grid on
title("Desired Acceleration")


