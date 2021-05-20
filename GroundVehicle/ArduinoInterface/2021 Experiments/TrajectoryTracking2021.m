function [pos,e,traj,time_GV,worldTrajAR2,time] = TrajectoryTracking2021(path,Motor1,path_AR2_J,Stepper1)
% This function takes a path_AR2_J, specified as 6 angle states followed by six
% angular velocity states (rad,rad/s) for each joint.
% Stepper1 is a stepper motor object for one of
% the joints on the arm. Arms must first be callibrated.

%% Trajectory Validation
validity = validateTrajectory(path_AR2_J);
if validity(1) == 0
  disp('The given worldTrajAR2 is invalid');
  return
end

%% Platform Geometry
L = .13; %distance from platform centroid to wheel
R = .05;  %radius of the wheel

%% PID GV

Kp = 8*.5*.5;
Ki = .4*.5*.5;


Kp = [Kp 0 0;
      0 Kp 0;
      0 0  -Kp*5*.5;]; %Gains for theta must be negative, not sure why yet
  
Ki = [Ki 0 0;
      0 Ki 0;
      0 0  -Ki*10;];

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

data = natnetclient.getFrame;	
if (isempty(data.RigidBody(1)))
    fprintf( '\tPacket is empty/stale\n' )
    fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
    return
end


%% Move to initial position
pause(5);
statesArray = [path_AR2_J(1,2),path_AR2_J(1,3),path_AR2_J(1,4)...
               path_AR2_J(1,5),path_AR2_J(1,6),path_AR2_J(1,7)...
               .25,.25,.25,.25,.25,.25];
Stepper1.updateStates(statesArray);
pause(5);

%% Main Loop

position = Demos.getTrajW(data);
thetaLast = position(3);
timestep = path(end,1)/length(path(:,1));

index = 1;
index2 = 1;
errorSum = 0;
tic;
figure;
hold on;
while(toc <= path_AR2_J(end,1))
    if (toc >= path_AR2_J(index,1))
        
        data = natnetclient.getFrame;	
		if (isempty(data.RigidBody(2)))
			fprintf( '\tPacket is empty/stale\n' )
			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
			return
        end
        
        
        
%       arm Data
        worldTrajAR2(index,:) = [data.RigidBody(2).x;-data.RigidBody(2).z;data.RigidBody(2).y;];
        time(index,:) = toc;


        statesArray = [path_AR2_J(index,2),path_AR2_J(index,3),path_AR2_J(index,4)...
                       path_AR2_J(index,5),path_AR2_J(index,6),path_AR2_J(index,7)...
                       path_AR2_J(index,8),path_AR2_J(index,9),path_AR2_J(index,10)...
                       path_AR2_J(index,11),path_AR2_J(index,12),path_AR2_J(index,13)];
        Stepper1.updateStates(statesArray);
        
        scatter3(worldTrajAR2(index,1),worldTrajAR2(index,2),worldTrajAR2(index,3))
        
        index = index + 1;
    end
    
    if (toc >= path(index2,1))
        
        trajectory = [path(index2,2);path(index2,3);path(index2,4)];
        trajectoryPrime = [path(index2,5);path(index2,6);path(index2,7)];  
        
        %data = natnetclient.getFrame;	
% 		if (isempty(data.RigidBody(1)))
% 			fprintf( '\tPacket is empty/stale\n' )
% 			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
% 			return
%         end
        
        position = Demos.getTrajW(data);
        theta = position(3);

        angleDiff2 = atan2(sin(theta-thetaLast), cos(theta-thetaLast));
        if abs((angleDiff2)) > .25/3
            theta = thetaLast;
        end
        angleDiff1 = atan2(sin(theta-trajectory(3)), cos(theta-trajectory(3)));

        position(3) = theta;
        
        error = position - trajectory;
        error(3) = angleDiff1;
        errorSum = errorSum + error*timestep;

        
        pTheta = [-sin(theta)               cos(theta)          L;
                  -sin((pi/3)-theta)       -cos((pi/3)-theta)   L;
                   sin((pi/3)+theta)       -cos((pi/3)+theta)   L;];
        
        setpointMetSec = pTheta*(trajectoryPrime - Kp*error - Ki*errorSum);
        setpointRadSec = setpointMetSec/R;
        
        Motor1.updateMotors(setpointRadSec*1.2);
        
        time_GV(index2,:) = toc;
        pos(index2,:) = position;
        e(index2,:) = error;
        traj(index2,:) = trajectory;
        index2 = index2 + 1;
        thetaLast = theta;
    end
    
    
end
Stepper1.default("REST")
Motor1.updateMotors([0.0,0.0,0.0]);

end