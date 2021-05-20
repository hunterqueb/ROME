function [pos,e,traj,time] = trajectoryTracking(path,Motor1)
%Trajectory first column is time, next three columns are states
%(X,Y,Theta),last three columns are derivatives (Xdot,Ydot,thetaDot).

%Examples [position,error,trajectory,time] = trajectoryTracking(CWTrajectory1,Motor1);

pause(4);

%NatNet Connection
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

model = natnetclient.getModelDescription;
if ( model.RigidBodyCount < 1 )
	return
end

data = natnetclient.getFrame;	
if (isempty(data.RigidBody(1)))
    fprintf( '\tPacket is empty/stale\n' )
    fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
    return
end

%%Platform Geometry
L = .13; %distance from platform centroid to wheel
R = .05;  %radius of the wheel

%%PID

Kp = 8*.5;
Ki = .4*.5;


Kp = [Kp 0 0;
      0 Kp 0;
      0 0  -Kp*5;]; %Gains for theta must be negative, not sure why yet
  
Ki = [Ki 0 0;
      0 Ki 0;
      0 0  -Ki*10;];

%Main Loop
timestep = path(end,1)/length(path(:,1)); %This is currently set equal to the interval for calculating
                %platform velocity and the inner loop PID update interval.
index = 1;
errorSum = 0;   %A forward euler integration will be performed.

position = Demos.getTrajW(data);
thetaLast = position(3);
        
tic;
while(toc <= path(end,1))
    
    if (toc >= path(index,1))
        
        trajectory = [path(index,2);path(index,3);path(index,4)];
        trajectoryPrime = [path(index,5);path(index,6);path(index,7)];  
        
        data = natnetclient.getFrame;	
		if (isempty(data.RigidBody(1)))
			fprintf( '\tPacket is empty/stale\n' )
			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
			return
        end
        
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
        
        time(index,:) = toc;
        pos(index,:) = position;
        e(index,:) = error;
        traj(index,:) = trajectory;
        index = index + 1;
        thetaLast = theta;
    end
end


%Set motors back to zero
Motor1.updateMotors([0.0,0.0,0.0]);

end

