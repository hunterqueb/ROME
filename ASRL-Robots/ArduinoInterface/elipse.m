function [pos,e,traj,time] = elipse(Motor1)

%example [position,error,trajectory,time] = elipse(Motor1)

pause(5);

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

%Platform Geometry
L = .13; %distance from platform centroid to wheel
R = .05;  %radius of the wheel

%PID
Kp = 8;
Ki = .4;

Kp = [Kp 0 0;
      0 Kp 0;
      0 0 -Kp;]; %Gains for theta must be negative, not sure why yet
  
Ki = [Ki 0 0;
      0 Ki 0;
      0 0 -Ki;];

%Main Loop
timestep = .02; %This is currently set equal to the interval for calculating
                %platform velocity and the inner loop PID update interval.
index = 0;
errorSum = 0;   %A forward euler integration will be performed.
numElipses = 1; %Determines the number of circles the platform will cycle
                %through before stopping.
                
tic;
while(toc < 2*pi*4*numElipses)
    
    if (toc >= timestep*index)
        
        trajectory = [cos(.25*toc);1.5*sin(.25*toc);0;]; %Elipse set to 1 meter, 1.5 meter radii. 8pi period.
        trajectoryPrime = [-.25*sin(.25*toc);.3755*cos(.25*toc);0;]; %Trajectory derivative
    
        data = natnetclient.getFrame;	
		if (isempty(data.RigidBody(1)))
			fprintf( '\tPacket is empty/stale\n' )
			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
			return
        end
        
        
        yaw = data.RigidBody(1).qy;
        pitch = data.RigidBody(1).qz;
        roll = data.RigidBody(1).qx;
        scalar = data.RigidBody(1).qw;
        q = quaternion(roll,yaw,pitch,scalar);
        qRot = quaternion(0,0,0,1);
        q = mtimes(q,qRot);
        a = EulerAngles(q,'zyx');
        theta = a(2); %yaw angle/rotation about the vertical

        position = [data.RigidBody(1).x;-data.RigidBody(1).z;theta;];
        error = position - trajectory;
        errorSum = errorSum + error*timestep;

        
        pTheta = [-sin(theta)               cos(theta)          L;
                  -sin((pi/3)-theta)       -cos((pi/3)-theta)   L;
                   sin((pi/3)+theta)       -cos((pi/3)+theta)   L;];
        
        setpointMetSec = pTheta*(trajectoryPrime - Kp*error - Ki*errorSum);
        setpointRadSec = setpointMetSec/R;
        
        Motor1.updateMotors(setpointRadSec);
        
        index = index + 1;
        time(index,:) = toc;
        pos(index,:) = position;
        e(index,:) = error;
        traj(index,:) = trajectory;
    end
end

%Set motors back to zero
Motor1.updateMotors([0.0,0.0,0.0]);

% plot(pos);
% hold on;
% plot(traj);
% hold off;

end


