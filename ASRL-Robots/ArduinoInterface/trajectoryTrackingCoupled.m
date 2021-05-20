function [pos,e,traj,time] = trajectoryTrackingCoupled(path,Motor1,Stepper1)

pause(15);

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

Kp = 8*.5;
Ki = .4*.5;


Kp = [Kp 0 0;
      0 Kp 0;
      0 0  -Kp;]; %Gains for theta must be negative, not sure why yet

Ki = [Ki 0 0;
      0 Ki 0;
      0 0  -Ki;];

%Main Loop
timestep = path(end,1)/length(path(:,1)); %This is currently set equal to the interval for calculating
                %platform velocity and the inner loop PID update interval.
index = 1;
errorSum = 0;   %A forward euler integration will be performed.

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

        Motor1.updateMotors(setpointRadSec*1.2);
        
        Stepper1.updateSteppers([path(index,8),path(index,9),path(index,10),path(index,11),path(index,12)]);

        time(index,:) = toc;
        pos(index,:) = position;
        e(index,:) = error;
        traj(index,:) = trajectory;
        index = index + 1;
    end
end


%Set motors back to zero
Motor1.updateMotors([0.0,0.0,0.0]);
Stepper1.updateSteppers([0.0,0.0,0.0,0.0,0.0,0.0]);

end
