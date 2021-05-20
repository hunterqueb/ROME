function [pos,e,traj,time] = newCircle(dcm1,dcm2,dcm3)
%This function takes a motor object as an input. It will have to be motor 1
%(the motor with the yellow coloring on the wheel) in order for the
%ordering to be correct. circle returns pos (3-element column
%vector with measured position in x, z, and theta), e (3-element column vector with
%difference in x, z, and theta between measured and reference
%trajectories), and traj (3-element column vector of the reference
%trajectory in x, z, and theta).

%example: [position,error,trajectory,time] = circle(Motor1)
MaxMotor = 12.5664;

dcm1.Speed = 0;
dcm2.Speed = 0;
dcm3.Speed = 0;

%Wait five seconds for setup time
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
Kp = 8*.25;
Ki = .4*.25;
% 
% Kp = 0;
% Ki = 0;

Kp = [Kp 0 0;
      0 Kp 0;
      0 0 -Kp;]; %Gains for theta must be negative, not sure why yet
  
Ki = [Ki 0 0;
      0 Ki 0;
      0 0 -Ki;];

%Main Loop0
timestep = .02; %This is currently set equal to the interval for calculating
                %platform velocity and the inner loop PID update interval.
index = 0;
errorSum = 0;   %A forward euler integration will be performed.
numCircles = 1; %Determines the number of circles the platform will cycle
                %through before stopping.
                
tic;
while(toc < 2*pi*4*numCircles)
    
    if (toc >= timestep*index)
        
        trajectory = [cos(.25*toc);sin(.25*toc);0;]; %Circle set to 1 meter radius, 8pi period.
        trajectoryPrime = [-.25*sin(.25*toc);.25*cos(.25*toc);0;]; %Trajectory derivative
    
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
        
        dcm1Speed=setpointRadSec(1)/MaxMotor;
        dcm2Speed=setpointRadSec(2)/MaxMotor;
        dcm3Speed=setpointRadSec(3)/MaxMotor;
        
        if dcm1Speed > 1
            dcm1Speed = 1;
        end
        if dcm1Speed < -1
            dcm1Speed = -1;
        end
        
        if dcm2Speed > 1
            dcm2Speed = 1;
        end
        if dcm2Speed < -1
            dcm2Speed = -1;
        end
        
        if dcm3Speed > 1
            dcm3Speed = 1;
        end
        if dcm3Speed < -1
            dcm3Speed = -1;
        end
        
        dcm1Speed=cast(dcm1Speed,'double');
        dcm2Speed=cast(dcm2Speed,'double');
        dcm3Speed=cast(dcm3Speed,'double');

        
        dcm1.Speed = dcm1Speed;
        dcm2.Speed = dcm2Speed;
        dcm3.Speed = dcm3Speed;
        
%         Motor1.updateMotors(setpointRadSec*1.2); %Multiply by 1.2 for significant improvement
        
        index = index + 1;
        time(index,:) = toc;
        pos(index,:) = position;
        e(index,:) = error;
        traj(index,:) = trajectory;
    end
end
%Set motors back to zero

dcm1.Speed = 0;
dcm2.Speed = 0;
dcm3.Speed = 0;

% Motor1.updateMotors([0.0,0.0,0.0]);
end


