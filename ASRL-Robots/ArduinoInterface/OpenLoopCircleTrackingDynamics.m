%NatNet Connection
natnetclient = natnet;
natnetclient.HostIP = '12f7.0.0.1';
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

xDoubleDot = -.25*cos(t);
yDoubleDot = -sin(t);
psiDoubleDot = 0;


trajectoryDoublePrime = [-cos(t);
                         -sin(t);
                          0;];

batteryVoltage = 12.0;
k2 = 13.4 * 10^-3; %N*m/A
k3 = 1.4 * 10^-3; %V/rpm
Ra = 1.9; %ohms
n = 1;
R = .05; %m
b0 = 1; %friction constant experimental, unknown

m = 5; %platform mass, unknown
Iz = 5; %platform inertia, unknown

H = [1/m 0 0;
     0 1/m 0;
     0 0   1/Iz;];

B = [0 cos(pi/6) -cos(pi/6);
    -1 sin(pi/6)  sin(pi/6);
     L         L          L;];

G = eye(3) + H*B*transpose(B)*n*n*J0/R^2;

timestep = .02; %This is currently set equal to the interval for calculating
                %platform velocity and the inner loop PID update interval.
index = 0;
errorSum = 0;   %A forward euler integration will be performed.
numCircles = 1; %Determines the number of circles the platform will cycle
                %through before stopping.

while(toc < 2*pi*4*numCircles)

    if (toc >= timestep*index)

        trajectory = [cos(.25*toc);sin(.25*toc);0;]; %Circle set to 1 meter radius, 8pi period.

        xDot = -.25*sin(.25*toc);
        yDot =  .25*cos(.25*toc);
        thetaDot = 0;

        xDoubleDot = -.25*.25*cos(.25*t);
        yDoubleDot = -.25*.25*sin(.25*t);
        psiDoubleDot = 0;

        trajectoryPrime = [xDot;yDot;thetaDot;]; %Trajectory derivative

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
        psi = a(2); %yaw angle/rotation about the vertical

        position = [data.RigidBody(1).x;-data.RigidBody(1).z;psi;];
        error = position - trajectory;
        errorSum = errorSum + error*timestep;


        bodyRate = [cos(psi),sin(psi),0;
                   -sin(psi),cos(psi),0;
                    0       ,0       ,0;]*trajectoryPrime;

        uDoubleDot = -sin(psi)*omega*xDot + cos(psi)*xDoubleDot + cos(psi)*omega*yDot + sin(psi)*yDoubleDot;
        vDoubleDot = -cos(psi)*omega*xDot - sin(psi)*xDoubleDot - sin(psi)*omega*yDot + cos(psi)*yDoubleDot;
        rDoubleDot = psiDoubleDot;

        bodyRatePrime = [uDoubleDot;
                         vDoubleDot;
                         rDoubleDot;];


        motorVoltages = inv(H*B)*(R*Ra/(k2*n))*G*bodyRatePrime - inv(H*B)*(R*Ra/(k2*n))*[bodyRate(2)*bodyRate(3);-bodyRate(1)*bodyRate(3);0] + transpose(B)*(Ra*n/(k2*R))*(k2*k3/Ra +b0)*bodyRate;
        motorPWMs = E1*255*(1/batteryVoltage);

        Motor1.setPWM(motorVoltages);

        index = index + 1;
        time(index,:) = toc;
        pos(index,:) = position;
        e(index,:) = error;
        traj(index,:) = trajectory;
    end
end
