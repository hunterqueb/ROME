function [offset] = getTransformationMatrix(theta,rotation)
%GETTRANSFORMATION This function obtains the transformation from the global
%frame to the AR2 workspace using the optiTrack camera system. Theta is
%specified as a 6x1 matrix containing the current joint angle states.
%   Detailed explanation goes here
%rotation in degrees
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

initialStatesWork = manipFK(theta');

%rotation matrix for how global rotates to reach work frame
rotz=[cosd(rotation) -sind(rotation) 0;
     sind(rotation) cosd(rotation)  0
     0              0               1];
%% Obtaining global frame position for 10 seconds and averaging
index = 1;
tic
while toc <= 10
    data = natnetclient.getFrame;
        if (isempty(data.LabeledMarker(1)))
			fprintf( '\tPacket is empty/stale\n' )
			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
			return
        end

    yaw = data.RigidBody(1).qy;
    pitch= data.RigidBody(1).qz;
    roll= data.RigidBody(1).qx;
    scalar = data.RigidBody(1).qw;
    q = quaternion(roll,yaw,pitch,scalar);
    qRot = quaternion(0,0,0,1);
    q = mtimes(q,qRot);
    a = EulerAngles(q,'zyz'); %radian output

    statesWorldPos(index,1:3) = [data.LabeledMarker(1).z data.LabeledMarker(1).x data.LabeledMarker(1).y]*1000;
    statesWorldPosRot(index,1:3)=rotz*statesWorldPos(index,1:3)';
    statesWorld(index,1:6) = [statesWorldPosRot(index,1:3) a(2) a(1) a(3)]; %might need to change angles from rad to deg.
    index = index + 1;
end

averageStatesWorld = sum(statesWorld)/length(statesWorld);
offset = averageStatesWorld - initialStatesWork';

end
