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

figure
hold on
tic;
while toc < 20
    data = natnetclient.getFrame;	
    scatter3(data.RigidBody(2).x,-data.RigidBody(2).z,data.RigidBody(2).y);
    pause(0.01)
end
daspect([1 1 1])
