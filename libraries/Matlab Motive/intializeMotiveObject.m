function natnetclient = intializeMotiveObject()

%% NatNet Connection
natnetclient = natnet;
natnetclient.HostIP = '127.0.0.1';
natnetclient.ClientIP = '127.0.0.1';
natnetclient.ConnectionType = 'Multicast';
natnetclient.connect;

if ( natnetclient.IsConnected == 0 )
    natnetclient = [];
end
