
function [trajectory,time] = trajectoryTrackingArm_Motive(path,Stepper1)
% This function takes a path, specified as 6 angle states followed by six
% angular velocity states (rad,rad/s) for each joint.
% Stepper1 is a stepper motor object for one of
% the joints on the arm. Arms must first be callibrated.

%% Trajectory Validation
validity = validateTrajectory(path);
if validity(1) == 0
  disp('The given trajectory is invalid');
  return
end

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

%% Move to initial position
pause(5);
statesArray = [path(1,2),path(1,3),path(1,4)...
               path(1,5),path(1,6),path(1,7)...
               .25,.25,.25,.25,.25,.25];
Stepper1.updateStates(statesArray);
pause(15);

%% Main Loop
index = 1;
tic;
while(toc <= path(end,1))
    if (toc >= path(index,1))
        
        data = natnetclient.getFrame;	
		if (isempty(data.LabeledMarker(1)))
			fprintf( '\tPacket is empty/stale\n' )
			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
			return
        end
        
        trajectory(index,:) = [data.LabeledMarker(1).x;-data.LabeledMarker(1).z;data.LabeledMarker(1).y;];
        time(index,:) = toc;


        statesArray = [path(index,2),path(index,3),path(index,4)...
                       path(index,5),path(index,6),path(index,7)...
                       path(index,8),path(index,9),path(index,10)...
                       path(index,11),path(index,12),path(index,13)];
        Stepper1.updateStates(statesArray);
        
        index = index + 1;
    end
end
end
