%Copyright © 2018 Naturalpoint
%
%Licensed under the Apache License, Version 2.0 (the "License");
%you may not use this file except in compliance with the License.
%You may obtain a copy of the License at
%
%http://www.apache.org/licenses/LICENSE-2.0
%
%Unless required by applicable law or agreed to in writing, software
%distributed under the License is distributed on an "AS IS" BASIS,
%WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%See the License for the specific language governing permissions and
%limitations under the License.

% Optitrack Matlab / NatNet Polling Sample
%  Requirements:
%   - OptiTrack Motive 2.0 or later
%   - OptiTrack NatNet 3.0 or later
%   - Matlab R2013
% This sample connects to the server and displays rigid body data.
% natnet.p, needs to be located on the Matlab Path.

%REMEMBER BE SURE TO CHECK WHAT AXIS MOTIVE IS USING AS THE VERTICAL AXIS

function NatNetPollingSample
	fprintf( 'NatNet Polling Sample Start\n' )

	% create an instance of the natnet client class
	fprintf( 'Creating natnet class object\n' )
	natnetclient = natnet;

	% connect the client to the server (multicast over local loopback) -
	% modify for your network
	fprintf( 'Connecting to the server\n' )
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

	% get the asset descriptions for the asset names
	model = natnetclient.getModelDescription;
	if ( model.RigidBodyCount < 1 )
        error('No Rigid Bodies Detected!')
		return
	end

	% Poll for the rigid body data a regular intervals (~1 sec) for 10 sec.
	fprintf( '\nPrinting rigid body frame data approximately every second for 10 seconds...\n\n' )
	for idx = 1 : 10  
		%java.lang.Thread.sleep( 996 ); %time in milliseconds for delay of transmitted data
		java.lang.Thread.sleep( 20 );
		data = natnetclient.getFrame; % method to get current frame
		
		if (isempty(data.RigidBody(1)))
			fprintf( '\tPacket is empty/stale\n' )
			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
			return
        end
        
		fprintf( 'Frame:%6d  ' , data.Frame )
		fprintf( 'Time:%0.2f\n' , data.Timestamp )
		for i = 1:model.RigidBodyCount
			fprintf( 'Name:"%s"  ', model.RigidBody( i ).Name )
			fprintf( 'X:%0.1fm  ', data.RigidBody( i ).x * 1 )
			fprintf( 'Y:%0.1fm  ', data.RigidBody( i ).y * 1 )
			fprintf( 'Z:%0.1fm\n', data.RigidBody( i ).z * 1 )
            
            x(idx) = data.RigidBody(i).x;
            y(idx) = data.RigidBody(i).y; 
            z(idx) = data.RigidBody(i).z; 
            time(idx) = data.Timestamp;
            
            
		end
    end 
%     normalize data
    x_start = x(1);
    y_start = y(1);
%     for idx = 1:10
%         x(idx) = x(idx)-x_start;
%         y(idx) = y(idx)-y_start;
%     end
    %initialize time to when polling started
    time_s = time(1);
    for idx = 1:10
        time(idx) = time(idx) - time_s;
    end
    ERR = Path_Error(time,x,y,x_start,y_start)
	disp('NatNet Polling Sample End' )
end







 
