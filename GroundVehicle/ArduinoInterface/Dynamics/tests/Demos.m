classdef Demos
    
    properties
       % omni object
       omni
       % arduino object
       motor1
       motor2
       motor3
    end
    
    methods
        function obj = Demos(omni,motor1,motor2,motor3)
            
            obj.omni = omni;
            obj.motor1 = motor1;
            obj.motor2 = motor2;
            obj.motor3 = motor3;
            
        end
        
        function [pos,e,traj,t] = openTrajectory(obj,pathObj)
            
            time = 0;
            
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
            
            tic
            index = 1
            while time <= pathObj.stop
                
                 %% Get frame info
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
                 phi = a(2); %yaw angle/rotation about the vertical
                 
                pathW = pathObj.getPathW(time);
                voltages = obj.omni.getE(pathW);
                voltages = -voltages;
                trajW = [data.RigidBody(1).x;-data.RigidBody(1).z;phi;];
                oError = trajW - [pathW(1);pathW(2);pathW(3)];
                
                 if toc >= time
                    %disp(voltages)
                    obj.motor1.updateVoltages(voltages);
                    time = time + pathObj.step;
                 end
                 
                 t(index,:) = toc;
                 pos(index,:) = trajW;
                 e(index,:) = oError;
                 traj(index,:) = pathW;
                 index = index + 1;
            end
            
            obj.motor1.updateVoltages([0,0,0]);
        end
        
        %% New Trajectory Code
        function [] = closedTrajectoryMatrix(obj,path)
            natnetclient = Demos.getClient();
            
            % Initialize integral error
            oErrorSum = 0;
            iErrorSum = 0;
            
            % Begin timing
            time = 0;
            tic
            index = 1;
            
            data = natnetclient.getFrame;
            lastTrajW = Demos.getTrajW(data);
            
            while time <= path(end,1)
                 %% Get frame info
                 data = natnetclient.getFrame;
                 
                 if (isempty(data.RigidBody(1)))
                     fprintf( '\tPacket is empty/stale\n' )
                     fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
                     return
                 end
                 
                 %% Update path and trajectory
                 trajW = Demos.getTrajW(data);
                 pathW = path(index,:);
                 
                 innerCommand = Demos.getVoltage(pathW,trajW,lastTrajW,oErrorSum,iErrorSum);

                 if toc >= time
                    obj.motor1.updateVoltages(innerCommand);
                    %disp(innerCommand)
                    time = time + pathObj.step;
                 end
                 
                 lastTrajW = trajW;
                 index = index + 1;
            end
            
        end
        
        %% For Path Objec
        function [pos1,pos2,e1,e2,traj1,traj2,t] = closedTrajectory(obj,pathObj)
            
            natnetclient = Demos.getClient();
            
            % Initialize integral error
            oErrorSum = 0;
            iErrorSum = 0;
            
            % Begin timing
            time = 0;
            tic
            index = 1;
            
            lastTrajW = [0;0;0];
            while time <= pathObj.stop
                 %% Get frame info
                 data = natnetclient.getFrame;
                 
                 if (isempty(data.RigidBody(1)))
                     fprintf( '\tPacket is empty/stale\n' )
                     fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
                     return
                 end
                 
                 %% Update path and trajectory
                 trajW = Demos.getTrajW(data);
                 pathW = pathObj.getPathW(time);
                 
                 angleDiff1 = atan2(sin(trajW(3)-pathW(3)), cos(trajW(3)-pathW(3)));
                 angleDiff2 = atan2(sin(trajW(3)-lastTrajW(3)), cos(trajW(3)-lastTrajW(3)));
                 
                 trajDotW = (trajW - lastTrajW)/pathObj.step;
                 trajDotW(3) = angleDiff2/pathObj.step;
                 
                 %% Filter
                 if (angleDiff2) > .25
                     trajW(3) = lastTrajW(3);
                 end
                 
                 for i = 1:2
                     if trajDotW(i) > .5;
                         trajDotW(i) = .5;
                     end
                     
                     if trajDotW(i) < -.5;
                         trajDotW(i) = -.5;
                     end
                 end
                 
                 if trajDotW(3) > .25
                     trajDotW(3) = .25;
                 end
                 if trajDotW(3) < -.25
                     trajDotW(3) = .25;
                 end
                
                 %% Calulate error
                 oError = trajW - [pathW(1);pathW(2);pathW(3)];
                 oError(3) = angleDiff1;
                 oErrorSum = oErrorSum + oError*pathObj.step;
                 tilde1 = obj.omni.getTilde1(pathW,oError,oErrorSum);
                 
                 pathDotB = obj.omni.worldToBody([pathW(4);pathW(5);pathW(6)],pathW(3));
                             
                 outerCommandB = pathDotB + tilde1;
                
                 E = obj.omni.getE(pathW);
                
                 % Equivalent to encoder measurment
                 trajDotB = obj.omni.worldToBody(trajDotW,phi);
                
                 % Error is actual - nominal
                 iError = trajDotB - outerCommandB;
                 iErrorSum = iErrorSum + iError*pathObj.step;
                
                 tilde2 = obj.omni.getTilde2(pathW,iError,iErrorSum);
                
                 innerCommand = E + tilde2;
                 
                for i = 1:3
                    if innerCommand(i) >= 12
                        %disp("POSITIVE SATURATED")
                        innerCommand(i) = 12;
                    elseif innerCommand(i) <= -12
                        innerCommand(i) = -12;
                        %disp("NEGATIVE SATURATED")
                    end
                end
                 
                 if toc >= time
                    % Omega directions fliped from my code to paper model
                    innerCommand = -innerCommand;
                    obj.motor1.updateVoltages(innerCommand);
                    %disp(innerCommand)
                    time = time + pathObj.step;
                 end
                 
                 %% Return values
                 
                 t(index,:) = toc;
                 pos1(index,:) = trajW;
                 pos2(index,:) = trajDotW;
                 e1(index,:) = oError;
                 e2(index,:) = iError;
                 traj1(index,1:3) = pathW(1:3);
                 traj2(index,4:6) = pathW(4:6);
                 
                 lastTrajW = trajW;
                 index = index + 1;
            end
            
            obj.motor1.updateVoltages([0,0,0]);
        end   
    end
    
    methods(Static)
        function natnetclient = getClient()
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
        end
        
        function trajW = getTrajW(data)
            yaw = data.RigidBody(1).qy;
            pitch = data.RigidBody(1).qz;
            roll = data.RigidBody(1).qx;
            scalar = data.RigidBody(1).qw;
            q = quaternion(roll,yaw,pitch,scalar);
            qRot = quaternion(0,0,0,1);
            q = mtimes(q,qRot);
            a = EulerAngles(q,'zxy');
            phi = a(3);% + pi;
            
            trajW = [data.RigidBody(1).x;-data.RigidBody(1).z;phi;];
        end
        
        function [innerCommand] = getVoltage(pathW,trajW,lastTrajW,oErrorSum,iErrorSum)
             
             angleDiff1 = atan2(sin(trajW(3)-pathW(3)), cos(trajW(3)-pathW(3)));
             angleDiff2 = atan2(sin(trajW(3)-lastTrajW(3)), cos(trajW(3)-lastTrajW(3)));

             trajDotW = (trajW - lastTrajW)/pathObj.step;
             trajDotW(3) = angleDiff2/pathObj.step;

             %% Filter
             if (angleDiff2) > .25
                 trajW(3) = lastTrajW(3);
             end

             for i = 1:2
                 if trajDotW(i) > .5
                     trajDotW(i) = .5;
                 end

                 if trajDotW(i) < -.5
                     trajDotW(i) = -.5;
                 end
             end

             if trajDotW(3) > .25
                 trajDotW(3) = .25;
             end
             if trajDotW(3) < -.25
                 trajDotW(3) = .25;
             end

             %% Calulate error
             oError = trajW - [pathW(1);pathW(2);pathW(3)];
             oError(3) = angleDiff1;
             oErrorSum = oErrorSum + oError*pathObj.step;
             tilde1 = obj.omni.getTilde1(pathW,oError,oErrorSum);

             pathDotB = obj.omni.worldToBody([pathW(4);pathW(5);pathW(6)],pathW(3));

             outerCommandB = pathDotB + tilde1;

             E = obj.omni.getE(pathW);

             % Equivalent to encoder measurment
             trajDotB = obj.omni.worldToBody(trajDotW,phi);

             % Error is actual - nominal
             iError = trajDotB - outerCommandB;
             iErrorSum = iErrorSum + iError*pathObj.step;

             tilde2 = obj.omni.getTilde2(pathW,iError,iErrorSum);

             innerCommand = E + tilde2;

            for i = 1:3
                if innerCommand(i) >= 12
                    %disp("POSITIVE SATURATED")
                    innerCommand(i) = 12;
                elseif innerCommand(i) <= -12
                    innerCommand(i) = -12;
                    %disp("NEGATIVE SATURATED")
                end
            end
            
            innerCommand = -innerCommand;
            
        end
    end
end
    