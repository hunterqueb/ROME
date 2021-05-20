function [pose_GV,error_GV,pos_AR2_W,error_AR2_W,traj_AR2_G,time] = trajectoryTrackingCoupledWind(path_GV_G,path_AR2_J,Motor1,Stepper1,path_AR2_WZ)

pause(15);

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

model = natnetclient.getModelDescription;
if ( model.RigidBodyCount < 1 )
	return
end

%% Trajectory Validation (Actually Path Validation REEEEEE)
validity = validateTrajectory(path_AR2_J);
if validity(1) == 0
  disp('The given trajectory is invalid');
  return
end

%% Platform Geometry
L = .13; %distance from platform centroid to wheel
R = .05;  %radius of the wheel

%% Gains

Kp_GV = 8*.5;
Ki_GV = .4*.5;


Kp_GV = [Kp_GV 0 0;
      0 Kp_GV 0;
      0 0  -Kp_GV;]; %Gains for theta must be negative, not sure why yet

Ki_GV = [Ki_GV 0 0;
      0 Ki_GV 0;
      0 0  -Ki_GV;];
  
Kp_AR2 = 4;
Ki_AR2 = 2*0.038;    
Kd_AR2 = 0.4*3;


%% Main Loop Prep
timestep = path_GV_G(end,1)/length(path_GV_G(:,1)); %This is currently set equal to the interval for calculating
                %platform velocity and the inner loop PID update interval.
                
errorSum_GV_G = 0;   %A forward euler integration will be performed.
thetaLast_GV = 0;
% Arm initialization
statesArray_AR2_J = [path_AR2_J(1,2),path_AR2_J(1,3),path_AR2_J(1,4)...
               path_AR2_J(1,5),path_AR2_J(1,6),path_AR2_J(1,7)...
               .25,.25,.25,.25,.25,.25];
Stepper1.updateStates(statesArray_AR2_J);
pause(10);

%% Setup: For FK and IK Functions
command_AR2_W = zeros(length(path_AR2_WZ(:,1)),3);
pathDot_AR2_W = zeros(length(path_AR2_WZ(:,2)),3);

pathDot_AR2_W(:,3) = path_AR2_WZ(:,2);
command_AR2_J = zeros(length(path_AR2_J(:,1)),6);
command0_AR2_J = path_AR2_J(1,2:7);
command_AR2_J(1,:) = command0_AR2_J;

command0_AR2_W = manipFK(command0_AR2_J);

% Offset between global and AR2 work frame
offset = getTransformation3(command0_AR2_J,7);


for i = 1:length(path_AR2_WZ(:,1))
   command_AR2_W_6(i,:) = manipFK(path_AR2_J(i,(2:7)));
   command_AR2_W(i,1:3)= command_AR2_W_6(i,1:3);
end

traj_AR2_W = zeros(length(path_AR2_J(:,1)),3);
traj_AR2_G = zeros(length(path_AR2_J(:,1)),3);
trajDot_AR2_W = zeros(length(path_AR2_J(:,1)),3);
error_AR2_W = zeros(length(path_AR2_J(:,1)),3);
time = path_AR2_J(:,1);

%offset(3)=offset(3)+21.12;
% initWorld = [-data.LabeledMarker(1).x -data.LabeledMarker(1).z data.LabeledMarker(1).y 0 0 0]*1000;
% initWork=reference(1,:);
% offset=initWorld-initWork;

commandDot_AR2_W = zeros(length(path_AR2_J(:,1)),3);

%% Main  Loop
index = 1;
disp("Starting");
tic;
while(toc < 60)

    if (toc >= path_GV_G(index,1))

        trajectory = [path_GV_G(index,2);path_GV_G(index,3);path_GV_G(index,4)];
        trajectoryPrime = [path_GV_G(index,5);path_GV_G(index,6);path_GV_G(index,7)];

        data = natnetclient.getFrame;
        if (isempty(data.RigidBody(1)) || isempty(data.LabeledMarker(7)))
            if isempty(data.RigidBody(1)) == 1
                disp("Ground Vehicle Lost Connection");
            elseif isempty(data.LabeledMarker(7))
                disp("AR2 Lost Connection");
            end
% 			fprintf( '\tPacket is empty/stale\n' )
% 			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
			Motor1.updateMotors([0 0 0]);
            Stepper1.default("REST");
            return
        end
        
        yaw_GV_G = data.RigidBody(1).qy;
        pitch_GV_G = data.RigidBody(1).qz;
        roll_GV_G = data.RigidBody(1).qx;
        scalar = data.RigidBody(1).qw;
        q = quaternion(roll_GV_G,yaw_GV_G,pitch_GV_G,scalar);
        qRot = quaternion(0,0,0,1);
        q = mtimes(q,qRot);
        a = EulerAngles(q,'zyx');
        theta_GV_G = a(2); %yaw angle/rotation about the vertical
        if abs(theta_GV_G - thetaLast_GV) >= (10)*(pi/180)
            theta_GV_G = thetaLast_GV;
        end
        
        yawYeet = theta_GV_G;
        
        position = [data.RigidBody(1).x;-data.RigidBody(1).z;theta_GV_G;];
        error_GV_G = position - trajectory;
        errorSum_GV_G = errorSum_GV_G + error_GV_G*timestep;

        yawYeet_error = theta_GV_G - path_GV_G(index,4);

        pTheta = [-sin(theta_GV_G)               cos(theta_GV_G)          L;
                  -sin((pi/3)-theta_GV_G)       -cos((pi/3)-theta_GV_G)   L;
                   sin((pi/3)+theta_GV_G)       -cos((pi/3)+theta_GV_G)   L;];

        command_GV_J = (1/R)*(pTheta*(trajectoryPrime - Kp_GV*error_GV_G - Ki_GV*errorSum_GV_G));
        
        Motor1.updateMotors(command_GV_J*1.2);
		%ARM Logic
        index=index+1;
        
    end
end
while(toc <= path_GV_G(end,1) && toc >= 60)

    if (toc >= path_GV_G(index,1))

        trajectory = [path_GV_G(index,2);path_GV_G(index,3);path_GV_G(index,4)];
        trajectoryPrime = [path_GV_G(index,5);path_GV_G(index,6);path_GV_G(index,7)];

        data = natnetclient.getFrame;
        if (isempty(data.RigidBody(1)) || isempty(data.LabeledMarker(7)))
            if isempty(data.RigidBody(1)) == 1
                disp("Ground Vehicle Lost Connection");
            elseif isempty(data.LabeledMarker(7))
                disp("AR2 Lost Connection");
            end
% 			fprintf( '\tPacket is empty/stale\n' )
% 			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
			Motor1.updateMotors([0 0 0]);
            Stepper1.default("REST");
            return
        end
        
        yaw_GV_G = data.RigidBody(1).qy;
        pitch_GV_G = data.RigidBody(1).qz;
        roll_GV_G = data.RigidBody(1).qx;
        scalar = data.RigidBody(1).qw;
        q = quaternion(roll_GV_G,yaw_GV_G,pitch_GV_G,scalar);
        qRot = quaternion(0,0,0,1);
        q = mtimes(q,qRot);
        a = EulerAngles(q,'zyx');
        theta_GV_G = a(2); %yaw angle/rotation about the vertical
        if abs(theta_GV_G - thetaLast_GV) >= (10)*(pi/180)
            theta_GV_G = thetaLast_GV;
        end
        
        yawYeet = theta_GV_G;
        
        position = [data.RigidBody(1).x;-data.RigidBody(1).z;theta_GV_G;];
        error_GV_G = position - trajectory;
        errorSum_GV_G = errorSum_GV_G + error_GV_G*timestep;

        yawYeet_error = theta_GV_G - path_GV_G(index,4);

        pTheta = [-sin(theta_GV_G)               cos(theta_GV_G)          L;
                  -sin((pi/3)-theta_GV_G)       -cos((pi/3)-theta_GV_G)   L;
                   sin((pi/3)+theta_GV_G)       -cos((pi/3)+theta_GV_G)   L;];

        command_GV_J = (1/R)*(pTheta*(trajectoryPrime - Kp_GV*error_GV_G - Ki_GV*errorSum_GV_G));

		%ARM Logic

        traj_AR2_G(index,:) = [data.LabeledMarker(7).z data.LabeledMarker(7).x data.LabeledMarker(7).y]*1000;
        traj_AR2_W(index,:) = traj_AR2_G(index,:) - offset;

		if index == 1
            error_AR2_W(index,:) = zeros(1,3);
            %error in pos!!!
        else
            error_AR2_W(index,:) = command_AR2_W(index,:)-traj_AR2_W(index,:);
        end

        if index == 1
            %use PID Controller:
            Up = zeros(1,3);
            Ui = zeros(1,3);
            Ud = zeros(1,3);
        elseif index == 2
            Up = Kp_AR2*(error_AR2_W(index,:));
            Ui = Ui + Ki_AR2*(error_AR2_W(index,:)*timestep);
            Ud = Kd_AR2*((error_AR2_W(index,:)-error_AR2_W(index-1,:))/timestep);
        else
            Up = Kp_AR2*(error_AR2_W(index,:));
            Ui = Ui + Ki_AR2*(error_AR2_W(index,:)*timestep);
            Ud = Kd_AR2*(3*(error_AR2_W(index,:)-4*error_AR2_W(index-1,:)+error_AR2_W(index-2,:))/2*timestep);
        end
        u = Up+Ui+Ud;
        %PID part
        commandDot_AR2_W(index,:) = pathDot_AR2_W(index,:)+u;
        commandDot_AR2_J_temp = trajectoryIK3(commandDot_AR2_W(index,:)',command_AR2_J(index,:));

        commandDot_AR2_J=zeros(1,6);
        commandDot_AR2_J(2)=commandDot_AR2_J_temp(1);
        commandDot_AR2_J(3)=commandDot_AR2_J_temp(2);
        commandDot_AR2_J(5)=commandDot_AR2_J_temp(3);

        statesArray_AR2_J = [command_AR2_J(index,1),command_AR2_J(index,2),command_AR2_J(index,3)...
                       command_AR2_J(index,4),command_AR2_J(index,5),command_AR2_J(index,6)...
                       commandDot_AR2_J(1),commandDot_AR2_J(2),commandDot_AR2_J(3)...
                       commandDot_AR2_J(4),commandDot_AR2_J(5),commandDot_AR2_J(6)];

        Stepper1.updateStates(statesArray_AR2_J);
        Motor1.updateMotors(command_GV_J*1.2);

		command_AR2_J(index+1,:)=command_AR2_J(index,:)'+(commandDot_AR2_J'*timestep);

        time(index,:) = toc;
        pose_GV(index,:) = [position(1) position(2) yawYeet];
        error_GV(index,:) = [error_GV_G(1) error_GV_G(2) yawYeet_error];
        pos_AR2_W(index,:) = [traj_AR2_W(index,1) traj_AR2_W(index,2) traj_AR2_W(index,3)];
        error_AR2_W(index,:) = [error_AR2_W(index,1) error_AR2_W(index,2) error_AR2_W(index,3)];
        traj_AR2_G(index,:) = [command_AR2_W(index,1) command_AR2_W(index,2) command_AR2_W(index,3)];
        index = index + 1;
    end
end


%Set motors back to zero
Motor1.updateMotors([0.0,0.0,0.0]);
Stepper1.default("REST");

end
