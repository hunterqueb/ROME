function [trajectory,time,error,initposWork,initposGlobal] = trajectoryTrackingArm_Feedback(path_AR2_J,refTraj,Stepper1,theta0)
% This function takes a path, specified as 6 angle states followed by six
% angular velocity states (rad,rad/s) for each joint.
% Stepper1 is a stepper motor object for one of
% the joints on the arm. Arms must first be callibrated.

pause(5);

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

%% Gains

Kp_AR2 = 4;
Ki_AR2 = 2*0.038;
Kd_AR2 = 0.4*3;


%% Main Loop Prep
timestep = path_AR2_J(end,1)/length(path_AR2_J(:,1)); %This is currently set equal to the interval for calculating
                %platform velocity and the inner loop PID update interval.

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
offsetTemp = getTransformationMatrix(command0_AR2_J,0);

offset = offsetTemp(1:3);
offsetAng = offsetTemp(4:6);

for i = 1:length(path_AR2_WZ(:,1))
   command_AR2_W_6(i,:) = manipFK(path_AR2_J(i,(2:7)));
end

traj_AR2_W = zeros(length(path_AR2_J(:,1)),3);
traj_AR2_G = zeros(length(path_AR2_J(:,1)),3);
trajDot_AR2_W = zeros(length(path_AR2_J(:,1)),3);
error_AR2_W = zeros(length(path_AR2_J(:,1)),3);
time = path_AR2_J(:,1);

commandDot_AR2_W = zeros(length(path_AR2_J(:,1)),3);

%% Main  Loop
index = 1;
disp("Starting");
tic;
while(toc <= path_AR2_J(end,1))

    if (toc >= path_AR2_J(index,1))

        data = natnetclient.getFrame;

		%ARM Logic
				%pos measure
        traj_AR2_G(index,:) = [data.RigidBody(1).z data.RigidBody(1).x data.RigidBody(1).y]*1000;
        traj_AR2_W(index,:) = traj_AR2_G(index,:) - offset;

				%angle measure
				yaw = data.RigidBody(1).qy;
        pitch = data.RigidBody(1).qz;
        roll = data.RigidBody(1).qx;
        scalar = data.RigidBody(1).qw;
        q = quaternion(roll,yaw,pitch,scalar);
        qRot = quaternion(0,0,0,1);
        q = mtimes(q,qRot);
        a = EulerAngles(q,'zyx');

				%orientation measure
				traj_AR2_OR_G(index,:) = [a(2), a(1), a(3)]; %yaw pitch
				traj_AR2_OR_W(index,:) = traj_AR2_OR_G(index,:) - offsetAng;

		%Error Calc and correction

		%pos correction
		if index == 1
            error_AR2_W(index,:) = zeros(1,3);
            %error in pos using PID!!!
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
        commandDot_AR2_J = trajectoryIK(commandDot_AR2_W(index,:)',command_AR2_J(index,:));


        statesArray_AR2_J = [command_AR2_J(index,1),command_AR2_J(index,2),command_AR2_J(index,3)...
                       command_AR2_J(index,4),command_AR2_J(index,5),command_AR2_J(index,6)...
                       commandDot_AR2_J(1),commandDot_AR2_J(2),commandDot_AR2_J(3)...
                       commandDot_AR2_J(4),commandDot_AR2_J(5),commandDot_AR2_J(6)];

        Stepper1.updateStates(statesArray_AR2_J);

		command_AR2_J(index+1,:)=command_AR2_J(index,:)'+(commandDot_AR2_J'*timestep);

        time(index,:) = toc;
        pos_AR2_W(index,:) = [traj_AR2_W(index,1) traj_AR2_W(index,2) traj_AR2_W(index,3)];
        error_AR2_W(index,:) = [error_AR2_W(index,1) error_AR2_W(index,2) error_AR2_W(index,3)];
        traj_AR2_G(index,:) = [command_AR2_W(index,1) command_AR2_W(index,2) command_AR2_W(index,3)];
        index = index + 1;
    end
end


%Set motors back to zero
Stepper1.default("REST");

end
