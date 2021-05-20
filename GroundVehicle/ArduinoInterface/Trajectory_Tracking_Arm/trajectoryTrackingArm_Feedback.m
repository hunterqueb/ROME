
function [trajectory,time,error,initposWork,initposGlobal] = trajectoryTrackingArm_Feedback(path,refTraj,Stepper1,theta0)
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
pause(10);

%% Setup: For FK and IK Functions
% kp=10;    ki=0;    kd=0.0003;
kp=0;    ki=0;    kd=0;
reference=zeros(length(refTraj(:,1)),6);
referenceVel=zeros(length(refTraj(:,2)),6);

referenceVel(:,3)=refTraj(:,2);

% thetad=zeros(6,1);
% thetaOld=zeros(6,1);

theta=theta0;

initialStatesWorld=manipFK(theta0);

reference(:,1)=initialStatesWorld(1);
reference(:,2)=initialStatesWorld(2);
reference(:,4)=initialStatesWorld(4);
reference(:,5)=initialStatesWorld(5);
reference(:,6)=initialStatesWorld(6);

for i=1:length(refTraj(:,1))
   temp=manipFK(path(i,(2:7)));
   reference(i,3)=temp(3);
end
initposWork = reference(1,1:3);
trajectory=zeros(length(path(:,1)),6);
trajectoryGlobal=zeros(length(path(:,1)),6);
trajectoryVel=zeros(length(path(:,1)),6);
time=path(:,1);
%error=zeros(length(path(:,1)),6);
        data = natnetclient.getFrame;
        initGlobal=[-data.LabeledMarker(1).x -data.LabeledMarker(1).z data.LabeledMarker(1).y 0 0 0]*1000;
        posWork=reference(1,1:3);
        initWork=[posWork 0 0 0];
        offset=initGlobal-initWork;
initposGlobal=[-data.LabeledMarker(1).x -data.LabeledMarker(1).z data.LabeledMarker(1).y]*1000;
% warmup(theta0);
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

        trajectoryGlobal(index,:) = [-data.LabeledMarker(1).x -data.LabeledMarker(1).z data.LabeledMarker(1).y 0 0 0]*1000;
        trajectory(index,:) = ([-data.LabeledMarker(1).x -data.LabeledMarker(1).z data.LabeledMarker(1).y 0 0 0]*1000)-offset;
        %Multiple by 1000 to convert from m to mm
        time(index,:) = toc;


        %function transformTrajectory needed?- input trajectory and output trajectory scaled to a new output
               %how do i transform the captured trajectory to our reference one?
        %Not needed. Velocity should not need a change of basis.

        %should error be handled in joint space or cartesian space?
        %current error is calced in cartesian space

        %error found by subtracting velocity of the trajectory in cartesian space by the
        %velocity captured by the camera system.

        if index == 1
            thetad=path(index,8:13);
            error(index,:) = zeros(1,6);
            trajectoryVel(index,:)=zeros(1,6);
        else
            %path(1,1) is timestep assuming uniform frequency
            if index > 1 && index < 7
                trajectoryVel(index,:)=(trajectory(index,:)-trajectory(index-1,:))/path(1,1);
            else
                trajectoryVel(index,:)=(10*trajectory(index-6,:)-72*trajectory(index-5,:)+255*trajectory(index-4,:)...
                    -400*trajectory(index-3,:)+450*trajectory(index-2,:)-360*trajectory(index-1,:)+147*trajectory(index,:))...
                    /60*path(1,1);
            end
            %error in pos!!!
            error(index,:) = reference(index,:)-trajectory(index,:);
            %use PD Controller: ki=0
            up=kp*(error(index,:));
            ui=ki*(error(index,:)*path(1,1));
            ud=kd*((error(index,:)-error(index-1,:))/path(1,1));

            u=up+ui+ud;
            %PD part
            correctedVel=referenceVel(index,:)-u;

            thetad=trajectoryIK(correctedVel',theta);

        end
        statesArray = [theta(1),theta(2),theta(3)...
                       theta(4),theta(5),theta(6)...
                       thetad(1),thetad(2),thetad(3)...
                       thetad(4),thetad(5),thetad(6)];

        Stepper1.updateStates(statesArray);
        theta=theta+(thetad*path(1,1));

        index = index + 1;
    end
end
data = natnetclient.getFrame;
trajectory(3000,:) = [-data.LabeledMarker(1).x -data.LabeledMarker(1).z data.LabeledMarker(1).y 0 0 0]*1000;
end
