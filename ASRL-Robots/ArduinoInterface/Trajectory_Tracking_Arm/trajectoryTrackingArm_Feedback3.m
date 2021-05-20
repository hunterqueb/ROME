
function [trajectory,reference,time,error,correctedVel,theta,toofast] = trajectoryTrackingArm_Feedback3(path,refTraj,Stepper1)
% This function takes a path, specified as 6 angle states followed by six
% angular velocity states (rad,rad/s) for each joint.
% Stepper1 is a stepper motor object for one of
% the joints on the arm. Arms must first be callibrated.
% path and refTraj are 13 columns, refTraj has z and zDot

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
kp=15;    ki=2*0.038;    kd=0.4;
reference=zeros(length(refTraj(:,1)),3);
referenceVel=zeros(length(refTraj(:,2)),3);

referenceVel(:,3)=refTraj(:,2);
theta=zeros(length(path(:,1)),6);
theta0=path(1,2:7);
theta(1,:)=theta0;

initialStatesWork=manipFK(theta0);

reference(:,1)=initialStatesWork(1);
reference(:,2)=initialStatesWork(2);


for i=1:length(refTraj(:,1))
   temp=manipFK(path(i,(2:7)));
   reference(i,3)=temp(3);
end

trajectory=zeros(length(path(:,1)),3);
trajectoryGlobal=zeros(length(path(:,1)),3);
trajectoryVel=zeros(length(path(:,1)),3);
error=zeros(length(path(:,1)),3);
time=path(:,1);
data = natnetclient.getFrame;

offset = getTransformation3(theta0);

% initWorld = [-data.LabeledMarker(1).x -data.LabeledMarker(1).z data.LabeledMarker(1).y 0 0 0]*1000;
% initWork=reference(1,:);
% offset=initWorld-initWork;

correctedVel=zeros(length(path(:,1)),3);
indexf=0;
toofast=zeros(1,4);
%% Main Loop
Ui = zeros(1,3);
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
            
        trajectoryGlobal(index,:) = [-data.LabeledMarker(1).z -data.LabeledMarker(1).x data.LabeledMarker(1).y]*1000;
        trajectory(index,:) = trajectoryGlobal(index,:) - offset;
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
            error(index,:) = zeros(1,3);
%             trajectoryVel(index,:)=zeros(1,6);
%         elseif index > 1 && index < 7
%             trajectoryVel(index,:)=(trajectory(index,:)-trajectory(index-1,:))/path(1,1);
%             error(index,:) = reference(index,:)-trajectory(index,:);
%         elseif index > 6
            %path(1,1) is timestep assuming uniform frequency
%             trajectoryVel(index,:)=(10*trajectory(index-6,:)-72*trajectory(index-5,:)+225*trajectory(index-4,:)-400*trajectory(index-3,:)+450*trajectory(index-2,:)-360*trajectory(index-1,:)+147*trajectory(index,:))/60*path(1,1);
            %error in pos!!!
        else
            error(index,:) = reference(index,:)-trajectory(index,:);
        end
        if index == 1
            %use PD Controller: ki=0
            Up=zeros(1,3);
            Ui=zeros(1,3);
            Ud=zeros(1,3);
        elseif index == 2
            Up=kp*(error(index,:));
            Ui= Ui + ki*(error(index,:)*path(1,1));
            Ud=kd*((error(index,:)-error(index-1,:))/path(1,1));
        else
            Up=kp*(error(index,:));
            Ui= Ui + ki*(error(index,:)*path(1,1));
            Ud=kd*(3*(error(index,:)-4*error(index-1,:)+error(index-2,:))/2*path(1,1));
        end   
            u=Up+Ui+Ud;
            %PD part
            correctedVel(index,:)=referenceVel(index,:)+u;
            
%             if sum(abs(correctedVel(index,3)) > abs(1.2*referenceVel(index,3))) || sum(abs(correctedVel(index,1:2)) > [2 2])
%                 indexf=indexf+1;
%                 toofast(indexf,:)=[index correctedVel(index,:)];
%                 correctedVel(index,3) = referenceVel(index,3) * 1.5;
%                 if correctedVel(index,1) > 0
%                     correctedVel(index,1) = 2;
%                 elseif correctedVel(index,1) < 0
%                     correctedVel(index,1) = -2;
%                 end
%                 if correctedVel(index,2) > 0
%                     correctedVel(index,2) = 2;
%                 elseif correctedVel(index,2) < 0
%                     correctedVel(index,2) = -2;
%                 end
%             end
            
            thetadtemp=trajectoryIK3(correctedVel(index,:)',theta(index,:));
            
            thetad=zeros(1,6);
            thetad(2)=thetadtemp(1);
            thetad(3)=thetadtemp(2);
            thetad(5)=thetadtemp(3);
            
            statesArray = [theta(index,1),theta(index,2),theta(index,3)...
                       theta(index,4),theta(index,5),theta(index,6)...
                       thetad(1),thetad(2),thetad(3)...
                       thetad(4),thetad(5),thetad(6)];
                    
        Stepper1.updateStates(statesArray);
        theta(index+1,:)=theta(index,:)'+(thetad'*path(1,1));
        
        index = index + 1;
        toc
    end
end
data = natnetclient.getFrame;
trajectory(end,:) = [-data.LabeledMarker(1).z -data.LabeledMarker(1).x data.LabeledMarker(1).y]*1000 - offset;

plotArmExp(trajectory,reference,error,path(end,1));
pause(2);
Stepper1.default("REST");
end
