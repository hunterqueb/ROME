%1 get init pose

%2 pre calculations

%3 in loop solve ode


%1 + 2 out of for loop

%data.RigidBody(2)

%assume J is defined here

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

%%

offset = getTransformationMatrix(command0_AR2_J,0);

data = natnetclient.getFrame();

initialStates_G=[data.RigidBody(1).z data.RigidBody(1).x data.RigidBody(1).y]*1000;

initial_W(index,:) = traj_AR2_G(index,:) - offset;

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



thetaIC=[q1;q2;q3;q4;q5;q6];

theta_ref = thetaIC;
thetadot_ref = zeros(6,1);

%add angles
traj_AR2_G(index,:) = [data.RigidBody(2).z data.RigidBody(2).x data.RigidBody(2).y]*1000;

%fix offset, how to do transform? do i need to do a C matrix transform
%again?
traj_AR2_W(index,:) = traj_AR2_G(index,:) - offset;
%^offset
%   for position, motive gives world frame/inertial frame
%       you can reuse the code you have right now!
%   for orentiation, motive gives body frame orentation.
%       remember, you cant subtract!

[x_init, theta_init] = AR2fkine(thetaIC);

Binv = eul2jac(theta_ref);
omega_ref = Binv*thetadot_ref;

%assuming inital error is zero

X0=zeros(12,1);

% 3 - in ode loop
%consider changing this to make faster

%t=0;
%dt=end(timeVector)/length(timeVector);
%for(i=1:length(timeVector)
%t=t+dt;
[~, theta] = AR2fkine(q);

C_ref = eul2r(theta_ref');
Crot = eul2r(theta');
[~,L] = getOrientErr(C_ref, Crot);

%gains
kp=1;
ko=1;
Kp = [kp 0 0
      0 kp 0
      0 0 kp];
Ko = [ko 0 0
      0 ko 0
      0 0 ko];

qdot = pinv(J)*[xdot_ref + Kp*ep; pinv(L)*(L'*omega_ref + Ko*eo)];
errdot = [xdot_ref; L'*omega_ref] - [eye(3), zeros(3); zeros(3), L]*J*qdot;
xdot = [qdot; errdot];

%send q dot directly

 
% to find pose
%     IK
%         vel > thetad > integrate > theta
%         

%find q using ODE methods

statesArray_AR2_J = [command_AR2_J(index,1),command_AR2_J(index,2),command_AR2_J(index,3)...
                       command_AR2_J(index,4),command_AR2_J(index,5),command_AR2_J(index,6)...
                       commandDot_AR2_J(1),commandDot_AR2_J(2),commandDot_AR2_J(3)...
                       commandDot_AR2_J(4),commandDot_AR2_J(5),commandDot_AR2_J(6)];
                   
%end
%repeat start of 3