% Function to propagte the kinematics of AR2 6 revolute joint robotic
% mamipulator
function xdot = AR2KinDE_anal(t, x0)
%% Reference Pose and Velocities
global Robot
x_ref = [10*sin(t)+200; 10*sin(t)+100; 10*sin(t)+200]; 
xdot_ref = [10*cos(t); 10*cos(t); 10*cos(t)];
tr = 5*pi/180*sin(2*pi/10/4.*t) + 60*pi/180;
theta_ref = [tr; tr; tr];
tr_dot = 5*pi/180.*cos(2*pi/10/4.*t);
thetadot_ref = [tr_dot; tr_dot; tr_dot];
Binv = eul2jac(theta_ref);
omega_ref = Binv*thetadot_ref;
% fprintf('%d\n',omega_ref*180/pi)
%% System variables
q = x0(1:6);
ep = x0(7:9);
eo = x0(10:12);
J = Jacobian0_analytical(q);
[~, theta] = AR2FKZYZ(q);
C_ref = eul2r(theta_ref');
Crot = eul2r(theta');
[~,L] = getOrientErr(C_ref, Crot);
omega_ref;
Kp = eye(3);
Ko = eye(3);
%% Differential Equations
qdot = pinv(J)*[xdot_ref + Kp*ep; pinv(L)*(L'*omega_ref + Ko*eo)];
errdot = [xdot_ref; L'*omega_ref] - [eye(3), zeros(3); zeros(3), L]*J*qdot;
xdot = [qdot; errdot];
end


