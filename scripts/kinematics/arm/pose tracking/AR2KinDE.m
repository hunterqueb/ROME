% Function to propagte the kinematics of AR2 6 revolute joint robotic
% mamipulator
function xdot = AR2KinDE(x0,xdot_ref,theta_ref,thetadot_ref)
%% Reference Pose and Velocities
Binv = eul2jac(theta_ref);
omega_ref = Binv*thetadot_ref;
%% System variables
q = x0(1:6);
ep = x0(7:9);
eo = x0(10:12);
% J = JacobionAR2(q);
J = Jacobian0_analytical(q);
[~, theta] = AR2FKZYZ(q);
C_ref = eul2r(theta_ref');
Crot = eul2r(theta');
[~,L] = getOrientErr(C_ref, Crot);
% omega_ref;
Kp = 1*eye(3);
Ko = 1*eye(3);
%% Differential Equations
qdot = pinv(J)*[xdot_ref + Kp*ep; pinv(L)*(L'*omega_ref + Ko*eo)];
errdot = [xdot_ref; L'*omega_ref] - [eye(3), zeros(3); zeros(3), L]*J*qdot;
xdot = [qdot; errdot];
end


