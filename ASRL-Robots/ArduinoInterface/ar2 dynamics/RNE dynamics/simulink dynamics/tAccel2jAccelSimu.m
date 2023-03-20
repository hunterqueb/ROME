function qddot = tAccel2jAccelSimu(u,Robot)
theta = u(1:6); thetadot = u(7:12); xddot = u(13:18);
    qddot = pinv(Jacobian0_analytical(theta)) * (xddot - Robot.jacob_dot(theta,thetadot));
%     returns the joint angle acceleration of the ar2 in units rad/s^2
end