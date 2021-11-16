function qddot = tAccel2jAccel(theta,thetadot,xddot,Robot)
    qddot = pinv(Jacobian0_analytical(theta)) * (xddot - Robot.jacob_dot(theta,thetadot));
%     returns the joint angle acceleration of the ar2 in units rad/s^2
end