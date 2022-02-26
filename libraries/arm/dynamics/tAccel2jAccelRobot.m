function qddot = tAccel2jAccelRobot(theta,thetadot,xddot,Robot)
    qddot = pinv(Jacobian0_analytical(theta)) * (xddot - Robot.jacob_dot(theta,thetadot));
%     returns the joint angle acceleration of the ar2 in units rad/s^2
end