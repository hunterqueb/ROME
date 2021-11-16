function xddot = jAccel2tAccelRobot(theta,thetadot,thetaddot,Robot)

xddot = Jacobian0_analytical(theta)*thetaddot + Robot.jacob_dot(theta,thetadot);

% outputs in the units of mm/s^2 and rad/s^2 as a 6x1 vector.

end