function xddot = jAccel2tAccel(theta,thetadot,thetaddot)

xddot = Jacobian0_analytical(theta)*thetaddot + Jacobiandot0_analytical(theta,thetadot)*thetadot;

% outputs in the units of mm/s^2 and rad/s^2 as a 6x1 vector.

end