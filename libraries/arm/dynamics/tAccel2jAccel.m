function qddot = tAccel2jAccel(theta,thetadot,xddot)
    qddot = pinv(Jacobian0_analytical(theta)) * (xddot - Jacobiandot0_analytical(theta,thetadot) * thetadot);
%     returns the joint angle acceleration of the ar2 in units rad/s^2
end