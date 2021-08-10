function xddot = jAccel2tAccelNoJd(theta,thetaddot)
xddot = Jacobian0_analytical(theta)*thetaddot;
end

