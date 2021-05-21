% Function to calculate the inital error in the effector pose

function [err,R] = getError(pos_world, theta_world, q)
[pos_base, theta_base] = AR2fkine(q);
e_pos = pos_world - pos_base;
C_world = eul2r(theta_world');
C_base = eul2r(theta_base');
[eo,~] = getOrientErr(C_world, C_base);
err = [e_pos; eo];
R=rotz(eo(3))*roty(eo(2))*rotx(eo(1));
end

%now you take output

% pos_base=pos_world-err(1:3)
%^this is the base position offset
% angleBaseMeasured=R*angleWorld
%^this is the angle offset

%consider using a filter to get this offset to be more accurate