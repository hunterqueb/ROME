% Funciton to calculate the orientation error based on the column
% representation of the rotation matrices
function [eo, L] = getOrientErr(C_ref, C)
%% Vectors of the rotation matricies
nd = C_ref(:,1);
sd = C_ref(:,2);
ad = C_ref(:,3);
ne = C(:,1);
se = C(:,2);
ae = C(:,3);
%% Skew Symmetric matrix representation
S_nd = Vec2Skew(nd);
S_sd = Vec2Skew(sd);
S_ad = Vec2Skew(ad);
S_ne = Vec2Skew(ne);
S_se = Vec2Skew(se);
S_ae = Vec2Skew(ae);
%% Ouputs
eo = 0.5*(cross(ne,nd) + cross(se,sd) + cross(ae,ad));
L = -0.5*(S_nd*S_ne + S_sd*S_se + S_ad*S_ae);
end