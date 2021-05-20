%GOALS
%Ri- inital orentation
%Rf - final orentation

%Rotiation matrix  Rif=[Rix, Riy, Riz]'*[Rfx, Rfy, Rfz] = [1 2 3;
%                                                          4 5 6;
%                                                          7 8 9;];
%for Ri=Rf, Rif=I

%nuef=acos((Rif(1,1)+Rif(2,2)+Rif(3,3)-1)/2)     %angle of rotation about axis r
%r=1/(2*sin(nuef) * [Rif(3,2)-Rif(2,3);Rif(1,3)-Rif(3,1);Rif(2,1)-Rif(1,2);];      %unit vector of the axis of rotation


%J already defined.

%required by defintions:

%reference position and orentation as a funciton of time
%for a specifc experiment, i could design a simple function to pose track, however, i shouldnt do that

%theta matrices like theta_ref are defined the orentation of the end effector in terms of zyx rotations


%Binv matrix required
Binv = eul2jac(theta_ref);
omega_ref = Binv*thetadot_ref;


%Solving Diffeq requires L matrix
%orientation error based on the column representation of the rotation matrices
C_ref = eul2r(theta_ref');
Crot = eul2r(theta'); %measured from cameras

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


%Solving Diff Eq
qdot = pinv(J)*[xdot_ref + Kp*ep; pinv(L)*(L'*omega_ref + Ko*eo)]; %qdot - angle of joints vel
errdot = [xdot_ref; L'*omega_ref] - [eye(3), zeros(3); zeros(3), L]*J*qdot; %error vel
xdot = [qdot; errdot]; %full output vel

%put into fast ode solver above to find - solve equations in from t1 to t2,
%put back into solution

%q and err
