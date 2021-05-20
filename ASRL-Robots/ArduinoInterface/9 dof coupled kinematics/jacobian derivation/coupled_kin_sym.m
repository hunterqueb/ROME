% ----------------------------------------------------------------------
% {Ahmed Seleit June 2019}
% Numerical simulation of the integrated forward kinematics for ROME
% A 6 dof mobile manipulator is fixed on top of a holonomic mobile
% manipulator
% The basr fram of the manipulator is asuumed to coincide with the ground
% vehicle frame

% Edited by Hunter Quebedeaux Feb 2021, added support for setting base
% frame offsets
clear all; close all; clc;

% % constants
% the distance from the base frames of the gv to the ar2
X_ORIGIN_DIST = 0;
Y_ORIGIN_DIST = 21.12; % in mm
Y_ORIGIN_DIST = 0;

%% Inputs
syms m1 m2 m3 m4 m5 m6 g
% DH parameters symbolic
syms theta1 theta2 theta3 theta4 theta5 theta6
syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6
syms d1 d2 d3 d4 d5 d6
syms a1 a2 a3 a4 a5 a6 psi L
% Link center of mass position vector wrt its frame

% Center of mass coordinates for 6 links
pc1 = sym('pc1_',[3 1]);
pc2 = sym('pc2_',[3 1]);
pc3 = sym('pc3_',[3 1]);
pc4 = sym('pc4_',[3 1]);
pc5 = sym('pc5_',[3 1]);
pc6 = sym('pc6_',[3 1]);

pc1 = [pc1; 1];
pc2 = [pc2; 1];
pc3 = [pc3; 1];
pc4 = [pc4; 1];
pc5 = [pc5; 1];
pc6 = [pc6; 1];      
%% DH transformatio matrices
DH1 = [cos(theta1) -sin(theta1)*cos(alpha1) sin(theta1)*sin(alpha1) a1*cos(theta1);
    sin(theta1) cos(theta1)*cos(alpha1) -cos(theta1)*sin(alpha1) a1*sin(theta1);
    0 sin(alpha1) cos(alpha1) d1;
    0 0 0 1];
DH2 = [cos(theta2) -sin(theta2)*cos(alpha2) sin(theta2)*sin(alpha2) a2*cos(theta2);
    sin(theta2) cos(theta2)*cos(alpha2) -cos(theta2)*sin(alpha2) a2*sin(theta2);
    0 sin(alpha2) cos(alpha2) d2;
    0 0 0 1];
DH3 = [cos(theta3) -sin(theta3)*cos(alpha3) sin(theta3)*sin(alpha3) a3*cos(theta3);
    sin(theta3) cos(theta3)*cos(alpha3) -cos(theta3)*sin(alpha3) a3*sin(theta3);
    0 sin(alpha3) cos(alpha3) d3;
    0 0 0 1];
DH4 = [cos(theta4) -sin(theta4)*cos(alpha4) sin(theta4)*sin(alpha4) a4*cos(theta4);
    sin(theta4) cos(theta4)*cos(alpha4) -cos(theta4)*sin(alpha4) a4*sin(theta4);
    0 sin(alpha4) cos(alpha4) d4;
    0 0 0 1];
DH5 = [cos(theta5) -sin(theta5)*cos(alpha5) sin(theta5)*sin(alpha5) a5*cos(theta5);
    sin(theta5) cos(theta5)*cos(alpha5) -cos(theta5)*sin(alpha5) a5*sin(theta5);
    0 sin(alpha5) cos(alpha5) d5;
    0 0 0 1];
DH6 = [cos(theta6) -sin(theta6)*cos(alpha6) sin(theta6)*sin(alpha6) a6*cos(theta6);
    sin(theta6) cos(theta6)*cos(alpha6) -cos(theta6)*sin(alpha6) a6*sin(theta6);
    0 sin(alpha6) cos(alpha6) d6;
    0 0 0 1];
%% Homogenous transformation matrices for the robotic manipulator
HT1 = DH1;      %look at the order ofd the mulatiplication 
HT2 = HT1*DH2;
HT3 = HT2*DH3;
HT4 = HT3*DH4;
HT5 = HT4*DH5;
HT6 = HT5*DH6;
%% Jacobian of the robotic manipulator
% Link center of mass position vector wrt base frame
pc10 = HT1 * pc1;
pc20 = HT2 * pc2;
pc30 = HT3 * pc3;
pc40 = HT4 * pc4;
pc50 = HT5 * pc5;
pc60 = HT6 * pc6;
% Translational Jacobians
theta = [theta1 theta2 theta3 theta4 theta5 theta6];
Jv1 = jacobian(pc10(1:3),theta);
Jv2 = jacobian(pc20(1:3),theta);
Jv3 = jacobian(pc30(1:3),theta);
Jv4 = jacobian(pc40(1:3),theta);
Jv5 = jacobian(pc50(1:3),theta);
Jv6 = jacobian(pc60(1:3),theta);
% Rotational Jacobians
Jw1 = [HT1(1:3,3) zeros(3,5)];
Jw2 = [HT1(1:3,3) HT2(1:3,3) zeros(3,4)];
Jw3 = [HT1(1:3,3) HT2(1:3,3) HT3(1:3,3) zeros(3,3)];
Jw4 = [HT1(1:3,3) HT2(1:3,3) HT3(1:3,3) HT4(1:3,3) zeros(3,2)];
Jw5 = [HT1(1:3,3) HT2(1:3,3) HT3(1:3,3) HT4(1:3,3) HT5(1:3,3) zeros(3,1)];
Jw6 = [[0 0 1]' HT1(1:3,3) HT2(1:3,3) HT3(1:3,3) HT4(1:3,3) HT5(1:3,3)];
% combined Jacobians
J1 = [Jv1; Jw1];
J2 = [Jv2; Jw2];
J3 = [Jv3; Jw3];
J4 = [Jv4; Jw4];
J5 = [Jv5; Jw5];
J6 = [Jv6; Jw6];
%% Wheels to Ground Vehicle frame Jacobian [6x3]
Jv = [-2/3*sin(psi) -2/3*sin(pi/3-psi) 2/3*sin(pi/3+psi);
      2/3*cos(psi) -2/3*cos(pi/3-psi) -2/3*cos(pi/3+psi);
      0 0 0 ;
      0 0 0;
      0 0 0;
      1/(3*L) 1/(3*L) 1/(3*L) ];
%% Ground vehicle to end effector Jacobian [6x6]
Pex = pc60(1);
Pey = pc60(2);
Jve = [1 0 0 0 0 -sqrt((Pex+X_ORIGIN_DIST)^2 + (Pey+Y_ORIGIN_DIST)^2)*sin(theta1);
       0 1 0 0 0 sqrt((Pex+X_ORIGIN_DIST)^2 + (Pey+Y_ORIGIN_DIST)^2)*cos(theta1);
       0 0 1 1 1 0 ;
       0 0 1 1 1 0;
       0 0 1 1 1 0;
       0 0 1 1 1 1];
%% Integrated system Jacobian
R_B2I = [cos(psi)   sin(psi)    0   0           0           0;
         -sin(psi)  cos(psi)    0   0           0           0;
         0          0           1   0           0           0;
         0          0           0   cos(psi)    sin(psi)    0;
         0          0           0   -sin(psi)   cos(psi)    0;
         0          0           0   0           0           1];
J = R_B2I*J6;
J_ROME = [J Jve*Jv];

fid1 = fopen('J_ROME.txt','wt');
fprintf(fid1, '%s \n', char(J_ROME));
fclose('all');


  
