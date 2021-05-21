% function to calculate analytical Jacobian 
function [states, orientation] = AR2fk(theta0)
%% Inputs
theta1 = theta0(1);
theta2 = theta0(2);
theta3 = theta0(3);
theta4 = theta0(4);
theta5 = theta0(5);
theta6 = theta0(6);
%% DH parameters
d1 = 169.77;    a1 = 64.2;  alpha1 = -90*pi/180;    
d2 = 0;         a2 = 305;   alpha2 = 0;             
d3 = 0;         a3 = 0;     alpha3 = 90*pi/180;     
d4 = -222.63;   a4 = 0;     alpha4 = -90*pi/180;    
d5 = 0;         a5 = 0;     alpha5 = 90*pi/180;     
d6 = -36.25;    a6 = 0;     alpha6 = 0;             
%% Direct Forward Kinematics
cth1 = cos(theta1);
cth2 = cos(theta2);
cth3 = cos(theta3);
cth4 = cos(theta4);
cth5 = cos(theta5);
cth6 = cos(theta6);
cth23 = cos(theta2 + theta3);
sth1 = sin(theta1);
sth2 = sin(theta2);
sth3 = sin(theta3);
sth4 = sin(theta4);
sth5 = sin(theta5);
sth6 = sin(theta6);
sth23 = sin(theta2 + theta3);

l1 = d1;
d2 = a1;
l2 = a2;
l3 = d4;
d7 = d6;

t11 = cth1*(cth23*(cth4*cth5*cth6 - sth4*sth6) - cth6*sth5*sth23) + sth1*(cth4*sth6 + cth5*cth6*sth4);
t21 = sth1*(cth23*(-sth4*sth6 + cth4*cth5*cth6) - cth6*sth5*sth23) - cth1*(cth4*sth6 + cth5*cth6*sth4);
t31 = sth23*(cth4*cth5*cth6 - sth4*sth6) + cth6*sth5*cth23;
t12 = cth1*(sth5*sth6*sth23 - cth23*(cth6*sth4 + cth4*cth5*sth6)) + sth1*(cth4*cth6 - cth5*sth4*sth6);
t22 = sth1*(sth5*sth6*sth23 - cth23 *(cth6*sth4 + cth4*cth5*sth6)) + cth1*(-cth4*cth6 + cth5*sth4*sth6);
t32 = -sth5*sth6*cth23 - sth23 *(cth6*sth4 + cth4*cth5*sth6);
t13 = sth1*sth4*sth5 + cth1*(cth5*sth23 + cth4*sth5*cth23);
t23 = -cth1*sth4*sth5 + sth1*(cth5*sth23 + cth4*sth5*sth23);
t33 = cth4*sth5*sth23 - cth5*cth23;
t14 = l3*cth1*sth23 + l2*cth1*cth2;
t24 = sth1*sth23*l3 + l2*cth2*sth1;
t34 = l2*sth2 - l3*cth23;

s11 = cth4*cth5*cth6 - sth4*sth6;
s21 = cth5*cth6*sth4 + cth4*sth6;
s31 = -cth6*sth5;
s12 = -cth6*sth4 - cth4*cth5*sth6;
s22 = cth4*cth6-cth5*sth4*sth6;
s32 = sth5*sth6;
s13 = cth4*sth5;
s23 = sth4*sth5;
s33 = cth5;

HT3 = [s11 s12 s13;
       s21 s22 s23;
       s31 s32 s33];
   
HT6 = [t11 t12 t13 t14;
       t21 t22 t23 t24;
       t31 t32 t33 t34;
       0    0   0   1];   

 
%% Position
x = HT6(1,4);
y = HT6(2,4);
z = HT6(3,4);
%% ZYZ
nu = atan2(sqrt(HT3(1,3)^2+HT3(2,3)^2), HT3(3,3));
psi = atan2(HT3(3,2),-HT3(3,1));
phi = atan2(HT3(2,3),HT3(1,3));
%% Outputs
orientation = [phi nu psi]';
RotE = HT3;
states =  [x y z]';
