% DH parameters symbolic
syms theta1 theta2 theta3 theta4 theta5 theta6
syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6
syms d1 d2 d3 d4 d5 d6
syms a1 a2 a3 a4 a5 a6

DH1 = [cos(theta1) -sin(theta1)*cos(alpha1) sin(theta1)*sin(alpha1) a1*cos(theta1);
       sin(theta1) cos(theta1)*cos(alpha1) -cos(theta1)*sin(alpha1) a1*sin(theta1);
        0           sin(alpha1)             cos(alpha1)             d1;
        0 0 0 1];
DH2 = [cos(theta2) -sin(theta2)*cos(alpha2) sin(theta2)*sin(alpha2)     a2*cos(theta2);
        sin(theta2) cos(theta2)*cos(alpha2) -cos(theta2)*sin(alpha2)    a2*sin(theta2);
        0           sin(alpha2)             cos(alpha2)                 d2;
    0 0 0 1];
DH3 = [cos(theta3) -sin(theta3)*cos(alpha3) sin(theta3)*sin(alpha3) a3*cos(theta3);
    sin(theta3) cos(theta3)*cos(alpha3) -cos(theta3)*sin(alpha3)    a3*sin(theta3);
    0 sin(alpha3) cos(alpha3)                                       d3;
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

HT1 = DH1;      %look at the order ofd the mulatiplication 
HT2 = HT1*DH2;
HT3 = HT2*DH3;
HT4 = HT3*DH4;
HT5 = HT4*DH5;
HT6 = HT5*DH6;

T = HT6;

atan2(T(3,2),T(3,3))
atan2(T(3,2),T(3,3))
atan2(T(3,2),T(3,3))

x = HT6(1,4);
y = HT6(2,4);
z = HT6(3,4);

THETA = atan2(sqrt(T(1,3)^2+T(2,3)^2),T(3,3));
PHI = atan2((T(3,2)/sin(THETA)),(T(3,1)/sin(THETA)));
PSI = atan2((T(2,3)/sin(THETA)),-(T(1,3)/sin(THETA)));





% nu = atan2(sqrt(HT3(1,3)^2+HT3(2,3)^2), HT3(3,3));
% psi = atan2(HT3(3,2),-HT3(3,1)); %
% phi = atan2(HT3(2,3),HT3(1,3)); %x axis rotation


% 
% x = T_0E.t(1);
% y = T_0E.t(2);
% z = T_0E.t(3);
% states = [x y z]';
% orient = tr2eul(T_0E)';

