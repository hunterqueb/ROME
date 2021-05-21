% function to calculate analytical Jacobian 
function states = manipFK(theta0)

theta1 = theta0(1);
theta2 = theta0(2);
theta3 = theta0(3)-(90*pi/180);
theta4 = theta0(4);
theta5 = theta0(5);
theta6 = theta0(6)+(180*pi/180);

d1 = 169.77;    a1 = 64.2;  alpha1 = -90*pi/180;    
d2 = 0;         a2 = 305;   alpha2 = 0;             
d3 = 0;         a3 = 0;     alpha3 = 90*pi/180;     
d4 = -222.63;   a4 = 0;     alpha4 = -90*pi/180;    
d5 = 0;         a5 = 0;     alpha5 = 90*pi/180;     
d6 = -36.25;    a6 = 0;     alpha6 = 0;             

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

HT6 = DH1*DH2*DH3*DH4*DH5*DH6;

x = HT6(1,4);
y = HT6(2,4);
z = HT6(3,4);
pitch = atan2(sqrt(HT6(1,3)^2+HT6(2,3)^2),-HT6(3,3));
yaw = atan2(HT6(3,1)/pitch,HT6(3,2)/pitch);
roll = atan2(HT6(1,3)/pitch,HT6(2,3)/pitch);


states =  [x y z roll pitch yaw]';
