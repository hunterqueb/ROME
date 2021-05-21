function [states, orient] = AR2fkine(theta0)

d1 = 169.77;    a1 = 64.2;  alpha1 = -90*pi/180;    
d2 = 0;         a2 = 305;   alpha2 = 0;             
d3 = 0;         a3 = 0;     alpha3 = 90*pi/180;     
d4 = -222.63;   a4 = 0;     alpha4 = -90*pi/180;    
d5 = 0;         a5 = 0;     alpha5 = 90*pi/180;     
d6 = -36.25;    a6 = 0;     alpha6 = 0;  

L(1) = Link([0 d1 a1 alpha1], 'R');
L(2) = Link([0 d2 a2 alpha2], 'R');
L(3) = Link([0 d3 a3 alpha3], 'R');
L(4) = Link([0 d4 a4 alpha4], 'R');
L(5) = Link([0 d5 a5 alpha5], 'R');
L(6) = Link([0 d6 a6 alpha6], 'R');
Robot = SerialLink(L);


theta1 = theta0(1);
theta2 = theta0(2);
theta3 = theta0(3);
theta4 = theta0(4);
theta5 = theta0(5);
theta6 = theta0(6);

T_0E = Robot.fkine([theta1 theta2 theta3 theta4 theta5 theta6]);

x = T_0E.t(1);
y = T_0E.t(2);
z = T_0E.t(3);
states = [x y z]';
orient = tr2eul(T_0E)';