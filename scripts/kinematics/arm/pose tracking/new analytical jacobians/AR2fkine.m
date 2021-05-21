function [states, orient] = AR2fkine(theta0,Robot)

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