function J = AR2Jacobiantool(theta0, Robot)
theta1 = theta0(1);
theta2 = theta0(2);
theta3 = theta0(3);
theta4 = theta0(4);
theta5 = theta0(5);
theta6 = theta0(6);
J = Robot.jacobn([theta1 theta2 theta3 theta4 theta5 theta6]);