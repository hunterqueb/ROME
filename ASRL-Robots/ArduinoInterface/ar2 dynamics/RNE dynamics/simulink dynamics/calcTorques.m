function torques = calcTorques(u,Robot)

Q = u(1:6);
QD = u(7:12);
QDD = u(13:18);
torques = Robot.rne(Q', QD', QDD');
