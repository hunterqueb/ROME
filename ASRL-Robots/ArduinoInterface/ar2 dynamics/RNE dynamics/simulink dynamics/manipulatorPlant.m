function qdd = manipulatorPlant(u,Robot)

 Q = u(1:6);
QD = u(7:12);
TORQUE = u(13:18);
qdd = Robot.accel(Q', QD', TORQUE');
