clear

%% % [theta1 d1 a1 alpha1], 'P'
length = 0.13;

L(1) = Link([0*pi/180 0 0 90*pi/180], 'R');
L(2) = Link([90*pi/180 0 0 90*pi/180], 'P');
L(3) = Link([90*pi/180 0 0 90*pi/180], 'P');
L(4) = Link([90*pi/180 0 0 0*pi/180], 'R');


L(2).jointtype = 'P';
L(3).jointtype = 'P';


rev1AngleLock = 90*pi/180;

Robot = SerialLink(L);

theta = 0*pi/180;

inputVelVector3 = [12; 10; 0*pi/180];
inputVelVector4 = [0;inputVelVector3];

pTheta = [-sin(theta)               cos(theta)          length;
                  -sin((pi/3)-theta)       -cos((pi/3)-theta)   length;
                   sin((pi/3)+theta)       -cos((pi/3)+theta)   length;];
               
dotGV = pTheta * inputVelVector3
               
pThetaInv = [-2/3 * sin(theta), -2/3 * sin(pi/3-theta), 2/3 * sin(pi/3+theta);
             2/3 * cos(theta), -2/3 * cos(pi/3-theta), 2/3 * cos(pi/3+theta);
             1/(3*length), 1/(3*length), 1/(3*length);]

Robot.fkine([rev1AngleLock 10 40 theta]);
% Robot.jacob0([revAngleLock 0 0 theta]);
dotRobot = Robot.jacob0([rev1AngleLock 0 0 theta]) * inputVelVector4;

xdotRobot = dotRobot(1);
ydotRobot = dotRobot(2);
zdotRobot = dotRobot(3);
thetadotRobot = dotRobot(6);

dotRobotinGV(1) = -sin(theta)*xdotRobot+cos(theta)*ydotRobot+length*thetadotRobot;
dotRobotinGV(2) = -sin(pi/3-theta)*xdotRobot-cos(pi/3-theta)*ydotRobot+length*thetadotRobot;
dotRobotinGV(3) = ((xdotRobot*sin(pi/3+theta))-(ydotRobot*cos(pi/3+theta))+length*thetadotRobot);
dotRobotinGV;

