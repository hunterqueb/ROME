clear

%% % [theta1 d1 a1 alpha1], 'P'
length = 0.13;

L(1) = Link([0*pi/180 0 0 -90*pi/180], 'P');
L(2) = Link([90*pi/180 0 0 90*pi/180], 'P');
L(3) = Link([90*pi/180 0 0 90*pi/180], 'R');

L(1).jointtype = 'P';
L(2).jointtype = 'P';


Robot = SerialLink(L);

theta = 0;

inputVector = [12; 01; 10];

pTheta = [-sin(theta)               cos(theta)          length;
                  -sin((pi/3)-theta)       -cos((pi/3)-theta)   length;
                   sin((pi/3)+theta)       -cos((pi/3)+theta)   length;];
               
dotGV = pTheta * inputVector;
               
pThetaInv = [-2/3 * sin(theta), -2/3 * sin(pi/3-theta), 2/3 * sin(pi/3+theta);
             2/3 * cos(theta), -2/3 * cos(pi/3-theta), 2/3 * cos(pi/3+theta);
             1/(3*length), 1/(3*length), 1/(3*length);];

Robot.fkine([10 40 theta])
Robot.jacob0([0 0 theta]);
dotRobot = Robot.jacob0([0 0 theta]) * inputVector;

xdotRobot = dotRobot(1);
ydotRobot = dotRobot(3);
zdotRobot = dotRobot(2);
thetadotRobot = dotRobot(5);

dotRobotinGV(1) = -sin(theta)*xdotRobot+cos(theta)*ydotRobot+length*thetadotRobot;
dotRobotinGV(2) = -sin(pi/3-theta)*xdotRobot-cos(pi/3-theta)*ydotRobot+length*thetadotRobot;
dotRobotinGV(3) = (sin(pi/3+theta)*xdotRobot+cos(pi/3+theta)*ydotRobot+length*thetadotRobot) - inputVector(3);

dotRobotinGV;