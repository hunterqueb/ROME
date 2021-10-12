d1 = 169.77;    a1 = 64.2;  alpha1 = -90*pi/180;    
d2 = 0;         a2 = 305;   alpha2 = 0;             
d3 = 0;         a3 = 0;     alpha3 = 90*pi/180;     
d4 = -222.63;   a4 = 0;     alpha4 = -90*pi/180;    
d5 = 0;         a5 = 0;     alpha5 = 90*pi/180;     
d6 = -36.25;    a6 = 0;     alpha6 = 0;  


% phantom link -- needed for 
L(1) = Link([0*pi/180 0 0 90*pi/180], 'R');
L(2) = Link([90*pi/180 0 0 90*pi/180], 'P');
L(3) = Link([90*pi/180 0 0 90*pi/180], 'P');

L(2).jointtype = 'P';
L(3).jointtype = 'P';

L(4) = Link([90*pi/180 0 0 0*pi/180], 'R');
L(5) = Link([0 d1 a1 alpha1], 'R');
L(6) = Link([0 d2 a2 alpha2], 'R');
L(7) = Link([0 d3 a3 alpha3], 'R');
L(8) = Link([0 d4 a4 alpha4], 'R');
L(9) = Link([0 d5 a5 alpha5], 'R');
L(10) = Link([0 d6 a6 alpha6], 'R');

Robot = SerialLink(L);

rev1AngleLock = 90 * pi/180;

%% FK Verification

% in mm and degrees
% X,Y, yaw
gvStates = [10 10 10];
% in degrees
% joints 1 - 6
armStates = [10 10 10 10 10 10];

robotqAnalytical = [gvStates,armStates];
% convert the angles from degrees to radians
robotqAnalytical(3:end) = robotqAnalytical(3:end)* pi/180;
robotqToolbox = [rev1AngleLock,robotqAnalytical];

RoboticsToolboxHTMAnswer = Robot.fkine(robotqToolbox);
AnalyticalHTMAnswer = ROMEFK(robotqAnalytical);

% FK of coupled system verified

%% IK Verification

% in mm and degrees
% X,Y, yaw
gvStates = [10 0 0];
% in degrees
% joints 1 - 6
armStates = [0 0 0 0 0 0];

robotqAnalytical = [armStates,gvStates];
% convert the angles from degrees to radians
robotqAnalytical(1:6) = robotqAnalytical(1:6)* pi/180;
robotqToolbox = [rev1AngleLock,robotqAnalytical];

gvMotorVel = [10;0;0];
armAngleVel = [0;0;0;0;0;0];
jointAngleVel = [armAngleVel;gvMotorVel];
jointAngleVel(1:6) = jointAngleVel(1:6) * pi/180;
jointAngleVelToolbox = [0;jointAngleVel(7:end);jointAngleVel(1:6)];

RoboticsToolboxJacobianAnswer = Robot.jacob0(robotqToolbox) * jointAngleVelToolbox
Robot.jacob_dot
AnalyticalJacobianAnswer = J_ROME(robotqAnalytical) * jointAngleVel




