function [endEffPos, endEffOrient] = ROMEFK(jointStates)
% INPUT - takes in the 9 joint states of the ROME platform, in the format
%         of the the last 3 being the x, y, and yaw/psi of the gv in mm and 
%         the first 6 being the joint states of the arm.

% OUTPUT - outputs the pose - position and orientation of the end effector
%          using the ZYZ rotation

% MAKE SURE ROBOTICS TOOLBOX IS ADDED TO PATH
% needs to be in mm
xGV = jointStates(7);
yGV = jointStates(8);
psiGV = jointStates(9);


theta1 = jointStates(1);
theta2 = jointStates(2);
theta3 = jointStates(3);
theta4 = jointStates(4);
theta5 = jointStates(5);
theta6 = jointStates(6);


% DH parameters for AR2 in mm
d1 = 169.77;    a1 = 64.2;  alpha1 = -90*pi/180;    
d2 = 0;         a2 = 305;   alpha2 = 0;             
d3 = 0;         a3 = 0;     alpha3 = 90*pi/180;     
d4 = -222.63;   a4 = 0;     alpha4 = -90*pi/180;    
d5 = 0;         a5 = 0;     alpha5 = 90*pi/180;     
d6 = -36.25;    a6 = 0;     alpha6 = 0;             

% Constants that describe the length between the frames of the gv and the
% ar2

X_ORIGIN_DIST = 0;
Y_ORIGIN_DIST = 21.12; % in mm
PSI_ORIGIN_ROT = 0;

Y_ORIGIN_DIST = 0; % in mm


% DH matrices for AR2
DH1AR2 = [cos(theta1) -sin(theta1)*cos(alpha1) sin(theta1)*sin(alpha1) a1*cos(theta1);
       sin(theta1) cos(theta1)*cos(alpha1) -cos(theta1)*sin(alpha1) a1*sin(theta1);
        0           sin(alpha1)             cos(alpha1)             d1;
        0 0 0 1];
DH2AR2 = [cos(theta2) -sin(theta2)*cos(alpha2) sin(theta2)*sin(alpha2)     a2*cos(theta2);
        sin(theta2) cos(theta2)*cos(alpha2) -cos(theta2)*sin(alpha2)    a2*sin(theta2);
        0           sin(alpha2)             cos(alpha2)                 d2;
    0 0 0 1];
DH3AR2 = [cos(theta3) -sin(theta3)*cos(alpha3) sin(theta3)*sin(alpha3) a3*cos(theta3);
    sin(theta3) cos(theta3)*cos(alpha3) -cos(theta3)*sin(alpha3)    a3*sin(theta3);
    0 sin(alpha3) cos(alpha3)                                       d3;
    0 0 0 1];
DH4AR2 = [cos(theta4) -sin(theta4)*cos(alpha4) sin(theta4)*sin(alpha4) a4*cos(theta4);
    sin(theta4) cos(theta4)*cos(alpha4) -cos(theta4)*sin(alpha4) a4*sin(theta4);
    0 sin(alpha4) cos(alpha4) d4;
    0 0 0 1];
DH5AR2 = [cos(theta5) -sin(theta5)*cos(alpha5) sin(theta5)*sin(alpha5) a5*cos(theta5);
    sin(theta5) cos(theta5)*cos(alpha5) -cos(theta5)*sin(alpha5) a5*sin(theta5);
    0 sin(alpha5) cos(alpha5) d5;
    0 0 0 1];
DH6AR2 = [cos(theta6) -sin(theta6)*cos(alpha6) sin(theta6)*sin(alpha6) a6*cos(theta6);
    sin(theta6) cos(theta6)*cos(alpha6) -cos(theta6)*sin(alpha6) a6*sin(theta6);
    0 sin(alpha6) cos(alpha6) d6;
    0 0 0 1];

HT1 = DH1AR2;      %look at the order of the multiplication 
HT2 = HT1*DH2AR2;
HT3 = HT2*DH3AR2;
HT4 = HT3*DH4AR2;
HT5 = HT4*DH5AR2;
TBE = HT5*DH6AR2;

% tranformation matrix from the GV to the base of the ar2 taking into account the 
% distance and rotation difference between the origins of the ar2 and gv
TGvB = [cos(PSI_ORIGIN_ROT)  sin(PSI_ORIGIN_ROT) 0  X_ORIGIN_DIST;
        -sin(PSI_ORIGIN_ROT) cos(PSI_ORIGIN_ROT) 0  Y_ORIGIN_DIST;
        0                    0                   1  0;
        0                    0                   0  1];

% transformation matrix from the inertia frame to the GV 
% taking into account the distance and rotation difference between the interial frame and the gv
TIGv = [cos(psiGV)  -sin(psiGV) 0  xGV;
        sin(psiGV) cos(psiGV) 0  yGV;
        0           0          1  0;
        0           0          0  1];

% the total tranformation matrix from the inertia frame to the end effector
% of the ROME platform
TIE = TIGv * TGvB * TBE;

xEndEff = TIE(1,4);
yEndEff = TIE(2,4);
zEndEff = TIE(3,4);
   
endEffOrient = tr2eul(TIE);

endEffPos = [xEndEff yEndEff zEndEff]';

endEffOrient = endEffOrient';

end