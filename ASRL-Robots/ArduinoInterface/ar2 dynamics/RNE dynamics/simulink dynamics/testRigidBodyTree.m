robot = rigidBodyTree;


body1 = rigidBody('b1');
body2 = rigidBody('b2');
body3 = rigidBody('b3');
body4 = rigidBody('b4');
body5 = rigidBody('b5');
body6 = rigidBody('b6');


L(1) = Link([0 169.77/1000 64.2/1000 -piHalves], 'R');
L(2) = Link([0 0 305/1000 0], 'R');
L(3) = Link([-piHalves 0 0 piHalves], 'R');
L(4) = Link([0 -222.63/1000 0 -piHalves], 'R');
L(5) = Link([0 0 0 piHalves], 'R');
L(6) = Link([pi -36.25/1000 0 0], 'R');


%load inertia matrices
inertias

%set inertia matrices for each link
L(1).I=I1;
L(2).I=I2;
L(3).I=I3;
L(4).I=I4;
L(5).I=I5;
L(6).I=I6;

%link masses taken from urdf

L(1).m=0.88065;
L(2).m=0.57738;
L(3).m=0.1787;
L(4).m=0.34936;
L(5).m=0.11562;
L(6).m=0.013863;

% center of mass of all links taken from urdf
L(1).r=[-0.022706 0.04294 -0.12205];
L(2).r=[0.064818 -0.11189 -0.038671];
L(3).r=[-0.00029765 -0.023661 -0.0019125];
L(4).r=[-0.0016798 -0.00057319 -0.074404];
L(5).r=[0.0015066 -1.3102E-05 -0.012585];
L(6).r=[2.9287E-10 -1.6472E-09 0.0091432];


% assuming friction to be zero in this model
L(1).Jm=0;
L(2).Jm=0;
L(3).Jm=0;
L(4).Jm=0;
L(5).Jm=0;
L(6).Jm=0;
