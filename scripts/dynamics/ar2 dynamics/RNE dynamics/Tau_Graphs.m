%AR2 DH Parameters
% http://www.petercorke.com/RTB/r9/html/Link.html
% Bm: link viscous friction- Link.B
% Tc: link Coulomb friction use the function- Link.Tc
% G: gear ratio- Link.G
% m- Link mass
% r- link centre of gravity (3x1)

%DH First, then inertia matrices for each 

%load inertia matrices
inertias

L(1) = Link([0 169.77 64.2 -1.5707], 'R');
L(2) = Link([0 0 305 0], 'R');
L(3) = Link([-1.5707 0 0 1.5707], 'R');
L(4) = Link([0 -222.63 0 -1.5707], 'R');
L(5) = Link([0 0 0 1.5707], 'R');
L(6) = Link([pi -36.25 0 0], 'R');

%set inertia matrices for each link
%link 1
I1=[0.0034 0.00042296 -0.00089231;
    0.00042296 0.00042296 0.0010848;
    -0.00089231 0.0010848 0.0027077];

%link 2
I2=[0.0047312 0.0022624 0.00032144;
    0.0022624 0.0020836 -0.00056569;
    0.00032144 -0.00056569 0.0056129];

%link 3
I3=[0.0001685 -2.7713E-05 5.6885E-06;
    -2.7713E-05 0.00012865 2.9256E-05;
    5.6885E-06 2.9256E-05 0.00020744];

%link 4
I4=[0.0030532 -1.8615E-05 -7.0047E-05;
    -1.8615E-05 0.0031033 -2.3301E-05;
    -7.0047E-05 -2.3301E-05 0.00022264];

%link 5
I5=[5.5035E-05 -1.019E-08 -2.6243E-06;
    -1.019E-08 8.2921E-05 1.4437E-08;
    -2.6243E-06 1.4437E-08 5.2518E-05];

%link 6
I6=[1.3596E-06 3.0585E-13 5.7102E-14;
    3.0585E-13 1.7157E-06 6.3369E-09;
    5.7102E-14 6.3369E-09 2.4332E-06];

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

%link center of gravity from urdf
L(1).r=[0 0 0];
L(2).r=[0.064818 -0.11189 -0.038671];
L(3).r=[-0.00029765 -0.023661 -0.0019125];
L(4).r=[-0.0016798 -0.00057319 -0.074404];
L(5).r=[2.9287E-10 -1.6472E-09 0.0091432];
L(6).r=[-0.000294 0 0.02117];

%Friction is zero in this sim.
L(1).Jm=0;
L(2).Jm=0;
L(3).Jm=0;
L(4).Jm=0;
L(5).Jm=0;
L(6).Jm=0;

Robot = SerialLink(L);
Robot.name = 'AR2';

Robot.nofriction('all');


%manuever
%time steps in manuever
T=100;
times=[1:T];

%joint space coordinates for the two motions.
q1=[0.11394 -40.1878 -68.04 1.7108 109.7497 2.035]*pi/180;
%angles offsets
q1(3) = q1(3)-90;
q1(6) = q1(6)+180;

q2=[0.10852 12.5147 -53.7355 2.3715 42.7753 -0.279]*pi/180;
q2(3) = q1(3)-90;
q2(6) = q1(6)+180; 
%create joint space trajectory w/ velocities and accelerations
[Q,Qd,Qdd]=jtraj(q1,q2,T);

%inverse dynamics calc
Tau=Robot.rne(Q,0*Qd,0*Qdd);

plot(times,Tau(:,1))
hold on
plot(times,Tau(:,2))
plot(times,Tau(:,3))
plot(times,Tau(:,4))
plot(times,Tau(:,5))
plot(times,Tau(:,6))







