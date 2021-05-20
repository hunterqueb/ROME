%% AR2 robotic manipulator closed loop resolved rate algorithm
clear
%% Robot Definition
d1 = 169.77;    a1 = 64.2;  alpha1 = -90*pi/180;    
d2 = 0;         a2 = 305;   alpha2 = 0;             
d3 = 0;         a3 = 0;     alpha3 = 90*pi/180;     
d4 = -222.63;   a4 = 0;     alpha4 = -90*pi/180;    
d5 = 0;         a5 = 0;     alpha5 = 90*pi/180;     
d6 = -36.25;    a6 = 0;     alpha6 = 0;  

L(1) = Link([0 d1 a1 alpha1], 'R');
L(2) = Link([0 d2 a2 alpha2], 'R');
L(3) = Link([0 d3 a3 alpha3], 'R');
L(4) = Link([0 d4 a4 alpha4], 'R');
L(5) = Link([0 d5 a5 alpha5], 'R');
L(6) = Link([0 d6 a6 alpha6], 'R');
global Robot
global xref_old
told = 0;
xref_old = zeros(3,1);
Robot = SerialLink(L);

%% Initial conditions and error calculations
ti = 0;
global manueverTime
manueverTime = 30;
% x_ref_coef = 10;
% x_ref_init = [x_ref_coef*sin(ti)+200; x_ref_coef*sin(ti)+100; x_ref_coef*sin(ti)+200]; 
% xdot_ref_init = [x_ref_coef*cos(ti); x_ref_coef*cos(ti); x_ref_coef*cos(ti)];
% tr = 5*pi/180.*(2*pi/10/4*ti) + 60*pi/180;
% theta_ref_init = [tr; tr; tr];

global q_init;
q_init = [-90 -80 80 0 0 0]'*pi/180;

global initPos;
global initOri;
[initPos,initOri] = AR2FKZYZ(q_init);


% x_ref_coef = 10;
% x_ref_init = [0*sin(ti)+initPos(1); 0*sin(ti)+initPos(2); x_ref_coef*cos(pi*ti/manueverTime)+initPos(3)]; 
% x_ref_init = [x_ref_coef*sin(ti)+initPos(1); x_ref_coef*sin(ti)+initPos(2); x_ref_coef*sin(ti)+initPos(3)]; 
% x_ref_init = [initPos(1);initPos(2);initPos(3)];
% 
% xdot_coef = 10;
% xdot_ref_init = [0*cos(ti); 0*cos(ti); -xdot_coef*pi/manueverTime*sin(pi*ti/manueverTime)];
% xdot_ref_init = [xdot_coef*cos(ti);xdot_coef*cos(ti); xdot_coef*cos(ti)];
% 
% % xdot_ref_init = [0;0;0];
% 
% thetr = 105*(pi/180)*(sin((pi*ti/manueverTime)-90*(pi/180)));
% % theta_ref_init = [initOri(1); thetr+initOri(2); initOri(3)];
% thetr = pi/4*sin(ti);
% theta_ref_init = [thetr+initOri(1); thetr+initOri(2); thetr+initOri(3)];
% 
% theta_ref_init = [initOri(1);initOri(2);initOri(3)];
% 
% thetr_dot = 105*(pi/180)*pi/manueverTime*(cos((pi*ti/manueverTime)-90*(pi/180)));
% % thetadot_ref_init = [0; thetr_dot; 0];
% 
% thetr_dot = pi/4*cos(ti);
% thetadot_ref_init = [thetr_dot; thetr_dot; thetr_dot];
% 
% x_ref_coef = 10;
% x_ref_init = [x_ref_coef*sin(ti)+initPos(1); x_ref_coef*sin(ti)+initPos(2); x_ref_coef*sin(ti)+initPos(3)]; 
% xdot_ref_init = [x_ref_coef*cos(ti); x_ref_coef*cos(ti); x_ref_coef*cos(ti)];
% tr = 5*pi/180*sin(2*pi/10/4.*ti) + 60*pi/180;
% theta_ref_init = [tr; tr; tr];
% tr_dot = 5*pi/180.*cos(2*pi/10/4.*ti);
% thetadot_ref_init = [tr_dot; tr_dot; tr_dot];

x_ref_init = x_ref(ti);
xdot_ref_init = xdot_ref(ti);
theta_ref_init = theta_ref(ti);
thetadot_ref_init = thetadot_ref(ti);



% thetadot_ref_init = [0;0;0];

% q_init = [28 -28 145 25 30 30]'*pi/180;
% q_init = 0*[0;-1.396263401595464;1.570796326794897;0;0;0];

err = getError_init(x_ref_init, theta_ref_init, q_init);
X0 = [q_init; err];
%% Simulation Setup
tspan = [0 manueverTime];
options = odeset('RelTol',1e-3,'AbsTol',1e-5);
tic
% [t, X] = ode45(@AR2KinDE, tspan, X0, options);
toc
% tic
% [t2, X2] = ode45(@AR2KinDE_anal, tspan, X0, options);
% toc
%new solver
tic
[tRK, XRK]=RK4_angles(@AR2KinDE,X0);
toc
%% Output
% q = X(:,1:6);
% err = X(:,7:12);

% %new output
% q2=X2(:,1:6);
% err2=X2(:,7:12);

% even newer output
qRK=XRK(:,1:6);
errRK=XRK(:,7:12);
%% Plots
% figure(1)
% plot(t,q*180/pi)
% xlabel('time [sec]')
% ylabel('Configuration variables [deg]')
% legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6')

%new plot
% figure
% plot(t2,q2*180/pi)

%new plot
figure
plot(tRK,qRK*180/pi)
xlabel('time [sec]')
ylabel('Configuration variables [deg]')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6')

% figure
% plot(t,q*180/pi)
% xlabel('time [sec]')
% ylabel('Configuration variables [deg]')
% legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6')

for i = 1:length(tRK)
   ex(i) = XRK(i,7);
   ey(i) = XRK(i,8);
   ez(i) = XRK(i,9);
   eth1(i) = XRK(i,10);
   eth2(i) = XRK(i,11);
   eth3(i) = XRK(i,12);
end
figure
plot(tRK,ex)
hold on
plot(tRK,ey)
hold on
plot(tRK,ez)
xlabel('time [sec]')
ylabel('Position error')
legend('x error', 'y error', 'z error')

figure
plot(tRK,eth1)
hold on
plot(tRK,eth2)
hold on
plot(tRK,eth3)
xlabel('time [sec]')
ylabel('Orientation error')
legend('\theta_4 error', '\theta_5 error', '\theta_6 error')

% 
% for i = 1: length(t)
%    q = X(i,1:6);
%    [pos, orient] = AR2fkine(q);
%    x(i) = pos(1);
%    y(i) =  pos(2);
%    z(i) = pos(3);
%    th1(i) = orient(1);
%    th2(i) = orient(2);
%    th3(i) = orient(3);
% end
% 
% xr = 10*sin(t)+200; 
% tr = 5*pi/180.*sin(2*pi/10/4*t) + 60*pi/180;
% 
% figure(2)
% plot(t, xr,'r'); hold on 
% plot(t,x)
% legend('Ref', 'actual')
% 
% figure(3)
% plot(t,xr-100,'r'); hold on
% plot(t,y)
% legend('Ref', 'actual')
% figure(4)
% plot(t,xr,'r'); hold on
% plot(t,z)
% legend('Ref', 'actual')
% 
% 
% figure(5)
% plot(t,tr*180/pi,'r');
% hold on
% plot(t,th1*180/pi)
% legend('Ref', 'actual')
% 
% figure(6)
% plot(t,tr*180/pi,'r');
% hold on
% plot(t,th2*180/pi)
% legend('Ref', 'actual')
% 
% figure(7)
% plot(t,tr*180/pi,'r');
% hold on
% plot(t,th3*180/pi)
% legend('Ref', 'actual')
