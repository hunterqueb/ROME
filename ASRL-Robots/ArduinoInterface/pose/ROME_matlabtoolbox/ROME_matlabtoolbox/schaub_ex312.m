clear;clc;close all;
x0 = [40, 30, 80]*pi/180;
tspan = [0 60];
[t, x] = ode45(@eul, tspan, x0);
plot(t,x*180/pi)
function xdot = eul(t, x)
omega = [sin(0.1*t), 0.01, cos(0.01*t)]'*20*pi/180;

psi = x(1);
theta = x(2);
phi = x(3);
B = 1/cos(theta)*[0              sin(phi)                cos(phi);
     0              cos(phi)*cos(theta)   -sin(phi)*cos(theta);
     cos(theta)     sin(phi)*sin(theta)     cos(phi)*sin(theta)];
 xdot = B*omega;
end
