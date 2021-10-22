%%

% Constructed circle has 8*pi period, 1 meter radius
circle = Circle(8*pi,1,.1);
omni = omni();
[t,result] = omni.trajectory(circle);

pathW = circle.getPathW(t);
pathX = pathW(:,1);
pathY = pathW(:,2);
pathTheta = pathW(:,3);

errorX = result(:,1) - pathX;
errorY = result(:,2) - pathY;
errorTheta = result(:,3) - pathTheta;

errorXDot = result(:,4) - pathW(:,4);
errorYDot = result(:,5) - pathW(:,5);
errorThetaDot = result(:,6) - pathW(:,6);



plot(result(:,1),result(:,2));
hold on
plot(pathX,pathY);
axis equal
xlabel("X Position (m)");
ylabel("Y Position (m)");
xlim([-1.25,1.25]);
ylim([-1.25 1.25]);
legend("Simulated Trajectory","Input Path");
grid on

figure
plot(t,errorX);
hold on
plot(t,errorY);
plot(t,errorTheta);
xlim([0 3]);
legend("Error in X Position (m)","Error in Y Position (m)", "Error in Theta Position (rad)");
xlabel("Time (s)");
ylabel("Position Error");
grid on

figure
hold on
plot(t,errorXDot);
plot(t,errorYDot);
plot(t,errorThetaDot);
xlim([0 3]);
legend("Error in X Velocity (m/s)","Error in Y Velocity (m/s)", "Error in Theta Velocity (rad/s)");
xlabel("Time (s)");
ylabel("Velocity Error");
grid on

