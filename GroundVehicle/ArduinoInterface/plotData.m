function plotData(pos,e,traj,time)

%example" plotData(position,error,trajectory,time)

xPos = pos(:,1);
yPos = pos(:,2);
thetaPos = pos(:,3);

xTraj = traj(:,1);
yTraj = traj(:,2);
thetaTraj = traj(:,3);

clf('reset')

%Produce plot of error
errorPlot = figure(1);
plot(time,e);
xlabel('time(s)');
ylabel('X(m),Y(m), Theta(rad)');
legend('X','Y','Theta');
 %saveas(errorPlot,'lineErrorPlot1.fig');
 %saveas(errorPlot,'lineErrorPlot1.jpg');

%Produce plot of trajectory vs position
trackingPlot = figure(2);
plot(xPos,yPos,'red','LineWidth',5);
hold on
plot(xTraj,yTraj,'black','LineWidth',1);
axis equal
xlabel('X(m)');
ylabel('Y(m)');
hold off

disp('Mean Error in X:');
disp(mean(abs(e(:,1))));

disp('Mean Error in Y:');
disp(mean(abs(e(:,2))));

disp('Mean Error in Theta:');
disp(mean(abs(e(:,3))));

 %saveas(trackingPlot,'lineTrackingPlot1.fig');
 %saveas(trackingPlot,'lineTrackingPlot1.jpg');

end