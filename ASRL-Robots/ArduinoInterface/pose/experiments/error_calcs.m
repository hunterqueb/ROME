% calculates the error of the experiments to the simulations

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


for i = 1:length(qSaved(:,1))
   exArm(i) = qSaved(i,8);
   eyArm(i) = qSaved(i,9);
   ezArm(i) = qSaved(i,10);
   eth1Arm(i) = qSaved(i,11);
   eth2Arm(i) = qSaved(i,12);
   eth3Arm(i) = qSaved(i,13);
end
figure
plot(qSaved(:,1),exArm)
hold on
plot(qSaved(:,1),eyArm)
hold on
plot(qSaved(:,1),ezArm)
xlabel('time [sec]')
ylabel('Position error')
legend('x error', 'y error', 'z error')

figure
plot(qSaved(:,1),eth1Arm)
hold on
plot(qSaved(:,1),eth2Arm)
hold on
plot(qSaved(:,1),eth3Arm)
xlabel('time [sec]')
ylabel('Orientation error')
legend('\theta_4 error', '\theta_5 error', '\theta_6 error')