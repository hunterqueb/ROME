figure
plot(qSaved(:,1),qSaved(:,2:7)*180/pi)
xlabel('time [sec]')
ylabel('Configuration variables [deg]')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6')
title("Experiments")
yticks(-200:20:200);

qRK=XRK(:,1:6);
errRK=XRK(:,7:12);

figure
plot(tRK,qRK*180/pi)
xlabel('time [sec]')
ylabel('Configuration variables [deg]')
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6')
title("Simulations")
yticks(-200:20:200);
