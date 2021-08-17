index = 1;
tFinal = 30;
for t = 0:0.001:tFinal
qd(index,:) = [      10*sin(t/tFinal)   10*sin(t/tFinal)   10*sin(t/tFinal)         10*sin(t/tFinal)       10*sin(t/tFinal)  10*sin(t/tFinal)  ]; % rad
qdotd(index,:) = [   10*cos(t/tFinal)   10*cos(t/tFinal)  10*cos(t/tFinal)         10*cos(t/tFinal)       10*cos(t/tFinal)   10*cos(t/tFinal)];
qddotd(index,:) = [  -10*sin(t/tFinal)  -10*sin(t/tFinal)  -10*sin(t/tFinal)        -10*sin(t/tFinal)           -10*sin(t/tFinal)    -10*sin(t/tFinal) ];

xddot(index,:) = jAccel2tAccel(qd(index,:)',qdotd(index,:)',qddotd(index,:)');
xddotNoJd(index,:) = jAccel2tAccelNoJd(qd(index,:)',qddotd(index,:)');

xddotError(index,:) = xddot(index,:) - xddotNoJd(index,:);

index = index + 1;
end

